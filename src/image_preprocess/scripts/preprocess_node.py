#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from dataclasses import dataclass

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


@dataclass
class BrightnessStats:
    mean: float = 0.0
    std: float = 0.0
    sat_ratio: float = 0.0  # ratio of pixels > sat_thr
    dark_ratio: float = 0.0 # ratio of pixels < dark_thr


class PreprocessNode:
    """
    Image preprocess for illumination robustness:
      - Gamma correction
      - CLAHE (on L channel)

    Auto re-tuning (optional):
      - Monitors brightness stats (mean/std/saturation/dark)
      - Slowly adjusts gamma & CLAHE clip-limit within bounds
      - Rate-limited updates (stable for RoboCup venue)
    """

    def __init__(self):
        rospy.init_node("image_preprocess_node")

        # Topics
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/camera/image_preprocessed")
        self.debug_topic = rospy.get_param("~debug_topic", "/camera/image_preprocess_debug")
        self.debug_enable = bool(rospy.get_param("~debug_enable", False))

        # Preprocess base params (used when auto_tune_enable=False)
        self.gamma = float(rospy.get_param("~gamma", 1.10))
        self.clahe_clip = float(rospy.get_param("~clahe_clip", 2.5))
        self.clahe_grid = int(rospy.get_param("~clahe_grid", 8))

        # Auto tuning enable
        self.auto_tune_enable = bool(rospy.get_param("~auto_tune_enable", False))

        # Auto tuning targets & thresholds
        self.target_mean = float(rospy.get_param("~target_mean", 125.0))  # 0..255
        self.dark_mean_thr = float(rospy.get_param("~dark_mean_thr", 90.0))
        self.bright_mean_thr = float(rospy.get_param("~bright_mean_thr", 170.0))
        self.low_contrast_std_thr = float(rospy.get_param("~low_contrast_std_thr", 35.0))

        self.sat_thr = int(rospy.get_param("~sat_thr", 245))
        self.dark_thr = int(rospy.get_param("~dark_thr", 10))
        self.sat_ratio_thr = float(rospy.get_param("~sat_ratio_thr", 0.12))
        self.dark_ratio_thr = float(rospy.get_param("~dark_ratio_thr", 0.12))

        # Auto tuning step sizes (small & safe)
        self.gamma_step = float(rospy.get_param("~gamma_step", 0.05))
        self.gamma_step_saturated = float(rospy.get_param("~gamma_step_saturated", 0.08))
        self.clahe_step = float(rospy.get_param("~clahe_step", 0.2))

        # Bounds
        self.gamma_min = float(rospy.get_param("~gamma_min", 0.70))
        self.gamma_max = float(rospy.get_param("~gamma_max", 1.60))
        self.clahe_min = float(rospy.get_param("~clahe_min", 1.2))
        self.clahe_max = float(rospy.get_param("~clahe_max", 3.8))

        # Rate limiting (avoid oscillation)
        self.update_every_n = int(rospy.get_param("~auto_tune_update_every_n", 8))  # frames
        self.min_update_interval = float(rospy.get_param("~auto_tune_min_update_interval", 0.25))  # sec

        # Smoothing (EMA)
        self.ema_alpha = float(rospy.get_param("~ema_alpha", 0.15))

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.pub_dbg = rospy.Publisher(self.debug_topic, Image, queue_size=1) if self.debug_enable else None
        self.sub = rospy.Subscriber(self.input_topic, Image, self.cb, queue_size=1)

        self._frame = 0
        self._last_update_t = 0.0
        self._ema = BrightnessStats()

        rospy.loginfo("image_preprocess ready: in=%s out=%s auto_tune=%s",
                      self.input_topic, self.output_topic, self.auto_tune_enable)

    def _compute_stats(self, bgr: np.ndarray) -> BrightnessStats:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        mean = float(gray.mean())
        std = float(gray.std())
        sat_ratio = float(np.mean(gray > self.sat_thr))
        dark_ratio = float(np.mean(gray < self.dark_thr))
        return BrightnessStats(mean=mean, std=std, sat_ratio=sat_ratio, dark_ratio=dark_ratio)

    def _ema_update(self, s: BrightnessStats):
        a = self.ema_alpha
        self._ema.mean = (1 - a) * self._ema.mean + a * s.mean
        self._ema.std = (1 - a) * self._ema.std + a * s.std
        self._ema.sat_ratio = (1 - a) * self._ema.sat_ratio + a * s.sat_ratio
        self._ema.dark_ratio = (1 - a) * self._ema.dark_ratio + a * s.dark_ratio

    def _auto_tune(self):
        """
        Adjust gamma & CLAHE based on EMA stats.
        Safe, small steps, bounded, rate-limited.
        """
        now = time.time()
        if (self._frame % self.update_every_n) != 0:
            return
        if (now - self._last_update_t) < self.min_update_interval:
            return

        mean = self._ema.mean
        std = self._ema.std
        sat = self._ema.sat_ratio
        dark = self._ema.dark_ratio

        gamma = self.gamma
        clahe = self.clahe_clip

        # Priority: saturation (white-out) first
        if sat > self.sat_ratio_thr:
            gamma = max(self.gamma_min, gamma - self.gamma_step_saturated)
            clahe = max(self.clahe_min, clahe - self.clahe_step)
        else:
            # Dark / bright control
            if mean < self.dark_mean_thr or dark > self.dark_ratio_thr:
                gamma = min(self.gamma_max, gamma + self.gamma_step)
            elif mean > self.bright_mean_thr:
                gamma = max(self.gamma_min, gamma - self.gamma_step)

            # Contrast control (gentle)
            if std < self.low_contrast_std_thr:
                clahe = min(self.clahe_max, clahe + self.clahe_step)
            else:
                # If contrast is already fine, slowly relax CLAHE back toward nominal (2.0~2.5)
                # to avoid over-enhancement noise.
                target = 2.3
                if clahe > target + 0.2:
                    clahe = max(self.clahe_min, clahe - self.clahe_step * 0.5)
                elif clahe < target - 0.2:
                    clahe = min(self.clahe_max, clahe + self.clahe_step * 0.3)

        # Apply
        if (abs(gamma - self.gamma) > 1e-6) or (abs(clahe - self.clahe_clip) > 1e-6):
            self.gamma = float(np.clip(gamma, self.gamma_min, self.gamma_max))
            self.clahe_clip = float(np.clip(clahe, self.clahe_min, self.clahe_max))
            self._last_update_t = now

            rospy.loginfo_throttle(
                1.0,
                "auto_tune preprocess: gamma=%.2f clahe=%.2f (mean=%.1f std=%.1f sat=%.2f dark=%.2f)",
                self.gamma, self.clahe_clip, mean, std, sat, dark
            )

    @staticmethod
    def _apply_gamma(bgr: np.ndarray, gamma: float) -> np.ndarray:
        if gamma <= 0:
            return bgr
        inv = 1.0 / gamma
        table = (np.linspace(0, 1, 256) ** inv * 255.0).astype(np.uint8)
        return cv2.LUT(bgr, table)

    @staticmethod
    def _apply_clahe(bgr: np.ndarray, clip: float, grid: int) -> np.ndarray:
        grid = max(2, int(grid))
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=float(clip), tileGridSize=(grid, grid))
        l2 = clahe.apply(l)
        lab2 = cv2.merge([l2, a, b])
        return cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)

    def cb(self, msg: Image):
        self._frame += 1
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        # stats on raw (before preprocess), then EMA + auto tune
        s = self._compute_stats(bgr)
        self._ema_update(s)

        if self.auto_tune_enable:
            self._auto_tune()

        # preprocess
        out = self._apply_gamma(bgr, self.gamma)
        out = self._apply_clahe(out, self.clahe_clip, self.clahe_grid)

        out_msg = self.bridge.cv2_to_imgmsg(out, "bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        # optional debug overlay
        if self.debug_enable and self.pub_dbg is not None:
            dbg = out.copy()
            cv2.putText(dbg, f"gamma={self.gamma:.2f} clahe={self.clahe_clip:.2f} grid={self.clahe_grid}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(dbg, f"mean={self._ema.mean:.1f} std={self._ema.std:.1f} sat={self._ema.sat_ratio:.2f} dark={self._ema.dark_ratio:.2f}",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
            dbg_msg.header = msg.header
            self.pub_dbg.publish(dbg_msg)


if __name__ == "__main__":
    PreprocessNode()
    rospy.spin()
