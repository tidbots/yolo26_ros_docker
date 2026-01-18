# image_preprocess
照明の変化に対応するための前処理ROSノード

Raw Imageを
- Gamma補正（暗所/白飛び対策）
  - 暗い → γ < 1.0
  - 明るすぎ → γ > 1.0

- CLAHE（局所コントラスト正規化）
  - 通常のHistogram Equalizationより安全
  - 実機照明のムラに強い

## 設計方針
- CPUのみ・軽量
- 照明が暗すぎ / 明るすぎ の両方に対応
- パラメータはROS paramで調整可能
- YOLO26向け（白飛び・黒つぶれを防止）
- Docker内でそのまま動作

## 前提
- Ubuntu 20.04 / 22.04
- Docker / Docker Compose v2（docker compose が使える）
- ROS Master（通常は同じマシン、roscore）
- USBカメラ or 既存 /camera/image_raw が発行されている


## 構成
```
/camera/image_raw   (sensor_msgs/Image)
        ↓
[ preprocess_node ]
  - 明るさ推定
  - 自動Gamma補正
  - CLAHE（局所コントラスト補正）
        ↓
/camera/image_preprocessed
```

## 準備
```
git clone
cd
docker compose build

```

##　動作確認
```
# 起動
roslaunch your_pkg_name preprocess.launch

# 可視化
rqt_image_view /camera/image_preprocessed
```


