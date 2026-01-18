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
git clone git@github.com:tidbots/image_preprocess.git
cd image_preprocess
docker compose build

```

##　動作確認
```
# 起動
roslaunch your_pkg_name preprocess.launch

# 可視化
rqt_image_view /camera/image_preprocessed
```

## パラメータチューニング指針（会場照明別）
対象ノード：
- image_preprocess（Gamma / CLAHE）

### 0. チューニングの基本方針（重要）
1. まず前処理（画像の見え）を安定させる
2. 次に YOLO の conf / tile を調整
3. 最後に Depth ROI を詰める

👉 いきなり YOLO 側を触らないのがコツ

### 1.照明パターン別・推奨設定
#### A. 暗い会場（夕方・照度不足・影が強い）
症状
- 全体が暗い
- 小物が背景に溶ける
- confidence が全体的に低い

image_preprocess
```
gamma: 1.3 〜 1.6
clahe_clip: 3.0
clahe_grid: 8
```

✅ ポイント
- 暗い会場では gamma ↑ が最優先

#### B. 明るすぎる会場（白飛び・直射照明）
症状
- 白い床・テーブルが飛ぶ
- ハイライトで物体輪郭が消える

image_preprocess
```
gamma: 0.75 〜 0.9
clahe_clip: 1.5
clahe_grid: 8
```

✅ ポイント
- gamma < 1.0 で白飛び抑制
- CLAHE を強くしすぎない（ノイズ化する）

#### C. ムラのある照明（スポットライト・影あり）
症状
- 場所によって明るさが違う
- 同じ物体が認識されたりされなかったり

image_preprocess
```
gamma: auto（暗→1.3 / 明→0.85）
clahe_clip: 2.5
clahe_grid: 8
```
※ auto は平均輝度で切り替え（実装済みなら有効）

#### D. 理想的な会場（均一・十分な照度）
症状
- 全体が見やすい
- 認識は安定

image_preprocess
```
gamma: 1.0
clahe_clip: 2.0
clahe_grid: 8
```

### 3. 会場入り後の「5分チューニング手順」
#### ① 画像を見る
```
rqt_image_view /camera/image_preprocessed
```
- 暗い → gamma ↑
- 白飛び → gamma ↓

### 4. 鉄板プリセット
万能スタート設定（迷ったらこれ）
```
gamma: 1.1
clahe_clip: 2.5
```


## 照明変化に対するパラメータの自動再チューニング
- image_preprocess：自動Gamma/CLAHE（照明変化の主因をここで吸収）

launch でON/OFF可

### 使い方（ON/OFF の切替）
自動再チューニング ON（推奨）
```
launch 内で auto_tune_enable=true
```

OFF（固定パラメータで運用）
```
launch 内で auto_tune_enable=false
```

### 方針
```
camera
  ↓
image_preprocess
  ├─ 輝度統計（mean / std）
  ├─ gamma 自動調整
  └─ clahe 自動調整
```

### 照明変化の検出（image_preprocess側）
#### ① 監視指標（軽量・確実）
各フレームで以下を計算：

指標	意味
- mean_luma	全体の明るさ
- std_luma	明るさのばらつき
- sat_ratio	白飛び率（>245）
- dark_ratio	黒潰れ率（<10）

#### 照明状態の分類（例）
状態	条件
- DARK	mean < 90
- BRIGHT	mean > 170
- SATURATED	sat_ratio > 0.15
- LOW_CONTRAST	std < 35
- NORMAL	上記以外

※ 10フレーム移動平均で判定（瞬間変化に反応しない）  

### Gamma / CLAHE の自動調整（preprocess）
基本ルール（安全側）
- 暗い → gamma ↑
- 明るい → gamma ↓
- 白飛び → gamma ↓ + clahe_clip ↓
- コントラスト低 → clahe_clip ↑

実際の制御例
```
if state == "DARK":
    gamma = min(gamma + 0.05, 1.6)
elif state == "BRIGHT":
    gamma = max(gamma - 0.05, 0.7)
elif state == "SATURATED":
    gamma = max(gamma - 0.08, 0.75)
    clahe_clip = max(clahe_clip - 0.2, 1.5)
elif state == "LOW_CONTRAST":
    clahe_clip = min(clahe_clip + 0.3, 3.5)
```
⚠️ 1フレームで大きく変えない（±0.05）


### フィードバックループ（重要）
```
照明変化
 ↓
preprocess 自動調整
 ↓
yolo confidence 改善？
 ↓
YES → 何もしない
NO  → yolo 側も微調整
```

絶対にやらないこと
- モデル切替
- imgsz 変更
- 再学習
- ノード再起動

### 競技向けフェイルセーフ設計
状態遷移（イメージ）
```
NORMAL
 ↓（照明変化）
ADJUSTING
 ↓（改善）
STABLE
 ↓（失敗）
DEGRADED（conf↓ tile↑ ROI緩和）
```

DEGRADED 状態でも動き続ける
- タスク中に止まらない


