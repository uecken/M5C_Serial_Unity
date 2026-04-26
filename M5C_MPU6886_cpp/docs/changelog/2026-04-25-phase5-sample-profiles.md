# 2026-04-25 Phase 5: サンプルプロファイル 5 種 + 3D 球固定

## 1. 3D ビュアの球を固定化 (ユーザー指示)

### 変更
旧仕様: 球体ワイヤーフレームと M5StickC モデルが**両方一緒に**回転していた  
(M5StickC が球の子オブジェクトだったため)

新仕様: **球体は scene 直下で固定、M5StickC モデルだけ回転**  
(旧 motion_controller.js / findradio.jp 仕様)

### 理由
- 球体は「基準フレーム」「視点の手がかり」として固定されているべき
- 回転すると基準が消えてユーザーが姿勢の参照点を失う
- 当初の Phase 4.1 移植時にこの差異が見落とされていた

### コード変更 (`Web/hidconfig/src/lib/IMUViewer.js`)
- `this.sphere.add(this.m5StickC)` → `this.scene.add(this.m5StickC)`
- `_tick()` で `this.sphere.quaternion.slerp(...)` → `this.m5StickC.quaternion.slerp(...)`
- Body 軸は M5StickC の子のままなので一緒に回転 (これは意図通り)
- 球面ドット (current/closest/reference) と World 軸 / 重力ベクトルは scene 直下のまま (固定)

## 2. サンプルプロファイル 5 種

### 配信形式
- リポジトリ同梱 (`Web/hidconfig/profiles/`)
- スキーマは `rule.add` 簡易形式 (内部 state machine 表現ではなく **Web UI 側の入力形式**)
- 起動時に `index.json` を fetch して一覧表示
- 「適用」ボタンで `rule.clear` → 各 `rule.add` 直列送信 → `profile.save` で永続化

### 実装方針: なぜ rule.add 形式で配布するか
- Profile.hpp の `serializeState()` 形式は state machine の内部表現で人間が読み書きしづらい
- `rule.add` 簡易スキーマなら `posture.euler`, `key`, `accel_abs_threshold` 等の高レベル指定で済む
- 人間がプロファイル JSON を直接編集して PR/共有できる (Phase 6 GitHub 共有リポジトリへの布石)

### 5 種の内訳

| ID | タイトル | 用途 | ルール数 |
|----|---------|------|---------|
| `presentation` | 🎤 プレゼンテーション | 右傾け→次 (n)、左傾け→前 (p)、振り→ポインタ (b) | 3 |
| `fps_wasd` | 🎮 FPS WASD (傾け移動) | 4 方向の HOLD_START_ONLY で WASD、振り→Space | 5 |
| `accessibility_one_motion` | ♿ アクセシビリティ - 1 モーション 1 キー | 振り 1 つで Space (1 秒クールダウン) | 1 |
| `media_remote` | 🎵 メディアリモコン | YouTube キー (k/j/l) を傾けで | 3 |
| `street_fighter` | 🥊 ストリートファイター 風 | 右傾け+加速→p、左傾け+加速→k、振り下ろし→h | 3 |

### Phase 6+ の発展余地
- **矢印キー / Function キー対応**: 現在 ASCII のみ。`r["key"]` を `"ARROW_RIGHT"` 等の特殊名でも受け付けるよう FW を拡張
- **マウス出力サンプル**: msg_format=2 の Mouse モード (FW で実装済) を Web UI で生成
- **SEQUENCE モードを使った真の波動拳**: 現状はストリートファイター "風" のみ、本格コマンド入力は SEQUENCE で
- **GitHub 共有リポ** (`motion-controller-profiles`): jsDelivr 経由で外部のコミュニティプロファイルも取得可能に

### スクリーンショット要点
- Connect 後、左ペインに「🎁 サンプルプロファイル」セクションが表示
- 5 種のカードが並ぶ (タイトル、説明、タグ、ルール数、適用ボタン)
- 適用中はステータスバナー表示 (`ルール 1/5: move_forward` 等)
- 完了で `✅ "🎮 FPS WASD" を 5 ルールで読み込み完了`

## ビルド結果
FW 変更なし。Web のみ。
公開: https://uecken.github.io/M5C_Serial_Unity/ (v20260425-214540)

## ファイル変更
- 改訂: `Web/hidconfig/src/lib/IMUViewer.js` (球固定化)
- 新規: `Web/hidconfig/profiles/index.json`
- 新規: `Web/hidconfig/profiles/presentation.json`
- 新規: `Web/hidconfig/profiles/fps_wasd.json`
- 新規: `Web/hidconfig/profiles/accessibility_one_motion.json`
- 新規: `Web/hidconfig/profiles/media_remote.json`
- 新規: `Web/hidconfig/profiles/street_fighter.json`
- 改訂: `Web/hidconfig/src/app.js` (samples state + handleSampleLoad + UI ブロック)
- 新規: `docs/changelog/2026-04-25-phase5-sample-profiles.md`
