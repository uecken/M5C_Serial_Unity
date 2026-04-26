# 2026-04-25 Phase 4: 旧 UI 機能の移植 + Closest Rule

## 背景
ユーザー指摘:
1. 旧版 (findradio.jp/motioncontroller) の **Quaternion 3D 球体表示**
2. **Roll/Pitch 2 次元表示**
3. **アクションルールの開始姿勢: 登録姿勢と現状姿勢に最も近いルールを適用** という旧仕様

これらが新版に移植されていなかったため、解析・実装した。

## 解析結果
**docs/architecture/legacy-features.md** に旧 UI の全機能を整理:
- ページ構成 5 タブ (シリアル&3D / キーマップ / FW書込み / 重力ベクトル / 傾斜)
- 球体ワイヤーフレーム + M5StickC モデル + 軸表示の構造
- Roll/Pitch 2D Canvas (`updatePitchRollCanvas` + `plotPoint` + `drawAxis`)
- Closest Reference 自動選択 (`quaternion.angleTo`)
- 重力ベクトル (4 軸グラフ含む)
- 傾斜インジケータ (水準器)
- Quaternion 軸変換 `THREE.Quaternion(-qx, qz, qy, qw)` の意味

## Phase 4.1: 3D ビュア大幅拡張

### IMUViewer.js リライト
- **球体ワイヤーフレーム** (旧版互換): `SphereGeometry(1, 24, 24)` + `wireframe: true`
- **M5StickC モデル**を球の子として配置 (球と一緒に回転)
  - Box 0.24×0.12×0.48、+Y 面が黒 LCD、-Z 面が白 (旧版互換のマテリアル割当)
  - LCD パネル + LED マーク
- **World 軸** (チェックボックスで切替): AxesHelper 1.8、scene 直下、固定
- **Body 軸**: AxesHelper 0.7、M5StickC の子、連動回転
- **重力ベクトル矢印**: CylinderGeometry + ConeGeometry、水色 0x00aaff、scene 直下
- **球面ドット**:
  - 🔴 現在姿勢 (`setCurrentDot`): SphereGeometry(0.06)
  - 🟢 最近傍 (`setClosestDot`): SphereGeometry(0.07)
  - 🟠 登録ルール (`setReferenceQuaternions`): SphereGeometry(0.05)
- **Init Yaw** ボタン: 現在 quat を `qRef` として保存、以降は base からの相対回転
- 軸変換: 旧版互換 `THREE.Quaternion(-qx, qz, qy, qw)` で M5C IMU 軸 → Three.js 軸

### PitchRollGrid.js (新規)
Roll [-180°, +180°] × Pitch [-90°, +90°] の 2D 平面に:
- 🔴 現在姿勢を**赤丸**
- 🟠 登録ルール姿勢を**橙丸** + 名前ラベル
- 🟢 最近傍を**緑丸** (大きめ)
- 30°毎のグリッド線 + 軸ラベル
- 中央十字 (0°ロール、0°ピッチ)
- Roll →、↑ Pitch のラベル

### 統合 (app.js)
- 3D ビューに **「Init Yaw」「Reset Base」**ボタン追加
- 軸切替 3 つのチェックボックス (旧版互換)
- センサーストリーム受信時:
  - 3D viewer の Quaternion 更新
  - 重力ベクトル更新 (sensor.ax/ay/az → 正規化)
  - 2D グリッドの現在ドット更新
  - **最近傍ルール計算** (Quaternion 内積から角度差、最小値の rule を選定)
- 「アクションルール」フラッシュ時に最近傍 ID を表示

## Phase 4.2: Closest Rule 選択

### FW
- `rule.list` 応答に **posture (euler, euler_tol)** を含めるよう拡張
- **新コマンド `rule.closest`**: 現在の `g_sensor_state.quat` と各 rule.states[0].posture.quat の**角度差最小**を返す
- 角度計算: `2 * acos(|dot|)` (0 ~ π rad)
- `rule.add` で `posture.quat` を JSON で受信、未指定なら現在 sensor の quat を保存

### Web
- `rule.add` の posture に `quat: [qw, qx, qy, qz]` を含める (姿勢キャプチャ時の sensor.qw/qx/qy/qz)
- `rule.list` 応答の posture からルール座標を 2D グリッドと 3D 球面に表示
- リアルタイム最近傍計算 (Quaternion 内積、ブラウザ JS)
- 「アクションルール」セクションに**最近傍ルール名**表示
- 旧版互換: `localStorage` に姿勢キャプチャ値を保存 (FW 応答にない場合のフォールバック)

### 検証 (実機)
```jsonc
→ rule.add r:{id:200, posture:{euler:[0,0,0],quat:[1,0,0,0]}, key:'a'}
→ rule.add r:{id:201, posture:{euler:[45,0,0],quat:[0.924,0.383,0,0]}, key:'b'}

→ rule.list
← rules:[{id:200, posture:{euler:[0,0,0], euler_tol:[10,10,60]}},
         {id:201, posture:[euler:[45,0,0], euler_tol:[10,10,60]}]]

→ rule.closest  (現在姿勢: ほぼ flat)
← {"id":200, "angle_rad":0.019822}   ← flat (id 200) が選ばれる、~1°
```

## 公開
https://uecken.github.io/M5C_Serial_Unity/

### スクリーンショット要点
- 球体ワイヤーフレームの中心に M5StickC モデル → 旧版とほぼ同じ見た目
- Roll/Pitch 2D グリッドが新パネルとして追加
- 「アクションルール」フラッシュ時に最近傍 ID 表示

## 残課題
- **HOLD_START_END で開始/終了で別キー** (現状同じキーの press/release)
- 4.2 で実装した Closest Rule を**実際のトリガー判定に組み込む**: 現状は表示のみ、ボタン押下時に最近傍だけ発火するモードを追加可能
- 重力比較グラフ (旧版 page4) は移植せず (デバッグ用機能、優先度低)
- 傾斜インジケータ (旧版 page5) は移植せず (重複機能)

## ファイル変更
- 新規: `Web/hidconfig/src/lib/PitchRollGrid.js`
- 新規: `docs/architecture/legacy-features.md` (旧 UI 解析資料)
- 大幅改訂: `Web/hidconfig/src/lib/IMUViewer.js`
- 改訂: `Web/hidconfig/src/app.js` (軸切替、Roll/Pitch グリッド統合、最近傍計算)
- 改訂: `src/main_v2.cpp` (rule.list に posture 含める、rule.closest 追加、rule.add で quat 保存)
