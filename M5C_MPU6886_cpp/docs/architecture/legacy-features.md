# 旧 Web UI (findradio.jp/motioncontroller) 機能解析

URL: https://findradio.jp/motioncontroller/motion_controller_updater.html
JS:  https://findradio.jp/motioncontroller/motion_controller.js (約 1700 行)

実装は jQuery + Bootstrap 4 + Three.js r128 + esp-web-tools v8。
新版 (Phase 2-3) で取り込み済の機能と、まだ移植していない機能を整理。

---

## ページ構成 (5 タブ)

1. **page1**: シリアル & 3D表示 (本ドキュメントのメイン対象)
2. **page2**: キーマップ書き込み (PK3 構造体)
3. **page3**: Firmware 書き込み (esp-web-tools)
4. **page4**: 重力ベクトル ビュー
5. **page5**: 傾斜インジケータ

---

## page1 「シリアル & 3D表示」の主要機能

### A. Quaternion 3D 球体表示 (`#threejsContainer`)
スクリーンショット: 白いワイヤーフレーム球の中心に小さなオレンジ M5StickC モデル、
Roll/Pitch のドット可視化が下に並ぶ。

#### シーン構成
```js
// 球体ワイヤーフレーム (スケール基準)
const sphereGeometry = new THREE.SphereGeometry(1, 32, 32);
sphereMaterial = MeshBasicMaterial({ color: 0xffffff, wireframe: true });

// M5StickC モデル (Box 0.24×0.12×0.48)
m5StickC = Mesh(stickGeometry, [
    orange,  // +X
    orange,  // -X
    black,   // +Y (天面)
    orange,  // -Y
    orange,  // +Z
    white    // -Z
]);
sphere.add(m5StickC);  // M5C を球の子に → 球と一緒に Quat 回転

// LCD パネル (Plane 0.2×0.05)
display = Mesh(planeGeo, blackMat);
display.position.set(0, 0.06, 0.24);
m5StickC.add(display);

// LED マーク (Plane 0.04×0.02)
mark.position.set(0, -0.06, 0.24);
m5StickC.add(mark);

camera.position.z = -2;
```

#### 軸表示 (チェックボックスで切替可)
| チェックボックス | 表示 | サイズ | 親 |
|----------------|------|--------|------|
| `Show World Axis` | ワールド座標軸 (赤=X, 緑=Y, 青=Z、固定) | AxesHelper 1.8 | scene |
| `Show Body Axis`  | M5C ボディ座標軸 (連動して回転) | AxesHelper 0.8 | m5StickC |
| `Show Gravity Vector` | 推定重力方向 (水色矢印) | CylinderGeometry 0.02×1 | scene |

#### 重力ベクトル詳細 (gravityArrowGroup)
- 軸: `CylinderGeometry(0.02, 0.02, 1, 8)` = 円柱で太い線
- 色: `0x00aaff` (水色)
- 動的に長さ更新: `updateArrow(arrow, x, y, z)` で重力ベクトルに合わせる

#### Quaternion 回転処理
```js
// ストリームの quat (qw,qx,qy,qz) を受信時:
// 内部表現: THREE.Quaternion(-qx, qz, qy, qw) ← 軸再マッピング
//   理由: M5StickC IMU 軸 → Three.js シーン軸の変換 (X反転、Y/Z 入替)
const quaternion = new THREE.Quaternion(-qx, qz, qy, qw);
const rotateQ = base_q.clone().multiply(q_ref).invert().multiply(quaternion);
sphere.quaternion.copy(rotateQ);  // 球体ごと回転 (M5C も回転する)
```

#### Init Yaw / Use Base Upright
- **Init Yaw ボタン**: 現在の quat を `base_q` として保存 → 以降は base からの相対回転
- **Use Base Upright チェック**: 直立基準 quat に切替 (`q_ref` プリセット)

### B. Roll/Pitch 2D グリッド (`#pitchRollCanvas`)
スクリーンショット: 青いグリッド (-180～180°ロール、-90～90°ピッチ)、現在位置に赤丸。

```js
// pitchRollCanvas: 360x180 程度のキャンバス
// 軸: X = roll [-180, +180], Y = pitch [-90, +90]
function updatePitchRollCanvas(pitch, roll) {
    ctx.clearRect(0, 0, w, h);
    drawAxis(ctx);  // グリッド + ラベル描画
    // 登録済み参照点 (青/オレンジ)
    bluePoints.forEach(point => { /* 描画 */ });
    // 現在位置 (赤)
    const x = (roll + 180) * (w / 360);
    const y = (pitch + 90)  * (h / 180);
    ctx.fillStyle = 'red';
    ctx.fill();
}
```

#### グリッド描画 (drawAxis)
- 白背景、灰色グリッド線
- X 軸ラベル: -180, -150, ..., 150 (30°毎)
- Y 軸ラベル: -60, -30, 0, 30, 60, 90 (30°毎)
- 軸交点 (0, 0) は赤、参照ポイントは青やオレンジ

### C. 球面投影 (Quaternion 角度差判定)
```js
function plotIntersectionWithSphere(q, type) {
    // 単位球 (z=1) ベクトルを quat で回転 → 球面上の点を得る
    const direction = new THREE.Vector3(0, 0, 1);
    rotate_q = base_q.clone().multiply(q_ref).invert().multiply(q);
    direction.applyQuaternion(rotate_q).normalize();
    const intersection = direction.clone().multiplyScalar(1);

    // 表示色 (用途別)
    let color;
    if (type === "selected_pk")        color = blue/purple;   // 現在姿勢
    else if (type === "pk_references") color = tomato/orange; // 登録姿勢
    else if (type === "closest_reference") color = 0x00ff00;  // 最近傍 (緑)

    // 球面に小さな球を配置
    const sphereDot = Mesh(SphereGeometry(0.05));
    sphereDot.position.copy(intersection);
    scene.add(sphereDot);
}
```

### D. **Closest Reference 自動選択** (重要、未移植)
これが「登録姿勢のうち現状姿勢に最も近いものを適用」のロジック。

```js
// sensor_data 受信時:
let minAngle = Infinity;
let closestReference = null;
referencesData.forEach(reference => {
    const refQuaternion = new THREE.Quaternion(-reference.qx, reference.qz, reference.qy, reference.qw);
    const angle = quaternion.angleTo(refQuaternion);  // クォータニオン角度差
    if (angle < minAngle) {
        minAngle = angle;
        closestReference = reference;
    }
});

if (closestReference) {
    // 球面上の最近傍点を 緑色 でハイライト
    plotIntersectionWithSphere(...closestReference, "closest_reference");
    // 0.5s 後に色をトマトに戻して、最新の selectedSphere を緑へ
    setTimeout(() => selectedSphere.material.color.set(0xff6347), 500);
}
```

**仕様**:
- 各登録 pk3 (referencesData) と現在 quat の **angleTo** を計算
- 最小角度の reference を選択 → その時点の HID 入力を発火するという思想
- FW 側でも `getClosestPK3` ロジックがあり、Web/FW 両方で同じ概念

**新版 Phase 3 では複数ルールが**全部独立評価される実装**だが、
旧版は**最近傍 1 個だけ採用**で、姿勢が連続的に変わる UI に向く。

### E. PK3 参照テーブル
- `pk_references` 行を受信したら `referencesData` 配列に push
- `displayReferencesTable()` で HTML テーブル化
- 球面と Roll/Pitch グリッド両方に座標プロット

---

## page4 「重力ベクトル」 (initGravityVisualization)

### 機能
- 別 Three.js シーン (`gravityScene`)
- 静止重力推定値 (`gravX/Y/Z`) を矢印で表示
- 加速度 (動的) と 推定重力 (Mahony 出力) を別矢印で対比
- 4 軸グラフ: X, Y, Z それぞれ accX vs gravX を時系列比較

### コード (関連関数)
```
initGravityVisualization()
createArrow(color, name)       矢印 (CylinderGeometry + ConeGeometry)
createM5StickCForGravity()
updateArrow(arrow, x, y, z)
addAxisLabels()
animateGravity()
updateGravityVisualization(accX, accY, accZ, gravX, gravY, gravZ)
updateGravityCompareCanvas()
drawAxisGraph(canvasId, axisName, accKey, gravKey)
```

### グラフ
- 各軸 (X/Y/Z) について 横軸=時間、縦軸=値
- 加速度生 = 細線、推定重力 = 太線
- 重力 fusion がうまく走っているかを視覚確認

---

## page5 「傾斜インジケータ」 (initTilt3D + updateTiltIndicator)

### 機能
- もう一つの 3D シーン: M5C を真上から見下ろした立体ビュー
- 傾き具合を「水準器」風 2D Canvas (drawBubbleLevel)
- アクションルール発火時に色変化 (`updateTriggerIndicators(pitch, roll)`)
- 姿勢インジケータ (drawAttitudeIndicator) — 飛行機のスタビライザー風

---

## 旧版が新版より進んでいた機能 (移植候補)

| 機能 | 旧版 | 新版 (Phase 3) | 移植優先度 |
|------|------|----------------|-----------|
| 球体ワイヤーフレーム | ✅ 中央に M5C | ❌ (グリッド+M5C のみ) | **高 (見栄え)** |
| World/Body 軸切替 | ✅ チェックボックス | ❌ | **高** |
| 重力ベクトル矢印 | ✅ 水色矢印 | ❌ | **中** |
| **Closest Reference 自動選択** | ✅ Quat angleTo 最小 | ❌ (全ルール独立評価) | **高 (アルゴリズム)** |
| Roll/Pitch 2D グリッド | ✅ ドットプロット | ❌ | **高 (登録時の補助)** |
| 球面ドット表示 | ✅ 緑 (最近傍)/青 (現在)/橙 (登録) | ❌ | **中** |
| Init Yaw / Base Upright | ✅ q_ref プリセット | ⚠ FW 側にコマンドあるが UI なし | 中 |
| 重力比較グラフ (X/Y/Z) | ✅ Acc vs Grav 線グラフ | ❌ | 低 (デバッグ用) |
| 傾斜インジケータ (page5) | ✅ 水準器風 + 計器盤風 | ❌ | 低 |
| Bubble Level 2D | ✅ 円形水準器 | ❌ | 低 |

### 旧版にあったが廃止すべき機能

- jQuery 依存 → Preact で代替済み
- Bootstrap 4 → Tailwind で代替済み
- HTML 文法エラーあり (壊れている、新版で再構築済)

---

## 移植計画 (Phase 4)

### Phase 4.1: 3D 強化 (このセッションで実装)
1. **球体ワイヤーフレーム** (`SphereGeometry wireframe`)
2. **M5C モデルを球の子に** (球の中心、向き連動)
3. **World/Body 軸切替** (チェックボックス)
4. **重力ベクトル矢印** (Mahony 出力から推定 grav 計算)
5. **Roll/Pitch 2D グリッド** (登録ポイント可視化付き)
6. **球面ドット**: 現在姿勢 (赤)、登録ルール姿勢 (橙)、最近傍 (緑)

### Phase 4.2: Closest Rule 選択ロジック (このセッションで実装)
- FW 側に新コマンド `rule.closest` 追加
  - 現在の quat と登録ルールの quat を比較
  - 角度差が最小のルールを返す
  - 「ボタン押下時のみ最近傍を発火」のような選択モードを追加
- Web UI: 球面ドットのリアルタイム最近傍ハイライト

### Phase 4.3: q_ref UI (将来)
- Init Yaw / Use Base Upright / Use Base Horizontal ボタン
- 現在 FW にコマンドはあるが UI なし

---

## 旧 → 新の Quaternion 軸変換式

旧版で使っていた `THREE.Quaternion(-qx, qz, qy, qw)` の意味:
- M5StickC IMU の軸は (X=右, Y=前, Z=上)
- Three.js デフォルトカメラは (X=右, Y=上, Z=前)
- 軸変換: M5C(X,Y,Z) → Three(X,Z,Y) で X 反転 (左手系→右手系の補正)

新版では IMUViewer.js が `setQuaternion(qw, qx, qy, qz)` で同様の変換をしているか要確認。
スクリーンショットでは正しく表示されているので問題なさそう。

---

## 引用ファイル
- `/c/tmp/legacy_mc.js` (motion_controller.js のローカル copy)
- `/c/tmp/legacy_mcu.html` (motion_controller_updater.html のローカル copy)
