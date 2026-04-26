# 2026-04-25 Phase 3 完了

## Phase 3.1: アクションルール大幅拡張

### Web UI
- **モード**: ONESHOT / HOLD_START_ONLY / **HOLD_START_END** (3種)
- **姿勢キャプチャ**:
  - 「📷 開始姿勢」: 現在の Euler を記録、許容 ± 度 設定可
  - 「📷 終了姿勢」: HOLD_START_END 時のみ表示、別姿勢を記録
  - キャプチャ後の値表示 (R/P/Y deg)
  - × でクリア
- **修飾キー**: Ctrl / Shift / Alt / Win チェックボックス
- **加速度トリガ**: 有効/無効 + 閾値 (0 で無効)
- **複合 AND**: 加速度+姿勢+ボタンの組合せが Condition に展開
- **ルール名**: 自由入力 (空なら自動)

### FW (rule.add 拡張)
- `r.modifiers` (uint8): bit0=Ctrl, bit1=Shift, bit2=Alt, bit3=GUI
- `r.posture / r.end_posture`: Euler + tol で開始/終了姿勢
- HOLD_START_END で states[0]=press、states[1]=release を自動構築
- 修飾キーは BleCombo の HID キーコード (0x80-0x83) で keys[] 先頭に配置 → fire_once で全 press → 待機 → 全 release

### 検証 (実機)
```jsonc
// Ctrl+C ONESHOT
→ rule.add r:{id:100, ui_mode:oneshot, accel_abs_threshold:2.0, key:'c', modifiers:0x01}
← {"ack":true, "id":100, "rule_count":1}

// HOLD_START_END (姿勢 +45° → -45° で Shift+A)
→ rule.add r:{id:101, ui_mode:hold_start_end,
              posture:{euler:[45,0,0], euler_tol:[15,15,90]},
              end_posture:{euler:[-45,0,0], euler_tol:[15,15,90]},
              key:'a', modifiers:0x02}
← {"ack":true, "id":101, "rule_count":2}

→ rule.list
← rules:[{id:100, states_count:1, loop:false}, {id:101, states_count:2, loop:true}]

→ profile.save name:'phase3_test'
← {"ack":true, "rule_count":2}
```

## Phase 3.2: 電源ボタン処理 (AXP192 PEK)

### 動作
- **短押し** (~256ms): sensor.stream を ON/OFF トグル
- **長押し** (~1.5s): BLE HID + NUS を起動 (未起動時)
- **超長押し** (~4s): AXP192 ハードシャットダウン (PMIC 機能)

イベント通知:
```jsonc
{"type":"power_button","press":"short","action":"stream_on"}
{"type":"power_button","press":"long","action":"ble_start"}
```

## Phase 3.3: 6 点キャリブレーション

### Web UI ウィザード
- 「⚡ 簡易キャリブ (1秒)」: gyro bias 補正のみ (起動時自動実行と同じ)
- 「🎯 フル 6 点キャリブ 開始」 → ステップウィザード:
  1. LCD を上向き (+Z)
  2. LCD を下向き (-Z)
  3. 右側面を上 (+X)
  4. 左側面を上 (-X)
  5. 上端を上 (+Y)
  6. 下端を上 (-Y)
- 各ステップで「📷 静止して キャプチャ」、6 面終了で「✅ 計算 + 適用」
- リアルタイム ax/ay/az 表示で正しい姿勢か確認可

### FW
- `calibrate.full.start / capture / finish / cancel` コマンド
- 各 face で 50 サンプル平均 → bias = (+面 + -面)/2、scale = G / ((+面 - -面)/2)
- `updateSensor()` で gyro bias + accel bias + accel scale を全て減算/補正
- 結果: `accel_bias_ms2[3]` + `accel_scale[3]` を JSON で返す

### 検証 (実機 protocol テスト)
```jsonc
→ calibrate.full.start
← {"step":0, "instruction":"LCD を上向き (+Z) に静置"}
→ calibrate.full.capture × 6
← face_avg_g + 次の指示
→ calibrate.full.finish
← {"ok":true, "accel_bias_ms2":[...], "accel_scale":[...]}
```

## Build メトリクス
- RAM: 11.8% (38664 bytes / 320KB)
- Flash: 56.5% (740KB / 1.31MB) — Phase 3 全機能で +2.5%

## GitHub Pages 公開機能 (gh-pages branch)

### 全 UI セクション (現在公開中: https://uecken.github.io/M5C_Serial_Unity/)
1. 🔧 デバイス: Ping/Info/BLE Start/Calibrate/Stream
2. 📊 センサー / 🎨 3D 姿勢 (Three.js)
3. 🧪 HID 直接テスト: 1秒遅延+カウントダウン、マウス即実行
4. 🎯 アクションルール: 一覧+発火フラッシュ、姿勢キャプチャ、修飾キー、HOLD_START_END
5. 🎯 加速度キャリブレーション: 簡易 + 6点ウィザード
6. ⚡ FW 書込み (esp-web-tools)
7. 📁 プロファイル (LittleFS): save/load/list/delete
8. 📜 通信ログ

## Phase 3 完了サマリ

| 機能カテゴリ | 状態 |
|------------|------|
| **アクションルール** | ✅ 3 モード + 修飾キー + 姿勢開始/終了 + 複合条件 |
| **キャリブレーション** | ✅ 簡易 (起動時自動 + 手動) + 6 点ウィザード |
| **電源ボタン** | ✅ AXP192 PEK 短/長押し検出、stream toggle / BLE start |
| **プロファイル永続化** | ✅ LittleFS、自動 active load |
| **3D 可視化** | ✅ Three.js M5StickC モデル + クォータニオン追従 |
| **LCD 表示** | ✅ BLE / Battery / Rules / Time / Euler |
| **バッテリ監視** | ✅ AXP192、98% / 4.13V / charging 表示 |
| **接続経路** | ✅ USB Serial 自動再接続 + BLE NUS Web Bluetooth |
| **FW 書込み (Web)** | ✅ esp-web-tools で 1 クリックフラッシュ |
| **公開** | ✅ GitHub Pages 24/7 アクセス可能 |

## 残作業 (Phase 4+)

- HOLD_START_END 用の終了アクションが今は「同じキーの release」固定。終了時に**別のキーを press**したい場合 (例: 開始で 'A'、終了で 'B' 押す) は要拡張
- ボタン入力 (M5StickC G37/G39 ボタン)
- LSM6DSV16X 等の他 IMU 対応
- M5Atom S3 / XIAO ESP32-S3 USB HID
- BLE HID Gamepad 拡張
- OTA FW 更新 (BLE NUS 経由)
- Adapter FW (Switch/PS4 中継)
- DeepSleep + 3g wake (要 IMU 変更)
- 6 点キャリブ結果を NVS 永続化 (現状 RAM のみ)
- アクセシビリティモード (シンプル UI、簡易ルール)
- プロファイル共有 (GitHub repo 経由のクラウドライブラリ)
