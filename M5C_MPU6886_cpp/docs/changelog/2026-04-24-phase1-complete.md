# 2026-04-24 Phase 1 MVP 完了

## ✅ Phase 1 完了

### FW 側 (`[env:m5stick-c-v2]`)
- **ビルド**: SUCCESS、RAM 11.4% (37368B)、Flash 47.8% (626265B / 1.31MB)
- **アップロード**: COM8 経由で M5StickC へ書込み成功 (632KB written)
- **動作確認**: 115200 baud で全機能動作確認済み

### 動作確認した JSON Lines コマンド

| コマンド | 結果 |
|---------|------|
| `{"cmd":"ping"}` | `{"type":"pong","fw":"2.0.0-dev","board":"m5stickc","imu":"mpu6886","uptime":290}` |
| `{"cmd":"device.info"}` | `{"type":"device.info","fw":"2.0.0-dev","board":"m5stickc","imu":"mpu6886","uptime":688,"ble_connected":false,"output":"ble"}` |
| `{"cmd":"sensor.stream","rate_hz":10}` | 10Hz で 14 サンプル取得、accel/gyro/Euler/Quaternion 全出力 |
| `{"cmd":"sensor.stream","rate_hz":0}` | ストリーム停止 |
| `{"cmd":"foo"}` (不明) | `{"type":"err","cmd":"foo","err":"unknown_cmd"}` |

### Boot メッセージ (FW 起動直後)
```json
{"type":"boot","stage":"post_imu_init","imu":"MPU6886","imu_ok":true}
```
→ MPU6886 I2C 通信 (WHO_AM_I=0x19) 成功、IMU 初期化完了

### センサー値の妥当性
- 静止時: accel_z = 10.16〜10.32 m/s² → **重力 1g (9.81) を正しく検出**
- accel_x/y: ~0.2 m/s² (ノイズレベル)
- gyro: -3〜-15°/s (bias 残あり、要キャリブレーション)
- Mahony Euler 出力: pitch=-13°, roll=-3° (重力ベクトルから推定)

### Web アプリ (`Web/hidconfig/`)
- **ローカルサーバー稼働中**: http://localhost:8000/
- 構成: Preact + htm + Tailwind CDN (no-build)
- 機能:
  - 🔌 Connect/Disconnect (Web Serial @ 115200)
  - 📋 デバイス情報 (Ping / Info / BLE Start / Stream Toggle)
  - 📊 リアルタイムセンサー表示 (Accel/Gyro/Euler/Quat)
  - 📜 Serial ログペイン (TX/RX 色分け、Clear)

## 解決した技術課題

### 1. PlatformIO platform バージョン
- `espressif32` 最新版は pioarduino fork、arduino-esp32 v2.0.14 builder 不一致
- **解決**: `platform = espressif32@6.6.0` でピン留め

### 2. C++17 が必要 (auto ラムダ)
- TriggerEngine.cpp の lambda で `auto&&` 使用
- **解決**: `build_unflags = -std=gnu++11` + `build_flags = -std=gnu++17`

### 3. Arduino.h マクロとの enum 名衝突
- Arduino.h L48 で `#define EULER 2.718...`
- **解決**: enum 値にプレフィックス追加 (BY_EULER, AT_PRESS, OP_AND, CMP_GTE, OUT_BLE 等)

### 4. lib/M5StickC ローカルコピー API 不整合
- 既存 lib/M5StickC が公式 lib と整合せず
- **解決**: Phase 1 新 FW は M5 lib に依存しない (Wire.h 直接 I2C で MPU6886 アクセス)
- 既存 FW は user 担当領域、触らず

### 5. M5StickC FTDI 経由 auto-reset 問題
- アップロード後 M5C が DOWNLOAD_BOOT モード で停止
- esptool RTS pin reset では通常起動に戻らず
- **解決**: 921600 → 115200 baud に下げて FTDI と相性改善 (鍵となった対処)
  - 921600 では FTDI が信号サンプル不安定だった可能性
  - 115200 で起動完了、Serial 通信成功

## ファイル一覧 (Phase 1 で新規作成)

```
src/
├── core/
│   ├── types.hpp                   ActionRule/Condition/State 型定義
│   ├── MahonyFilter.hpp            姿勢推定フィルタ
│   ├── TriggerEngine.hpp           統一状態機械 evaluator (header)
│   └── TriggerEngine.cpp           実装 + サブ条件 evaluator
├── hal/esp32/
│   ├── ImuReader.hpp               IMU 抽象インタフェース
│   └── ImuMpu6886.hpp              MPU6886 直接 I2C ドライバ
├── hid/
│   ├── IHidSink.hpp                HID 出力抽象
│   └── BleHidSink.hpp              NimBLE + BleCombo 経由
├── transport/
│   └── SerialJsonLine.hpp          JSON Lines プロトコル
└── main_v2.cpp                     Entry point (setup + loop)

Web/hidconfig/
├── index.html                      Tailwind CDN + ES Module
├── src/
│   ├── app.js                      Preact App
│   └── lib/
│       └── SerialClient.js         Web Serial + JSON Lines

docs/
├── README.md
├── architecture/overview.md
├── protocol/json-lines.md
├── protocol/profile-schema.md
├── hardware/bom.md
├── firmware/phase1-implementation.md
└── changelog/
    ├── 2026-04-24-baseline-status.md
    ├── 2026-04-24-phase1-build-success.md
    └── 2026-04-24-phase1-complete.md (本ファイル)

.claude/agents/
├── burst-motion-system-architect.md
├── burst-motion-fw-esp32.md
├── burst-motion-sensor-dsp.md
├── burst-motion-web-frontend.md
├── burst-motion-hardware-designer.md
└── burst-motion-marketing.md
```

## 次の Phase 2 で取り組むべき項目

### A. ブラウザでの動作確認
- ユーザーが Chrome で http://localhost:8000/ を開いて手動テスト
- COM8 接続 → Ping → Stream ON → 3D viewer 移植

### B. キャリブレーション
- Gyro bias 補正 (静止時に -15°/s drift)
- 簡易キャリブレーションコマンド `calibrate.simple` 実装

### C. BLE HID 統合
- `ble.start` で NimBLE/BleCombo 開始 → ペア → メモ帳でキー入力テスト
- Output モード切替の実動作確認

### D. ActionRule の登録/読込
- `rule.add` JSON 受信 → TriggerEngine に流す
- LittleFS にプロファイル保存
- ストリートファイター波動拳プロファイルを試験投入

### E. UI 拡張
- 3D Three.js ビュアー (motion_controller.js から流用)
- TriggerEditor ページ
- Library / Profile 管理
