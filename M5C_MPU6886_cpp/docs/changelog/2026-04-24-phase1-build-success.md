# 2026-04-24 Phase 1 新 FW ビルド成功

## 成果

**`[env:m5stick-c-v2]` が build + upload 成功**。
- RAM 11.4% (37368/327680 bytes)
- Flash 47.8% (626265/1310720 bytes)

## 構築内容

### 新規ファイル (Phase 1 MVP)
```
src/
├── core/
│   ├── types.hpp              ActionRule/Condition/State データ型 (ボード非依存)
│   ├── MahonyFilter.hpp       姿勢推定 (Quaternion + Euler)
│   ├── TriggerEngine.hpp/.cpp 統一状態機械 evaluator
├── hal/esp32/
│   ├── ImuReader.hpp          IMU 抽象インタフェース
│   └── ImuMpu6886.hpp         M5StickC MPU6886 直接 I2C (M5 lib 非依存)
├── hid/
│   ├── IHidSink.hpp           HID 出力抽象
│   └── BleHidSink.hpp         NimBLE + BleCombo ラッパ
├── transport/
│   └── SerialJsonLine.hpp     JSON Lines over Serial
└── main_v2.cpp                Entry point
```

### 対応する JSON コマンド (Phase 1 最小セット)
- `ping` → `pong + fw + board + imu + uptime`
- `device.info` → FW 情報
- `sensor.stream` (rate_hz) → センサーデータストリーム
- `output.set` (ble/usb/both/none) → HID 出力モード切替
- `ble.start` → BLE HID 遅延開始
- `watch.set` → トリガー発火通知の ON/OFF

## 解決した技術課題

### 1. platformio.ini platform ピン留め
- `espressif32` 最新版は pioarduino fork、arduino-esp32 v2.0.14 builder を含まない
- **解決**: `platform = espressif32@6.6.0` で v2.0.14 ネイティブ builder を明示
- 既存 `9aabaef cannot compile M5C` の本質的原因

### 2. C++17 で auto ラムダ引数
- TriggerEngine.cpp の `auto&& predicate` は C++14+ が必要
- **解決**: `build_unflags = -std=gnu++11` + `build_flags = -std=gnu++17`

### 3. Arduino.h マクロとの enum 名衝突
- `Arduino.h` L48 で `#define EULER 2.718...`
- `PostureJudge::EULER` 等が展開されて parse error
- **解決**: enum 値にプレフィックス付与:
  - `PostureJudge::BY_EULER`, `BY_QUAT`
  - `ActionType::AT_NONE`, `AT_PRESS`, `AT_RELEASE`, `AT_FIRE_ONCE`, ...
  - `LogicOp::OP_AND`, `OP_OR`
  - `Comparison::CMP_GTE`, `CMP_LTE`
  - `HidOutputMode::OUT_NONE`, `OUT_BLE`, `OUT_USB`, `OUT_BOTH`

### 4. lib/M5StickC ローカルコピーの API 不整合
- 公式 M5StickC@0.2.3 と lib/M5StickC 内の API が食い違う
- **Phase 1 ポリシー**: 新 FW は M5 lib に一切依存しない
- Wire.h 直接 I2C で MPU6886 アクセス (ImuMpu6886.hpp)
- 既存 FW は user が調整中、触らない

## 未解決の課題

### M5C の FTDI auto-reset 問題 (MVP テスト阻害)

```
python -m esptool --port COM8 chip_id:
  Features: Wi-Fi, BT, Dual Core + LP Core, 240MHz, ...
  MAC: e8:9f:6d:0d:31:04
```

FW は正常にアップロードされる。しかし reset 後:
```
ets Jun  8 2016 00:22:57
rst:0x1 (POWERON_RESET),boot:0x3 (DOWNLOAD_BOOT(UART0/UART1/SDIO_REI_REO))
waiting for download
```

→ **ESP32 が DOWNLOAD_BOOT モードで停止**、FW が実行されない。

**原因推定**:
- FTDI USB-Serial (VID:0403 PID:6001) の DTR/RTS が M5StickC の EN/GPIO0 に適切に配線されていない、または配線されているが pyserial / esptool の RTS/DTR 操作と相性が悪い
- GPIO0 が LOW 固定されて boot mode が DOWNLOAD になる

**対策**:
1. **物理的にリセット**: M5StickC の電源ボタンを 6 秒長押し → 再起動 → FW 実行
2. または **M5StickC 内蔵の Power IC (AXP192) 経由のリセット**: これは FW 側でコントロールできる
3. または FTDI adapter 側の DTR/RTS 配線を確認

**次のアクション**: ユーザーに電源ボタン長押し依頼 OR 並行で Web アプリスケルトン作成
