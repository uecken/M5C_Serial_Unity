# Burst Motion アーキテクチャ概要

## 全体構成

```
┌──────────────────────────────────────────────────────────────┐
│  Web 設定アプリ (Cloudflare Pages、motion.findradio.jp)       │
│  ├─ Connect / Live IMU / Trigger Editor / Library / Flash     │
│  └─ Web Serial (115200) ↔ Controller                          │
└──────────────────┬───────────────────────────────────────────┘
                   │ USB Serial (JSON Lines)
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  Burst Motion Controller                                      │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  src/transport/ SerialJsonLine (ArduinoJson v7)          ││
│  │  src/core/ TriggerEngine / Profile / MahonyFilter        ││
│  │  src/hid/ BleHidSink (NimBLE) + UsbHidSink (S3 TinyUSB) ││
│  │  src/hal/esp32/ imu_*.cpp / buttons / display            ││
│  └─────────────────────────────────────────────────────────┘│
│                                                              │
│  ┌─ Storage ──────────────────────┐                          │
│  │ NVS (Preferences)              │  軽量設定値              │
│  │ LittleFS /profiles/*.json      │  プロファイル            │
│  └────────────────────────────────┘                          │
│                                                              │
│  Output:                                                     │
│  ├─ BLE HID (NimBLE + BleCombo / 将来 BLE-CompositeHID)     │
│  ├─ USB HID (ESP32-S3、TinyUSB composite)                   │
│  └─ ESP-NOW → Adapter (Phase 8)                              │
└──────────────────────────────────────────────────────────────┘
                   │ BLE HID / USB HID
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  Target: PC / Phone / Switch / PS4 / Xbox                    │
└──────────────────────────────────────────────────────────────┘
```

## レイヤ構成（FW）

```
┌─── 上位層 ─────────────────────────────────────┐
│  transport/SerialJsonLine   hid/IHidSink       │
│  (JSON プロトコル)          (HID 抽象)          │
├────────────────────────────────────────────────┤
│  core/ (ボード非依存、ホストテスト可)            │
│  ├─ TriggerEngine (状態機械、if-else のみ)      │
│  ├─ Profile (JSON SerDe)                       │
│  ├─ MahonyFilter (姿勢推定)                    │
│  ├─ QuatOffset (q_ref)                         │
│  └─ types.hpp (ActionRule/Condition/State)     │
├────────────────────────────────────────────────┤
│  hal/esp32/ (ボード依存)                        │
│  ├─ imu_mpu6886 / imu_bmi270 / imu_lsm6dsv16x │
│  ├─ buttons_gpio                               │
│  └─ display_m5stickc                           │
├────────────────────────────────────────────────┤
│  hid/ (実装は MCU 依存)                         │
│  ├─ BleHidSink (NimBLE, ESP32)                 │
│  └─ UsbHidSink (TinyUSB, ESP32-S3)             │
└────────────────────────────────────────────────┘
```

## データフロー

```
[IMU 生データ]
    ↓ bias 補正 (個体差、NVS)
    ↓ 軸リマップ (機種別、定数)
[Body frame センサ値]
    ↓ Mahony フィルタ (MPU6886/BMI270) or SFLP/DMP (LSM6DSV16X/ICM-20948)
[Body frame Quaternion (q_body)]
    ↓ q_ref* 適用 (ユーザー基準姿勢)
[User frame Quaternion (q_user) + Euler]
    ↓
[TriggerEngine: 状態機械評価]
    ↓ 一致したルール
[HID Dispatcher]
    ↓
[BLE HID / USB HID / ESP-NOW]
```

## Phase 構成

| Phase | 内容 | スコープ |
|-------|------|---------|
| 1 | 新 FW コア層、platformio env 整備 | `src/core/*`、`platformio.ini` |
| 2 | HAL + 姿勢推定 + BLE HID | `src/hal/esp32/*`、Mahony、BleHidSink |
| 3 | Web アプリ新規構築 | `Web/hidconfig/*` |
| 4 | ESP32-S3 USB HID | TinyUSB composite、Gamepad |
| 5 | 高度機能 + HW 再設計 | MCP23017、Touch、DeepSleep、新 PCB |
| 6 | クラウド共有プロファイル | GitHub repo + jsDelivr |
| 7 | バックエンド (任意) | CF Workers + R2 + D1 |
| 8 | Adapter FW | RP2040 W / M5Atom S3 Adapter |
| 9 | 量産準備 | 認証、QC、パッケージ |

## 複数 Controller/IMU 対応

抽象化戦略:
- **`src/core/` は MCU/IMU 非依存**、ホスト上で Catch2 テスト可能
- **`src/hal/esp32/imu_*.cpp`** を IMU チップ別に分離
- コンパイル時 `-D IMU_TYPE=LSM6DSV16X` で切替
- **プロファイル JSON は IMU 種別に依らず互換**

## 設計方針

1. **シンプルルールベース**: ML/DTW 不使用、全トリガー判定は if-else
2. **統一状態機械モデル**: ONESHOT / HOLD_* / SEQUENCE を 1 フレームで表現
3. **Euler + Quaternion 両方保存**、judge_by で判定切替
4. **NVS + LittleFS ハイブリッド**: 軽量設定は NVS、可変長は LittleFS
5. **MPU6886 は検証継続、量産は LSM6DSV16X/BMI270**
6. **USB Serial 設定 + BLE HID 出力** を MVP 基本、並行稼働可

## 関連ドキュメント

- [JSON Lines プロトコル](../protocol/json-lines.md)
- [プロファイルスキーマ](../protocol/profile-schema.md)
- [Phase 1 実装](../firmware/phase1-implementation.md)
