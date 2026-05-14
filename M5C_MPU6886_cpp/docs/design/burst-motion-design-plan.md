# Burst Motion 設計 — モーション→HID Webアプリ + FW + HW 包括計画

## Context（なぜこの変更が必要か）

[M5C_MPU6886_cpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/) は既に MPU6886 IMU + BLE HID (NimBLE + BleCombo) + PK3 トリガー登録 + Web Serial 設定UI (`motion_controller_updater.html`) を持つ成熟した基盤。用途は Protopedia #1988「Motion Controller」— PC/スマホのフィジカル入力デバイス、ゲーム/家庭/オフィス用、および手の障害を持つ人の支援入力デバイス。

不足しているのは:
1. **リポジトリ内セルフホストの設定 Web アプリ** — 現状 findradio.jp 依存
2. **統合トリガー→HID マッピング UI** — 現 pk3 は CSV コマンド手打ち前提
3. **マウス/ゲームパッド HID** — キーボードのみ（ゲームパッドは ESP32-S3 のみ）
4. **USB HID 出力（ESP32-S3）** — ハード対応だが未実装
5. **出力経路の実行時切替** (BLE HID / USB HID / 両方 / 無効)

**MVP ポリシー**: 「USB Serial（設定書き込み）+ BLE HID（出力）」で当面動作。S3 の USB HID は第2段階で追加。S3 では TinyUSB CDC+HID コンポジットで両立するので「Serial⇔HID切替」は実行時フラグで出力先を選ぶだけになり、電源再起動は不要。

**対象ハードウェア（Controller + Adapter の 2 段構成）**:

### Controllers (モーション検出側)
| HW | MCU | BLE HID | USB HID | RF 送信 |
|----|-----|---------|---------|---------|
| **Motion Burst** (自作) | ESP32-S3 (暫定) | ✅ | ✅ (USB OTG) | ESP-NOW |
| **M5StickC** | ESP32 PICO D4 | ✅ | ❌ | ESP-NOW |
| **M5StickC Plus** | ESP32 PICO D4 | ✅ | ❌ | ESP-NOW |
| **M5StickC 2** | ESP32 PICO-V3-02 | ✅ | ❌ | ESP-NOW |
| **M5Atom S3** | ESP32-S3 | ✅ | ✅ (USB OTG) | ESP-NOW |

### Adapters (将来拡張、Phase 8 にまとめ)
用途: PS4 / Nintendo Switch / Xbox など **BLE HID 入力を受け付けない機器** に USB HID として入力する。Controller→Adapter 間を RF (ESP-NOW / nRF ESB) で中継し、Adapter が USB HID デバイスとして振る舞う。

**現時点のスコープ**: 実装は Phase 8 にまとめて先送り。MVP (Phase 1-5) では**設計の布石のみ**(プロファイルの `hardware.adapter` フィールド、RF 抽象インタフェース等)を残す。

| HW | MCU | USB HID | BLE HID | RF 受信 |
|----|-----|---------|---------|---------|
| **Motion Burst Adapter** | ESP32-S3 (暫定、要確認) | ✅ | ✅ | ESP-NOW |
| **RP2040 W** | RP2040 + CYW43439 | ✅ (TinyUSB で Keyboard+Mouse+Gamepad 合成) | ✅ (CYW43 + BTStack) | ESP-NOW 非対応 → nRF24L01 外付け / BT Classic 受信 / WiFi UDP のいずれか |
| **M5Atom S3 Adapter** | ESP32-S3 | ✅ (USB OTG) | ✅ | ESP-NOW |

**RP2040 W の HID 確認**: RP2040 は native USB 1.1 ホスト/デバイス対応、TinyUSB 経由で Keyboard+Mouse+Gamepad 複合 HID として動作。Arduino-Pico (earlephilhower) 環境で標準対応。CYW43439 チップで BLE HID も可能。Adapter ハードウェアとしては十分機能する。

### RF プロトコルの使い分け
| RF | 遅延 | 対応 MCU | 利点 | 欠点 |
|----|------|----------|------|------|
| **ESP-NOW** | 2-4ms | ESP32/S3/C3/C6 | ペアリング不要、最大250B/pkt | ESP32 系列限定 (RP2040 W 不可) |
| **nRF ESB** | 1-2ms | nRF52 native / nRF24L01 外付け (SPI) | 超低遅延、ESP32/RP2040 両対応 | 外付けモジュール、ピン消費 |
| **BT Classic HID ホスト** | 7-15ms | RP2040 W (CYW43) 等 | 既存 BLE HID Controller をそのまま受信可 | スイッチ機の認識検証必要 |
| **WiFi UDP (LAN)** | 5-20ms | ESP32/RP2040 | LAN 経由、複数台対応 | 遅延大、AP 依存 |

→ **推奨**: **ESP32 同士は ESP-NOW**（シンプル）。**RP2040 W** は用途次第で選択（Phase 8 検討）。

### 今回スコープ
- **Phase 1-5 (MVP)**: Controller 単独動作（直接 BLE HID / USB HID、Adapter 無し）
- **Phase 8 (将来拡張、まとめ実施)**: Adapter FW + Controller→Adapter RF 中継。Controller 側 FW に `RfHidSink` を布石として追加する
- **XIAO ESP32-C3**: 今回スコープ外（後回し、illumiTrack 4-button 用途は維持可）

---

## 全面リデザイン方針（FW + Web、5年前の設計を刷新）

現行 FW・Web は 5 年前の設計で以下の負債があり、ユーザー許諾のもとに刷新する：

### FW 側の刷新ポイント

**捨てるもの**:
- FreeRTOS 5 タスク構成（ImuLoop / ReadSession / WriteSession / ButtonSession / hidSession）と 3 つのミューテックス → 複雑すぎる
- pk / pk2 / pk3 の進化系譜（3 種の類似構造体）→ 混乱のもと
- `MotionController` 1311 行の god class → 層分離
- ボード分岐の深い `#ifdef` ネスト → HAL 層で吸収
- `execSF_HIDInputs` の特定用途専用ロジック → TriggerEngine に汎用化
- CSV + バイナリハイブリッドの serial プロトコル → JSON Lines に統一
- Arduino `String` の過剰使用 → `const char*` / std::string_view 主体

**採用する構成**:

```
M5C_MPU6886_cpp/
├── src/
│   ├── main.cpp              # setup() + super-loop のみ
│   ├── core/                 # ボード非依存、PC上でも単体テスト可能
│   │   ├── TriggerEngine.hpp/cpp
│   │   ├── HidDispatcher.hpp/cpp
│   │   ├── Profile.hpp/cpp       (JSON load/save)
│   │   ├── SensorState.hpp       (現在のIMU/ボタン状態の snapshot)
│   │   ├── MahonyFilter.hpp/cpp  (姿勢推定、全 Controller 共通)
│   │   ├── AxisRemap.hpp         (IMU chip frame → body frame 変換)
│   │   ├── QuatOffset.hpp/cpp    (ユーザー初期姿勢 offset、NVS 永続化)
│   │   └── types.hpp             (Trigger, Condition, Output 構造体)
│   ├── hal/                  # ボード依存、board フラグで片方だけビルド
│   │   ├── imu_m5stickc.cpp      (M5.Imu 生データ + 機種別 AxisRemap)
│   │   ├── imu_m5atom_s3.cpp     (M5Atom S3 IMU + 機種別 AxisRemap)
│   │   ├── imu_motion_burst.cpp  (Motion Burst 独自、要実測)
│   │   ├── imu_xiao_s3.cpp       (XIAO ESP32-S3 + MPU6050 DMP)
│   │   ├── axis_remaps.hpp       (機種別リマップ行列の定義集)
│   │   ├── buttons_gpio.cpp
│   │   ├── buttons_i2c_expander.cpp  (MCP23017、Phase 5)
│   │   └── display_m5stickc.cpp  (LCD、S3 にはなし)
│   ├── hid/                  # HID 出力の実装差し替え
│   │   ├── IHidSink.hpp          (抽象インタフェース)
│   │   ├── BleHidSink.cpp        (BleCombo ベース、BLE HID Gamepad 拡張)
│   │   └── UsbHidSink.cpp        (S3 のみ、TinyUSB)
│   └── transport/            # 外部通信プロトコル
│       ├── SerialJsonLine.cpp    (JSON Lines over USB Serial)
│       └── Protocol.hpp          (コマンド定義)
├── lib/                      # 外部ライブラリ（NimBLE、ArduinoJson、BleCombo）
├── test/                     # Catch2 でコア層の単体テスト
├── platformio.ini
└── Web/hidconfig/            # Web アプリ
```

**単一メインループ + 最小タスク**:
- `IMUTask` (100Hz) — IMU 読取りだけ、SensorState を atomic 更新
- `loop()` — TriggerEngine 評価 → HidDispatcher 発火 → Serial コマンド処理 → 状態ストリーム送信、すべて非ブロッキング
- FreeRTOS タスクは 2 つで完結、ミューテックスは `std::atomic` または `critical section` に置換

**制御フォーマット**: **JSON Lines over Serial** 統一

現在の CSV + バイナリ混在プロトコルを廃止し、改行区切り JSON に一本化。
```jsonc
// Web → FW
{"cmd":"profile.load", "name":"fps"}
{"cmd":"trigger.add", "t":{"id":1,"name":"right-tilt","mode":"hold_start_end", ...}}
{"cmd":"trigger.list"}
{"cmd":"output.set", "target":"ble"}          // ble | usb | both | none
{"cmd":"hw.buttons.set", "layout":[...]}
{"cmd":"q.init.upright"}                       // 従来 QINITU 相当
{"cmd":"ping"}

// FW → Web
{"type":"sensor","t":12345,"ax":0.1,"ay":0.2,"az":0.98,"gx":0,"gy":0,"gz":0,"pitch":1,"roll":2,"yaw":3,"qw":1,"qx":0,"qy":0,"qz":0}
{"type":"trigger.hit","t":12346,"id":1,"phase":"start"}
{"type":"trigger.release","t":12400,"id":1,"phase":"end"}
{"type":"status","uptime":12345,"ble":true,"usb":false,"output":"ble","profile":"fps","fw":"1.0.0"}
{"type":"ack","cmd":"trigger.add","ok":true}
{"type":"err","cmd":"profile.load","err":"not_found"}
```

**大量データ転送の扱い**:
- 従来 `savepk3vectol2file` のバイナリブロック転送は廃止
- プロファイル全体を JSON 1 行または JSON Object として送信 (`{"cmd":"profile.write","name":"fps","data":{...}}`)
- サイズが大きい場合は複数行に分割 (`{"cmd":"profile.chunk","seq":0,"data":"..."}`) だが、実用上 1 プロファイル ≈ 10KB 以下なら一括で問題なし
- 200ms 待ち等の**タイミング依存プロトコルを完全排除**（行単位のバッファリングで確実）

**ArduinoJson ライブラリ追加**:
- `lib_deps += bblanchon/ArduinoJson@^7.0.0`
- v7 の `JsonDocument` はスタック割当てもヒープも両対応、RAM 効率良好

**トリガーデータモデル (新 `struct Trigger`)**:
```cpp
enum class TriggerMode : uint8_t { ONESHOT, HOLD_START_END, HOLD_START_ONLY, SEQUENCE };
enum class OutputType : uint8_t { KEYBOARD, MOUSE, GAMEPAD, CONSUMER, MACRO };

struct Condition {
    bool button_enabled;
    uint8_t button_idx;      // 1-16
    bool button_pressed;     // true=押下で成立
    bool posture_enabled;
    bool posture_use_quat;   // false=Euler, true=Quaternion
    float euler[3];          // roll, pitch, yaw (center)
    float euler_tol[3];      // tolerance
    float quat[4];           // w,x,y,z (if quat mode)
    float quat_dot_min;      // inner product threshold
    bool accel_enabled;
    float accel_th[4];       // x,y,z,abs
    bool gyro_enabled;
    float gyro_th[4];
};

struct Trigger {
    uint16_t id;
    char name[32];
    TriggerMode mode;
    Condition start;
    Condition end;           // unused if mode != HOLD_START_END
    OutputType out_type;
    uint8_t out_payload[32]; // 種別別にパース (keyboard: mod+keys, mouse: dx/dy/buttons, ...)
    uint16_t interval_ms;
    uint16_t cooldown_ms;
    int8_t priority;         // 大きいほど優先
    // runtime state (RAM only)
    mutable bool armed_cooldown;
    mutable bool holding;
    mutable uint32_t last_fire_ms;
};
```

合計 ~180B/trigger（RAM）。LittleFS 上は JSON で約 400-600B/trigger。

### Web App 側の刷新ポイント

**捨てるもの**:
- jQuery 3.5 + Bootstrap 4 + 全グローバル scope (motion_controller.js 710行)
- 壊れた HTML (motion_controller_updater.html の文法エラー)
- esp-web-tools v8 と生 JS の混在

**採用する構成**: **Preact + htm + Tailwind CSS（ビルド不要、CDN のみ）**
- Preact (3KB) + htm (1KB) = React 風の宣言的 UI が**ビルド無しで動く**
- Tailwind CSS は Play CDN 版（本番は Twind や CLI ビルドに切替可能）
- esp-web-tools 10.x (最新) を ES Module として直 import

**ディレクトリ構成**:
```
Web/hidconfig/
├── index.html                    # シェル、Preact マウントポイント
├── src/
│   ├── app.jsx                   # ルーティング（ハッシュベース）
│   ├── lib/
│   │   ├── SerialClient.js       # Web Serial 接続 + JSON Lines parser
│   │   ├── TriggerModel.js       # Trigger データモデル + JSON SerDe
│   │   ├── IMUViewer.js          # Three.js 3D (既存移植、モジュール化)
│   │   └── i18n.js               # 日/英 切替
│   └── pages/
│       ├── Connect.jsx           # 接続 + 現状ステータス
│       ├── LiveView.jsx          # IMU リアルタイム可視化
│       ├── TriggerEditor.jsx     # トリガー追加/編集 (メイン機能)
│       ├── Library.jsx           # プロファイル管理
│       ├── Output.jsx            # BLE/USB 切替 + HW ボタン設定
│       └── Flash.jsx             # esp-web-tools ラッパ
├── manifests/
│   ├── m5stickc.json
│   └── xiao_s3.json
├── profiles/                     # サンプルプロファイル (FPS, Fighting, Presentation, Accessibility)
│   └── *.json
└── sw.js                         # Service Worker (PWA、オフライン対応)
```

**UX 強化**:
- **モバイル対応**: Chrome Android は Web Serial 非対応だが Web Bluetooth は対応。将来 BLE 設定プロトコルを追加する拡張口を残す
- **アクセシビリティ**: ARIA ラベル、キーボード操作、コントラスト、日本語/英語切替
- **PWA 化**: Service Worker でオフライン動作、ホーム画面追加
- **プロファイル URL シェア**: 設定 JSON を圧縮→base64→URL ハッシュに入れて共有可能
- **インタラクティブトリガー登録**: 「録画→再生→確認→登録」フロー

**テスト**:
- Vitest でトリガー評価・JSON SerDe の単体テスト
- Playwright で Chrome のモック Serial を使った E2E テスト

---

## 全体アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────┐
│  Web 設定アプリ (Chrome 上 navigator.serial + esp-web-tools)     │
│  M5C_MPU6886_cpp/Web/hidconfig/index.html (新規)                 │
│   ├ Connect   : USB シリアル接続                                 │
│   ├ Live IMU  : 3D 姿勢＋加速度リアルタイム (既存 Three.js 流用) │
│   ├ Triggers  : 姿勢/加速/ボタン→キー/マウス/ゲームパッド登録UI  │
│   ├ Library   : プロファイル保存/読込 (JSON、LittleFSへ転送)     │
│   ├ Output    : BLE/USB/両方/無効 切替（S3のみ USB 選択可）      │
│   └ Flash     : esp-web-tools (ボード別 manifest_*.json)         │
└─────────────────┬───────────────────────────────────────────────┘
                  │ USB Serial 115200 (CSV + バイナリPK4)
                  ▼
┌─────────────────────────────────────────────────────────────────┐
│  FW: IMU_BLEorSerial_tester.cpp + MotionController.hpp          │
│   ├ ImuLoop (50Hz) → 加速度/ジャイロ/クォータニオン              │
│   ├ TriggerEngine (新規、置換) ← pk4_vector を走査              │
│   ├ HidOut   (新規 layer) ─┬─ BleHidSink  (BleCombo 既存流用)    │
│   │                        └─ UsbHidSink  (TinyUSB, S3 のみ新規) │
│   ├ SerialCommand (既存、拡張: SET_OUTPUT, dumppk4, addpk4...)   │
│   └ EEPROM/LittleFS (既存、savepk4vectol2file で pk4 保存)       │
└─────────────────────────────────────────────────────────────────┘
```

---

## DeepSleep + Wake 戦略（電池駆動、1年超の待機時間）

### Wake ソース比較

| Wake source | ESP32 PICO D4 | ESP32-S3 | nRF52840 | 消費電力 |
|-------------|---------------|----------|----------|----------|
| GPIO 外部割込み (物理ボタン) | ✅ EXT0/EXT1 | ✅ | ✅ SENSE | — |
| **Native Touch** (静電容量、MCU 内蔵) | ✅ **10 ピン、DeepSleep wake 可** | ✅ **14 ピン** | ❌ | ~1-3µA |
| **Wake-on-motion (IMU INT)** | ✅ GPIO 経由 | ✅ | ✅ | ~3-13µA |
| Timer wake (周期) | ✅ ULP | ✅ | ✅ RTC | ~1µA |
| **DeepSleep 消費 (MCU のみ)** | ~10µA | ~5µA | **~1µA** 🏆 | — |

### IMU 各チップの Wake-on-Motion 対応

| IMU | Any-Motion INT | 監視消費 | 補足 |
|-----|----------------|---------|------|
| MPU6886 | ✅ | ~13µA | 閾値 4mg〜 |
| **BMI270** | ✅ | **~3.5µA** 🏆 | Any/No-motion + Orientation + Tilt |
| **LSM6DSV16X** | ✅ | ~3-5µA | MLC + QVAR INT も wake 源に |
| LSM6DS3TR-C | ✅ | ~8µA | 基本 Wake-up |

### 電池駆動試算（150mAh 前提）

| 状態 | 消費 | 寿命 |
|------|------|------|
| Active (BLE HID) | ~50mA | ~3 時間 |
| Light sleep (BLE 維持) | ~5mA | ~30 時間 |
| **Deep sleep + wake 監視** | **~10µA** | **~1.5 年** |

### 🚨 重要: MPU6886 の 3g Wake 不可問題

ユーザー要件「**3g 以上の激しい振りで wake したい**」に対する調査結果:

| IMU | WoM 閾値上限 | **3g wake** | 理由 |
|-----|-------------|-------------|------|
| **MPU6886** (現 M5StickC) | **~1.02g** | ❌ **不可** | 固定 4mg/LSB × 8bit、ハード制約 |
| MPU6050 / MPU-9250 | ~1.02g | ❌ 不可 | 同ファミリ |
| ICM-20948 (native WoM) | ~1.02g | ❌ native 不可 | DMP3 で custom 閾値なら可 |
| **BMI270** | ±16g FS で 16g | ✅ **可** | 12-bit × (FS/4096)、~3.5µA |
| **LSM6DSV16X** | ±16g FS で 16g | ✅ **可** | 6-bit × (FS/64)、MLC/FSM で複雑条件も |
| LSM6DS3TR-C (手元 XIAO) | ±16g FS で 16g | ✅ 可 | 同ファミリ、~8µA |
| ICM-42688-P | ±16g FS で ~15.9g | ✅ 可 | ~3µA |
| BNO085/086 | Shake + custom | ✅ 可 | SH-2 内蔵 |

**意味すること**:
- **MPU6886 では 3g wake を実現できない** — WoM レジスタが 1g 上限でハード制約
- 「FW 側で常時加速度を読んで判定」は DeepSleep と矛盾（MCU が動き続ける必要）
- **量産版で LSM6DSV16X / BMI270 への移行が必須要件** に昇格（元々推奨していたが、これで確定）

**3g wake 設定例**:

```cpp
// BMI270
accel_fs_range = BMI2_ACC_RANGE_16G;
any_motion.threshold = (3.0f * 4096 / 16.0f);  // 768 → 3g
any_motion.duration = 10;                       // 連続10サンプル以上

// LSM6DSV16X
ctrl8_xl = LSM6DSV16X_XL_FS_16g;
wake_up_ths = (3.0f * 64 / 16.0f);  // 12 → 3g
wake_up_dur = 2;                     // 2サンプル以上
```

**UX 設計**:
```
机に置いた静止状態     → DeepSleep ~10µA
手に持つ (1g 程度の動き) → Wake しない (誤起動防止)
「さあ開始」と強く振る   → Wake (3g 超) → BLE HID 接続開始
```
意図のある動作のみ wake、微振動では誤起動しない。

### 推奨 Wake 戦略（3 経路並列）

```
DeepSleep 中の監視:
  1. 物理 PWR ボタン (GPIO 割込み) — 強制 wake、誤起動防止
  2. Native Touch パッド 1 個 — 「起動タッチ」
  3. IMU Any-motion INT — 動かすだけで wake
  → いずれかの割込みで MCU wake

Wake 後:
  - バッテリ電圧確認
  - BLE HID 再接続 or 維持
  - 無操作 60 秒 (Web UI で可変) で自動 DeepSleep 復帰
```

### QVAR vs Native Touch の使い分け

**MCU だけで QVAR は利用できるか？** → **QVAR は LSM6DSV16X チップ内蔵機能**。独立チップではない。ただし LSM6DSV16X を **QVAR 単独モード (~1-2µA)** で動作させれば、IMU 機能を切っても QVAR だけ使える。

**ESP32 / ESP32-S3 は native touch を内蔵**、これで QVAR は不要:

| 方式 | 電極数 | DeepSleep wake | 追加チップ | 消費 |
|------|--------|----------------|------------|------|
| **ESP32 native touch** | 10 | ✅ | 不要 | ~1-3µA |
| **ESP32-S3 native touch** | 14 | ✅ | 不要 | ~1-3µA |
| LSM6DSV16X QVAR | 2 | ✅ | IMU 自体 | ~1-2µA |
| 外付け MPR121 | 12 | ✅ | +$1.5 | ~29µA |

→ **ESP32/S3 採用時は native touch で十分、QVAR は不要**。nRF52840 採用時のみ QVAR が活きる。

### HW 再設計の推奨（量産版 ESP32-S3 ベース、$25 BOM）

```
[ESP32-S3-WROOM-1-N8]  $3.5
 │
 ├─ USB-C (CDC + HID composite)
 ├─ I2C/SPI → [LSM6DSV16X] $4.10
 │              ├─ INT1 → GPIO (wake-on-motion)
 │              └─ QVAR (予備、未使用)
 ├─ TOUCH1-6 (native) ← 6 個の静電パッド (PCB パターン)
 │                      DeepSleep wake 対応
 ├─ GPIO0 ← 物理 PWR ボタン (強制 wake)
 └─ バッテリ + TP4056 + USB-C 充電

BOM 試算: $17.6 (ESP32-S3 $3.5 + IMU $4.1 + 電池 $3 + 充電 $1 + PCB $2
               + 筐体 $3 + USB-C $0.5 + 物理ボタン $0.5)
→ $25 内で余裕
```

**HW 変更ポイント**（現 M5StickC 物理 3 ボタン → 新設計）:
| 要素 | 現 M5StickC | 新量産版 (推奨) |
|------|------------|-----------------|
| 物理ボタン | 3 個 | **1 個 (PWR/wake のみ)** |
| 入力ボタン | 物理 | **Native Touch パッド 4-6 個** |
| IMU | MPU6886 | LSM6DSV16X |
| Wake 源 | ボタンのみ | PWR + Touch + Motion INT |
| ボタン摩耗 | あり | ほぼなし |
| 防水性 | 困難 | 容易 (タッチは接点なし) |

### Web UI の DeepSleep 設定タブ

```
DeepSleep 設定:
  自動スリープ:
    ○ 無効  ○ 10秒  ● 1分  ○ 5分  ○ 10分
  
  Wake ソース (複数可):
    ☑ 加速度閾値 [500 mg ─▼]
    ☑ Touch パッド 1 (起動用)
    ☐ Touch パッド 2-6 (任意)
    ☑ 物理 PWR ボタン (常時有効、無効化不可)
  
  現在のバッテリ: 85% (推定稼働: 2.8時間)
```

### Phase 6 以降の実装

- Phase 5 で量産版 HW の PCB 設計
- Phase 6 で DeepSleep FSM 実装
  - `esp_sleep_enable_ext1_wakeup` (物理 PWR + Touch)
  - `esp_sleep_enable_touchpad_wakeup`
  - IMU INT → GPIO wake
- Phase 7 で Web UI の DeepSleep 設定 UI

### アクセシビリティ観点

- **Touch/QVAR**: 軽い接触で反応 → 指の力が弱い方にも優しい
- **Wake-on-motion**: 手を動かせる方ならボタン操作不要
- **物理 PWR**: 誤起動を確実に防ぐ安全装置
- **3 経路並列**で、利用者の身体能力に合わせて使える wake ソースを選べる

---

## IMU チップ選定（最終製品向け）

### 現状の検証環境
- **M5StickC (現)**: MPU6886 内蔵、M5.IMU ライブラリで MCU 側 Mahony 相当フィルタ実行
- **Burst Motion (現)**: MPU6050、i2cdevlib で DMP 経由 quaternion。**MPU6050 は EOL (生産終了)、代替必須**

### 比較（最終製品向け候補）

| IMU | 軸数 | 内蔵 fusion | 出力 | MCU flash 消費 | 応答遅延 | 価格 (チップ) | 推奨度 |
|-----|------|-------------|------|----------------|----------|----------------|--------|
| MPU6886 (現) | 6 | 限定 DMP | 加速度/ジャイロ生 | ほぼ 0 (M5 lib) | ~20ms | ~$1 | 🟡 検証用継続 |
| MPU6050 (現) | 6 | ✅ DMP | Quaternion 可 | ~3KB (DMP image) | ~20ms | ~$0.5 | ❌ **EOL** |
| **ICM-20948** | 9 | ✅ DMP3 | Quaternion + 9 軸 | ~14KB (DMP3 image) | ~10-20ms | $7-15 | 🥈 コスト重視 |
| **BNO085 / BNO086** | 9 + ARM M0+ | ✅✅ SH-2 内蔵 | Quaternion 直出力 | ~5KB (lib のみ) | **<1ms (GIRV)** | $20-25 | 🏆 **最推奨** |
| LSM6DSV16X (ST) | 6 + ISPU | ◎ MLC | Quaternion 可 | ~10KB | ~5ms | $3-5 | 🟡 ST 専用習熟要 |
| BMI270+BMM150 (Bosch) | 6+3 別 IC | なし | 生データのみ | MCU 側 fusion 要 | MCU 次第 | ~$5 | 🟡 コスト最優先 |

### ユーザー質問への回答

**Q. ICM-20948 と BNO085 は IC 側で姿勢推定できたか？**
- **ICM-20948**: ✅ はい（DMP3 on-chip）。ただし **DMP3 firmware image ~14KB を MCU flash に持って起動時ロード**が必要
- **BNO085 / BNO086**: ✅✅ **より強力**。IC 内部に ARM Cortex-M0+ + CEVA SH-2 firmware が焼込み済み。**MCU 側は通信ライブラリ 5KB のみ**、fusion firmware 書込み不要

**Q. DMP の SW を FLASH に書く必要があり、ROM が不足するかどうか**
- M5StickC (4MB flash) なら DMP3 14KB も余裕（実質問題なし）
- BNO086 なら **fusion firmware を MCU に持たなくて済み、逆に ROM 消費が減る**（Mahony + キャリブレーションコード約 10KB 不要）
- 実際の ROM 圧迫要素は BLE stack (NimBLE ~30KB + NIMBLE_HID ~10KB)、ArduinoJson ~40KB、LittleFS ~20KB、Web アプリなし
- **ROM 不足の心配は ICM-20948 含めて無用**

### LSM6DSV16X の QVAR 機能（差別化要素）

**QVAR = Charge Variation (電荷変動)** = **静電容量センシング機能**。LSM6DSV16X に内蔵、BMI270 には無い独自機能。

- IMU チップに **QVAR 専用 2 ピン**、外部に金属パッド (PCB 銅 or 金属箔) を接続
- 指/物体が近づくと静電容量変化を検出
- ジャイロ/加速度と**同じ FIFO に統合格納**、低遅延 (~20-50ms)

**Motion Controller での活用案**:
1. **追加ボタン代用 (タッチセンサ)**: M5StickC の物理 3 ボタン → QVAR で +2〜4 ボタン追加（PCB パターンのみで実現、MCP23017 不要）
2. **非接触ジェスチャ (アクセシビリティ)**: 指を近づけるだけで入力、触れなくてよい — 身体障害のある方向けに画期的
3. **筐体デザイン自由度**: 物理ボタン配置に縛られず、QVAR 電極は任意形状の銅箔で OK

**実装タイミング**: Phase 5 末〜 Phase 6 で LSM6DSV16X 採用時に有効化。pk4 の `button_idx` で QVAR 電極もボタンとして扱える（button_idx 4-7 を QVAR 割当て等）。

**アクセシビリティプロファイル例**:
```json
{
  "name": "Accessibility - Hand Proximity",
  "hardware": {"imu_features":["qvar"]},
  "triggers": [
    {"mode":"hold_start_only", "start":{"button":{"idx":4,"state":"pressed"}}, "output":{"type":"keyboard","keys":["SPACE"]}}
  ]
}
```

---

### BOM $25 (¥1500) 制約 + LCSC 在庫での再評価

ユーザー要件: 最終製品の原価 $25 以内、LCSC で調達可能。BOM 内訳試算:
- MCU (ESP32-S3-WROOM-1-N8): ~$3.5
- IMU: ~$3-8 (現実的な枠)
- 電池+充電 IC+PCB+筐体+その他: ~$12-18
- IMU に使える予算は **$5-8** が実質上限

### LCSC 在庫調査結果（2026/04 時点）

| IMU | 単価 1pc | 100pc | 在庫 | 内蔵 fusion | 評価 |
|-----|---------|-------|------|-------------|------|
| **[LSM6DSV16X (ST)](https://www.lcsc.com/product-detail/C5267406.html)** | **$4.10** | ~$2-3 | ✅ **2377 pcs** | ✅ SFLP (Sensor Fusion Low-Power) + MLC | 🏆 **BOM最適** |
| [BMI270 (Bosch)](https://www.lcsc.com/product-detail/C2836813.html) | **$2.72** | ~$1.5 | ✅ 在庫あり | なし | 🥈 最安、MCU fusion |
| [ICM-20948 (TDK)](https://www.lcsc.com/product-detail/C726001.html) | $7.27 | ~$5 | ✅ 在庫あり | ✅ DMP3 9軸 | 🥉 絶対方位要時 |
| [ICM-42688-P (TDK)](https://www.lcsc.com/product-detail/C1850418.html) | $8.89 | ~$6 | ✅ 在庫あり | なし | 高精度低ノイズ用 |
| [BNO085 (CEVA)](https://www.lcsc.com/product-detail/C5189642.html) | $20.03 | $16.06 | ❌ **Out of Stock** | ✅✅ SH-2 | 予算NG + 欠品 |
| MPU6886 | - | - | ❌ LCSC 扱いなし | - | 非採用 |

### 量産製品推奨: **LSM6DSV16X @ $4.10**

**決定要因**:
1. **SFLP モード搭載**: IC 側で Quaternion 直出力可能、MCU 側 Mahony ほぼ不要 (BNO086 の半額以下でほぼ同等の UX)
2. **MLC (Machine Learning Core) 搭載**: 決定木 8 本で波動拳コマンド等の複雑ジェスチャ分類を IC オフロード可能 (Phase 9 拡張)
3. **LCSC 在庫 2377 pcs**: 量産即応
4. **$4.10 @ 1pc, 100pc で $2-3**: BOM 制約内
5. **ST 公式 Arduino ライブラリ**: [STMicroelectronics/LSM6DSV16X](https://github.com/stm32duino/LSM6DSV16X)
6. **LGA-14**: PCB 設計容易、超低消費 9µA

**弱点**:
- 6 軸（磁気なし）→ yaw ドリフトあり。Game Rotation Vector 相当モードで実用上問題なし（磁気擾乱対策が逆に不要）
- MLC 学習は ST Unico-GUI 必要（ただしオプション機能）

### 代替案

**現行互換優先**: **BMI270 @ $2.72**
- 既存 MPU6886 FW の Mahony コードをそのまま流用可能
- MCU 側 fusion、移行コスト最小
- BOM に $2 の余裕ができる

**9 軸必須**: **ICM-20948 @ $7.27**
- yaw ドリフトなし、絶対方位必要な用途 (AR/ロボット/コンパス機能)
- DMP3 14KB を MCU flash に持つが問題なし
- BOM タイトだが収まる

**プレミアム版 ($35 BOM 許容時)**: **BNO086**
- 性能最高、MCU コード最小
- LCSC 欠品中、DigiKey/Mouser で調達可（同価格帯）
- 量産初期は他 IMU で立ち上げ、プレミアムラインで採用の二本立てもあり

### 最終判断（ユーザー確定: LSM6DSV16X + BMI270 両対応、MCU は ESP32 系主力）

| フェーズ | MCU | IMU | 用途・理由 |
|----------|-----|-----|-----------|
| **検証 (現行)** | ESP32 PICO D4 (M5StickC) | MPU6886 (内蔵) | 既存 HW 継続、現行 FW 動作検証 |
| 検証 (現行) | ESP32-S3 (Burst Motion) | MPU6050 (EOL 注意) | Burst Motion 現行 HW |
| 検証 (手元活用、任意) | nRF52840 (XIAO Sense Plus) | LSM6DS3TR-C (内蔵) | Adafruit nRF52 Core、Mahony 検証 |
| **量産 v1 低価格** | ESP32-S3 | **BMI270** ($2.72) | 既存 Mahony 流用、$18-22 |
| **量産 v1 標準** | ESP32-S3 | **LSM6DSV16X** ($4.10) | SFLP + MLC、$22-26 |
| 量産 v2 プレミアム | nRF52840 (将来) | LSM6DSV16X | 低消費電力、$30-35、Phase 6+ |

### nRF52840 vs ESP32 の主力選定

**主力: ESP32 系** (M5StickC + M5Atom S3) を採用。

理由:
1. **ユーザー必須要件**: `arduino-esp32 v2` framework = ESP32 系
2. **M5StickC (ESP32 PICO D4) 既存コード流用**
3. **Motion Burst (ESP32-S3 暫定)** — ESP-NOW で Adapter 中継、nRF52840 は非対応
4. **5 年分の FW 資産**は ESP32 ベース、nRF52840 ポートは全面書き直し

**補助: nRF52840 (XIAO nRF52840 Sense Plus) を手元検証ボードとして活用**:
- Phase 1-5 は触らず ESP32 系に集中
- Phase 6 以降で nRF52840 HAL を追加、低消費電力版として評価
- HAL 抽象化を最初から行っておけば移植コスト ~2-3 週間

### HAL 抽象化でマルチ MCU/IMU 対応

```
src/
├── core/              # MCU/IMU 非依存
│   ├── TriggerEngine, MahonyFilter, Profile, ...
│   └── IImuSensor.hpp — 抽象インタフェース
├── hal/
│   ├── esp32/         # Phase 1-5 実装
│   │   ├── imu_mpu6886.cpp      (M5StickC)
│   │   ├── imu_bmi270.cpp       (量産低価格)
│   │   ├── imu_lsm6dsv16x.cpp   (量産標準、SFLP 使用)
│   │   └── imu_icm20948.cpp     (代替 9 軸)
│   └── nrf52840/      # Phase 6+ 追加
│       ├── imu_lsm6ds3trc.cpp   (手元 XIAO)
│       ├── imu_bmi270.cpp       (共通)
│       └── imu_lsm6dsv16x.cpp   (共通)
├── hid/
│   ├── IHidSink.hpp
│   ├── BleHidSink_nimble.cpp      (ESP32 系)
│   ├── BleHidSink_bluefruit.cpp   (nRF52840、Phase 6+)
│   ├── UsbHidSink_tinyusb_esp32.cpp
│   └── UsbHidSink_tinyusb_nrf52.cpp (Phase 6+)
```

プロファイル JSON は MCU/IMU に依らず同じスキーマ → **どの HW でも同じプロファイルが動く**。

### platformio.ini の env 構成

```ini
[env:m5stickc-esp32]            # 現検証
platform = espressif32
board = m5stick-c
framework = arduino
build_flags = -D IMU_TYPE=MPU6886 -D BOARD_M5STICKC

[env:m5atom_s3-esp32s3]          # 量産候補 (BMI270)
platform = espressif32
board = m5stack-atoms3
framework = arduino
build_flags = -D IMU_TYPE=BMI270 -D BOARD_M5ATOM_S3 -D USB_HID_ENABLE

[env:m5atom_s3-lsm6dsv16x]       # 量産候補 (LSM6DSV16X)
platform = espressif32
board = m5stack-atoms3
framework = arduino
build_flags = -D IMU_TYPE=LSM6DSV16X -D BOARD_M5ATOM_S3 -D USB_HID_ENABLE

[env:xiao_nrf52840_sense_plus]   # Phase 6+、手元検証
platform = nordicnrf52
board = xiaoblesense_adafruit
framework = arduino
build_flags = -D IMU_TYPE=LSM6DS3TRC -D BOARD_XIAO_NRF52840 -D USB_HID_ENABLE
```

これで「暫定で両対応」の設計意図を保ちつつ、**Phase 1-5 では ESP32 系のみ実装に集中**、nRF52840 は Phase 6 以降に HAL 層を追加するだけで対応可能な構造を維持します。

Sources:
- [LSM6DSV16XTR @ LCSC ($4.10, 2377 pcs)](https://www.lcsc.com/product-detail/C5267406.html)
- [BMI270 @ LCSC ($2.72)](https://www.lcsc.com/product-detail/C2836813.html)
- [ICM-20948 @ LCSC ($7.27)](https://www.lcsc.com/product-detail/C726001.html)
- [ICM-42688-P @ LCSC ($8.89)](https://www.lcsc.com/product-detail/C1850418.html)
- [BNO085 @ LCSC (Out of Stock, $16-20)](https://www.lcsc.com/product-detail/C5189642.html)
- [LSM6DSV16X product page (ST)](https://www.st.com/en/mems-and-sensors/lsm6dsv16x.html)
- [stm32duino/LSM6DSV16X Arduino library](https://github.com/stm32duino/LSM6DSV16X)

---

### （旧記述の参照用、検討過程）

**量産版: BNO086** (第一候補)
- 理由:
  - IC 側 fusion、MCU 計算負荷ゼロ
  - **Gyro-Integrated Rotation Vector モードで <1ms latency** — ゲーム用途で絶大
  - factory-tuned、磁気擾乱対策強化 (BNO086 の改善点)
  - Step Counter / Tap Detector / Activity Classifier も IC 内蔵、機能拡張余地大
  - Adafruit BNO08x / SparkFun BNO080 Arduino ライブラリが成熟
- 弱点: 価格 ($20-25)、入手性が他より劣る

**コスト版: ICM-20948** (第二候補)
- 価格半分 ($7-15)、調達容易
- DMP3 で BNO086 と同機能レベル（latency は劣る）
- SparkFun lib で実用的

**検証環境継続**: MPU6886 / MPU6050 (MCU 側 Mahony)
- 既存 M5StickC / Burst Motion の検証をそのまま使用
- 最終製品で BNO086 / ICM-20948 に切替

### 設計インパクト

**両対応する FW 構成** (`src/hal/imu_*.cpp` を IMU チップ別に用意):
| ファイル | IMU | Fusion 場所 | Mahony 使用 |
|----------|-----|-------------|-------------|
| `imu_mpu6886.cpp` (M5StickC) | MPU6886 | MCU | ✅ core/MahonyFilter |
| `imu_mpu6050.cpp` (Burst Motion 現) | MPU6050 | IC 側 DMP | ❌ (DMP quaternion 使用) |
| `imu_icm20948.cpp` (コスト版) | ICM-20948 | IC 側 DMP3 | ❌ |
| `imu_bno086.cpp` (量産版最推奨) | BNO086 | IC 側 SH-2 | ❌ |

コンパイル時 `-D IMU_TYPE=BNO086` 等で選択。`SensorState` にクォータニオン/生データを統一フォーマットで渡すので、core 層は IMU 種別を意識しない。

### Phase 調整

Phase 2 で MPU6886 (M5StickC) と MPU6050 (Burst Motion) の HAL を先行実装、Phase 5 以降で **BNO086 と ICM-20948 の HAL 追加**を最終製品向け作業として実施。両方動けば製品選択の自由度が増す。

Sources:
- [Adafruit BNO085 IMU Fusion Breakout](https://www.adafruit.com/product/4754)
- [BNO08X Datasheet (CEVA)](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)
- [SparkFun ICM-20948 Arduino Library DMP](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/DMP.md)
- [SlimeVR IMU Comparison](https://docs.slimevr.dev/diy/imu-comparison.html)
- [xioTechnologies/Fusion (Mahony 実装参考)](https://github.com/xioTechnologies/Fusion)

---

## IMU 軸の HW 依存オフセット & 姿勢推定

### 問題
Controller ごとに IMU チップの取付向きが異なる：
- M5StickC / Plus / Plus2: ケース前面 (LCD 側) を基準、IMU は裏面実装
- M5Atom S3: LED ドット表示面を基準、IMU は底面実装
- Motion Burst (ESP32-S3 暫定): 独自 PCB、軸は要測定

これにより、**同じ物理動作でも Controller 機種によって生の加速度/ジャイロの XYZ 値が違う**。そのままではプロファイルの互換性が取れず、Mahony/Madgwick フィルタ出力のクォータニオン・Euler も各機種でバラバラになる。

### 設計方針

**3 段階の座標変換を明示的に用意**:

```
[IMU 生データ (chip frame)]
    ↓ 1. 機種別軸リマップ (コンパイル時、HAL 層)
[Body frame = 標準デバイス向き]
    ↓ 2. Mahony/Madgwick 姿勢推定
[Quaternion in body frame (world 基準)]
    ↓ 3. ユーザー初期姿勢 offset (実行時、NVS 保存)
[User-frame quaternion & Euler (pitch/roll/yaw)]
```

### 1. 機種別軸リマップ（FW ハードコード、HAL 層）

各 Controller の `src/hal/imu_*.cpp` に軸リマップ行列を定義。以下は**標準 body frame 規約**に従う：
- **+X**: 右
- **+Y**: 前 (LCD 側 / 表示面)
- **+Z**: 上 (ディスプレイ上方向)

```cpp
// Example: M5StickC (LCD 横持ち、USB-C 左)
const AxisRemap M5STICKC_REMAP = {
  .accel_sign = { +1, -1, +1 },  // imu_x→body_x, imu_y→-body_y, imu_z→body_z
  .accel_perm = { 0, 2, 1 },     // imu [x,y,z] → body [x, z, y]
  .gyro_sign  = { +1, -1, +1 },
  .gyro_perm  = { 0, 2, 1 }
};
// M5Atom S3 / Motion Burst は別途キャリブレーション必要
```

実装:
```cpp
Vec3 remap(const Vec3& raw, const AxisRemap& r) {
  Vec3 v = { raw[r.perm[0]] * r.sign[0],
             raw[r.perm[1]] * r.sign[1],
             raw[r.perm[2]] * r.sign[2] };
  return v;
}
```

### 2. 姿勢推定フィルタ（body frame → quaternion）

**現状**:
- M5StickC: M5.IMU.getAhrsData() — M5 ライブラリ内蔵の Mahony 系複素数フィルタ
- XIAO ESP32-S3 (MPU6050): DMP 内蔵（Madgwick/Mahony 系）
- Mahony は姿勢+加速度 + ジャイロ統合、ジャイロドリフト補正付き

**新方針**: **FW 側で統一 Mahony フィルタを実装**、すべての Controller で同じ姿勢推定アルゴリズムを使用
- Mahony は軽量（数十 µs/update）、ESP32 の 100Hz 処理で余裕
- 既存 M5 IMU / DMP からの切替えで機種横断の再現性向上
- パラメータ:
  - `twoKp` (Proportional gain): 加速度による補正強度、デフォルト 2.0
  - `twoKi` (Integral gain): ジャイロドリフト補正、デフォルト 0.0（M5StickC で実績値 推奨）
  - サンプリング周波数 100Hz
- 実装: `src/core/MahonyFilter.hpp/cpp` に単一クラス、全 Controller 共通使用
- 参考: [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) (MIT license)、または古典的 Mahony 実装

**ソース例**:
```cpp
class MahonyFilter {
  float q0=1, q1=0, q2=0, q3=0;        // quaternion (body → world)
  float integralFBx=0, integralFBy=0, integralFBz=0;  // integral feedback
  float twoKp, twoKi;
public:
  void update(Vec3 gyro_rad_s, Vec3 accel_ms2, float dt);
  void toEuler(float* roll, float* pitch, float* yaw);
  void getQuat(float q[4]);
};
```

Mahony フィルタ固有のパラメータは Web UI の「センサー設定」タブで調整可能（上級ユーザー向け）。

### 3. ユーザー初期姿勢オフセット（実行時、NVS 保存）

既存の `QINIT` / `QINITH` / `QINITU` 概念を踏襲しつつ整理：

```cpp
struct UserQuatOffset {
  float q_ref[4];       // 「基準姿勢」を world frame quaternion で保存
  bool enabled;
  char name[16];        // "horizontal", "upright", "custom_1" 等
};
// NVS に up to 8 プリセット保存可
```

Serial コマンド:
```
{"cmd":"calibrate.qref.set", "name":"horizontal", "source":"current"}    // 現在の姿勢を保存
{"cmd":"calibrate.qref.set", "name":"upright",    "source":"current"}
{"cmd":"calibrate.qref.apply","name":"horizontal"}                       // これを基準に
{"cmd":"calibrate.qref.list"}                                            // 一覧
{"cmd":"calibrate.qref.delete","name":"horizontal"}
```

実行時のユーザー姿勢計算:
```
q_user = q_ref* × q_body     // q_ref の逆をかけて「基準姿勢で原点」
```
`QOFFSET` 相当は `calibrate.qref.set` with `source:"current"` + 即 apply。

### 4. 加速度/ジャイロ生データの bias キャリブレーション（3 レベル）

**レベル 1: 簡易（水平 1 点、10 秒、MVP 既定）**
- 既存 `calibrateMPUtoLittleFS()` の流れを踏襲
- デバイスを水平静止 → accel bias (aOX, aOY, aOZ) + gyro bias (gOX, gOY, gOZ) の 6 変数
- 日常用途・アクセシビリティで十分
```
{"cmd":"calibrate.simple", "duration_ms":10000}
```

**レベル 2: フル 6 点キャリブレーション（約 1 分、上級ユーザー向け）**
- ± 3 軸各方向 (+X, -X, +Y, -Y, +Z, -Z) に静置、各 10 秒
- 12 変数推定: 3 軸 bias + 3 軸スケール + 3 軸間クロストーク
- **効果**: accel bias ±20mg→±2mg、scale ±3%→±0.5%、Euler 誤差 ±2°→±0.5°
- ゲーム用途、高精度姿勢検出が必要な場合
- Web UI ウィザードで各面を向けるよう指示
```
{"cmd":"calibrate.full.start"}
{"cmd":"calibrate.full.capture", "step":1}  # step 1-6
...
{"cmd":"calibrate.full.finish"}
```

**レベル 3: 磁気キャリブレーション（9 軸 IMU 採用時のみ）**
- ICM-20948 / BNO086 で磁気センサを絶対方位に使う場合
- 「8 の字を描くように 30 秒振る」→ Hard iron + Soft iron 補正
- **BNO086 は自動**（Interactive Calibration 機能）、ユーザー操作不要
- **ICM-20948 は手動**（DMP3 dynamic magnetometer cal に移行するのが妥当）
- MPU6886/MPU6050 (6 軸) には無関係
```
{"cmd":"calibrate.magnetometer", "duration_ms":30000}
```

**初回起動の自動ウィザード**:
- NVS `calibration_done` フラグを起動時確認
- 未校正なら LCD に「水平に置いてください」表示 + 10 秒カウントダウン
- 自動で簡易キャリブレーション実行 → NVS 保存 → 以降は読込のみ
- ユーザーはいつでも Web UI から再校正可能

**推奨マトリクス**:
| ユーザータイプ | 推奨レベル |
|--------------|----------|
| 一般ゲーマー / プレゼン / リモコン | 1 (簡易) |
| FPS / 格闘ゲーム / VR | 2 (フル) |
| 絶対方位活用 (ロボット、AR) | 2 + 3 |
| アクセシビリティ（身体制約） | 1 のみ、または BNO086 自動 |

### 5. プロファイルのクォータニオン互換性

pk4 の `start.quaternion` / `start.euler` は**ユーザー姿勢フレーム（q_ref 適用後）**で記録する：
- 別機種 Controller にインポートしても、各機種の軸リマップ + その機種のユーザー基準姿勢適用後は同じ「ユーザー frame」になるので、プロファイルは互換
- 例: M5StickC で登録した「右に傾ける→W」は、M5Atom S3 でも「右に傾ける→W」として機能

ただし機種ごとに軸リマップの定義が正しくないと一致しない → **機種別リマップの正確な実測キャリブレーション**が必須（Phase 2 の各 HAL 実装時に実施）。

### 6. q_ref の保存場所（LittleFS / NVS の統合）

**ユーザー質問への回答**: プロファイル JSON 側に **q_ref を埋込む**（LittleFS 側に統合）。

```json
// /profiles/cs2-basic.json
{
  "schema_version": 2,
  "id": "cs2-basic",
  "calibration_context": {
    "qref_name": "horizontal",
    "qref_quat": [1.0, 0.0, 0.0, 0.0]     // +16B、プロファイル作者の基準姿勢
  },
  "hardware": {...},
  "triggers": [...]
}
```

**保存先の整理**:
| データ | 保存先 | 理由 |
|--------|--------|------|
| プロファイル本体 + qref 埋込み | **LittleFS `/profiles/*.json`** | 自己完結、可搬 |
| ユーザー固有 qref プリセット (最大8個) | **NVS `qref_0` 〜 `qref_7`** | プロファイル独立、デバイス固有の基準姿勢ライブラリ |
| 現在 active な qref (プロファイル追従) | **NVS `current_qref`** | プロファイル切替で自動更新 |

**可搬性**: プロファイル JSON を別デバイスにインポート → インポート時に `qref_quat` を NVS `current_qref` に書込み → 即動作。ユーザー固有プリセット (qref_0〜7) とは独立なので上書きしない。

**Web UI の動作**:
- プロファイル保存時: 現在の active qref を自動埋込み
- プロファイルインポート時: 「このプロファイル作者の基準姿勢 (horizontal) を使いますか？」確認ダイアログ → 承認で `current_qref` 更新

### 6. Web UI の対応ページ

新「センサー設定」ページ:
- 現在の生加速度/ジャイロ表示（chip frame）
- 軸リマップ適用後の body frame 表示
- Mahony フィルタ出力の quaternion / Euler 表示
- 「現在の姿勢を基準に設定」「水平基準」「直立基準」ボタン
- Mahony ゲイン (twoKp, twoKi) スライダ
- Bias キャリブレーション実行ボタン
- プリセット一覧と切替

これにより、**ユーザーが機種を変えても同じプロファイルが動く**ことと、**センサーがずれても簡単に再キャリブレーションできる**ことを両立。

---

## マウス出力の実装方針（M5C/S3 共通）

マウスは**2系統**の出力形式をサポート：

### A. トリガー発火型マウス（pk4 msg_format=2）
姿勢や加速度が条件を満たすと**離散的に**マウスイベントを送信。
- **用途**: 「頭を振る→左クリック」「右に傾ける→右クリック」「振り下ろす→ホイール下」
- **inputs_msg[20] レイアウト**: `[0]=buttons`, `[1..2]=dx int16 LE`, `[3..4]=dy int16 LE`, `[5]=wheel int8`, `[6]=hWheel int8`, `[7]=interval_ms`
- **実装**: BleCombo.move(x, y, wheel) / BleCombo.press(MOUSE_LEFT) を TriggerEngine から呼ぶ。S3 USB HID も同 API を USBHIDMouse で実装

### B. 連続運動型マウス（MODE_MOUSE、既存温存+改良）
IMU の姿勢/ジャイロから**連続的に**マウスカーソルを動かす（エアマウス）。
- **用途**: プレゼン/PC 操作、ヘッドマウスとしての使用
- **既存実装**: `MotionController.hpp` に MODE_MOUSE (mode=4) と `moveMouse(x, y)` があり、改良して使用
- **新規パラメータ**: 感度（sensitivity）、デッドゾーン、反転軸、有効化スイッチ（Fnキー相当のボタン押下中のみ有効）
- **NVS 保存**: `mouse_config` (sensitivity_x/y, deadzone, invert_flags, enable_mode)
- **Serial コマンド**: `SET_MOUSE,sens_x=10,sens_y=10,deadzone=2,invert_y=1,enable_mode=hold_btn0`
- **Web UI**: 「マウスモード」タブで感度スライダ + テスト用 XY プレビュー

### C. 両方混在（推奨構成）
通常は連続マウス、ボタン押下中に pk4 トリガーでクリック/ホイールを発行。BLE HID は単一デバイスで OK。USB HID は単一 USBHIDMouse インスタンス。

---

## ボタン入力の拡張（4個以上対応）

### 現状
| ボード | 現在のボタン | 内訳 |
|--------|--------------|------|
| M5StickC | 3 個 | GPIO 0 (電源, PU), GPIO 36 (INPUT), GPIO 26 (PU) |
| XIAO ESP32-S3 (illumiTrack) | 4 個 | GPIO 2/3/4/5 (容量式、小指/薬指/中指/人差し指) |
| XIAO ESP32-C3 | 1 個 | GPIO 2 (容量式) |

### 製品版で 4+ ボタンに対応するための選択肢

**M5StickC 側（物理 GPIO が枯渇気味）**
- ✅ **推奨**: **Grove I2C GPIO エクスパンダ** (MCP23017 = 16ch, PCF8574 = 8ch)。Grove ポート (GPIO 32/33) 経由で接続。割込みピンを 1 本（GPIO 25 = Grove IN）に繋げばスキャン負荷ゼロ。コスト約 300 円。
- ⭕ 代替: **Grove 4-Button Unit** (M5Stack 純正) = 4 ボタンが I2C で読める完成品。
- ⭕ 代替: **アナログボタンマトリクス**（抵抗分圧で 1 ADC ピンに 4-8 ボタン）。配線簡素、精度要注意。
- ❌ 非推奨: GPIO 25 (Grove 単一ピン) を digital-in 化して足す。Grove I2C を潰すことになる。
- ❌ 非推奨: GPIO 35 (IR LED) を奪う。IR 機能が失われる。

**XIAO ESP32-S3 側**
- ✅ 標準状態で **D0〜D10 の 11 GPIO が使用可能**。容量式なら 4 指既に実装済み、さらに D6/D7/D8/D9/D10 を使える。
- ✅ **Grove I2C 拡張**も可（I2C ピン: D4/D5）。

### FW 側の実装（ボタン数の可変対応）
- `MotionController.hpp` の `button[N]` 配列を**ランタイム可変**に：
  ```cpp
  struct ButtonConfig {
      uint8_t source; // 0=GPIO, 1=I2C_EXPANDER, 2=CAPACITIVE, 3=ANALOG_MATRIX
      uint8_t addr_or_pin;
      uint8_t bit_mask;
      uint8_t pull_mode; // 0=PU, 1=PD, 2=NONE
  };
  ButtonConfig buttons_cfg[MAX_BUTTONS]; // MAX_BUTTONS = 16
  uint8_t button_count;
  ```
- NVS に `button_layout` として永続化。Web UI の「ハードウェア設定」タブから選択。
- **pk4 の `button_idx` は uint8_t (0-255)** なので 16 個まで余裕で拡張可能（既存互換）。
- **I2C スキャンタスク**: 新規に追加（`ButtonI2CLoop`、50Hz）。MCP23017 の INT 信号で割込み化（消費電力最適化、将来）。
- **既存 `switchRead()` をボタンソース別に分岐**させる抽象化関数 `readButton(idx)` を追加。

### Web UI での設定
- 「ハードウェア設定」新タブ
- ボタン本数選択（1-16）
- 各ボタンの source (GPIO 直結 / I2C エクスパンダ / 容量式) をドロップダウン
- 「現在押下中のボタンを登録」ボタンで自動検出
- `SET_BUTTONS` Serial コマンドでデバイスに書き込み（基本情報と一緒に）

### サポート優先順
1. **MVP**: 既存の 3 ボタン (M5StickC) / 4 ボタン (S3 illumiTrack) を TriggerEngine で引き続き扱う。`button_count` は定数のまま。
2. **Phase 5**: MCP23017 対応を実装、Web UI で本数可変。製品版 (4+) はこの時点で揃う。

---

## 5 年前設計との整合（アクションルール概念の継承）

**ユーザーの 5 年前設計メモの要点**:

> アクションルール = 主体 + トリガー + アクション
> - 主体: 本体 / IMU センサ
> - トリガー: ボタン / 姿勢 / 加速度 / ジャイロ
> - アクション: キーボード / ゲームパッド / マウス
> 
> 詳細:
> - 複数トリガーを **AND / OR 条件** で結合
> - 姿勢は "roll,pitch,yaw" CSV、**4 要素ならクォータニオン**
> - 加速度/ジャイロは **合成値 (ABS) 1 つ** で指定
> - アクションルールは**ドラッグで順番変更**、上から優先度 index
> - 送信ボタンで **Web Serial 経由でアクションルールリスト送信**

### 現計画への反映 (3 点)

#### 反映 1: 複数トリガーの AND / OR 論理演算
現計画の Condition は**暗黙の AND のみ** → `logic_op` を追加:
```cpp
enum class LogicOp : uint8_t { AND, OR };
struct ActionRule {
    LogicOp start_logic;   // 開始条件の結合
    Condition start;
    LogicOp end_logic;     // 終了条件の結合 (HOLD_START_END 時)
    Condition end;
    ...
};
```
Web UI で `条件結合: [ すべて満たす (AND) ▼ ] or [ いずれか (OR) ]` で切替。

#### 反映 2: 加速度/ジャイロの合成値 (ABS) 優先入力
```cpp
struct Condition {
    bool accel_use_abs_only;     // true: 合成値のみ、false: 軸別
    float accel_abs_threshold;    // 合成 ABS 閾値 (単一値)
    float accel_per_axis[3];      // 軸別 (optional、詳細モード時)
    bool gyro_use_abs_only;
    float gyro_abs_threshold;
    float gyro_per_axis[3];
    ...
};
```
Web UI は **合成値 1 つ入力**を既定、「軸別にする」ボタンで詳細モード展開。
5 年前の設計思想: **ユーザー簡素化** (最初は合成値で十分、上級者のみ軸別)。

#### 反映 3: ドラッグ&ドロップで優先度編集
Web UI の Trigger Editor (「アクションルール」ページ) で:
- ルールリストを**ドラッグで並替え可能**
- 上から順に priority index を自動採番
- 内部構造 (priority フィールド) は変更なし、UI 表現のみ変更

### 用語統一: UI 表示は「アクションルール」

5 年前の「アクションルール」はユーザーフレンドリーな表現。**内部コードは `Trigger` 維持**、**Web UI 表示のみ「アクションルール」で統一**:

| 層 | 用語 |
|----|------|
| Web UI 表示 | **アクションルール (Action Rule)** |
| JSON スキーマ | `rules[]` (旧 `triggers[]` からリネーム検討) |
| FW 内部構造体 | `Trigger` / `ActionRule` 両表記可 |
| プロトコル | `rule.add`, `rule.list`, `rule.remove` に変更候補 |

### Web UI のアクションルール編集デザイン

```
┌─────────────────────────────────────────────────────────┐
│ 🎮 アクションルール                                      │
│                                                          │
│ ≡ ルール 1: 波動拳 [ONESHOT]         [ ⚙ ][ × ]         │ ← ドラッグで並替え
│ ≡ ルール 2: 右傾け W 連打 [HOLD]     [ ⚙ ][ × ]         │
│ ≡ ルール 3: 振って射撃 [ONESHOT]     [ ⚙ ][ × ]         │
│                                                          │
│ [ ＋ アクションルール追加 ]                              │
│                                                          │
│ [ 📤 Controller へ送信 ]                                 │
└─────────────────────────────────────────────────────────┘

詳細編集モーダル:
┌─────────────────────────────────────────────────────────┐
│ ルール名: [ 波動拳 Hadoken         ]                    │
│ モード: [ ONESHOT ▼ ]                                   │
│                                                          │
│ ─── トリガー（開始条件）──────────────────────────       │
│ 条件結合: [ すべて満たす (AND) ▼ ]                       │
│                                                          │
│ ☑ ボタン:   (  ) A  (  ) B  ( ● ) C                    │
│ ☐ 姿勢:     [ 0, -45, 0 ]  許容±[ 10, 10, 10 ]         │
│ ☑ 加速度:   合成値 ≥ [ 3.0 g ]  [ 軸別にする ]         │
│ ☐ ジャイロ: 合成値 ≥ [ ___ °/s ]                        │
│                                                          │
│ ─── アクション ─────────────────────────────────────       │
│ 種別: ( ● ) キーボード  (  ) ゲームパッド (  ) マウス    │
│ キー: [ ↓→↓→P ] [ 16 進入力 ▼ ]                        │
│ 修飾: ☐ Ctrl  ☐ Shift  ☐ Alt                           │
│                                                          │
│ 連続時間: [ 30 ] ms    クールダウン: [ 100 ] ms          │
│                                                          │
│ [ キャンセル ] [ 保存 ]                                  │
└─────────────────────────────────────────────────────────┘
```

---

## 動作キャプチャ記録（Motion Capture Recording）

ユーザー要件: **「手動設定 + 動作記録」両対応、シンプルルールベース、推論（ML/DTW）は遅延が懸念**。

### 🏆 採用: 統一状態機械モデル（全モード統一、UI は 1-2 状態のみ当面露出）

**ユーザー確定方針**: 
- **FW 内部は統一状態機械モデル**（1-N 状態対応、汎用 evaluator）
- **UI は 1-2 状態のモードのみ表示**（ONESHOT / HOLD_START_ONLY / HOLD_START_END）
- **SEQUENCE (3+ 状態) は FW 実装済みだが UI 非表示、将来拡張枠**

**設計の本質**:
- ONESHOT / HOLD_START_ONLY / HOLD_START_END / SEQUENCE は**すべて状態機械の特殊ケース**
- N=1 状態 = ONESHOT または HOLD_START_ONLY
- N=2 状態 = HOLD_START_END
- N=N 状態 = SEQUENCE
- **複数ルール同時処理 OK** — 各ルールが独立した状態を保持、毎 tick で並列評価

### 統一データ構造

```cpp
struct State {
    Condition match_condition;   // この状態にいる条件
    uint16_t min_dwell_ms;       // 最小滞在時間 (誤検出防止)
    uint16_t max_dwell_ms;       // タイムアウト (超えたらリセット)
    OnEnterAction on_enter;      // 状態入場時 (press / release / fire_macro / none)
    OnExitAction  on_exit;       // 状態離脱時
};

struct ActionRule {
    uint16_t id;
    char name[32];
    std::vector<State> states;   // 1-N 状態
    bool loop;                   // true=循環 (HOLD_*)、false=一方通行 (ONESHOT/SEQUENCE)
    
    // runtime (各ルール独立、RAM のみ)
    int current_state;           // -1=idle
    uint32_t state_enter_ms;
};
```

### UI プリセット ↔ 内部構造のマッピング

| UI モード | 内部 states[] | loop | MVP UI 表示 |
|-----------|--------------|------|-------------|
| ONESHOT | 1 状態: `{match, on_enter:fire+cooldown}` | false | ✅ |
| HOLD_START_ONLY | 1 状態: `{match, on_enter:press, on_exit:release}` | true | ✅ |
| **HOLD_START_END** | **2 状態: `[{start,press}, {end,release}]`** | true | **✅ (ご要望の中核)** |
| SEQUENCE | N 状態 | false | ❌ 非表示（将来拡張） |

ユーザーは UI プリセット経由で設定、内部は統一表現で処理。

### 汎用 Evaluator (全モード共通、if-else のみ、遅延ゼロ)

```cpp
void evaluateStateMachine(ActionRule& r, SensorState& s, uint32_t now) {
    if (r.current_state < 0) {
        // idle: 最初の state 条件チェック
        if (matchCondition(r.states[0].match_condition, s)) {
            r.current_state = 0;
            r.state_enter_ms = now;
            executeAction(r.states[0].on_enter);
        }
    } else {
        State& cur = r.states[r.current_state];
        
        // タイムアウト
        if (cur.max_dwell_ms > 0 && (now - r.state_enter_ms) > cur.max_dwell_ms) {
            executeAction(cur.on_exit);
            r.current_state = -1;
            return;
        }
        
        // 次状態遷移
        int next = r.current_state + 1;
        if (next >= r.states.size()) {
            if (r.loop) next = 0;  // HOLD_* 循環
            else next = -1;         // ONESHOT/SEQUENCE 完了
        }
        
        if (next >= 0 && matchCondition(r.states[next].match_condition, s)) {
            executeAction(cur.on_exit);
            r.current_state = next;
            r.state_enter_ms = now;
            executeAction(r.states[next].on_enter);
        }
    }
}

void tick(SensorState& s, uint32_t now) {
    for (auto& rule : action_rules) {
        evaluateStateMachine(rule, s, now);  // 各ルール独立、並列処理
    }
}
```

### HID 出力の衝突対応 (複数ルールが同じキーを押す場合)

```cpp
// キー押下カウンタで多重 press/release を管理
void pressKey(uint8_t key) {
    press_count[key]++;
    if (press_count[key] == 1) bleHid.press(key);  // 初めの 1 回のみ
}
void releaseKey(uint8_t key) {
    press_count[key]--;
    if (press_count[key] == 0) bleHid.release(key);  // 全ルール離れたら release
}
```

### 🔽 以下は検討過程の資料（参考）、上の統一モデル方針が最終確定

---

### 設計方針 (旧議論): Level 1 = ルールベース (全トリガーモード)、Level 2 = SEQUENCE のみ Phase 6+、Level 3 = 不採用

#### Level 1: ルールベース (MVP 主力、🏆 全トリガーモード対応)

**Level 1 は「アルゴリズムの分類」でありモード限定ではない**:
- 判定は if-else のみ、ML/DTW 一切なし、遅延ゼロ
- **以下の全トリガーモードを Level 1 で実装**:
  - **ONESHOT** — 開始条件一致で 1 発発火、クールダウン付き
  - **HOLD_START_END** — 開始条件で press、**終了条件で release**（別条件指定、2 値判定）
  - **HOLD_START_ONLY** — 開始条件で press、条件離脱で自動 release
- 出力は msg_format 別に keyboard / mouse / gamepad / consumer / macro すべて対応

**判定ロジック全パターン (Level 1、すべて if-else)**:
```cpp
void evaluateOneshot(ActionRule& r, SensorState& s) {
    if (!r.armed_cooldown && matchCondition(r.start, s)) {
        fireAction(r.action);  // msg_format に従い keyboard/mouse/gamepad/macro 発火
        r.armed_cooldown = true;
    } else if (r.armed_cooldown && !matchCondition(r.start, s)) {
        r.armed_cooldown = false;
    }
}

void evaluateHoldStartEnd(ActionRule& r, SensorState& s) {
    bool start_match = matchCondition(r.start, s);
    bool end_match   = matchCondition(r.end, s);    // 別条件
    if (!r.holding && start_match) {
        hidSink.press(r.action);
        r.holding = true;
    } else if (r.holding && end_match) {
        hidSink.release(r.action);
        r.holding = false;
    }
}

void evaluateHoldStartOnly(ActionRule& r, SensorState& s) {
    bool start_match = matchCondition(r.start, s);
    if (!r.holding && start_match) {
        hidSink.press(r.action);
        r.holding = true;
    } else if (r.holding && !start_match) {
        hidSink.release(r.action);   // 開始条件離脱で自動 release
        r.holding = false;
    }
}
```

**具体的な用途例**:
- **ONESHOT + Macro**: 「右に振る」→ 波動拳コマンド "↓→↓→P" 順次送信
- **HOLD_START_END**: 「右傾け」→ Shift 押下、「左傾け」→ Shift 解除（開始/終了が別状態）
- **HOLD_START_ONLY**: 「右傾けてる間ずっと」→ D キー押下継続、戻したら離す

**利点**:

**用途例 (モード別)**:

| モード | 例 | 記録方法 |
|--------|----|---------|
| ONESHOT | 右に振る → 波動拳 "↓→↓→P" Macro 出力 | 1 姿勢 + 加速度ピークをキャプチャ |
| **HOLD_START_END** | **右傾けで Shift 押下、左傾けで解除** | **開始姿勢と終了姿勢を個別にキャプチャ** |
| HOLD_START_ONLY | 右傾けてる間だけ D キー連打 | 1 姿勢をキャプチャ、離脱で自動 release |

**利点**:
- 判定は毎 IMU tick で if-else 数回、ML/DTW 一切不要、遅延ゼロ
- 記録は姿勢 + 加速度閾値をキャプチャするだけ
- 既存 pk3 / execSF_HIDInputs の完全上位互換
- MPU6886 でも余裕で動作、5 年前の資産そのまま活用可
- **開始・終了 2 値判定 (HOLD_START_END) も同じ if-else で実装可能、追加複雑度ゼロ**

**記録フロー (モード別)**:

**ONESHOT (波動拳等)**:
```
1. 「動作キャプチャ」選択
2. Controller を目的の姿勢に構えて「▶ 振ってください」
3. 3g 超の加速度検出で自動記録停止
4. 記録された姿勢 + 加速度ピーク値を表示
5. 出力キー順序 (例: "↓,→,↓,→,P") を入力
6. 保存 → デバイスへ送信
```

**HOLD_START_END (開始・終了 2 値判定、ご要望の中核機能)**:
```
1. 「動作キャプチャ」選択、モード "HOLD_START_END"
2. Controller を開始姿勢に構えて「📷 開始姿勢をキャプチャ」
   → 記録: Euler + Quaternion 両方保存
3. Controller を終了姿勢に構えて「📷 終了姿勢をキャプチャ」
   → 記録: Euler + Quaternion 両方保存
4. 許容範囲 (Euler ±10° 等) 自動設定、調整可
5. アクションキー (例: Shift) を選択
6. 保存 → 開始条件一致で press、終了条件一致で release される
```

**HOLD_START_ONLY (条件継続中 press)**:
```
1. 「動作キャプチャ」選択、モード "HOLD_START_ONLY"
2. Controller を目的の姿勢に構えて「📷 姿勢をキャプチャ」
3. アクションキーを選択
4. 保存 → 姿勢一致中は press、離れたら自動 release
```

#### Level 2: 姿勢遷移判定（状態機械、Phase 6+ オプション）

**入力判定**: 複数姿勢を順次通過 (SEQUENCE モード)
- 状態機械で「↓→→」の姿勢遷移を実際にセンサで検出
- CPU 負荷小 (if-else のみ、ML/DTW なし)
- Level 1 より「動きの方向で厳密に区別」できる
- **多くの用途では Level 1 で十分** → MVP で不採用、需要に応じて Phase 6+ で追加

#### Level 3: DTW / ML-based（不採用）

- DTW: 時間軸ゆらぎに強いが数 ms の追加遅延
- MLC (LSM6DSV16X 内蔵): 学習データ作成コスト、遅延懸念
- **採用しない方針**、LSM6DSV16X の MLC は Phase 9+ で余裕があれば検討

### Level 1 の記録データ構造（既存 pk3 互換）

```json
{
  "id": "sf2-hadoken",
  "name": "波動拳",
  "mode": "oneshot",
  "start": {
    "posture":   {"euler":[0,90,0], "euler_tol":[15,15,180], "quat":[...], "judge_by":"euler"},
    "accel_abs": {"threshold": 3.0}
  },
  "action": {
    "type":     "keyboard",
    "msg_format": 5,                            // Macro
    "keys":     ["DOWN","RIGHT","DOWN","RIGHT","P"],
    "interval_ms": 30
  }
}
```

### Level 2 用のシーケンスキャプチャ（Phase 6+ 実装、参考）

将来的に Level 2 を追加する場合の設計（MVP では実装しない）:
- 「▶ 記録開始」→ IMU 50Hz 取得 → 「⏹ 停止」or 自動停止
- キーフレーム自動抽出（加速度ピーク、停留点検出）
- 状態機械で順次通過判定
- 計画は別途、需要が確認できた時点で具体化

### Euler + Quaternion 両方保存（ユーザー要望確定）

```json
{
  "posture": {
    "euler":       [0, -45, 0],         // 常に保存
    "euler_tol":   [10, 10, 180],
    "quat":        [0.92, 0, -0.38, 0], // 常に保存
    "quat_dot_min": 0.95,
    "judge_by":    "euler"              // "euler" | "quat"、判定に使う方
  }
}
```

**利点**:
- **3D 可視化は常に Quaternion** (連続表現、ジンバルロックなし)
- **判定は Euler 既定** (UI 直感性、通常姿勢範囲では十分)
- 特殊姿勢では `judge_by: "quat"` に切替可
- プロファイル完全可搬、インポート先で判定方法切替可

### 3D ライブビュー統合の Trigger Editor UI

```
┌──────────────────────────────────────────────────────────┐
│ 🎮 アクションルール編集                                   │
│                                                           │
│ ┌──────────────────────┐  ┌─────────────────────────────┐│
│ │                      │  │ ルール名: [波動拳         ] ││
│ │    🟦 3D モデル       │  │ モード: [ SEQUENCE ▼ ]      ││
│ │  (Three.js、Live)    │  │                             ││
│ │                      │  │ ─ トリガー記録 ──────────   ││
│ │  Pitch: -15°         │  │ 方式:                       ││
│ │  Roll:  +30°         │  │ ( ● ) 動作キャプチャ        ││
│ │  Yaw:   +5°          │  │ (  ) 手動入力                ││
│ │                      │  │                             ││
│ │  Quat:               │  │ [ ▶ 記録開始 ] [ ⏹ 停止 ]  ││
│ │  [0.92,0,-0.38,0]    │  │                             ││
│ │                      │  │ 記録キーフレーム:           ││
│ │  Accel: 0.3g         │  │  1. 0ms    p=0, r=0         ││
│ │  Gyro:  5°/s         │  │  2. 150ms  p=0, r=-45 ⟳    ││
│ │                      │  │  3. 300ms  p=45, r=0 🔥ピーク││
│ │  [ 再生プレビュー ]  │  │  4. 500ms  2.8g (発火点)    ││
│ │                      │  │                             ││
│ └──────────────────────┘  │ 許容: ±[15]°, 時間±[100]ms  ││
│                           │                             ││
│ [ 📷 単一姿勢キャプチャ ]  │ ─ アクション ────────────   ││
│                           │ キー: [ ↓→↓→P ]             ││
│                           │ [ キャンセル ] [ 保存 ]      ││
│                           └─────────────────────────────┘│
└──────────────────────────────────────────────────────────┘
```

**特徴**:
- **左ペイン: 常時 3D ライブビュー** — 現在の動きが見える
- **右ペイン: ルール編集** — 動作を見ながら設計
- 記録後**再生プレビュー**（タイムラインスクラブで 3D 再現）

### MVP 判定ロジック（Level 1、シンプル）

既存 `execSF_HIDInputs` (5 年前実装) をそのまま TriggerEngine 内に汎用化：

```cpp
void evaluateOneshot(ActionRule& rule, SensorState& s) {
    if (matchStartCondition(rule.start, s) && !rule.armed_cooldown) {
        fireMacroAction(rule.action);  // Macro 展開 = 複数キー順次送信
        rule.armed_cooldown = true;
    }
    if (!matchStartCondition(rule.start, s) && rule.armed_cooldown) {
        rule.armed_cooldown = false;  // 条件離脱でクールダウン解除、再 arm
    }
}

void fireMacroAction(Action& a) {
    // msg_format=5 Macro
    for (size_t i = 0; i < a.keys.size(); i++) {
        hidSink.press(a.keys[i]);
        vTaskDelay(a.interval_ms);
        hidSink.release(a.keys[i]);
    }
}
```

**CPU 負荷**: 毎 tick で全 pk4 エントリを O(N) 走査、各エントリ if-else 数回のみ。N=256 エントリでも余裕で 100Hz 処理可。ML/DTW 一切なし、遅延ゼロ。

### Level 2 (Phase 6+ オプション、参考)

将来必要なら状態機械で姿勢遷移判定:
```cpp
// Level 2 は MVP では実装しない、需要確認後に追加
void evaluateSequence(ActionRule& rule, SensorState& s, uint32_t now_ms) {
    // 状態機械、if-else のみ、ML/DTW なし
    ...
}
```
CPU 負荷も小さい (if-else のみ) が、MVP では Level 1 で十分。

### データモデル追加

```cpp
struct SequenceKeyframe {
    uint16_t time_offset_ms;
    float euler[3];
    float quat[4];
    float accel_abs;
    float gyro_abs;
    bool is_peak;              // 加速度ピークフラグ (発火 trigger のヒント)
};

struct Sequence {
    std::vector<SequenceKeyframe> keyframes;
    uint16_t time_tolerance_ms;
    float posture_tolerance_deg;
    float accel_tolerance_g;
};

struct ActionRule {
    TriggerMode mode;
    Condition start, end;      // ONESHOT/HOLD 用
    Sequence sequence;          // SEQUENCE 用 (新規)
    ...
};
```

### プロファイル JSON 例（シーケンス付き、波動拳）

```json
{
  "id": "sf2-hadoken",
  "name": "波動拳",
  "mode": "sequence",
  "sequence": {
    "keyframes": [
      {"t":0,   "euler":[0,0,0],    "quat":[1,0,0,0],         "accel_abs":0.2},
      {"t":150, "euler":[0,0,-45],  "quat":[0.92,0,0,-0.38],  "accel_abs":0.3},
      {"t":300, "euler":[0,0,0],    "quat":[1,0,0,0],         "accel_abs":0.4},
      {"t":500, "euler":[0,0,45],   "quat":[0.92,0,0,0.38],   "accel_abs":2.8, "is_peak":true}
    ],
    "time_tolerance_ms": 100,
    "posture_tolerance_deg": 15,
    "accel_tolerance_g": 0.5
  },
  "action": {
    "type": "keyboard",
    "keys": ["DOWN", "RIGHT", "DOWN", "RIGHT", "P"]
  }
}
```

### 実装タスク（Level 1 のみ、MVP で完結）

**Phase 2 (FW)**:
- `src/core/TriggerEngine.hpp/cpp` — ONESHOT 判定ロジック統合、既存 execSF_HIDInputs を汎用化
- Macro 出力（msg_format=5）の実装、連続キー順次送信

**Phase 3 (Web UI)**:
- `src/pages/TriggerEditor.jsx` に 3D ライブビュー埋込み
- 「振ってください」単発キャプチャボタン（姿勢 + 加速度ピークを記録）
- ユーザーが出力キー順序を入力するフォーム（カンマ区切り or ドラッグ&ドロップで並べる）
- 記録された姿勢の Euler / Quaternion 両方を JSON に保存
- Web UI で記録→許容範囲自動提案→ユーザー確認→保存の流れ

**Phase 6+ オプション** (需要確認後):
- Level 2 姿勢遷移判定（状態機械）
- `SequenceRunner.hpp/cpp`、`MotionRecorder.js`、`KeyframeExtractor.js`

**不採用**:
- Level 3 DTW / ML — 遅延懸念、必要性低

---

## トリガー発火モデル（開始状態のみ／開始＋終了状態）

### 現状再確認
現 FW は **開始状態のみ (one-shot)** の実装：
- `hidSessionLoop` ([IMU_BLEorSerial_tester.cpp:491-523](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/IMU_BLEorSerial_tester.cpp#L491)) が `events_bool[0] == true`（= ボタン立ち上がり）を検知すると、その瞬間の姿勢で `getClosestPK3` から最近傍 pk3 を選び、1 キー発火して `events_bool[0] = false`
- 離す側は `prev_button_state` 更新のみ、専用リリースイベントなし
- `execSF_HIDInputs` は押下→30ms→次キー…→加速度閾値到達で最終キーを離す、という**内蔵シーケンス**（格闘ゲーム特化）

### 新トリガーモデル（pk4 で拡張）

pk4 に **`trigger_mode` フィールド (uint8_t)** を追加し、4 種類の発火モードをサポート：

| trigger_mode | 名称 | 挙動 |
|--------------|------|------|
| 0 `ONESHOT` | **開始状態のみ（従来互換）** | 開始条件を満たした瞬間、HID を 1 回発火 (press→interval ms→release)。再発火は条件を一度離れて再度満たすまでロック（チャタリング防止） |
| 1 `HOLD_START_END` | **開始状態 + 終了状態（新規）** | 開始条件一致で HID を press のまま保持。**終了条件** 一致または開始条件の離脱で release |
| 2 `HOLD_START_ONLY` | **開始状態のみで自動リリース** | 開始条件一致で press、開始条件から外れた瞬間に release（= 終了条件 = 開始条件の否定、省略形）|
| 3 `SEQUENCE` | **コマンドシーケンス（従来互換）** | 既存 execSF_HIDInputs 互換。`inputs_msg` の ASCII をシーケンス実行、最終キーは加速度閾値到達時のみ発火 |

#### 開始条件・終了条件の定義
「状態」を構成する要素は、ユーザーが意図した通り **姿勢 + ボタン + 加速度/ジャイロ** の組み合わせ：

| 要素 | 開始条件の指定 | 終了条件の指定 |
|------|----------------|----------------|
| 姿勢 (Euler) | `rpy[3]` + `rpy_tolerance[3]` (角度±幅) | `end_rpy[3]` + `end_rpy_tolerance[3]` |
| 姿勢 (クォータニオン) | `quaternion[4]` + `q_dot_min` (cos角度差の閾値) | `end_quaternion[4]` + `end_q_dot_min` |
| 加速度 | `acc_triger[4]` (X/Y/Z/ABS 閾値、0=無視) | `end_acc_triger[4]` |
| ジャイロ | `gyro_triger[4]` | `end_gyro_triger[4]` |
| ボタン | `button_idx` (uint8_t、0=無指定)、`button_state` (0=離、1=押) | `end_button_idx`, `end_button_state` |

**論理合成**: 各要素は AND 結合（すべての指定要素が満たされたとき発火）。`trigger_mode=2` (HOLD_START_ONLY) では終了条件は省略（開始条件の否定で自動 release）。

#### レコード形式の決定（サイズ制約）
pk3 は 84B、全要素を開始＋終了の両方持たせると ~160B になりメモリ/ファイル肥大。3 つの選択肢：

**選択肢 A: pk4 を 84B 据え置き、単独レコードで one-shot/hold の両方を表現（ペア不要）**
- 終了条件を別途持たず、`trigger_mode=0/2/3` のみサポート（「開始条件の否定」で自動終了）
- 利点: 実装単純、互換性高い
- 欠点: 「右手を上げ始めたら Shift 押下開始、左手を叩いたら Shift 解除」のような**開始と終了が別状態**は表現不可

**選択肢 B: pk4 を 2 種類（pk4_start, pk4_end）のレコードに分け、`pair_id` で紐付け（推奨）**
- 各レコードは 84B 維持。ファイル上は `pair_id` (uint8_t) で開始・終了をペアリング
- `trigger_mode=1 (HOLD_START_END)` のときのみ終了レコードを要求
- 利点: 既存 pk3 と同じファイル構造、サイズ固定のまま
- 欠点: Web UI で「開始・終了ペア」を編集する必要あり（1 トリガー = 2 レコードのエンティティ扱い）

**選択肢 C: pk4 を 168B に拡張（開始＋終了をひとつのレコードに）**
- 利点: 編集しやすい、ロジックもシンプル
- 欠点: LittleFS サイズ 2 倍、pk3 との互換性低下

→ **推奨 B**: Web UI のエンティティモデルで「トリガー (pair)」を1単位として扱い、ストレージは 84B × 2 として既存構造を壊さない。

#### ROM/RAM 容量試算（M5StickC ベース、HW 再設計含む）

**ハードウェア**: M5StickC = ESP32-PICO-D4 + **4MB flash** (一部リビジョン 8MB)、RAM 520KB (BLE 等で実効 ~300KB)

**現在のストレージ内訳**:
- **NVS 経由 EEPROM** (`EEPROM_MAX_SIZE=4094B`): MOTION_CONT 800B + CAL 100B + MESSAGE 3190B
  - MOTION_CONT 800B に pk3 (84B) を格納した場合 = **9 エントリのみ**（既存）
- **LittleFS (SPIFFS パーティション)**: [custom_partition.csv](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/custom_partition.csv) なら 1MB、[default.csv](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/default.csv) なら 1.4MB
  - `savepk3vectol2file` は LittleFS 上にファイルとして保存

**pk4 の格納可能数（M5StickC, 4MB flash）**:

| 選択肢 | 1 トリガー | LittleFS 1MB 理論上限 | RAM 実用上限 | 選定 |
|--------|------------|----------------------|--------------|------|
| A (終了条件なし) | 84B | 12,500 | ~500 | 機能不足 |
| **B (start/end ペア)** | 84B × 2 = 168B | 6,250 | ~250 ペア | ✅ **推奨** |
| C (1レコードに拡張) | 168B | 6,250 | ~250 | 実質 B と同等、互換性低下 |

**重要な観察**: B と C は**物理容量で差がない**。採用基準は「既存 pk3 資産との互換性」と「Web UI の編集性」で判断 → Web UI で「1 トリガー = 1 論理単位」として扱えば B でも編集性は損なわれない。

**実用上の上限**: ~250 ペア / 500 one-shot 程度が安全圏（RAM バッファ、BLE スタック、Three.js 解析レイアウト余裕を加味）。一般ユーザーが手動登録する量（10〜50 個）を大きく超えるので問題なし。

**HW 再設計の余地（今後の製品版向け、MVP スコープ外）**:
- **選択肢 R1**: M5StickC 筐体維持、パーティション再割当て (app を 500KB → LittleFS を 2.5MB) → **pk4 ペア 15,000 相当**。`board_build.partitions = M5C_MPU6886_cpp/custom_partition_large_fs.csv` の新規追加だけで済む。
- **選択肢 R2**: XIAO ESP32-S3 に HW を切替 (**8MB flash + 最大 8MB PSRAM**)。PSRAM 使えば pk4 を数万個 RAM 展開可能。マウス/ゲームパッド/USB HID の強みもある。
- **選択肢 R3**: M5StickC Plus2 (4MB flash + 2MB PSRAM) — 容量余裕、筐体同等、液晶大。

→ **判断**: MVP は R1 相当（現 M5StickC + LittleFS 活用）で**十分**。ユーザーが 50〜100 個登録しても容量枯渇しない。製品版の最終決定は別途。

#### FW 改修時のストレージ方針（案 A: NVS + LittleFS ハイブリッド、推奨）

ユーザー質問「案 A でもよいのか？」→ **はい、案 A が実際の推奨**。

**理由**:
1. **組込み開発の業界標準**
2. **起動時間が速い** (NVS 直読 1-10ms vs LittleFS JSON パース 50-100ms)
3. **電源断時の config 破損リスク低** (NVS atomic write)
4. **5 年前の設計パターン踏襲** — 継続性
5. **頻繁な切替値** (output_mode 等) に wear leveling が効く

**Web UI 一括バックアップの懸念は** `config.export` コマンドで解決：
```json
// {"cmd":"config.export"} に対する応答
{
  "type": "config.export",
  "data": {
    "nvs": { "output_mode":"ble", "ble_name":"...", "accel_bias":[...], ... },
    "profiles": [ {"name":"sf2","data":{...}}, ... ]
  }
}
```
→ 案 A の長所（高速・原子性）と 案 B の長所（一括 import/export）両立。

**抽象化レイヤ**（将来の切替余地を残す）:
```cpp
class IConfigStorage {
public:
    virtual bool loadConfig(Config& out) = 0;
    virtual bool saveConfig(const Config& in) = 0;
    virtual bool loadProfile(const char* name, Profile& out) = 0;
    virtual bool saveProfile(const char* name, const Profile& in) = 0;
    virtual std::vector<std::string> listProfiles() = 0;
};
class HybridStorage : public IConfigStorage { /* 案 A */ };
class LittleFsOnlyStorage : public IConfigStorage { /* 案 B、必要なら */ };
```

---

### 案 A 詳細（確定版）

**NVS 側 (起動時高速アクセス、小さい設定値)**:
```
output_mode           uint8   1B
ble_name              str     <32B
usb_name              str     <32B
accel_bias            [f32]×3 12B
gyro_bias             [f32]×3 12B
accel_scale           [f32]×3 12B
qref_default          [f32]×4 16B  (現在 active な q_ref)
mahony_kp             f32     4B
mahony_ki             f32     4B
active_profile        str     <32B
btn_layout            blob    <512B
calibration_done      bool    1B
total                 <1KB
```

**LittleFS 側 (可変長、複数エントリ)**:
```
/profiles/*.json      pk4 プロファイル (5KB × N 個)
/messages/*.txt       メッセージテンプレート
/calibration.dat      bias バックアップ (旧互換)
```

**Web UI からのアクセス API**:
```
// 個別取得
{"cmd":"config.get", "key":"output_mode"} → {"type":"config","key":"output_mode","value":"ble"}
{"cmd":"config.set", "key":"output_mode", "value":"usb"}

// 一括 export (NVS + LittleFS すべて含む)
{"cmd":"config.export"} → {
  "type":"config.export",
  "data":{
    "nvs":{"output_mode":"ble","ble_name":"...",...},
    "profiles":[{"name":"sf2","data":{...}},...]
  }
}

// 一括 import (バックアップからの復元)
{"cmd":"config.import","data":{...}} → 各 NVS キーに展開 + LittleFS にファイル保存
```

Web UI は `config.export` でバックアップ、`config.import` で復元。NVS の中身も JSON として可視化される（案 B 相当の UX）。

---

### 参考: 案 B (pure LittleFS、却下案)

**既存メモリマップ（5 年前設計、現行 FW）**:
| 領域 | 開始 | サイズ | 用途 |
|------|------|--------|------|
| `EEPROM.MOTION_CONT` (NVS) | 0 | 800B | pk3 ベクトル (84B × ~9 件) |
| `EEPROM.CAL_SPACE` (NVS) | 800 | 100B | 加速度/ジャイロ bias |
| `EEPROM.MESSAGE_SPACE` (NVS) | 900 | 3190B | メッセージ (56B × 56 件) |
| LittleFS `/calibration.dat` | - | 24B | キャリブレーション backup |
| LittleFS (savepk3vectol2file 出力) | - | 可変 | pk3 バイナリファイル群 |

**新設計: LittleFS に統合（シンプル）**:

ユーザー質問「なぜ NVS と LittleFS を分ける？」への回答を受けて、**全設定を LittleFS 上の JSON ファイルに統合**する方針に変更：

```
/littlefs/
  config.json         # 全デバイス設定 (output_mode, BLE名, bias, qref_*, mahony_kp 等)
  active_profile      # 1 行テキスト、現在 active なプロファイル名
  profiles/
    default.json      # 出荷時デフォルト
    sf2.json          # ストリートファイターII（必須同梱）
    cs2-basic.json
    ...
  messages/
    *.txt             # メッセージテンプレート（旧 EEPROM.MESSAGE 後継）
  calibration.dat     # 旧互換バイナリ backup（任意）
```

**`config.json` の構造**:
```json
{
  "schema_version": 1,
  "hardware": {
    "board": "m5stickc",
    "imu_type": "mpu6886"
  },
  "hid": {
    "output_mode": "ble",
    "ble_name": "MotionController",
    "usb_name": "burst-motion"
  },
  "sensor": {
    "calibration": {
      "accel_bias": [-0.003, 0.018, 0.085],
      "gyro_bias":  [3.516, -6.023, -5.140],
      "accel_scale": [1.0, 1.0, 1.0],
      "accel_crosstalk": [[1,0,0],[0,1,0],[0,0,1]],
      "calibrated_at": "2026-04-24T10:00:00Z",
      "level": "simple"
    },
    "mahony": { "kp": 2.0, "ki": 0.0 },
    "qref_presets": {
      "horizontal": [1.0, 0.0, 0.0, 0.0],
      "upright":    [0.444, 0.469, -0.532, -0.5],
      "custom_1":   [0.9, 0.0, 0.436, 0.0]
    },
    "current_qref": "horizontal"
  },
  "buttons": {
    "layout": [
      {"source":"gpio", "pin":0, "pull":"up"},
      {"source":"gpio", "pin":36, "pull":"none"},
      {"source":"gpio", "pin":26, "pull":"up"}
    ]
  }
}
```

**利点**:
- **Web UI で `config.json` をそのままバックアップ/復元可能** — 設定全体を 1 ファイルで扱える
- **`profiles/` 配下もそのままエクスポート** — 移行作業が単純
- **デバッグ可視性**: Web UI の File Browser タブで中身を直接見られる
- **コード単純化**: Storage 抽象化レイヤは LittleFS のみを扱えばよい
- **電源断耐性**: LittleFS の journaling で十分（案 B でのリスクは実用上無視できる）

**ファイル表現の欠点**:
- 起動時に config.json パースで +50-100ms 遅延（ゲーム開始まで秒オーダーなので実害なし）
- NVS の atomic write ほどの原子性はない（ただし LittleFS の bit-level rollback で復旧可能）

**NVS の最小限使用**:
- `first_boot_flag` (bool): 初回起動検出用の 1 byte
- `littlefs_mount_fallback_config` (blob): LittleFS マウント失敗時の緊急フォールバック設定（BLE 名くらい）
- **実質ほぼ使わない**。Preferences 呼出しが数箇所に限定される

**旧 EEPROM 3 ゾーンの段階的退役**:
| 旧ゾーン | 新設計 |
|----------|--------|
| `MOTION_CONT` (pk3 800B) | 旧 FW 互換のためリードオンリーで残す。新書込みは LittleFS `/profiles/*.json` へ |
| `CAL` (bias 100B) | NVS キーに移行。両方存在すれば NVS 優先 |
| `MESSAGE` (3190B) | LittleFS `/messages/*.txt` に移行 |

**マイグレーションツール**:
```
{"cmd":"migrate.legacy"}
→ 旧 EEPROM 3 ゾーンと LittleFS 上の旧 pk3 ファイルを読み出し、新保存先に変換
→ 結果: {"type":"migration.done","profiles_moved":3,"messages_moved":12,"bias_moved":true}
```

**永続化レイヤ**:
- `src/core/Storage.hpp/cpp` — Preferences (NVS) + LittleFS を抽象化
- Arduino `Preferences.h` 採用（型安全、`EEPROM.h` より推奨）

**容量試算（M5StickC 4MB flash）**:
- NVS パーティション: 20KB デフォルト、KV ペア数百で十分
- LittleFS: 1MB 以上（必要なら custom_partition で拡大）
  - pk4 プロファイル 1 個 ~5KB → **200 プロファイル保存可**
  - 1 プロファイル内 trigger 数: 実用 5〜50 個
- App 領域 500KB で BLE stack + ArduinoJson + TriggerEngine 含め余裕

**設定実装方針**:
1. **pk4 ベクトルは `std::vector<Trigger>` でランタイム可変**。起動時に active_profile を LittleFS から読込
2. NVS `active_profile` でどのプロファイルをロードするか記録、切替は再起動不要
3. `MAX_TRIGGERS_PER_PROFILE = 256` を `#define`、Web UI が超過を検知して警告

#### 同時マッチ時の優先度ルール
ユーザー要件: **「ボタン押下は最優先。次にボタン未押下時は傾きや姿勢を優先」**
```
read current sensor state (button bitmap + posture + accel + gyro)
button_matched = []
posture_only = []
for each Trigger E in trigger_vector:
    if E.start.button_enabled and button_bitmap has E.start.button_idx:
        if evaluate_start(E, state) is true:
            button_matched.append(E)
    elif not E.start.button_enabled:
        if evaluate_start(E, state) is true:
            posture_only.append(E)

if button_matched is not empty:
    # ボタン押下マッチを全部発火（複数キー同時入力に対応）
    # 同じボタンで複数候補がある場合は priority 降順 → 最初の 1 つ
    active = dedupe_by_button_idx(button_matched, by="priority")
else:
    # ボタン未押下マッチの中から姿勢距離最近傍 1 つを選択（既存 getClosestPK3 思想の継承）
    active = [closest_by_posture(posture_only, state)]

for E in active:
    apply_trigger_mode(E, state)  # ONESHOT / HOLD_* / SEQUENCE
```

**デッドロック回避**: HOLD_START_END で press 中のトリガーは優先度計算から除外して常に維持評価（end 条件のみ判定）。

#### 発火ロジック（TriggerEngine）
```
for each IMU tick (100Hz):
  for each trigger entry E in pk4_vector:
    matched = evaluate_start_condition(E, current_state)
    switch E.trigger_mode:
      case ONESHOT:
        if matched and not E.armed_cooldown:
          fire_press_release(E)
          E.armed_cooldown = true
        else if not matched and E.armed_cooldown:
          E.armed_cooldown = false  # re-arm when condition left
      case HOLD_START_ONLY:
        if matched and not E.holding:
          fire_press(E); E.holding = true
        else if not matched and E.holding:
          fire_release(E); E.holding = false
      case HOLD_START_END:
        end_matched = evaluate_end_condition(E.pair, current_state)
        if matched and not E.holding:
          fire_press(E); E.holding = true
        else if end_matched and E.holding:
          fire_release(E); E.holding = false
      case SEQUENCE:
        if matched and not E.sequencing:
          start_sequence_task(E); E.sequencing = true
```

各 pk4 エントリは engine 内部に `armed_cooldown` / `holding` / `sequencing` の状態を持つ（RAM 上、NVS 保存不要）。

#### Web UI でのトリガー編集
- トリガー編集画面は 3 セクション：**開始条件** / **終了条件** / **出力**
- 「トリガーモード」ドロップダウンで ONESHOT/HOLD_START_END/HOLD_START_ONLY/SEQUENCE を選ぶと、**終了条件セクションは HOLD_START_END の時だけ有効化**される
- 「現在の姿勢を開始条件に使う」/「現在の姿勢を終了条件に使う」ボタンを分けて配置
- 既存 pk3 の one-shot はすべて `trigger_mode=0 (ONESHOT)` としてロード、UI では「開始条件のみ」で表示

### pk4 出力データモデル

現 pk3 (84B) は入力側 (姿勢/加速度/ボタン/ジャイロ) + 出力 (ASCII 20B) を持つが、マウス XY/ホイール、ゲームパッドボタン/スティック、修飾キー、Consumer (音量等) を扱えない。

**拡張方針**: pk3 を残し、新規 `pk4` 構造体を追加。`msg_format` フィールドの値で出力種別を識別し、`inputs_msg[20]` を種別別にパースする：

| msg_format | 出力種別      | inputs_msg[20] の解釈                                                    |
|------------|---------------|--------------------------------------------------------------------------|
| 0          | ASCII Keys    | 現行互換 (null-terminated string)                                        |
| 1          | Key + Mods    | [0]=mods bitmap, [1..6]=HID usage codes, [7]=duration, [8..19]=reserved  |
| 2          | Mouse         | [0]=buttons, [1..2]=dx int16, [3..4]=dy int16, [5]=wheel, [6]=interval   |
| 3          | Gamepad (S3)  | [0..1]=buttons u16, [2..9]=axes int8×8, [10]=hat, [11..19]=reserved      |
| 4          | Consumer      | [0..1]=usage code u16, [2]=hold ms, [3..19]=reserved                     |
| 5          | Macro         | [0]=num steps, [1..19]=steps × (msg_format ref + duration)               |

**理由**: pk3 の外形バイト数 (84B) を極力維持し LittleFS 上の既存ファイルと後方互換を取りつつ、`msg_format` で意味を拡張。Web 側は種別に合わせて UI を切り替える。

**追加トリガー条件**: 現状の `acc_triger[4]` (X,Y,Z,ABS 閾値) + `gyro_triger[4]` + `rpy[3]` (Euler 範囲) + `button_idx` で必要十分。ホールド/クールダウンは `hid_input_interval` に含まれ、追加フィールド不要。

---

## 修正/新規ファイル

### 新規作成 (Web 側)

| パス | 役割 |
|------|------|
| [Web/hidconfig/index.html](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/index.html) | 5タブ SPA (Bootstrap 5 + vanilla JS、CDN依存最小) |
| [Web/hidconfig/app.js](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/app.js) | ルーティング、Serial 接続管理 (既存 `motion_controller.js` の `connectSerial`/`readLoop`/`processBuffer` を流用) |
| [Web/hidconfig/trigger_editor.js](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/trigger_editor.js) | 姿勢キャプチャ UI (「現在の姿勢を使う」ボタン)、加速度閾値スライダ、キー選択 (修飾キーチェック+キー名ドロップダウン)、マウス/ゲームパッド切替 |
| [Web/hidconfig/live_view.js](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/live_view.js) | Three.js 3D表示 (既存 `motion_controller.js` から抽出・モジュール化) |
| [Web/hidconfig/pk4_codec.js](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/pk4_codec.js) | pk4 バイナリ encode/decode (DataView little-endian、84B per entry) |
| [Web/hidconfig/profiles/](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/profiles/) | サンプル JSON プロファイル (FPS、ストリートファイター、プレゼンテーション、音量リモコン、アクセシビリティ) |
| [Web/hidconfig/manifest_s3.json](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/manifest_s3.json) | esp-web-tools 用 (既存 `manifest_m5stickc.json` を踏襲) |

### FW 側

| パス | 変更 |
|------|------|
| [platformio.ini](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/platformio.ini) | `[env:esp32s3]` に `-D USB_HID_ENABLE`、`board_build.arduino.usb_mode = 1` (Hardware CDC+HID)、`board_upload.use_1200bps_touch = yes` 追加 |
| [src/MotionController.hpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/MotionController.hpp) | `pk4` 構造体追加 (L120付近 pk3 の直後)、`HidSink` 抽象レイヤ (press/release/move/gamepadReport を仮想メソッドで包む) |
| [src/HidSink.hpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/HidSink.hpp) *新規* | `class HidSink { virtual void sendKey(...); virtual void sendMouse(...); virtual void sendGamepad(...); };` と `BleHidSink`, `UsbHidSink` 実装 |
| [src/TriggerEngine.hpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/TriggerEngine.hpp) *新規* | pk4 走査・評価・ディスパッチ。ImuLoop から毎フレーム呼出 |
| [src/IMU_BLEorSerial_tester.cpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/IMU_BLEorSerial_tester.cpp) | Serial コマンド拡張: `SET_OUTPUT,{ble\|usb\|both\|none}`, `addpk4,...`, `savepk4vectol2file,...`, `loadpk4vector,...`, `WATCH,{on\|off}` (live trigger debug echo)。`hidSessionLoop` を削除し TriggerEngine に置換 |
| [src/UsbHidSink.cpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/UsbHidSink.cpp) *新規、S3 のみ* | `#ifdef ARDUINO_USB_MODE` で囲み、`USBHIDKeyboard`, `USBHIDMouse`, `USBHIDGamepad` を実装 |
| [lib/ESP32-BLE-Combo-master_peter/](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/lib/ESP32-BLE-Combo-master_peter/) | 変更なし (既存 API をそのまま使う) |

### 再利用する既存の関数・仕組み

- `connectSerial()`, `readLoop()`, `processBuffer()` — [Web/motion_controller.js:69-173](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/motion_controller.js#L69)
- `sendCommand()` の textbox 経由パターン — [Web/motion_controller.js](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/motion_controller.js) (可視化できる)
- 3D クォータニオン可視化 (Three.js r128) — 既存 `page1` ロジック流用
- `savepk3vectol2file` のバイナリ転送プロトコル (ヘッダ CSV → 200ms 待ち → バイナリ本体) — そのまま pk4 へ流用
- `flushPKvector()` / EEPROM レイアウト (0-800: MOTION_CONT, 800-900: CAL, 900-4094: MESSAGE) — 維持
- IMUReader の yaw/roll アンラップ — そのまま TriggerEngine で使用
- NimBLE + BleCombo — 既存 press/release/move/releaseAll API をそのまま `BleHidSink` でラップ
- esp-web-tools + 既存 `manifest_m5stickc.json` のパターン — S3 用に複製

---

## 実装ロードマップ

### Phase 1 — 新 FW コア層の構築（既存 FW と並行、branch で分離）
1. [platformio.ini](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/platformio.ini) に `[env:m5stick-c-v2]`, `[env:xiao-s3-v2]` を追加（既存 env は壊さない）、`lib_deps` に `bblanchon/ArduinoJson@^7` を追加
2. `src/core/types.hpp` — Trigger / Condition / Output 構造体定義
3. `src/core/Profile.hpp/cpp` — LittleFS `/profiles/*.json` の load/save (ArduinoJson v7)
4. `src/core/TriggerEngine.hpp/cpp` — evaluate / dispatch ロジック (ボード非依存)、PC 上の Catch2 テストで検証
5. `src/core/HidDispatcher.hpp/cpp` — trigger_mode に従って press/release/sequence を出力
6. `src/hid/IHidSink.hpp` — 抽象インタフェース (`sendKey`, `sendMouse`, `sendGamepad`, `sendConsumer`)

### Phase 2 — HAL + 姿勢推定 + BLE HID（M5StickC 先行、S3 並行）
7. `src/core/MahonyFilter.hpp/cpp` — 統一 Mahony フィルタ実装、PC 上 Catch2 で精度検証
8. `src/hal/axis_remaps.hpp` — 機種別軸リマップ行列定義（M5StickC/Plus/Plus2/AtomS3/MotionBurst）
9. `src/hal/imu_m5stickc.cpp` — M5.Imu 生データ取得、axis_remap 適用、Mahony に投入、100Hz
10. `src/hal/imu_m5atom_s3.cpp` — M5Atom S3 IMU 同様
11. `src/core/QuatOffset.hpp/cpp` — NVS に q_ref プリセット保存、calibrate.qref.* コマンド受付
12. `src/hal/buttons_gpio.cpp` — GPIO 直結ボタン
13. `src/hid/BleHidSink.cpp` — **ESP32-BLE-CompositeHID (Mystfit)** 採用、Keyboard/Mouse/Gamepad/Consumer 対応
14. `src/transport/SerialJsonLine.cpp` — JSON Lines 入出力 (ArduinoJson + Serial)
15. `src/main.cpp` — setup + super-loop、IMU Task 生成
16. **動作確認**: M5StickC でビルド → Web で軸リマップ確認 (3D 表示が物理動きと一致) → qref 登録 → トリガー登録 → BLE HID で 'a' 送信成功 → M5Atom S3 に移植して同じプロファイルが動く確認

### Phase 3 — Web App 新規構築
13. [Web/hidconfig/index.html](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/index.html) + Preact + htm + Tailwind Play CDN スケルトン
14. `src/lib/SerialClient.js` — Web Serial 接続 + JSON Lines 読み書き + コマンド応答キュー
15. `src/lib/TriggerModel.js` — Trigger JSON SerDe + Vitest 単体テスト
16. `src/lib/IMUViewer.js` — Three.js クォータニオン可視化 (既存コード抽出リファクタ)
17. **Connect ページ** — 接続/切断、デバイス情報 (FW version, board, profile 表示)
18. **LiveView ページ** — IMU 3D + 加速度グラフ（Chart.js or 自前 canvas）
19. **TriggerEditor ページ** — 3 セクション UI（開始条件/終了条件/出力）、「今の姿勢を開始/終了に使う」ボタン、トリガーモード切替
20. **Library ページ** — プロファイル一覧、JSON エクスポート/インポート、LittleFS と同期
21. **Output ページ** — BLE/USB/Both/None 切替（S3 以外は USB グレーアウト）
22. **Flash ページ** — esp-web-tools 10.x、ボード別 manifest

### Phase 4 — ESP32-S3 USB HID
23. `[env:xiao-s3-v2]` に `board_build.arduino.usb_mode = 1`, `board_upload.use_1200bps_touch = yes` 追加
24. `src/hid/UsbHidSink.cpp` — Arduino USBHIDKeyboard/Mouse/Gamepad を使った実装
25. `src/hal/imu_xiao_s3.cpp` — MPU6050 DMP ラッパ
26. CDC + HID コンポジット動作確認 (USB Serial 設定中でも USB HID 出力可能)
27. BLE HID Gamepad 拡張 (BleCombo に gamepad descriptor 追加、または ESP32-BLE-Gamepad 並行サーバー)

### Phase 5 — 高度機能 + ドキュメント
28. **HW ボタン拡張**: MCP23017 サポート `src/hal/buttons_i2c_expander.cpp`、Web UI の HW 設定タブ
29. **サンプルプロファイル 5 種** (FPS/格闘ゲーム/プレゼン/メディアリモコン/アクセシビリティ)
30. **アクセシビリティ機能**: Web UI 簡易モード、1-motion = 1-key のシンプルなテンプレート
31. **PWA 化**: Service Worker、オフライン動作、ホーム画面追加
32. **多言語**: 日本語/英語切替
33. **README + 動画チュートリアル**: 起動→接続→登録→使用の全体フロー

### Phase 6 — クラウド共有プロファイル（**バックエンドなし**、MVP 流儀）
34. GitHub リポジトリ `motion-controller-profiles` を新設、`profiles/<game_id>/<author>-<name>.json` 構成で curated プロファイルを管理
35. Web UI の Library ページに「Community Library」タブ追加 — jsDelivr CDN 経由で GitHub リポジトリの `index.json` を fetch（**CORS フリー、バックエンド不要**）、ゲーム別一覧・タグ検索・プレビュー・インポート
36. 「このプロファイルを共有」機能 — Web UI から GitHub PR ページを pre-fill URL で開き、ユーザーが PR 作成 → メンテナがレビュー merge → `index.json` が Actions で自動再生成 → CDN に反映
37. **ここまでで Phase 1-6 全体がバックエンドゼロで完結**（サーバー代も運用工数もゼロ）

### Phase 8 — Adapter FW（将来拡張、まとめ実施）

**目的**: PS4 / Nintendo Switch / 一部 PC 等の BLE HID を受け付けない機器に対し、Controller → RF → Adapter → USB HID の経路で入力する。

**今回の扱い**:
- Phase 1-5 の MVP では Adapter 実装**なし**
- ただし Controller FW 側には布石として以下を用意：
  - プロファイルに `hardware.adapter` と `output_paths: ["rf_to_adapter"]` フィールド（schema_version 2 で対応済み）
  - `HidSink` インタフェースの実装として `RfHidSink` を Stub レベルで用意（`output.set,rf` 指令の受け皿）
  - ESP-NOW 初期化コードを `#ifdef ENABLE_RF_ADAPTER` で囲み、Phase 8 で有効化

**アーキテクチャ**:
```
[Controller: M5StickC / M5Atom S3 / Motion Burst]
   TriggerEngine で HID イベント生成
       ↓ ESP-NOW (ESP32同士) or nRF ESB (nRF24L01 外付け)
       ↓ 14-byte パケット / 1-2ms 遅延
[Adapter: M5Atom S3 Adapter / RP2040 W / Motion Burst Adapter]
   RF 受信 → パケット変換 → USB HID レポート送信
       ↓ USB HID (TinyUSB composite: Keyboard+Mouse+Gamepad)
[Target: PS4 / Switch / PC / Xbox]
```

**設計判断**:
- **評価は Controller 側で実施** — TriggerEngine は Controller FW 内、Adapter は単純な RF→HID フォワーダ
  - 理由: RF 帯域節約、Adapter 実装簡略化、プロファイルは 1 箇所で管理
- **RF パケットフォーマット**（14 B、ESP-NOW と nRF ESB 共通）:
  ```cpp
  struct AdapterPacket {
    uint8_t magic;      // 0xA5
    uint8_t type;       // 1=keyboard, 2=mouse, 3=gamepad, 4=consumer, 5=ping
    uint8_t seq;        // 再送検出用シーケンス番号
    union {
      struct { uint8_t mods; uint8_t keys[6]; }              kb;   // 7 B
      struct { int16_t dx, dy; uint8_t btn; int8_t wheel; }  ms;   // 6 B
      struct { uint16_t btn; int8_t axes[6]; uint8_t hat; }  gp;   // 9 B
      struct { uint16_t usage; }                             cc;   // 2 B
    } data;
    uint16_t crc;       // パケット破損検知
  };
  ```
- **遅延目標**: Controller IMU → USB HID 出力まで合計 ≤ 15ms（実用ゲーム可能水準）
- **ペアリング**: Controller と Adapter は工場でペア済みアドレス（Web UI でペア設定変更可）

**Phase 8 作業項目**:
41. 新リポジトリ `motion-controller-adapter-fw` を作成（Controller FW とは分離、ただし共通プロトコル定義は git submodule か共有ヘッダ）
42. `src/rf/EspNowLink.hpp/cpp` — ESP-NOW 送受信、Controller/Adapter 両方で使用
43. `src/rf/NrfEsbLink.hpp/cpp` — nRF24L01 モジュール経由（RP2040 W Adapter 向け）
44. `src/rf/AdapterProtocol.hpp` — 共通パケット定義、CRC、シーケンス管理
45. **Controller FW 側追加**: `HidSink` の実装として `RfHidSink` を新規追加、`output.set,rf` 指令で切替
46. **Adapter FW 新規**:
    - M5Atom S3 Adapter: Arduino-ESP32 + TinyUSB HID + ESP-NOW
    - RP2040 W: Arduino-Pico (earlephilhower) + TinyUSB HID + nRF24L01 SPI ライブラリ (`RF24`)
    - 両者で共通の Adapter 抽象層、MCU 依存部のみ差し替え
47. **Web UI 拡張**:
    - Output ページに「RF to Adapter」を追加（Adapter 有り判定で有効化）
    - 新ページ「Pairing」 — Controller と Adapter の ESP-NOW MAC ペアリング UI（QR コード表示/スキャン、あるいは手動入力）
48. **Adapter 専用 Web UI**: Adapter にも Web Serial 接続できるようにし、以下を設定可能に
    - ペアリング対象の Controller MAC
    - USB HID デバイス名/VID/PID
    - RF 受信通知（動作確認用ログ）
49. **Motion Burst Adapter** は既存 RF を尊重 — 既存プロトコルが nRF ESB なら互換、ESP-NOW なら新プロトコルに統合。現物検証後に方針決定
50. **遅延測定**: Controller IMU サンプリング → USB HID 到達までの end-to-end 測定、15ms 目標を検証

**Adapter 単体での Web 設定**:
- Adapter を USB で PC に接続 → Web Serial で `motion.findradio.jp/adapter` ページから設定
- Controller とは独立した設定 UI（ペアリング対象、HID 名、出力モード）
- 同じ `hardware.adapter` 語彙でプロファイル互換性を判定

### Phase 9 — バックエンドが「必要になったら」追加（**スケール対応、任意**）
条件: 利用者が数百〜千人規模、PR 審査が回らなくなった、ユーザー直接投稿/評価/コメント機能が欲しい

**推奨スタック: Cloudflare Workers + R2 + D1**
38. Workers で認証 API (GitHub OAuth)、投稿受付 API、モデレーション Queue
39. R2 でプロファイル JSON 保存 (10GB 無料)、D1 でメタデータ/評価/DL カウンタ (500万行無料)
40. 無料枠を超えても月 $5〜のスケール

**代替: Supabase**
- Postgres 慣れ・RLS 活用したい場合。Free tier の active users 上限には注意。

**非推奨: 自前 VPS**
- 保守工数（セキュリティ更新、バックアップ、監視）が本業 (デバイス開発) を奪う。利用者 1 万超＆有料化着手まで不要。

### Phase 7 — 旧 FW との互換橋渡し（任意）
38. 旧 pk3 の JSON エクスポータ: 既存 LittleFS 上の pk3 バイナリファイルを Web UI から取り込み→新 JSON プロファイルに変換
39. 旧 FW からのマイグレーションドキュメント

---

## 公開 Web アプリ（Motion Controller 専用ページ）

### ホスティング戦略

**MVP 構成（推奨）**:
```
https://motion.findradio.jp/                           # メイン Web アプリ (Cloudflare Pages)
https://github.com/uecken/motion-controller-profiles/  # curated プロファイル (public repo)
https://cdn.jsdelivr.net/gh/uecken/motion-controller-profiles@main/...  # 配信 CDN
```

| 項目 | 採用 | 理由 |
|------|------|------|
| Web アプリホスト | **Cloudflare Pages** (GitHub Pages でも可) | 帯域幅無制限・preview URL・build log 優秀、無料、自動 HTTPS。GitHub Pages は 100GB/月制限あり、CF Pages はそれを外せる保険 |
| FW バイナリ配信 | 同じ Pages 上の `/firmware/` | CORS 問題なし、esp-web-tools v10 の manifest が相対パスで参照可 |
| curated プロファイル | **GitHub public repo + jsDelivr CDN** | push 即配信、バージョン管理付き、PR ベースで品質維持、無料、`raw.githubusercontent.com` と違い `Access-Control-Allow-Origin: *` ヘッダ付きなので **CORS フリー**で fetch() 可能 |
| ユーザー投稿 | **Phase 6**: GitHub PR ベース / **Phase 7**: Cloudflare Workers + R2 + D1 | 初期は PR、スケール時に CF スタック（Workers 10万req/日、R2 10GB、D1 5M行 すべて無料） |
| ドメイン | `motion.findradio.jp` を CNAME で CF Pages に | 既存ドメイン資産活用、信頼感、リンク永続性 |

**jsDelivr による CORS フリー配信とは**:
ブラウザの `fetch()` はクロスオリジンに対し `Access-Control-Allow-Origin` ヘッダを要求する。GitHub raw URL (`raw.githubusercontent.com/...`) は**このヘッダを付けない**ためブラウザから直接 fetch できない。jsDelivr (`cdn.jsdelivr.net/gh/user/repo@main/...`) は**`Access-Control-Allow-Origin: *` 付き**で再配信するので、どんなオリジンからも fetch 可能。これが「CORS フリー配信」の意味。

**Cloudflare vs GitHub Pages 詳細比較**:

| 指標 | GitHub Pages | Cloudflare Pages |
|------|--------------|------------------|
| 月間帯域幅 | 100GB | 無制限 |
| ビルド回数 | Actions の 2000 min/月 | 500 回/月 |
| preview URL (PR 毎) | 非対応（要 Actions 自作） | ✅ 標準 |
| グローバル CDN | GitHub の CDN | Cloudflare の 300+ 拠点 |
| アクセスログ/解析 | 基本なし | Web Analytics 付属 |
| エッジ関数 | 不可 | Workers でアップグレード可能 |
| DNS 連携 | CNAME のみ | ネームサーバ移譲でフル機能 |

**設計方針**: どちらでも動くが、**Cloudflare Pages を優先**。帯域制限リスクがなく、Phase 7 で Workers + R2 に拡張する際も CF スタック内で完結。既存 findradio.jp ドメインがあるので CNAME で subdomain を向けるだけ。

**GitHub Actions で自動デプロイ** (`.github/workflows/pages.yml`):
- `Web/hidconfig/` 配下を変更→自動で Pages にデプロイ
- FW ビルド成果物 (`pio run -e m5stick-c-v2`) を `Web/hidconfig/firmware/m5stickc/` にコピー→Pages に配置
- ESP-web-tools の manifest も自動生成（`manifest_m5stickc.json`, `manifest_xiao_s3.json`）

### USB Serial による FW 書込み（既要件）

esp-web-tools 10.x を Flash ページに統合済み：
- ボード選択ラジオ → `manifest_<board>.json` 切替
- ESP32-S3 は USB CDC モード切替のため BOOT ボタン長押し手順を UI でガイド表示
- FW 書込み完了後、自動で Web Serial 接続を促す

### 設定ワークフローの運用パターン

**パターン A: USB Serial 経由（MVP、常時利用可）**
```
Controller <BLE HID> ──> Target PC/Phone (常時ペアリング、ゲーム中)
Controller <USB CDC> ──> 設定用 PC (Web アプリ接続)
```
- USB Serial と BLE HID は別トランスポート、完全独立、**同時稼働 OK**
- 書込み中に誤入力を避けたければ `{"cmd":"output.set","target":"none"}` で一時無効化→完了で復元

**パターン B: BLE Serial (NUS) 経由（Phase 2 以降、ケーブルレス）**
```
Controller <BLE HID + NUS 同一接続> ──> PC/Phone
                                        Web アプリが NUS GATT に書込み
```
- NimBLE が同一接続で複数 GATT サービス提供可能
- HID ペアリング済み PC/スマホの Web アプリから、同じ接続上で NUS に書込める
- 制約: ESP32 は 1 セントラル接続推奨。「HID=PC / NUS 設定=別スマホ」の同時接続は非推奨

**典型運用フロー**:
1. 初回: Controller を USB 接続 → Web で初期設定書込み
2. 日常: USB 外して BLE HID でゲーム
3. 設定変更: USB 再接続 OR 同ホストの Web Bluetooth → NUS → 書込み → HID 継続

### Android / スマホ対応（Chrome 148 Beta で Web Serial 解禁）

**2026 年 4 月 8 日、Chrome 148 Beta for Android に Web Serial API が入り**、スマホからも USB OTG 経由で Controller の設定書込みが可能に。

- `navigator.serial` が Android Chrome Beta で動作
- USB OTG ケーブルで Android に ESP32 を直結 → Web アプリで設定書込み
- 2026 Q2 に対応デバイス拡大予定 (Android Serial API 経由)

**Web アプリ側の対応**:
- `navigator.serial` の有無で機能検出、対応していれば USB Serial 接続 UI 表示
- 非対応環境（iOS Safari、Firefox 等）は「対応ブラウザで開いてください」案内
- Android Beta は機能検出で有効化、β マークを UI に表示

**利用シーン拡大**:
- スマホだけで完結する初期設定（PC 不要）
- ゲーム中のスマホで同時に設定変更可能
- Adapter 設定も Android USB OTG で可能

**実装方針**:
- Web アプリは Chrome/Edge (desktop) + Android Chrome を明示サポート
- モバイル UI (Preact + Tailwind) はタッチ操作前提で設計済みなのでそのまま使える
- Three.js 3D ビュアは WebGL なので Android Chrome で問題なく動く

### クラウドプロファイル共有（コミュニティ機能）

**データモデル**（`profiles/<game_id>/<author>-<name>.json`）:
```json
{
  "schema_version": 2,
  "id": "cs2-basic-right-handed",
  "game": {
    "id": "cs2",
    "title": "Counter-Strike 2",
    "platform": ["PC", "Steam"],
    "icon": "https://.../cs2.png"
  },
  "profile": {
    "name": "Basic FPS (right-handed)",
    "description": "傾けて WASD、振って射撃",
    "author": "uecken",
    "version": "1.0.0",
    "created_at": "2026-04-24",
    "tags": ["fps", "beginner", "right-handed"]
  },
  "hardware": {
    "controllers": ["m5stickc", "m5stickc_plus", "m5stickc_2", "m5atom_s3"],
    "adapter": null,
    "output_paths": ["ble_hid_direct"],
    "button_count_min": 3,
    "required_features": ["ble_hid_keyboard"],
    "target_consoles": ["pc", "android_usb", "ios"]
  },
  "triggers": [
    { "id":1, "name":"Forward", "mode":"hold_start_only", "start":{...}, "output":{"type":"keyboard", "keys":["W"]} },
    ...
  ]
}
```

### hardware メタデータの語彙

| フィールド | 許可値 | 用途 |
|-----------|--------|------|
| `controllers` | `motion_burst` / `m5stickc` / `m5stickc_plus` / `m5stickc_2` / `m5atom_s3` | 対応する Controller 機種（配列、複数可） |
| `adapter` | `null` / `motion_burst_adapter` / `rp2040_w` / `m5atom_s3_adapter` | Adapter 経由時の対象機種（null = 直接接続） |
| `output_paths` | `ble_hid_direct` / `usb_hid_direct` / `rf_to_adapter` | 出力経路（複数可、いずれかで動作すれば OK） |
| `button_count_min` | integer | 最低必要ボタン数（不足時は警告） |
| `required_features` | `ble_hid_keyboard` / `ble_hid_mouse` / `ble_hid_consumer` / `ble_hid_gamepad` / `usb_hid_keyboard` / `usb_hid_mouse` / `usb_hid_gamepad` / `rf_esp_now` / `rf_nrf_esb` | 必須機能タグ |
| `target_consoles` | `pc` / `android_usb` / `android_ble` / `ios_ble` / `switch_usb` / `ps4_usb` / `ps5_usb` / `xbox_usb` | 接続先ゲーム機/OS（検索フィルタ用） |

### Web UI の互換チェック
プロファイルインポート時、接続中の Controller/Adapter 情報 (FW から取得) と突合：
- ✅ Compatible — `required_features` が全て満たされ `controllers` に該当機種がある
- ⚠️ Partial — 一部機能が使えない（例: gamepad 要件だが BLE HID のみのデバイス）
- ❌ Incompatible — 機種違い or 必須機能が欠けている
Community Library のプロファイルカードには「🎮 お使いの M5StickC に対応」等のバッジ表示。

### プロファイル互換例
| プロファイル | controllers | adapter | output_paths | target_consoles |
|--------------|-------------|---------|---------------|-----------------|
| Street Fighter II (PC) | 全 Controller | null | ble_hid_direct | pc, android_usb |
| Smash Bros. (Switch) | m5atom_s3 (direct) or 全 Controller (via adapter) | rp2040_w または m5atom_s3_adapter | usb_hid_direct, rf_to_adapter | switch_usb |
| PS4 FPS (via Adapter) | m5stickc 系 | motion_burst_adapter or rp2040_w | rf_to_adapter | ps4_usb |
| Media Remote | 全 Controller | null | ble_hid_direct | pc, android_ble, ios_ble |

**リポジトリ構成** (`motion-controller-profiles`):
```
/
├── README.md                          # 投稿ガイドライン
├── index.json                         # 自動生成インデックス (すべての game と profile の一覧)
├── games/
│   ├── cs2.json                       # ゲームメタデータ (タイトル、アイコン、公式サイト)
│   ├── smash-bros.json
│   └── ...
├── profiles/
│   ├── cs2/
│   │   ├── uecken-basic.json
│   │   └── community-pro.json
│   ├── smash-bros/
│   │   └── uecken-default.json
│   └── accessibility/
│       └── single-motion-enter.json
└── .github/
    └── workflows/
        └── build-index.yml            # profile 追加時に index.json を再生成
```

**Web UI のインポートフロー**:
1. Library ページ →「Community」タブ
2. ゲーム名/タグ/ハードウェアで検索
3. プロファイルカードクリック → プレビュー (トリガー一覧、作者、説明)
4. 「My Library に追加」→ `Web/hidconfig` の localStorage に JSON コピー
5. 「デバイスに書込み」→ Serial で `{"cmd":"profile.save","name":"cs2-basic","data":{...}}` 送信
6. Active profile 設定 → 再起動不要で即有効化

**投稿フロー（Phase 6 MVP、PR ベース）**:
1. Library で自作プロファイルを選択
2. 「Share this profile」ボタン → モーダルでゲーム選択/タグ/説明入力
3. 「GitHub で PR 作成」ボタン → `github.com/uecken/motion-controller-profiles/new/main/profiles/<game>/` に pre-filled JSON で新規ファイル作成 URL を開く
4. ユーザーが PR 作成、メンテナがレビューして merge
5. GitHub Actions で `index.json` 自動再生成、CDN キャッシュ破棄

**投稿フロー（Phase 7 オプション、クラウド版）**:
1. ユーザー登録・ログイン（GitHub OAuth via Supabase or CF Workers）
2. Web UI 内で直接アップロード（レビュー待ちキューに格納）
3. コミュニティ評価・コメント機能
4. 人気ランキング、新着、公式推奨の表示

**プライバシー・審査**:
- プロファイル内の `author` は GitHub アカウント名または自由入力（匿名可）
- 不適切なプロファイル（例: マクロツール禁止ゲームに不正コマンドを送るもの）は GitHub Issue 報告 → メンテナ削除
- ゲームタイトルは既知リストから選択（`games/*.json` に定義）、新規ゲームも PR で追加可

### デフォルトプロファイル（MVP 同梱）

リポジトリに最初から含めるプロファイル 5〜10 種：
| ゲーム/用途 | 内容 | 必須 |
|-------------|------|------|
| 🥊 **Street Fighter II** | 波動拳 (↓↘→+P) / 昇竜拳 (→↓↘+P) / 竜巻旋風脚 (↓↙←+K) を傾け+加速度で再現。既存 `execSF_HIDInputs` のノウハウ移植 | ✅ **必須** |
| 🎮 Generic FPS (Counter-Strike 2 等) | 傾け WASD + 振り射撃 | ✅ |
| 🎤 Presentation | 右スワイプ=次ページ / 左=前ページ / 振る=ポインタ | ✅ |
| 🎵 Media Remote | 傾け音量、タップ再生/停止 | ✅ |
| ♿ Accessibility 1-key | 単純な振り動作 1 つで Enter | ✅ |
| ♿ Accessibility tilt-scroll | 縦傾きでスクロール | ✅ |
| 🏎 Racing | 左右傾け=ハンドル、前後=アクセル/ブレーキ | |
| 🥷 Smash Bros. | ニンテンドーSwitch プロコンエミュ (S3 USB HID Gamepad 利用) | |
| 📐 Air Mouse | 姿勢連動カーソル + ボタンクリック | |
| 👨‍💻 Vim Editor | HJKL ナビ、モード切替 | |

**Street Fighter II プロファイル詳細** (trigger 定義の具体例):
- 波動拳: button=0 押下中、3 秒以内に「↓」→「↘」→「→」の姿勢遷移 → `P` キー (Punch) 最終キーは加速度閾値
- HOLD モードと SEQUENCE モードの組合せで実装
- 既存 `execSF_HIDInputs` のシーケンスロジックを TriggerEngine に汎用化して取り込む

---

## プロトコル仕様（JSON Lines over USB Serial）

### コマンド（Web → FW）
| cmd | 引数 | 目的 |
|-----|------|------|
| `ping` | - | 疎通確認 (`{"type":"pong","fw":"1.0.0","board":"m5stickc"}` 応答) |
| `profile.load` | `name` | LittleFS `/profiles/<name>.json` を読込 |
| `profile.save` | `name`, `data` | LittleFS に保存 |
| `profile.list` | - | LittleFS の一覧取得 |
| `profile.delete` | `name` | 削除 |
| `profile.active` | `name` | 起動時ロード対象を NVS に記録 |
| `trigger.add` | `t` (Trigger obj) | ランタイムに追加 |
| `trigger.remove` | `id` | 削除 |
| `trigger.list` | - | 現ランタイムの全トリガー取得 |
| `trigger.clear` | - | 全削除 |
| `output.set` | `target` (ble/usb/both/none) | HID 出力先切替、NVS 保存 |
| `watch.set` | `enabled` (bool) | トリガーヒット通知の ON/OFF |
| `sensor.stream` | `rate_hz` (0=停止) | センサーデータ配信レート設定 |
| `q.init` | `mode` (upright/horizontal/current) | クォータニオン offset 設定 |
| `calibrate` | - | IMU キャリブレーション実行（1 秒静止要求） |
| `hw.buttons.set` | `layout` | ボタン構成変更 |
| `hw.basicinfo.set` | `name`, `vendor_id`, ... | HID デバイス名等の基本情報 |
| `reboot` | - | 再起動 |
| `factory_reset` | - | NVS/LittleFS クリア |

### イベント（FW → Web）
| type | フィールド | 意味 |
|------|-----------|------|
| `sensor` | `t, ax, ay, az, gx, gy, gz, pitch, roll, yaw, qw, qx, qy, qz` | IMU データ（sensor.stream で ON） |
| `trigger.hit` | `t, id, phase` (start/end) | トリガー発火（watch.set で ON） |
| `trigger.release` | `t, id, phase` | リリース通知 |
| `status` | `uptime, ble, usb, output, profile, fw` | 周期的ステータス（10 秒毎） |
| `ack` | `cmd, ok` | コマンド成功 |
| `err` | `cmd, err, detail` | コマンドエラー |
| `log` | `level, msg` | デバッグログ |

### HID 出力先切替
`{"cmd":"output.set","target":"ble"}` — NVS キー `hid_output_mode` に保存、起動時復元：
- `ble`: BLE HID のみ
- `usb`: USB HID のみ（S3 でのみ許可、M5StickC は err 応答）
- `both`: 両方同時（二重入力に注意、デバッグ用）
- `none`: 出力無効（トリガー評価とイベント通知のみ）

---

## Serial ↔ HID 併存の仕組み（重要）

「切替」は電源再起動を伴わず、`SET_OUTPUT` による**実行時の出力先選択**で実現：

- **M5StickC**: USB Serial と BLE HID は物理的に別トランスポート。常時両方 ON で併存。`SET_OUTPUT` は BLE HID を抑止するだけ。
- **ESP32-S3 (Phase 4 以降)**: TinyUSB コンポジットで USB CDC (Serial) + USB HID (Keyboard/Mouse/Gamepad) が同時エンドポイントとして存在。Web 側から Serial で書込み中でも HID 出力は可能。ただし「設定中に誤入力したくない」場合は書込み開始時に自動で `SET_OUTPUT,NONE` → 完了時に元に戻すロジックを Web 側に実装。
- **BLE HID の接続状態は Serial の動作に影響しない** — BleCombo.isConnected() を見て送信可否だけ決める。

---

## 検証計画

### ビルド確認
```bash
cd /c/Users/thefu/Documents/M5C_Serial_Unity/M5C_MPU6886_cpp
pio run -e m5stick-c       # M5StickC
pio run -e esp32s3         # ESP32-S3 (Phase 4 以降で USB HID 有効)
```

### ユニット動作確認（FW 単体）
- Serial に `{"cmd":"ping"}` を送り `{"type":"pong",...}` 応答
- `{"cmd":"trigger.list"}` でランタイムトリガー取得
- `{"cmd":"output.set","target":"none"}` → トリガー発火時に `trigger.hit` イベントのみ流れ、実 HID 出力なし
- `{"cmd":"output.set","target":"ble"}` で Windows Bluetooth キーボードとしてペアリング → メモ帳に入力される

### E2E 動作確認（Web ↔ FW）
1. Chrome で [Web/hidconfig/index.html](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/index.html) を開く（ローカルは `python -m http.server`、公開は GitHub Pages）
2. M5StickC を USB 接続 → Connect ページで Web Serial 接続
3. LiveView ページで 3D 姿勢追従、加速度グラフ表示確認
4. TriggerEditor で「右に傾ける (roll=45±10°) → 'D' キー」を登録、トリガーモード HOLD_START_ONLY（傾けている間だけ D 押下）
5. Output で BLE 選択、BLE ペアリング後にメモ帳で動作確認：右傾斜中 'ddddd'、戻すと停止
6. トリガーモード HOLD_START_END で「右傾→Shift 押下、左傾→Shift 解除」のペア登録テスト
7. Output で NONE に切替 → 傾けても入力されないこと（ただし watch 有効時は trigger.hit イベント発生）
8. S3 では output=usb 選択 → USB HID として認識、キー入力検出
9. Library でプロファイル JSON エクスポート → ブラウザダウンロード → 別デバイスにインポート → 同じトリガー動作再現

### コア層の単体テスト（PC 上 Catch2）
```bash
cd M5C_MPU6886_cpp
pio test -e native_test  # core/ 配下のみビルドしてホスト上で実行
```
- TriggerEngine: 姿勢閾値判定、クールダウン、HOLD_START_END のペア動作
- Profile: JSON SerDe の往復
- Condition: Euler とクォータニオン両方式の判定精度

### アクセシビリティシナリオ確認
- 「大きく振る 1 動作で Enter キー」プロファイルを作成 → 実際に片手でも入力できるか

---

## 後回しにするもの（明示）

- XIAO ESP32-C3 での USB HID（ハードは対応だが Arduino-ESP32 の安定性と USB-JTAG との排他が厄介。BLE HID のみで継続）
- BLE Serial（NUS）経由の設定書込み — USB Serial で動いてから拡張。**BLE HID と BLE NUS は同一ペリフェラルで共存可能**（NimBLE は複数 GATT サービス登録対応、[NuS-NimBLE-Serial](https://github.com/afpineda/NuS-NimBLE-Serial) がそのまま使える）。スマホ Chrome の Web Bluetooth でケーブルレス設定が実現する
- ゲームパッド BLE HID — NimBLE で gamepad descriptor 実装が別工数。Phase 4 の後に検討
- Unity 側 [Assets/Assembly-CSharp/M5C_Serial.cs](../../../Documents/M5C_Serial_Unity/Assets/) — 受信側なので変更不要
- `M5 AtomS3 Lite`, `M5StickC Plus2` 等 M5 亜種の manifest — Phase 5 で追加可

---

## 既存資産の評価（有効活用できるか）

### ✅ そのまま流用できる（強い資産）
| 資産 | 所在 | 評価 |
|------|------|------|
| IMU 読取り（M5StickC の M5.Imu、XIAO の MPU6050+DMP） | [MotionController.hpp L261-366](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/MotionController.hpp#L261) | 50Hz サンプリング、AHRS/DMP 両対応。変更不要 |
| クォータニオンの offset/アンラップ/water/upright プリセット | [MotionController.hpp L368-418](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/MotionController.hpp#L368), [IMUReader.cpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/IMUReader.cpp) | 姿勢安定化のノウハウが詰まっている。トリガー判定の前段として有用 |
| NimBLE + BleCombo (キーボード+マウス+Consumer) | [lib/ESP32-BLE-Combo-master_peter/BleCombo.h L136-194](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/lib/ESP32-BLE-Combo-master_peter/BleCombo.h#L136) | `press/release/move/sendReport` API が揃う。MediaKeyReport (2B) で Consumer キー対応済み |
| LittleFS 上の pk3 バイナリ保存・読込パイプライン | [IMU_BLEorSerial_tester.cpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/IMU_BLEorSerial_tester.cpp) の `savepk3vectol2file`/`loadpk3vector` | ヘッダ CSV + 200ms 待ち + バイナリ本体 の手順は堅い。pk4 でもそのまま使う |
| EEPROM レイアウト (NVS 上の 4094B) + キャリブレーション永続化 | [lib/EEPROM.cpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/lib/EEPROM.cpp) | そのまま温存。`SET_OUTPUT` の保存先に空き領域を使う |
| esp-web-tools + manifest per board | [Web/manifest_m5stickc.json](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/manifest_m5stickc.json) | 1ファイル複製で S3 対応可能 |
| ボタン読取り / チャタリング除去（`switchRead()` + `prev_button_state` エッジ判定） | [MotionController.hpp L518-545](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/MotionController.hpp#L518), [IMU_BLEorSerial_tester.cpp ButtonSessionLoop](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/IMU_BLEorSerial_tester.cpp) | プルアップ/プルダウンのピン仕様ごとの扱いがある。TriggerEngine の入力側にそのまま差す |

### 🟡 リファクタが必要だが骨格は使える
| 資産 | 所在 | 課題 | 対処 |
|------|------|------|------|
| [motion_controller.js](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/motion_controller.js) 710行 | Web/ | 全てグローバルスコープ、UI/ロジック密結合、readLoop が巨大 | `connectSerial`/`readLoop`/`processBuffer`/`encodePK3` は抽出してモジュール化。残りは新 UI で書き直し |
| [motion_controller_updater.html](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/motion_controller_updater.html) | Web/ | **文法破綻あり** (L237 `s` 単独、`cl  ass` タイポ、タグ閉じ忘れ、「姿勢判定方法」フォームが2回出現) | 既存ページは保守用に残し、新規 `Web/hidconfig/index.html` は白紙から書く |
| Serial コマンドパーサ (`ReadSessionLoop` L1008-1143) | [IMU_BLEorSerial_tester.cpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/IMU_BLEorSerial_tester.cpp) | 長大な if/else チェーン、コマンドごとに重複コード | 既存は壊さず、ディスパッチテーブル化しつつ `SET_OUTPUT`/`addpk4` を末尾に追加 |
| pk3 構造体 (84B) | [MotionController.hpp L121-135](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/MotionController.hpp#L121) | マウス/ゲームパッド/修飾キー不可、`msg_format` は 0/1 のみ使用 | pk4 は同サイズで `msg_format` を拡張。既存 pk3 レコードはローダで読み続ける |
| `execSF_HIDInputs` の押下/解放シーケンス | [MotionController.hpp L549-604](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/MotionController.hpp#L549) | 波動拳コマンド特化、汎用性なし。`vTaskDelay(30)` がハードコード | 汎用 TriggerEngine 内で「押下→interval 待ち→解放」シーケンスを抽象化、pk4 の `hid_input_interval` を尊重 |
| hidSessionLoop (50Hz タスク) | [IMU_BLEorSerial_tester.cpp L491](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/IMU_BLEorSerial_tester.cpp) | コメントアウト状態で半完成 | 丸ごと削除 → TriggerEngine を ImuLoop 内から直呼び（別タスク不要、ミューテックス削減）|

### 🔴 使えない / 新規実装が必要
| 欠けている機能 | 理由 | 実装の指針 |
|----------------|------|-----------|
| **BLE HID ゲームパッド** | BleCombo は Keyboard + Mouse + MediaKeys のみ。HID Descriptor にゲームパッド無し | **推奨 [Mystfit/ESP32-BLE-CompositeHID](https://github.com/Mystfit/ESP32-BLE-CompositeHID) への移行** — Keyboard+Mouse+Gamepad(XInput/Generic)+Consumer を 1 デバイスに合成、NimBLE ベース、Arduino-ESP32 v2 対応、使わないモジュールを除外可。現 BleCombo を順次置換 |
| **USB HID (S3)** | TinyUSB 呼出しコード皆無 | Arduino-ESP32 2.0.14 以降の `USBHIDKeyboard.h`/`USBHIDMouse.h` と `USBHIDGamepad.h`（※ 非公式、要 USB Descriptor 実装）を使用。CDC と HID の composite は `board_build.arduino.usb_mode = 1` で有効化 |
| **出力先ランタイム切替** | 常時 BLE のみ | `HidSink` 抽象化 + `outputMask` フラグ（NVS 永続化）|
| **統合トリガー→HID マッピング UI** | 現 page2 は raw hex 入力の開発者 UI | 「今の姿勢を使う」「キー選択ドロップダウン」「テスト発火」ボタン等で非エンジニアが登録可能に |
| **プロファイル保存/読込** | 現状 LittleFS ファイル名でしか区別できず、Web UI に一覧機能なし | JSON エクスポート/インポート + `listDir` コマンドで LittleFS 上のプロファイル一覧取得 |
| **BLE Serial 設定書込み (NUS)** | NUS サービスなし | Phase 2 で `NuS-NimBLE-Serial` を追加、BLE HID と同一ペリフェラル上で共存。ESP32 PICO D4 + Arduino v2 + NimBLE 1.4.x でそのまま動作 |

### 流用率の概算
- **FW**: 約 65% 流用（IMU読取り/BLE HIDベース/LittleFS保存/Serialパーサ基本構造）、35% 新規/改修（TriggerEngine, HidSink 抽象, USB HID, pk4, SET_OUTPUT, BLE gamepad）
- **Web**: 約 25% 流用（Serial接続/Three.js/pk3 encode/esp-web-tools 呼出し）、75% 新規（UI 全面刷新、pk4 codec、プロファイル管理、トリガーエディタ）

---

## 要検討事項（承認前に判断が必要）

### 必須の判断ポイント（このまま進めると困る）

**Q1. BLE HID ゲームパッドの実装方式は？**
現 BleCombo にゲームパッドなし。3択：
- **A**: BleCombo を拡張して gamepad descriptor を追加（単一 BLE デバイス、実装工数: 中〜大）
- **B**: 別ライブラリ `ESP32-BLE-Gamepad` を追加し BLE サーバーを2つ持つ（OS 側から別デバイスに見える可能性、実装工数: 小〜中）
- **C**: ゲームパッドは USB HID (S3) のみサポートし、BLE ではキーマップ経由（例: 左スティック = WASD）で代用（実装工数: 小）
→ **推奨 C** で MVP、その後 A を追加（ゲームパッド BLE の確実性を優先するなら）。

**Q2. Arduino-ESP32 のバージョンは？**
現在 `framework-arduinoespressif32 @ 2.0.14` 固定。USB HID Gamepad は 2.0.14 の `USBHIDGamepad` に依存。TinyUSB 周りの API 安定性検証が必要。アップデートすると既存の M5StickC ビルドが影響を受ける可能性。
→ **推奨**: S3 環境のみ 3.0.x 系に上げて試す（`[env:esp32s3]` 単独設定）。M5StickC は 2.0.14 維持。

**Q3. LittleFS の pk4 と pk3 ファイルの共存ポリシー？**
既存デバイスに pk3 データがある場合、pk4 書込みで消える？ 残す？
→ **推奨**: `listDir` で一覧取得し、Web 側で「pk3 は旧形式」と表示。pk4 書込みは別ファイル名 (`profile_*.pk4`) で既存 pk3 と物理的に分離。

**Q4. Web 設定アプリのホスティング戦略？**
- **A**: ローカル `python -m http.server` のみ（最小、HTTPS なしで動くが Web Serial は HTTPS 必須）
- **B**: GitHub Pages に pushして HTTPS 付きで公開
- **C**: findradio.jp に push（既存と並べる）
→ **推奨 B**: リポジトリ内にコードがあれば GitHub Pages 公開は `.github/workflows/pages.yml` 一本で済む。既存 findradio.jp 版と並存。
→ ※ Web Serial API は `localhost` なら HTTP でも動くが、LAN 経由なら HTTPS 必須。

### 追加検討事項

**Q5. トリガー評価周期は？**
現 ImuLoop は 50Hz (20ms)。トリガー判定もこの周期で行うと最大 20ms の遅延。ゲーム用途（特に格闘）なら 100Hz (10ms) が望ましい。MPU6886 は最大 1kHz 対応、DMP は最大 200Hz。
→ **推奨**: IMU は 100Hz 化、write/read タスクは 151ms/201ms 維持。

**Q6. 「姿勢」トリガー判定のアルゴリズム**
現 pk3 は Euler 値 + 許容幅。しかし Euler はジンバルロックが発生。クォータニオン dot 積（角度差）なら安定するが UI が難しい。
→ **推奨**: UI では Euler (pitch/roll/yaw) で提示、FW 内部でもそのまま比較（簡易）。格闘ゲーム用途では Euler 判定で実績あり。将来 dot 積ベースの判定をオプション追加。

**Q7. HID リリースのタイミング**
BLE HID で `press` 後 `release` しないとキーが押しっぱなしになる。現 `execSF_HIDInputs` は vTaskDelay(30) + releaseAll() で終端処理。トリガー連発時に press/release が噛み合わなくなる可能性。
→ **推奨**: TriggerEngine 内で各 pk4 エントリに「press 中フラグ」を持ち、トリガー条件が解除された時点で release。`hid_input_interval` は press〜release の最小ホールド時間とする。

**Q8. ESP32-S3 の USB 書換え時の挙動**
S3 で USB HID (composite) を有効化すると、通常の書込みが `esptool.py` から見えなくなる可能性。`board_upload.use_1200bps_touch = yes` + `board_upload.wait_for_upload_port = yes` で自動復帰するはず。
→ **推奨**: README にリセットボタン長押し（BOOT モード）手順を明記。esp-web-tools でも同じ問題があるが、INSTALL 前にリセット案内を出す。

**Q9. Unity 連携は維持するか？**
`M5C_Serial.cs` は `sensor_data,...` CSV を受信する。SET_OUTPUT=NONE + Unity との併用で「HID 出力なしでセンサデータだけ Unity に流す」が可能。
→ **推奨**: 維持（既存の CSV 出力は変えない）。既存の「SERIAL ON/OFF」と独立した `SET_OUTPUT` 概念とする。

**Q10. アクセシビリティシナリオの具体要件**
「手に障害がある人の入力デバイス」として、片手/片指のみで操作する場合、トリガーは「大きな加速度 1 つ」「傾斜の 1 軸のみ」等シンプルな方が良い。
→ **推奨**: サンプルプロファイル「Accessibility - 1 motion = 1 key」を付属。Web UI に「簡単モード」トグルを設け、トリガー登録時に複数条件を隠す。

---

## 運用・信頼性に関わる追加仕様（MVP 要件）

### 1. OTA (Over-The-Air) ファームウェア更新

**方針**: USB Serial 経由の esp-web-tools 書込みを基本としつつ、将来的に BLE / WiFi OTA に拡張可能な構造にする。

**Phase 4 MVP**:
- esp-web-tools v10 経由の USB フラッシュ（既定）
- Web UI の「FW 更新」タブで manifest 切替、INSTALL ボタン
- FW バージョン確認: `{"cmd":"device.info"}` 応答の `fw` フィールドと manifest の最新版を比較、更新推奨通知

**Phase 7+ 拡張**:
- **WiFi OTA** (ESP32): ArduinoOTA ライブラリ or ESP HTTP OTA、自宅 LAN 上で自動更新
- **BLE OTA DFU** (nRF52840 標準、ESP32 は NimBLE でカスタム実装)
- ロールバック: OTA パーティション A/B 切替、失敗時は旧 FW に自動復帰

**バージョン管理**:
- セマンティックバージョニング (`major.minor.patch`)
- NVS に `fw_version` 保存、起動時にログ出力
- プロファイルに `min_fw_version` を指定、Web UI が互換性チェック

### 2. エラー処理・復旧戦略

**致命的エラーの分類と対応**:

| エラー | 検出 | 対応 |
|--------|------|------|
| IMU I2C/SPI 通信失敗 | `imu.begin()` 戻り値 false、通信タイムアウト | LED 赤点滅、セーフモード (設定は受けるが HID 出力停止)、3 秒後再初期化リトライ |
| BLE HID 切断 | NimBLE disconnect callback | 自動再接続（advertising 再開）、接続失敗 60 秒で DeepSleep 移行 |
| USB HID 失敗 | TinyUSB init エラー | USB HID 無効化、BLE HID のみで動作継続 |
| LittleFS マウント失敗 | `LittleFS.begin()` 戻り値 false | フォーマット → デフォルトプロファイル書込み → 再起動 |
| LittleFS 個別ファイル破損 | JSON パースエラー | そのプロファイルをスキップ、Web UI に警告通知 |
| NVS 破損 | `nvs_get_*` 失敗 | NVS 全消去 → 工場出荷状態 → 初回キャリブレーションウィザードへ |
| バッテリ異常 (過放電・過充電) | ADC 測定値異常 or TP4056 状態異常 | 即座 DeepSleep、LED 赤速点滅 |

**Watchdog Timer**:
- ESP32: Task Watchdog (TWDT) 有効化、各タスクで定期 feed
- 10 秒間 feed なしで自動再起動
- Crash 時: `esp_core_dump` を NVS 領域に保存、次回起動で Web UI から取得可

**Web UI エラー表示**:
- Connect ページに「デバイス状態」パネル
- 直近のエラーログ（FW → JSON Lines で出力）
- トラブルシューティング診断 (「BLE 接続できません → ペアリング解除＆再接続」等)

### 3. LED・フィードバック UX

**視覚フィードバック（LED 配置）**:
| LED | 機能 | パターン |
|-----|------|----------|
| **ステータス LED** (青/緑/赤 3 色 or RGB) | 接続状態 | 青点灯=BLE接続、緑=USB接続、赤点滅=切断/エラー |
| **バッテリ LED** (赤 単色) | 残量 | 常時点灯=充電中、1/3 点滅=20%以下、速点滅=5%以下 |
| **トリガー発火 LED** (白 or RGB) | 視覚確認 | 発火時 50ms フラッシュ、プロファイル名別に色分け可 |

**聴覚フィードバック（オプション、BOM +$0.3）**:
- 圧電ブザー: 起動音、Wake 確認音、エラー音
- アクセシビリティで視覚が使えない方向けに重要

**触覚フィードバック（Phase 5+ 量産版で検討）**:
- 小型 LRA 振動モータ ($0.5、BOM 余裕あり)
- トリガー発火時に 50ms 振動
- 視覚・聴覚に依らず確認可能、アクセシビリティで重要

**Web UI での有効化設定**:
```json
{
  "feedback": {
    "led_trigger_flash": true,
    "led_trigger_color": "by_rule",   // "fixed" | "by_rule"
    "buzzer_wake": true,
    "buzzer_error": true,
    "haptic_trigger": false
  }
}
```

### 4. レイテンシ予算・性能目標

**End-to-end レイテンシ目標**:
```
モーション発生 → HID 出力到達 ≤ 16ms (60fps ゲーム相当)
```

**内訳と最適化余地**:
| 区間 | 予算 | 実装 |
|------|------|------|
| IMU サンプリング | 10ms (100Hz) | Mahony ベース、200Hz 化で半減可 |
| フィルタ + トリガー判定 | < 1ms | if-else のみ、CPU 負荷小 |
| HID 送信 | 5-15ms | BLE HID Connection Interval 7.5ms、USB HID 1-2ms |
| **合計** | **16ms 前後** | USB HID は <15ms、BLE HID は 15-25ms |

**BLE HID の低レイテンシ化**:
- Connection Interval 最小化: 7.5ms (NimBLE 設定で要指定)
- Connection Event Length 最大化
- Slave Latency = 0 (低レイテンシ優先、電力増)
- **ゲーミング用プロファイル**: 低 latency 優先、電池消費 +20%

**測定方法 (Phase 4 QA)**:
- オシロスコープで IMU INT → BLE advertise 開始のタイミング
- Wireshark + BLE sniffer で空中パケット観測
- 高速カメラでモニター画面の反映まで計測（実用ベンチマーク）

**プロファイルごとの latency モード設定**:
```json
{
  "latency_mode": "low"   // "low" | "balanced" | "power_saving"
}
```

### 5. マルチコントローラー対応

**シナリオ 3 種**:

**A. 2 人対戦 (2 Controllers → 1 PC)**
- 各 Controller が個別に BLE HID ペアリング
- PC/OS レベルで 2 つのキーボード/マウスとして認識
- **HID Descriptor の Report ID 差別化** で両手識別可能 (Player 1/2 区別)
- プロファイルに `player_id` 0/1 フィールド、同じキーが衝突しないよう自動マッピング (P1=WASD, P2=IJKL 等)

**B. 両手構成 (2 Controllers → 1 User)**
- 右手 + 左手で 1 セット
- **複合トリガー**: 右手 A + 左手 B の姿勢組合せで特殊技
- 必要: Controller 間通信 (ESP-NOW 直接通信) or PC 経由で中央集権判定
- **Phase 6+ 実装**: Controller A が Master、B が Slave、ESP-NOW で A に状態通知、A が複合判定

**C. 1 Controller 複数 PC 切替**
- BLE HID の bond 複数保持（Windows・iPad 等）
- ボタン長押しで bond 切替
- nRF52840 は BTStack でネイティブ対応、ESP32 は NimBLE カスタム実装

**プロファイルに「device_role」フィールド追加**:
```json
{
  "hardware": {
    "device_role": "solo",       // "solo" | "player1" | "player2" | "right_hand" | "left_hand"
    "peer_controller_id": null   // 複合トリガー時、ペアの Controller MAC
  }
}
```

**MVP スコープ**: **A (2 人対戦) のみ対応**。B (両手複合) は Phase 7+、C (複数 PC) は Phase 8+。

### 6. バッテリ管理・低電力時の挙動

**電池電圧の監視**:
- ADC で Li-ion バッテリ電圧測定（分圧抵抗経由）
- 10 秒毎にサンプリング、移動平均
- 状態を NVS `battery_state` に書込み、Web UI に報告

**残量と挙動**:

| 残量 | 電圧 (Li-ion 1S) | LED 表示 | 動作 |
|------|-----------------|----------|------|
| 100-50% | 4.2-3.7V | 無し (正常) | フル機能 |
| 50-20% | 3.7-3.5V | バッテリ LED 低速点滅 | フル機能、Web UI 通知 |
| **20-10%** | 3.5-3.3V | 中速点滅 | **警告通知、高消費機能 warn** |
| **10-5%** | 3.3-3.1V | 速点滅 | **Wake-on-motion のみ、HID 動作継続** |
| **5-1%** | 3.1-3.0V | 非常速点滅 | **設定保存、BLE 切断、DeepSleep** |
| < 1% | < 3.0V | 消灯 | **強制シャットダウン** (Li-ion 過放電保護) |

**充電中の挙動**:
- TP4056 の STAT ピンを GPIO で監視
- 充電中: 赤 LED 常時点灯
- 満充電: 青 LED 常時点灯
- 充電中でも HID 動作継続可（USB 経由で電力供給されるので）

**保管時の自己放電対策 (Ship Mode)**:
- 初回起動前は Latching Switch で電池完全切断 (Ship Mode)
- 輸送中・店頭陳列中の自己放電防止
- 初回起動はユーザーがスイッチ操作で通電開始

**Web UI でのバッテリ表示**:
```
┌──────────────────────────────┐
│ 🔋 バッテリ: 85% (推定 2.8h) │
│ 充電中: いいえ               │
│ 最終充電: 2日前              │
│ サイクル数: 47 回            │
└──────────────────────────────┘
```

**ユーザー設定**:
- 警告表示しきい値 (デフォルト 20%)
- 省電力モード (Wake 高閾値化、BLE interval 広げる)

---

## プロジェクト Agent 体制（開発チーム構成）

### コア Agent（全フェーズで必須）

| # | Agent 役割 | 主な担当 | 投入フェーズ |
|---|-----------|---------|-------------|
| 1 | **System Architect** | 全体アーキテクチャ整合性、pk4 データモデル、JSON プロトコル、HAL 抽象化、Phase 遷移管理 | 全期間 |
| 2 | **Embedded Firmware Engineer (ESP32)** | Arduino ESP32 v2、NimBLE、TinyUSB、`src/core/*` `src/hal/esp32/*` 実装 | Phase 1〜5 |
| 3 | **Sensor / DSP Engineer** | IMU 特性評価、Mahony/SFLP/DMP、軸リマップ、6点キャリブレーション、Wake-on-motion 閾値 | Phase 1〜6 |
| 4 | **Web Frontend Developer** | Preact+htm+Tailwind、Web Serial、Three.js、esp-web-tools、`Web/hidconfig/*` | Phase 1〜7 |
| 5 | **UX / UI Designer** | トリガーエディタ UX、キャリブレーションウィザード、日英多言語、モバイル対応 | Phase 3〜7 |

### 専門 Agent（特定フェーズで投入）

| # | Agent 役割 | 主な担当 | 投入フェーズ |
|---|-----------|---------|-------------|
| 6 | **Hardware / Electrical Designer** | 回路/PCB/BOM、LCSC 調達、Touch パッド設計、電源/充電、BLE アンテナ | Phase 5〜 |
| 7 | **Hardware / Mechanical Designer** | 筐体 3D CAD、射出成型/3D プリント、エルゴノミクス、防水 | Phase 5〜 |
| 8 | **BLE Protocol Specialist** | HID Descriptor 最適化、NUS 共存、OS 互換性 (Win/Android/iOS) | Phase 2〜4 |
| 9 | **Embedded Firmware Engineer (nRF52840)** | Adafruit nRF52 Core、Bluefruit、TinyUSB nRF52 | Phase 6〜 |
| 10 | **RF / Wireless Engineer** | ESP-NOW、nRF ESB、遅延計測、チャネル最適化 | Phase 8 |
| 11 | **Cloud / DevOps Engineer** | GitHub Actions、CF Pages、jsDelivr、Workers+R2+D1 | Phase 5〜7 |
| 12 | **QA / Test Engineer** | E2E (Playwright)、互換性 (PC/Android/iOS/Switch)、遅延計測 | Phase 3〜 |

### 非技術系 Agent

| # | Agent 役割 | 主な担当 | 投入フェーズ |
|---|-----------|---------|-------------|
| 13 | **Accessibility Consultant** | 身体障害ユーザー向け検証、リハ療法士連携、当事者テスト | Phase 3〜（全期間） |
| 14 | **Community Manager** | profile repo モデレーション、PR レビュー、Discord 運営 | Phase 6〜 |
| 15 | **Technical Writer** | README、チュートリアル、動画、API ドキュメント、日英 | Phase 4〜 |
| 16 | **Safety / Regulatory** | 技適、FCC、CE、電池安全 (UL1642、PSE)、RoHS | Phase 9（量産直前） |
| 17 | **Marketing / SNS / Growth** | SNS 運用 (X/YT/TikTok)、Protopedia/Qiita 記事、デモ動画、インフルエンサー連携、クラウドファンディング、プレスリリース、LP 運用 | **Phase 1〜（早期開始推奨）** |

### #17 Marketing / SNS / Growth Agent の詳細

**小規模チームでも必須**。プロダクトが市場に届かないと技術成果も伝わらない。

**担当範囲**:
| 領域 | 具体タスク |
|------|-----------|
| SNS 運用 | Twitter/X、YouTube、TikTok、Instagram、LinkedIn で定期投稿。開発進捗 GIF、技術解説、ユーザー投稿リポスト |
| 日本向けメディア | Protopedia (既 #1988)、Qiita、Note、Zenn、はてな |
| 海外向けメディア | Hackaday、Hackster.io、Tindie、Make:、Medium、Dev.to |
| 動画 | デモ動画 (motion→keypress 可視化)、組立てチュートリアル、アクセシビリティ事例 |
| コミュニティ | Discord サーバー、Reddit (`r/esp32`, `r/arduino`, `r/accessibility_gaming`)、M5Stack Community |
| インフルエンサー | アクセシビリティ系 YouTuber (AbleGamers、SpecialEffect)、ゲーム Streamer、メイカー系 |
| 広告 (任意) | Meta/Google/X/YouTube Ads、少額から。アクセシビリティ市場はオーガニック優先 |
| LP | `motion.findradio.jp` にプロダクト説明、購入リンク、動画 |
| プレス | PR Times、MakerZine Japan、ITmedia、4Gamer 等へ量産時投げ込み |
| クラファン | Kickstarter / Makuake / CAMPFIRE — 量産資金 + 市場検証 |
| コンテスト | Hackaday Prize、Good Design Award、Protopedia Award、Sony Product Design Award |
| 販路開拓 | スイッチサイエンス、秋月、Amazon、Adafruit、SparkFun 代理店打診 |

**Phase 別タスク例**:
```
Phase 1-2 (開発初期):  進捗ツイート、「作ってます」アピール → フォロワー獲得
Phase 3-4 (MVP):       デモ動画、β テスター募集、コミュニティ形成
Phase 5 (HW 設計):      設計過程共有、色/形状の投票
Phase 6-7 (機能拡充):   ユーザー事例、プロファイル共有コンテスト
Phase 8 (Adapter):     ゲーマー向け訴求
Phase 9 (量産):         クラウドファンディング + プレスリリース + 販売開始
継続:                   新プロファイル投稿紹介、ユーザー作品拡散
```

**3 セグメント別訴求戦略**:

1. **ゲーマー**: 「体で操作する新しいゲーム体験」
   - Twitch/YouTube Streamer に提供 → プレイ動画拡散
   - 格闘ゲーム/VR/リズムゲームデモ

2. **メイカー/開発者**: 「カスタマイズできる OSS 入力デバイス」
   - GitHub star、技術記事
   - ハッカソン採用事例
   - OSS プロファイル投稿の仕組み訴求

3. **アクセシビリティユーザー**: 「手の障害があっても PC/ゲームができる」
   - **最重要セグメント** — Xbox Adaptive Controller ($100) より圧倒的に安価 ($25 目標)
   - 支援学校、リハビリ施設、日本障害者協議会への紹介
   - 当事者インタビュー動画
   - 医療機器ではなく「支援ツール」として位置付け

**チーム規模別の扱い**:
| チーム | Marketing 対応 |
|-------|---------------|
| **1 人 (個人)** | **必須兼任**。技術開発と並行、最低週 1 投稿 |
| 2-3 人 | 専任 1 名 or 全員 20% 負担 |
| 5-8 人 (スタートアップ) | 専任 1 名 (Phase 3〜、できれば Phase 1 から) |
| 10+ | 2-3 名 (SNS / コンテンツ / 広告分業) |

**Claude Code Subagent prompt 例**:
- "デモ動画の台本（30 秒、モーション→キー入力可視化）"
- "Protopedia 進捗報告テキスト草案"
- "アクセシビリティ系 YouTuber 宛ピッチメール"
- "Kickstarter キャンペーンストーリー構成（問題→解決→動画→リワード）"
- "ハッシュタグ戦略（日英、セグメント別）"
- "次週の投稿スケジュール 7 日分"

### フェーズ別投入スケジュール

```
Phase:    1  2  3  4  5  6  7  8  9
---------|--|--|--|--|--|--|--|--|--|
1 Arch   |██|██|██|██|██|██|██|██|██|
2 FW-ESP |██|██|██|██|██|  |  |  |  |
3 Sensor |██|██|██|██|██|██|  |  |  |
4 Web FE |██|██|██|██|██|██|██|  |  |
5 UX/UI  |  |  |██|██|██|██|██|  |  |
6 HW-EE  |  |  |  |  |██|██|██|██|██|
7 HW-ME  |  |  |  |  |██|██|██|  |██|
8 BLE    |  |██|██|██|  |  |  |  |  |
9 FW-nRF |  |  |  |  |  |██|██|  |  |
10 RF    |  |  |  |  |  |  |  |██|  |
11 Cloud |  |  |  |  |██|██|██|  |  |
12 QA    |  |  |██|██|██|██|██|██|██|
13 A11y  |  |  |██|██|██|██|██|██|██|
14 Com   |  |  |  |  |  |██|██|██|██|
15 Doc   |  |  |  |██|██|██|██|██|██|
16 Safety|  |  |  |  |  |  |  |  |██|
17 MKT   |██|██|██|██|██|██|██|██|██| ← 全期間、早期開始が効果的
```

### 現実的なチーム規模

**個人 or 小規模 (1-3 人)**:
- System Architect (兼プロジェクトリード)
- Embedded FW + Sensor 兼任
- Web Frontend + UX 兼任
- **Marketing / SNS は必ず誰かが担う** (週 1 投稿最低、できれば週 3)
- HW は Phase 5 以降に外部委託
- その他は必要時に委託

**スタートアップ (5-8 人)**:
- コア 5 役を各 1 名専任
- HW は 2 役兼任
- BLE/RF/nRF は短期投入
- QA/A11y は専門契約

**中規模 (10-15 人)**:
- 全 16 役を時期差で投入

### Claude Code Subagent 実装候補

`.claude/agents/` に以下のファイルを用意すると、各専門分野の Agent を呼出しで使える:

```
.claude/agents/
├── system-architect.md
├── firmware-engineer-esp32.md
├── firmware-engineer-nrf52.md
├── sensor-dsp-engineer.md
├── web-frontend-developer.md
├── hardware-designer.md
├── ble-specialist.md
├── ux-ui-designer.md
├── accessibility-consultant.md
├── qa-test-engineer.md
├── cloud-devops.md
├── technical-writer.md
└── marketing-growth-agent.md   # SNS 草案、記事構成、Kickstarter ピッチ、デモ動画台本
```

各 Agent ファイルには「専門分野」「参照ドキュメント」「OK/NG 行動」「Phase 内の具体タスク例」を定義。**本計画の実装段階で、専門性を要する場面で該当 Agent に委譲**することで、個別の深い調査・設計作業を並列化できる。

---

## リスク・不確定要素

1. **TinyUSB + BLE (NimBLE) の共存メモリ** — ESP32-S3 で NimBLE スタックと USB HID スタックを同時に立ち上げた時の RAM/Flash 使用量。NimBLE で約 80KB、TinyUSB HID で約 20KB。S3 (512KB SRAM) なら余裕だが、要実測
2. **ESP32-S3 で USB CDC がアップロード後に消える既知問題** — `use_1200bps_touch` とリセット手順を README に明記
3. **BleCombo の release タイミング** — 既存 `execSF_HIDInputs` で delay(20-50ms) が入っており、TriggerEngine で同等のインターバルを維持しないと素早い姿勢遷移時にキーが押しっぱなしになる
4. **pk4 と pk3 のファイル共存** — ファイル名で区別 (`pk3_*.bin` vs `profile_*.pk4`)、Web UI は pk4 のみを扱い、pk3 は互換維持のため FW 側のローダだけ残す
5. **BLE HID Gamepad の OS 認識** — Windows はベンダー/プロダクト ID 次第で認識が不安定。PS5 は BLE Gamepad を認識しない場合あり。要実機検証
6. **Arduino-ESP32 3.x 移行** — S3 の TinyUSB 安定化のため 3.x 系に上げる必要があるが、NimBLE + M5StickC ビルドが破綻する可能性。切替は S3 env だけに限定
7. **Web Serial の HTTPS 要件** — GitHub Pages 公開なら自動的に HTTPS、LAN 共有するなら自己署名証明書必要

---

## Phase 5.33 — Harry Potter Wand Magic Motion 機能 (新規)

### Context (なぜ実装するか)

Burst Motion の SEQUENCE モード (`State states[4]`、`src/core/types.hpp:114-142`) は FW で既に完全実装されているが、UI からは到達できない「眠っている」機能。アクセシビリティ/格闘ゲーム用途は ONESHOT / HOLD で十分で出番がなかったが、**ハリーポッターの「振り動作 → 呪文発動」というショーケース用途は、4 waypoint シーケンス検出にぴったり合う**。

Kano 公式の Harry Potter Coding Wand は 2018-2023 に 18 万本売れたが、Kano vs Warner Bros の知財係争 ([TechCrunch 2023](https://techcrunch.com/2023/01/27/warner-bros-swiped-our-harry-potter-wand-ip-says-kano/)) で 2026 現在は Kano 直販停止 / WB の Magic Caster Wand も発売直後に販売停止 ([MuggleNet](https://www.mugglenet.com/2023/04/warner-bros-suspends-sales-of-magic-caster-wand-possibly-due-to-dispute-with-kano/))。eBay/Best Buy で残在庫が散在するのみで、**ハリポタワンド市場は実質空白**。Burst Motion で再現すれば、デモ訴求力 + ファン文化への接続 + ハイレベル機能 (SEQUENCE) の実証 という 3 つの効果が得られる。

[Hogwarts-Legacy-Wand GitHub](https://github.com/Thats-so-Mo/Hogwarts-Legacy-Wand) に Kano ワンドの 30+ 呪文の方向シーケンスが**コードに直書きで全て載っており**、これをそのまま MVP の典拠として借用できる。

### 設計サマリ

**検出モデル: 8 方向シーケンス (Kano 流、ユーザー確定)**

各 waypoint = 8 方向のうち 1 つ。FW 内部で固定 Pitch/Roll 領域にハードコードマッピング:

| dir | Pitch | Roll | dir | Pitch | Roll |
|-----|-------|------|-----|-------|------|
| U   | +30°  | +90° | UR  | +30°  | +120° |
| D   | -30°  | +90° | UL  | +30°  | +60° |
| L   | 0°    | +60° | DR  | -30°  | +120° |
| R   | 0°    | +120°| DL  | -30°  | +60° |

各方向の許容: Pitch ±15°、Roll ±20°、Yaw 無視。**M5StickC を LCD 左向き縦持ち基本姿勢 R+90/P0 から振った時、Phase 5.30 と同じ姿勢領域を再利用**するので、既存 SF プロファイルが動いていれば追加調整不要。

**出力: 呪文名テキスト入力 (ユーザー確定)**

最後の waypoint で `fire_text` を BLE HID Keyboard に送信:
```
"wingardium leviosa\n"  → Notepad / ChatGPT / 任意のテキストエリアに自動入力
```

**MVP 5 呪文 (ユーザー確定 — 基本セット)**

| 呪文 | 方向シーケンス | 入力テキスト | 由来 |
|------|--------------|--------------|------|
| Lumos | `["U", "U"]` | `lumos\n` | Hogwarts-Legacy-Wand 互換 |
| Wingardium Leviosa | `["DR", "R", "UR", "D"]` | `wingardium leviosa\n` | 同上 |
| Expelliarmus | `["R", "UR"]` | `expelliarmus\n` | 「右へ振り上げる」近似 |
| Protego | `["DR", "UR"]` | `protego\n` | Hogwarts-Legacy-Wand 互換 |
| Stupefy | `["DL", "R", "DL"]` | `stupefy\n` | 同上 |

### 実装ロードマップ (~1 日)

#### 1. FW: rule.add に `directions[]` ショートハンド追加 (~2h)

`src/main_v2.cpp` の rule.add 処理 (line 925-1140) を拡張。
- `r.directions: ["DR","R","UR","D"]` を受け取り、内部で 4 状態の `State states[]` に自動展開
- 各状態 = `match_condition.posture` (固定 Pitch/Roll 領域)、`max_dwell_ms: 800`、`min_dwell_ms: 50`
- 最終状態の `on_enter.action = FIRE_MACRO`、`keys` = 呪文名テキストの文字列展開

新規ヘルパー関数 (in `main_v2.cpp` 内):
```cpp
static void directionToCondition(const char* dir, Condition& match_out) {
    // dir = "U" → match_out.posture.euler = [90, 30, 0], euler_tol = [20, 15, 180]
    // ... 8 方向すべて
}

// rule.add 内の追加処理
if (in["r"]["directions"].is<JsonArray>()) {
    JsonArray dirs = in["r"]["directions"].as<JsonArray>();
    rule.states_count = min((int)dirs.size(), 4);
    rule.loop = false;  // SEQUENCE
    const char* fire_text = in["r"]["type_text"] | "";
    for (int i = 0; i < rule.states_count; i++) {
        directionToCondition(dirs[i].as<const char*>(), rule.states[i].match_condition);
        rule.states[i].max_dwell_ms = 800;
        rule.states[i].min_dwell_ms = 50;
        if (i == rule.states_count - 1) {
            // 最終 waypoint: 呪文名テキストを連続キー入力として on_enter にセット
            rule.states[i].on_enter.action_type = ACT_FIRE_MACRO;
            // fire_text を 1 文字ずつ keys[] に展開
            for (size_t j = 0; j < strlen(fire_text) && j < MAX_MACRO_KEYS; j++) {
                rule.states[i].on_enter.keys[j] = (uint8_t)fire_text[j];
            }
        }
    }
}
```

修正ファイル:
- `src/main_v2.cpp` (rule.add 処理、~50 行追加)
- (optional) `src/core/types.hpp` に `UI_MODE_SEQUENCE_DIRECTIONS` 列挙追加 — 既存 mode 列挙との衝突回避のため

#### 2. サンプルプロファイル追加 (~30min)

`Web/hidconfig/profiles/harry_potter_wand.json` 新規作成:
```json
{
  "schema": "burst_motion_sample_v1",
  "id": "harry_potter_wand",
  "title": "🪄 Harry Potter Wand (5 呪文)",
  "description": "M5StickC を魔法の杖として振り、5 つの呪文を発動。最後の振りで呪文名がテキスト入力される。基本姿勢 R+90 (縦持ち、LCD 左)。",
  "hardware_required": ["m5stickc", "m5stickc_plus", "m5stickc_2"],
  "engine": {
    "closest_only": false,
    "cooldown_ms": 1500
  },
  "rules": [
    {
      "name": "lumos",
      "directions": ["U", "U"],
      "type_text": "lumos\n"
    },
    {
      "name": "wingardium_leviosa",
      "directions": ["DR", "R", "UR", "D"],
      "type_text": "wingardium leviosa\n"
    },
    {
      "name": "expelliarmus",
      "directions": ["R", "UR"],
      "type_text": "expelliarmus\n"
    },
    {
      "name": "protego",
      "directions": ["DR", "UR"],
      "type_text": "protego\n"
    },
    {
      "name": "stupefy",
      "directions": ["DL", "R", "DL"],
      "type_text": "stupefy\n"
    }
  ]
}
```

`Web/hidconfig/profiles/index.json` に entry 追加:
```json
{
  "id": "harry_potter_wand",
  "title": "🪄 Harry Potter Wand (5 呪文)",
  "description": "Lumos / Wingardium Leviosa / Expelliarmus / Protego / Stupefy — 8 方向シーケンスでテキスト入力",
  "tags": ["wand", "magic", "harry-potter", "sequence", "demo"],
  "rule_count": 5,
  "file": "harry_potter_wand.json"
}
```

#### 3. Web UI のサンプルカード追加 (~1h)

- 既存のサンプル import フローで完結 (新規 UI 不要)。Library タブで「🪄 Harry Potter Wand」が他のサンプルと並んで表示される。
- **オプション**: ヘッダ近くに現在 active な方向シーケンスの可視化 (Three.js canvas の隣に small canvas、waypoint progress バー)。Phase 5.33.1 以降で追加可。

#### 4. M5StickC LCD フィードバック (オプション、~30min)

`g_lcd` 行 8 に直近の trigger.hit のルール名を 1.5s 表示。
- `runOnTriggerHit()` で `lcd_show_text("Lumos", 1500)` を呼び出す。
- 既存 LCD 描画分散ロジック (`updateLcdRow`) と整合する形で実装。

#### 5. テスト + キャリブレーション (~2h)

- 実機で各方向検出を 1 つずつ確認 (姿勢を作って 0.5s 静止 → LCD/Web で current_state の遷移が見えるか)
- 必要なら方向マッピング微調整 (Pitch ±15→±20、Roll +60/+120 → +50/+130 等)
- 5 呪文を実演、誤検出率と取りこぼし率を測定
- 連続呪文発動時の cooldown 効果確認 (1.5s 設定で十分か、3s 必要か)

### 関連ファイル一覧

| 既存ファイル | 役割 | 変更内容 |
|------------|------|---------|
| `src/core/TriggerEngine.cpp:266-350` | SEQUENCE evaluate ロジック | **変更不要** (既存 4-state 実装をそのまま使用) |
| `src/core/types.hpp:114-142` | State / ActionRule 構造体 | **変更不要** (state_count=4 が足りる) |
| `src/main_v2.cpp:925-1140` | rule.add JSON 処理 | `directions[]` ショートハンド対応追加 (+~50 行) |
| `src/core/Profile.hpp:179-294` | profile JSON serializer | **変更不要** (既存 posture スキーマで足りる) |
| `Web/hidconfig/src/app.js` | Web UI | サンプルカード表示は自動 (index.json 経由) |
| `Web/hidconfig/profiles/index.json` | サンプル一覧 | entry 1 件追加 |

| 新規ファイル | 内容 |
|------------|------|
| `Web/hidconfig/profiles/harry_potter_wand.json` | 5 呪文の directions[] + type_text |
| `src/main_v2.cpp` 内ヘルパー | `directionToCondition()` static 関数 |

### 検証手順

1. FW build & flash:
   ```bash
   cd C:\Users\thefu\Documents\M5C_Serial_Unity\M5C_MPU6886_cpp
   pio run -e m5stick-c-v2 -t upload --upload-port COM8
   ```
2. Web デプロイ:
   ```bash
   cd Web\hidconfig && python deploy_ghpages.py "Phase 5.33: Harry Potter Wand 5 spells"
   ```
3. https://uecken.github.io/M5C_Serial_Unity/ にアクセス、USB or BLE で接続
4. Library タブから「🪄 Harry Potter Wand」を import → デバイスに適用
5. **Engine モードに切替** (Mouse モードだと evaluate されない)
6. Notepad / メモ帳を開いて focus
7. **Lumos テスト**: M5C 縦持ち→上にチョン、上にチョン → "lumos" が打鍵される (~1.5s 以内)
8. **Wingardium Leviosa テスト**: DR (右下) → R (右) → UR (右上) → D (下) と滑らかに振る → "wingardium leviosa" が打鍵される
9. **誤動作テスト**: ランダムに動かして 30 秒、意図しない呪文が発動しないか確認 (cooldown 1.5s で十分か)
10. **取りこぼしテスト**: 5 呪文を各 5 回試行、検出率を記録 (目標 80%+)

### 拡張余地 (将来)

- **10 呪文** (Accio / Reducto / Aguamenti / Riddikulus / Nox 追加) — `harry_potter_wand_extended.json`
- **25 呪文** (Kano 公式セット完全互換) — Hogwarts-Legacy-Wand リポジトリの定義をそのまま借用
- **Web UI で方向シーケンスを drag-drop 編集** (Phase 6+) — 8 方向ボタンを並べる UI コンポーネント `WandSequenceEditor.js`
- **自由 Euler waypoint モード (B 案)** への拡張 — SEQUENCE_DIRECTIONS と SEQUENCE_FREE_EULER の併存、より深い flick / より厳しい判定をカスタマイズしたい場合
- **呪文発動時の効果音** (BLE HID Consumer "play media key" 経由でホスト側 PC で wav 再生 — または Stream Deck 連携)
- **VR / Unity 連携** (Unity 側で呪文名テキストを受信 → AR エフェクト/パーティクル)
- **スマホ Web Bluetooth で「魔法学校アプリ」** (呪文一覧、ジェスチャ判定スコア、ランキング、Hogwarts 寮分け診断)

### リスク

1. **検出精度**: 方向間の遷移を 50ms 以上維持しないと取りこぼす可能性。逆に max_dwell_ms 800 → 一連の振り動作を 3.2s 以内 (4 waypoint × 800ms) で完結する必要がある。実機で要確認、長すぎれば 1000ms 等に調整。
2. **基本姿勢からの誤起動**: 最初の方向条件が常時マッチしてしまうと SEQUENCE が誤起動する。Idle state (base posture 領域に戻る) を**最初の waypoint より厳しい条件**で構成するか、`min_dwell_ms` で連続検出を抑制する。
3. **基本姿勢の握り方**: 「LCD 左向き縦持ち」は SF ゲーム用の規約で、ハリポタワンドの自然な持ち方 (横持ち、tip が前を向く) とは若干違う。実機で違和感あれば Phase 5.33.1 で「wand 専用基本姿勢 (R=0/P=0 横持ち)」モードを engine フラグで追加切替可。
4. **呪文連発**: cooldown 1.5s では「Lumos → 即 Nox」のような対の呪文が打てない。プロファイルレベルではなく rule ごとの cooldown_ms で細かく制御するか、cooldown 短縮 (500ms) + 誤動作率トレードオフ。
5. **FW_PHASE バンプ**: 5.32 → 5.33。Web UI ヘッダの「FW Phase < 5.33 警告」を表示する必要 (rule.add の `directions[]` が旧 FW で `unknown_field` エラー or 無視される) — 当該ユーザーは Web から FW 再フラッシュが必要。

---

## Phase 5.33.1 — 部分シーケンス衝突問題と Btn3 ゲート方式 (実機検証で発覚)

### 検出された問題

Phase 5.33 実機検証 (USB Serial デバッグ + 2D Roll/Pitch マップ可視化) で、サンプル 5 呪文のシーケンスが互いに**部分シーケンス衝突**を起こすことが確認された。

| 呪文 A | 呪文 A のシーケンス | 呪文 B | A 内に含まれる B のシーケンス |
|--------|-----------------|--------|------------------------------|
| Wingardium Leviosa | `DR,R,UR,D` | Expelliarmus | `R,UR` を内包 → Expelliarmus 巻き添え発火 |
| Wingardium Leviosa | `DR,R,UR,D` | Protego | `DR,UR` を内包 (途中に R 挟むが state[0]共有) |
| Stupefy | `DL,R,DL` | Expelliarmus | `R` 通過時に Expelliarmus state[0] 発動 → R で state[1]=UR を待つが UR 来なければタイムアウト ※実害低 |

実害が大きいのは Wingardium Leviosa の試行で **必ず Expelliarmus も巻き添え発火**するケース。メモ帳には `expelliarmus\nwingardium leviosa\n` の 2 連発が記録される。

### 根本原因

`TriggerEngine` は N 個の `ActionRule` を**完全に独立した並列状態機械**として走らせる。`rule[i].current_state` が独立、互いの状態を見ない。共通の waypoint を踏むと両方が前進し、先に終端に達した方が発火する。

これは Hogwarts-Legacy-Wand と異なる設計選択 — あちらは「1 つのジェスチャ蓄積バッファ + ジェスチャ終了後に全パターン最長一致」という**単一分類器モデル**を採用している。

### 採用案: Btn3 ゲート (Phase 5.33.1 MVP)

全 5 呪文 rule に `button_idx: 3, button_state: 1` を追加 (FW Phase 5.33 で実装済みのオプション機能を利用)。

**動作:**
- Btn3 押下中だけ各 rule の state 遷移条件が満たされる (`Condition.button` も AND 結合)
- Btn3 を離した瞬間、いずれの rule も次の state 条件が満たされなくなる → `max_dwell_ms` (800ms) でタイムアウトして idle へ
- 1 振り = 1 押下サイクル → 並列に進む複数 rule が共存しても、ユーザーの 1 押下で発火するのは **1 つだけ** (cooldown_ms によって他は抑制される)

**残課題:** Btn3 押下中に Wingardium 振りをしている間、依然として Expelliarmus と Protego の state[0]/state[1] が並走する。最初に終端に達するのは Expelliarmus (2 state) のため、ユーザーが Wingardium 完遂前に Expelliarmus が発火する**可能性は残る**。ただし engine cooldown_ms 1500ms によって、Expelliarmus 発火後 1.5s は他 rule の発火がブロックされる → Wingardium は事実上不発になる。

UX 上「Btn3 押下中だけ受付」「呪文発火後すぐ Btn3 を離す」が運用ルールになる。

### 実装変更

修正済みファイル:
- `Web/hidconfig/profiles/harry_potter_wand.json`: 全 5 rule に `button_idx: 3, button_state: 1` 追加
- `Web/hidconfig/profiles/index.json`: タイトルを「(要 Btn3)」へ、description 更新

FW 側変更不要 (Phase 5.33 で `button_idx` 対応済み、`directionToCondition` 結果に対し全 state へ button condition を上書き)。

### より良い改善案 (Phase 5.34+ 候補)

#### 案 A: Engine cooldown のグローバル化 (短期、~30 分)

現状の engine cooldown_ms (1.5s) は「直前に発火した rule」が再発火しないだけで、他 rule の発火は妨げない。これを「**直近の発火後は engine 全体で次の発火を抑制**」する仕組みに変更すれば、Expelliarmus が先に発火しても Wingardium 完遂時にもう一度発火するのを止められる。逆に意図しない 1 件目 (Expelliarmus) が必ず先に出てしまう問題は残るので不完全。

#### 案 B: 最長一致優先 (中期、~2h)

複数 rule が SEQUENCE の終端 (= 発火条件) に達した瞬間に「**より長い states_count の rule を優先**」して発火、短い rule は idle にリセット。Wingardium (4 state) と Expelliarmus (2 state) が同タイミングで終端に達した場合は Wingardium 優先。

```cpp
// TriggerEngine::tick 中、発火しようとしている rule の優先度比較
ActionRule* winner = nullptr;
for (auto& r : rules) {
    if (r.about_to_fire) {
        if (!winner || r.states_count > winner->states_count) {
            winner = &r;
        }
    }
}
// winner だけ発火、他は reset
```

実装複雑度: 中。`evaluateRule()` を「即発火」から「発火フラグ立て」に分離し、1 tick 内で全 rule 走査後に winner 選択 → 発火。

#### 案 C: 単一ジェスチャ分類器 (本格、~4h)

並列 state machine をやめて、1 つの **GestureBuffer** に方向遷移を記録する設計に変更:

```cpp
class GestureBuffer {
    static constexpr int MAX_SEQ = 8;
    char directions[MAX_SEQ];   // 'U','D','L','R','UL','UR','DL','DR'... を 1 char で表現
    uint8_t length;
    uint32_t last_motion_ms;
    
public:
    void recordDirection(char d, uint32_t now);
    bool isIdleTimeout(uint32_t now, uint16_t gap_ms);
    
    // 蓄積されたシーケンスと全パターンを最長一致で照合
    const ActionRule* classify(const std::vector<ActionRule>& patterns);
};

// tick 内:
// 1. 現在の posture から direction を判定 (基本姿勢領域なら "none")
// 2. 前回と異なる方向に切り替わった瞬間に buffer に追記
// 3. 基本姿勢に 200ms 留まったら「ジェスチャ完結」→ classify → 発火
// 4. cooldown 後、buffer クリア
```

**利点:**
- 完全な衝突解消 (4 state の Wingardium > 2 state の Expelliarmus が確実に勝つ)
- ボタンゲート不要 → Kano-canonical な「純ジェスチャ呪文」UX 復活
- Hogwarts-Legacy-Wand と同じ動作モデル → コミュニティ既存資産が直接使える

**欠点:**
- TriggerEngine の根本変更 (rule.add で `directions[]` 受けるとき、parallel state machine の代わりに pattern table に登録)
- 既存 SEQUENCE rule との共存 (ハリポタ用とそれ以外用) を考慮した分離アーキテクチャが要る
- FW 改修 + Web UI のサンプル import 経路調整

実装するなら **Phase 5.34** として独立フェーズ。FW Phase は 5.33 から 5.34 にバンプ、Web 側で「Phase 5.34+ の場合は GestureBuffer 経路を使う」フラグ。

#### 案 D: シーケンス重複ないように呪文セット差し替え

Kano-canonical を一部諦めて、互いに包含しない 5 呪文セットを採用:

| 呪文 | sequence | 重複なし理由 |
|------|----------|--------------|
| Lumos | `U,U` | U 開始は他になし |
| Nox | `D,D` | D 開始は他になし |
| Alohomora | `L,L` | L 開始は他になし |
| Reducto | `UL,DR` | 対角線、他と非重複 |
| Wingardium Leviosa | `DR,R,UR,D` | DR 開始は Reducto と被るが Reducto の state[1]=DR と Wingardium の state[0]=DR が一致するだけで、Reducto は単独で完結する |

Wingardium Leviosa を保持しつつ Expelliarmus / Protego / Stupefy を捨て、代わりに対の Nox / 弱衝突 Alohomora / Reducto を採用。MVP の「5 呪文」枠を維持しつつボタン不要 UX を実現できる。

ただし「ハリーポッター 5 呪文セット」としては **Expelliarmus がないのは寂しい** (映画/原作で最頻出の呪文の一つ)。ユーザーが「Kano-canonical で行く」と判断したので案 D は不採用。

### 推奨ロードマップ

- **今 (Phase 5.33.1)**: 案 Btn3 ゲートをサンプルに適用 → 即動作確認できる
- **次 (Phase 5.34)**: 案 C (GestureBuffer) を実装、ボタン不要モードへ戻す
- **長期**: Web UI で「呪文セット」をユーザー選択 (Kano-canonical 5 呪文 vs 衝突無し 5 呪文 vs 10 呪文 vs 全 25 呪文) → 各セットに最適な evaluation mode (パラレル / GestureBuffer) を自動選択

---

## Phase 5.35 — 周期/帯域 & IMU FS リファレンス (パフォーマンス調査結果)

### 背景

ユーザーが「Web Serial で stream + 3D viewer + 2D grid を同時表示すると 2020 PC で重い」と報告。
両側を実測するため Web UI に Perf Overlay (sensor Hz、bytes/s、render Hz、grid draw Hz、RAF fps) を実装し、
両側の周期と帯域を体系化。

### MPU6886 IMU 設定 (現状)

[ImuMpu6886.hpp](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/hal/esp32/ImuMpu6886.hpp) で設定:

| 項目 | レジスタ / 値 | 設定値 | LSB/単位 |
|------|--------------|--------|----------|
| **Accel FS** | `ACCEL_CONFIG = 0x10` | **±8 g** | 4096 LSB/g (0.244 mg/LSB) |
| **Gyro FS** | `GYRO_CONFIG = 0x18` | **±2000 dps** | 16.4 LSB/dps (0.061 dps/LSB) |
| **Internal Sample Rate** | `SMPLRT_DIV = 0x00` | **1 kHz** | 1000 Hz / (1+SMPLRT_DIV) |
| **DLPF (Digital Low-Pass)** | `CONFIG = 0x03` | **41 Hz** | Accel/Gyro 共通、anti-aliasing |
| **I²C Bus** | `Wire.setClock(400000)` | 400 kHz | Fast Mode |
| **Power Mode** | `PWR_MGMT_1 = 0x01` | Wake + AutoClock | 通常動作 |

**Accel ±8g の理由**: 通常重力 (1g) + 振り動作 (3-5g for wand flick) + 軽い衝撃に余裕あり。Wand 用途で飽和しない。±16g にすると上限 2 倍だが精度半減 (0.488 mg/LSB)。

**Gyro ±2000 dps の理由**: 杖振り最高速 ~1500 dps (実測)、ペン回し ~700 dps 程度 → 余裕あり。MPU6886 のハード最大。

**3g wake-on-motion 不可制約 (将来課題)**: MPU6886 の WoM レジスタは 1g 上限がハードウェア固定 (4mg/LSB × 8bit = 1.02g 上限)。3g wake は **LSM6DSV16X / BMI270 移行**で初めて実現可能 (Phase 6+)。

### M5C (FW) 側の全周期表

| 項目 | 周期 / 頻度 | コード位置 / 仕組み |
|------|------------|-------------------|
| **MPU6886 内部サンプル** | **1 kHz** | レジスタ `SMPLRT_DIV=0`、ハードウェア生成 |
| MPU6886 DLPF | 41 Hz LPF | レジスタ `CONFIG=0x03`、anti-aliasing |
| **main loop イテレーション** | **~1 kHz** | `loop() + delay(1)` で cooperative yield |
| **IMU read + Mahony update** | **100 Hz** | [`updateSensor()`](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/src/main_v2.cpp#L1360), `last_imu_loop >= 10ms` |
| **TriggerEngine tick** | **100 Hz** | `updateSensor()` 内で `g_engine.tick()` |
| **Mouse mode tick** | **100 Hz** | `updateSensor()` 内で `runMouseModeTick()` (engine と排他) |
| **Sensor stream 送信** | **0/20/50/100 Hz 可変** | `g_stream_rate_hz`、PWR 短押し ON で 50Hz デフォルト |
| **Serial command 処理** | loop 毎 2 回 | `g_serial.process()` (応答性確保) |
| **LCD 更新** | 1 行/loop、12 行で ~500 ms 周期 | `updateLcd()` 分散描画、~5ms/行 |
| **PWR ボタン処理** | loop 毎 (~1 kHz) | エッジ検出 |
| **BLE Connection Interval** | **15-30 ms** | Phase 5.31 で min=12/max=24 (×1.25ms) |
| **BLE supervision timeout** | **4 s** | Phase 5.31 (720ms→4s で切断耐性向上) |
| **NUS notify chunking** | MTU-3 単位 + `taskYIELD()` | Phase 5.31 (delay(5) 撤去) |
| Profile auto-save | event-driven のみ | `rule.add/remove/clear` 後 |

### Web (Chrome) 側の全周期表

| 項目 | 周期 / 頻度 | コード位置 |
|------|------------|------------|
| **3D viewer RAF** | **60 fps** (画面 refresh) | [`IMUViewer._tick()`](../../../Documents/M5C_Serial_Unity/M5C_MPU6886_cpp/Web/hidconfig/src/lib/IMUViewer.js#L225) browser native |
| **3D quaternion slerp** | 60 fps、factor 0.3 | RAF 内、滑らかな補完 |
| **2D PitchRollGrid draw** | **sensor 受信ごと** = stream Hz | `setCurrent()` → `draw()` |
| **React App 再 render** | **sensor 受信ごと** = stream Hz | `setSensor()` state 更新 (★重い、Phase 5.34 で SEQUENCE バッジ計算追加) |
| **Perf overlay 更新** | **1 Hz** | `setInterval(1000)` でカウンタ集計 |
| **closestRuleIdx 計算** | sensor 受信ごと | rule.list と現在姿勢比較 (軽量 N=5) |
| **gravity arrow 更新** | sensor 受信ごと | `setGravityVector()` (Phase 5.34.1 修正、軽量) |
| **BLE auto-reconnect retry** | 指数 backoff 200/600/1400 ms | Phase 5.31 `_gattConnectWithRetry()` |
| **Web Serial send queue** | async 直列化 | Phase 5.31 並列 send 破損防止 |
| **BLE chunk size** | 180 B/chunk | Phase 5.31 (旧 20B から 9 倍) |
| **hw.buttons.get auto-poll** | **5 Hz** (200 ms 間隔) | stream OFF 時のみ |
| **profile.list / rule.list** | event-driven のみ | ack 受信後の再取得 |

### Mahony フィルタ設定 (姿勢推定の精度)

| 項目 | 値 | 評価 |
|------|----|----|
| **twoKp (比例補正)** | 2.0 | 標準値、加速度→姿勢補正の強度 |
| **twoKi (積分補正)** | **0.0** | ⚠ 動的 gyro bias 補正なし、長期 Yaw ドリフト |
| **Roll/Pitch 精度** | 静止 ±1°、動的 ~5° | ✅ Wand に十分 |
| **Yaw 精度** | 累積ドリフト **0.1-1°/分** | ⚠ 磁気センサ無し、絶対基準なし |
| **Gyro bias 補正** | 静的 (`calibrate.simple`) | 起動時の固定値、温度変化未補正 |

**Wand 用途の Yaw 影響**: `directionToCondition()` で全方向に `euler_tol[2] = 180°` を設定 → **Yaw を判定に使わない** → ドリフト影響ゼロ。SF プロファイル (shoryuken/tatsumaki) も同様。

### stream rate 別の負荷シナリオ

USB Serial 115200 baud の実効スループット = **~11.5 KB/s** (8N1 で 10 bits/byte)。
sensor packet 平均サイズ = **~200 bytes** (JSON で type/t/ax/ay/az/gx/gy/gz/pitch/roll/yaw/qw/qx/qy/qz/btn)。

| stream rate | M5C 帯域使用 | M5C 負荷 | Web 再 render | Web 負荷 | 体感 |
|-------------|-------------|----------|---------------|----------|------|
| **0 Hz (停止)** | 0 KB/s (0%) | 100Hz 内部のみ | 0 Hz | 0 Hz | 軽快 |
| **20 Hz** | 4 KB/s (35%) | 100Hz | 20 Hz | 軽 | 滑らか |
| **50 Hz (デフォルト)** | 10 KB/s (**87%**) | 100Hz | 50 Hz | **★★★ 重** | 重さ感じる |
| **100 Hz** | 20 KB/s (**174% NG**) | 100Hz | 100 Hz | 大幅オーバー | 帯域オーバーで取りこぼし |

### 改善案 (効果順)

| 対象 | 現状 | 改善案 | 期待効果 | 実装コスト |
|------|------|--------|----------|-----------|
| **USB Serial baud** | 115200 | **921600** に昇格 | 帯域 8 倍、100Hz 余裕 | 30 分 (FW + Web 同時変更) |
| **stream rate デフォルト** | 50 Hz | **20 Hz** | 帯域・Web 両方軽量、3D は RAF 60fps で滑らか維持 | 1 行 |
| **React `setSensor` throttle** | 毎 packet | **10 Hz** 間引き | App 再 render 1/5 に | 30 分 (sensor は ref に分離、表示用は throttle) |
| **PitchRollGrid draw rate cap** | 毎 packet | **30 Hz cap** | 2D 再描画コスト半減 | 15 分 |
| **Mahony Ki** | 0.0 | **0.005** | Yaw 動的 drift 補正、長期精度改善 | 1 行 |
| **Accel/Gyro FS** | ±8g / ±2000dps | 現状維持 (Wand 用途で十分) | — | — |
| **IMU 周期** | 100 Hz | 現状維持 (200Hz でも Wand には差なし) | — | — |

### Phase 5.36 候補: baud 昇格 + 三段 throttle

ユーザーの体感重さの実測結果次第:
1. **baud 921600 昇格** が最大効果。FW `[env:m5stick-c-v2]` の `-D SERIAL_BAUD=115200` を 921600 に、Web `SerialClient.js` のデフォルトも追従
2. stream rate デフォルトを 20Hz に変更
3. それでも重ければ Web 側 React state throttle (10Hz) + 2D draw cap (30Hz)

実装着手の前に、Phase 5.35 perf overlay の実測値で「どの数字がボトルネック」を確定させてから手を打つのが安全。

---

## Phase 5.39 — Hold Start End + 中間姿勢 (waypoints) 拡張 (新規)

### Context (なぜこの変更が必要か)

現状の HOLD_START_END モードは「開始姿勢で press → 終了姿勢で release」の 2 状態判定のみで、**中間の通過姿勢を強制できない**。例えば Wingardium Leviosa の swish & flick (DR→R→UR→D) を Hold モードで組みたい場合、開始と終了だけ条件指定すると「途中で別の振り方をしても release 時の姿勢が合えば成立」してしまい、誤動作が起きる。

ユーザーの要件 (本セッション):
1. **Hold Start End モードに中間姿勢 (0-2 個) を追加した新モードが必要**
2. 本来は絶対クォータニオン軌跡で判定したいが、Mahony フィルタの **Yaw ドリフト** (磁気センサ無し、0.1-1°/分の累積) のため見送り
3. しかし **「ボタン押下時点を基準とした相対クォータニオン軌跡」なら Yaw ドリフトの影響を回避できる** ことに気づき、検討対象に追加
4. Euler のジンバルロック (Phase 5.38 で Pitch のみ判定 OFF 可能化済) も別の回避手段として継続
5. **中間姿勢は最重要要素** だが、各 state では既存 Condition (button + posture + accel + gyro の AND) を維持し、加速度ピーク併用等も可能

ユーザー回答 (AskUserQuestion):
- 判定基準: **絶対 Euler / 相対 Quaternion の両方併存**
- 中間姿勢の数: **0-2 個可変** (既存 `State states[4]` 範囲内、FW 配列拡張不要)
- ref 起点: 「検討する」→ **最良案: state[0] match の瞬間に q_ref 自動取得** (button-driven / posture-driven の切替は state[0].Condition の組み方で自動決定、別フィールド不要)

### 用語の定義 (★ ユーザー明示確定 ★、地磁気との混同回避)

このプロジェクトでは「絶対 / 相対」を以下のように定義する。**地磁気による絶対方位 (北極基準) ではない** ことに注意:

| 用語 | 定義 | 基準時点 | 例 |
|------|------|---------|-----|
| **絶対 Euler / 絶対 Quaternion** | **Mahony フィルタ起動時 (またはキャリブレーション実行時) を基準とした姿勢** | デバイス起動 / `calibrate.simple` 実行 | Mahony 出力 `sensor.qw/qx/qy/qz` をそのまま使う |
| **相対 Quaternion** | **ある明示的トリガ (ボタン押下、開始姿勢一致等) から見た相対姿勢変化** | rule の state[0] match の瞬間 (q_ref として保存) | `q_rel = quat_conj(q_ref) ⊗ q_current` |

地磁気を使わない理由 (ユーザー方針):
- **屋内では磁気擾乱 (鉄筋、家電、PC モニタ) で 9 軸 IMU でも誤差大きく実用に堪えない場合がある**
- 屋外でも、磁北・真北の偏角や局所磁気異常で不安定
- 現 HW の MPU6886 は 6 軸 (磁気センサなし)、将来採用候補の LSM6DSV16X / BMI270 も 6 軸が主流。9 軸の ICM-20948 / BNO086 は採用しても **magnetometer は OFF or 補助のみ** で使う方針

これに伴う「絶対」表現の真の意味:
- **Roll / Pitch は重力ベクトル (Mahony の加速度補正) で安定**して取れる — 起動後ずっと信頼可能
- **Yaw は Mahony 起動時を 0 として、以降ジャイロ積分のみで更新** — 累積誤差 (drift) が 0.1-1°/分で蓄積、長時間使用で 10° 以上ずれる
- → 「絶対 Yaw」は実用上「起動からの経過時間に応じてドリフトする値」、長期信頼性なし
- → これを補うのが「**相対モード**」で、ボタン押下時に q_ref をリセットすれば過去の drift を無視できる

絶対モードが使える場面:
- **起動直後〜数分以内** の短期セッション (drift がまだ小さい)
- **Yaw を判定軸から外す** (Phase 5.38、`postureUseYaw = false`) → Roll/Pitch だけで判定すれば drift 無関係
- **Pitch / Roll だけのジェスチャ** (上下振り、左右傾け) — Mahony の加速度補正で安定

絶対モードが使えない場面:
- 長時間連続使用 (例: PC ゲームで数時間プレイ後)
- Yaw を判定軸に含むジェスチャ (左右スワイプを Yaw で取る等)
- → このとき**相対モード必須**

#### 開始時のキャリブレーション (q_init / q_baseline)

ユーザーが「起動時やキャリブレーション時を基準」と言及した通り、絶対モードでも実用上は **「起動直後に水平静止でキャリブレーション → そこを基準姿勢にリセット」** という工程を経る:

- 既存 FW の `calibrate.simple` で gyro/accel bias を取得 + Mahony 初期姿勢を更新
- もしくは `q.init` コマンドで現在姿勢を基準にする (rule に対する q_baseline) — 既存 FW に存在する場合
- これにより、「絶対 Euler」と言っても**実質的には「キャリブレーション基準からの相対」** であり、ただし基準は rule 単位ではなくデバイス単位 (1 回だけ)

→ 結論: 「絶対」と「相対」は **「基準を取り直す頻度」の違い** に過ぎない:
- 絶対: デバイス起動 / キャリブレーション時に 1 回 (drift する)
- 相対: rule の state[0] enter のたびに毎回取り直す (drift なし)

### 判定アーキテクチャの選択 (FSM vs DTW vs ML、Kano/WB 比較)

ユーザーからの問い: **「状態遷移と機械学習判定のどちらがよいのか。Kano は何を使っていたのか？」**

#### Web 調査結果サマリ (2026/05、Sources は末尾)

| 製品 / 実装 | 判定方式 | 学習データ | 開始トリガ | 呪文数 |
|------------|---------|-----------|----------|-------|
| **Kano Coding Wand (2018-2023)** | **機械学習** (詳細非公開、特許 US20210165506A1) | 数千の学習済みジェスチャ (公式) | ボタンホールド + 画面カーソルトレイル | 数十 (非公開) |
| **Warner Bros Magic Caster (2023)** | **DTW** (公式仕様 + 逆工学で確認) | 1-3 振りで呪文テンプレート保存 | 静止 (重力のみ) で開始・終了検出 | 50+ |
| **Hogwarts-Legacy-Wand (OSS、Kano ハック)** | **8 方向 状態遷移 (moosegesture)** | 不要 (方向シーケンス記述) | ボタン | 30+ |
| **MagicWand ESP32+MPU6050 (OSS)** | **DTW** (ESP32 で動作実証) | 1 振り | ボタン | カスタム |
| **TensorFlow Lite Micro magic_wand** | **CNN 量子化** | 100-500 サンプル | 加速度ピーク | 3-5 |

#### 各判定アーキテクチャの詳細比較・解説

##### 詳細比較表 (10 軸評価、★ = Burst Motion 視点での優位性)

| 評価軸 | FSM (状態遷移) | DTW (動的時間正規化) | HMM (隠れマルコフ) | ML 浅層 (RF/SVM) | ML 深層 (CNN/LSTM) | HW アクセラ (LSM MLC) |
|--------|----------------|----------------------|--------------------|-------------------|--------------------|-----------------------|
| **動作原理** | 状態遷移グラフ。各状態の条件式 (if-else) で次状態へ。決定論的 | 入力時系列と各テンプレート時系列を弾性整合、最小コスト経路長で類似度算出 | 状態遷移確率 + 出力確率の確率モデル。Viterbi で最尤系列推定 | 特徴量 (mean/std/FFT 等) を抽出して決定木 / 超平面で分類 | 生時系列を NN に入力、畳み込み/再帰でパターン抽出して分類 | IMU チップ内蔵の decision tree (8 本) で IMU が直接分類結果を出力 |
| **入力データ** | 各 tick の姿勢 (Euler / Quat) + ボタン + 加速度のスナップショット | 時系列軌跡 (例: 加速度 / quat 全フレーム) | 同上、シーケンス化 | 統計的特徴量ベクトル (各次元 32-128 個) | 生波形 (Time × Channel テンソル) | 加速度 / ジャイロの統計特徴量 (IMU 内蔵 FSM が抽出) |
| **学習データ** | **不要** ★★★ (ルールを設計者/ユーザーが直接記述) | 1-3 サンプル ★★ (各呪文 1 回振らせて記録) | 20-50 サンプル/クラス | 50-100 サンプル/クラス | 100-500 サンプル/クラス、データ拡張で増 | 20-50 サンプル + ST Unico-GUI で学習 |
| **モデルサイズ** | rule あたり ~200B (RAM) ★★★ | テンプレートあたり数 KB | 数 KB-数十 KB | 数 KB-数十 KB | 100 KB-数 MB | IMU 内蔵 (MCU 0B) |
| **判定遅延 (motion 完了→結果)** | **数 ms (tick 直後)** ★★★ | 10-50 ms | 数十 ms | 数 ms | 50-200 ms | **1-2 ms** ★★★ |
| **CPU 負荷 (ESP32 100Hz)** | 1% 未満 ★★★ | 5-15% | 5-10% | 1-3% | 30-80% (S3 / PSRAM 推奨) | 0% (IMU 内処理) ★★★ |
| **時間方向ばらつき耐性** | △ (waypoint タイムアウト窓で吸収) | ★★★ (DTW の本領) | ★★★ | ★★ (特徴量に時間情報なし) | ★★★ | ★★ |
| **ユーザー追加性** (新呪文/ジェスチャ登録) | ★★★ (Web UI で姿勢キャプチャ → 即追加) | ★★★ (1 回振らせる UX) | ★ (学習ツール必要) | ★ (CSV 投入 → 学習) | ★ (データ収集 → 再学習 → 再 flash) | △ (ST Unico-GUI、再学習で UCF 再書込) |
| **透明性 (なぜその結果か説明できるか)** | ★★★ (ルールが可視) | ★★ (テンプレートマッチング過程が見える) | △ (確率遷移、半透明) | △ (決定木は見えるが特徴量抽象) | × (ブラックボックス) | × (UCF はブラックボックス) |
| **誤検出率 (false positive)** | 中 (中間 waypoint + cooldown で抑制) | 低 | 低 | 中 | 低 | 中 |
| **取りこぼし率 (false negative)** | 中 (条件厳しめだと取りこぼし) | 低 | 低 | 中 | 低 | 中 |
| **Burst Motion 適合度** | ★★★ MVP 最適 | ★★ Phase 6+ で追加候補 | × | ★ | ★ (S3 / PSRAM HW 限定) | ★★ HW 変更前提で将来 |
| **代表実装** | Hogwarts-Legacy-Wand、Burst Motion 現状 | WB Magic Caster Wand、flooxo/MagicWand-Gesture-recognition | Kinect SDK | Edge Impulse + ESP32 | TFLite magic_wand (CNN)、Kano Wand 推定 | STMems-machine-learning-core |

##### 各手法の解説 (200 語以内 / 手法)

###### 1. FSM (Finite State Machine、状態遷移)
**原理**: 状態 (state) と遷移条件 (transition) のグラフ。Burst Motion の `State states[4]` + `evaluateRule()` の if-else 評価 (TriggerEngine.cpp) がそのまま該当。状態 = 姿勢 waypoint + Condition (button/accel/gyro)、遷移 = 条件成立で次状態へ進む。

**Burst Motion 視点での強み**:
- **ルールが完全に可視** — ユーザーが Web UI で各 waypoint の Roll/Pitch を直接編集できる
- **学習データ不要** — 「ボタン押下時に姿勢キャプチャ → 保存」だけで新ルール完成
- **超低遅延** — 100Hz tick で if-else 数回、ML より 1-2 桁高速
- **既存資産そのまま** — Phase 5.33 SEQUENCE 機構を流用、FW 拡張最小

**弱み**: 時間方向のばらつき (例: 同じ振りでも 200ms と 400ms で完了) に弱い。これは `max_dwell_ms` / `min_dwell_ms` でタイムアウト窓を設けて吸収。Wand 用途は「ゆっくり振る vs 急ぐ」の許容範囲が広いので実用上問題なし。

**結論**: **Burst Motion MVP 採用、Phase 5.39 で中間姿勢拡張**。

###### 2. DTW (Dynamic Time Warping、動的時間正規化)
**原理**: 2 つの時系列 X[1..M] と Y[1..N] の各要素ペアの距離を計算した M×N コスト行列を、対角に近い経路で最小コスト累積する動的計画法。同じ動作を速度が違って振っても、テンプレートとの最適なアライメントで類似度を測れる。WB Magic Caster Wand と OSS の flooxo/MagicWand-Gesture-recognition が採用。

**Burst Motion 視点での強み**:
- **時間方向のばらつきに最強** — Wand 用途で「速く振っても遅く振っても同じ呪文」が成立
- **1-3 振りでテンプレート登録可能** — Kano/WB と同じ「振って覚えさせる」UX
- **ESP32 で動作実証あり** (flooxo の OSS、MPU6050 + ESP32)
- 50+ 呪文を WB が実装している → スケールする

**弱み**: 計算量 O(M·N)、典型 M=N=200 で 40,000 演算 / 認識 → 100Hz 連続評価は重い (ジェスチャ終了検出で 1 回だけ走らせる設計が標準)。学習データ収集 UX が必要 (ユーザーが振ってあげる + ラベル付け)。テンプレート修正は再学習が必要。

**結論**: **Phase 6+ 候補**。WB / Kano と同じ UX を目指す場合に追加検討。

###### 3. HMM (Hidden Markov Model、隠れマルコフモデル)
**原理**: 観測系列の背後に隠れた状態系列があると仮定し、状態遷移確率 + 出力確率の積で系列尤度を計算。Viterbi 復号で最尤状態系列を求める。手書き文字認識・音声認識で実績、Kinect の手話認識でも採用された時期がある。

**Burst Motion 視点での強み**: 時間方向のばらつき耐性、雑音耐性 (確率モデルなので)。

**弱み**: 学習データが 20-50 サンプル必要 (Baum-Welch アルゴリズム)、ハイパーパラメータ調整 (状態数、混合数) が職人技、ユーザー追加が困難 (再学習必要)、透明性低い (確率遷移はユーザーに見せにくい)。

**結論**: Burst Motion 不採用。FSM で十分代替可、DTW より UX で劣る。

###### 4. ML 浅層 (Random Forest / SVM / 決定木)
**原理**: 時系列から特徴量 (mean / std / max / min / FFT / 自己相関 等) を計算してベクトル化、決定木の集合や超平面で多クラス分類。Edge Impulse が ESP32 向けに自動特徴量抽出 + 浅層学習を提供。

**Burst Motion 視点での強み**: 計算軽量 (推論数 ms)、量産後再学習不要、Edge Impulse の Web UI で開発容易。

**弱み**: **特徴量設計依存** — どの特徴量を取るかで精度が劇的に変わる、職人技。時間情報を直接保持しないので「振りの順序」を捉えにくい (例: 上→右 と 右→上 を区別困難)。学習データ 50-100 サンプル/クラス必要。

**結論**: Burst Motion 不採用。同程度の UX なら DTW の方が時間順序を素直に扱える。

###### 5. ML 深層 (CNN / LSTM / Transformer)
**原理**: 生波形 (加速度 / ジャイロの time × channel テンソル) を NN に入力、Conv1D / LSTM / Attention で時間方向のパターンを自動抽出して softmax で分類。TFLite Micro の petewarden/magic_wand が CNN で 5 呪文認識を実証。

**Burst Motion 視点での強み**: 高精度 (Kano Wand の正体である可能性)、時間方向耐性最高、ノイズ耐性高。

**弱み**:
- **モデルサイズ 100KB-数 MB** → ESP32 PICO D4 (520KB RAM) では実用困難、S3 + PSRAM 必須
- 推論遅延 50-200 ms → Wand リアルタイム性に厳しい
- 学習データ 100-500 サンプル/クラス → 個人開発では集めにくい
- 新呪文追加は再学習 + 再 flash → ユーザーが気軽に増やせない

**結論**: Burst Motion 不採用 (HW 制約)。BOM $25 制約と矛盾。Phase 6+ で量産版 LSM6DSV16X + S3 + PSRAM に移行できれば検討余地あり。

###### 6. HW アクセラレーション (LSM6DSV16X MLC)
**原理**: STMicro の LSM6DSV16X IMU 内蔵の Machine Learning Core (8 本の決定木) で、IMU 自身が分類結果を割り込み出力。MCU の関与は割り込み受信のみ、超低消費 (~3.5µA)。

**Burst Motion 視点での強み**: **CPU 負荷ゼロ + 1-2 ms 超低遅延 + 超低消費電力**、Phase 5 の量産候補 IMU として既に挙がっている。

**弱み**:
- **現 HW (MPU6886) では使用不可** — IMU 置き換え必須
- 決定木 8 本制限 → 多クラス分類はクラス数制限あり
- 学習に ST Unico-GUI 必要、UCF ファイル経由で書込み → ユーザー追加困難
- 透明性低い (UCF はブラックボックス)

**結論**: 量産 v1 (LSM6DSV16X 採用) で **「典型ジェスチャ 5-8 個を MLC で IMU 側オフロード、追加カスタムは FSM」** のハイブリッド構成が理想。Phase 6+ HW 移行後に検討。

##### Burst Motion の判定アーキテクチャ選定ロードマップ

| Phase | アーキテクチャ | 姿勢表現 | 用途 |
|-------|------------|--------|------|
| Phase 1-5 (現状-MVP) | **FSM** | 絶対 Euler (起動基準) | SF / プレゼン / 単純 1 動作 |
| **Phase 5.39 (本計画)** | **FSM + 中間姿勢拡張** | **絶対 Euler / 相対 Quat (rule 選択)** | Wand 呪文、両手構え保持、複雑コンボ |
| Phase 6+ (HW 移行後) | **FSM + DTW (ハイブリッド)** | 相対 Quat 主体 | ユーザーが「振って覚えさせる」呪文追加、WB 互換 UX |
| Phase 7+ (量産 LSM6DSV16X 後) | **FSM + DTW + MLC ハードアクセラ** | 相対 Quat | 量産製品、典型ジェスチャはチップ内処理、カスタムは MCU 側 |
| Phase 9+ (要 S3 + PSRAM) | **+ TFLite (オプション)** | 生波形 + 相対 Quat | プレミアム版で高精度認識 |

→ Phase 5.39 は **「FSM × 相対」象限を埋める** ことで、業界標準 (Kano/WB の相対方式) と整合しつつ、実装コストを最小化する戦略。

#### 手法別比較 (Burst Motion 文脈)

| 手法 | 判定遅延 | RAM | 学習要否 | ユーザー追加性 | 時間ばらつき耐性 | ESP32 互換 |
|------|---------|-----|---------|-------------|---------------|-----------|
| **FSM (状態遷移)** | 数 ms | 低 (rule あたり ~200B) | 不要 | ★★★ (1 ボタンでルール記述) | △ (waypoint タイムアウトで吸収) | ◎ |
| **DTW** | 10-50 ms | 中 (テンプレート × サンプル長) | 1-3 振り | ★★★ (振って覚えさせる) | ★★★ (時間方向弾性) | ◎ |
| **HMM** | 数十 ms | 中 | 20-50 サンプル | ★ (学習ツール必要) | ★★★ | ○ |
| **TFLite CNN/LSTM** | 50-200 ms | 高 (300KB モデル) | 100-500 サンプル | ★ (再学習・再コンパイル) | ★★★ | △ (S3 / PSRAM 必須) |
| **LSM6DSV16X MLC** | 1-2 ms | 極低 | 20-50 | △ (ST Unico-GUI) | ★★ | × (MPU6886 非互換) |

#### Burst Motion の選択: FSM を主軸、DTW は将来拡張

**MVP (Phase 5.39): FSM 継続採用** の理由:

1. **既存 SEQUENCE 機構 (state[4]) で実装ほぼ完了** — Phase 5.39 で追加するのは「中間姿勢拡張 + 相対 Quat 判定」のみ、判定エンジン自体は変更不要
2. **学習データ不要** — Wand 呪文を「ボタン 1 回押し → 姿勢キャプチャ × N → 保存」で登録可能、ML のような数百サンプル収集が不要
3. **遅延ゼロ** — 100Hz tick で if-else 評価、Kano の ML (数十 ms) や DTW (10-50 ms) より低レイテンシ
4. **小型 IMU (MPU6886) + 小型 MCU (ESP32 PICO D4 520KB RAM)** に最適 — TFLite は S3 + PSRAM が現実的
5. **ユーザー直感的** — 「Roll +30°、Pitch -15°」と数値で確認できる (ML は内部状態がブラックボックス)
6. **Hogwarts-Legacy-Wand と同じ路線** — OSS コミュニティで実証済み、Kano ハック資産 (8 方向呪文定義) を取り込める

**「特に中間は姿勢を重要視」というユーザー方針との整合**:
- FSM の waypoint = 「通過すべき姿勢」を直接記述できる
- ML だと「学習データに偏った特徴を内部抽出」して結果的に姿勢に近い判定にはなるが、**設計時に「ここを通せ」と明示できない**
- DTW は時系列全体マッチングなので、中間状態を特に重要視する意味では中間的

**Phase 6+ で DTW 追加検討の余地**:
- ユーザーが「自分で振って覚えさせる」UX を強く望む場合 (Kano/WB 互換)
- 既存 OSS [flooxo/MagicWand-Gesture-recognition](https://github.com/flooxo/MagicWand-Gesture-recognition) を参考に C++ 移植、ESP32 + MPU6050 で動作実証済
- MVP では実装しない (FSM で 95% のユースケースは充足)

#### Kano / WB の方式を「絶対/相対」の観点で再評価 (★ ユーザー定義に基づく再考察 ★)

ユーザー定義に基づく「絶対 = Mahony 起動基準」「相対 = トリガ基準」を当てはめると、両社の方式は以下のように分解できる:

**Kano Coding Wand: 相対 (トリガ基準) + 機械学習 (★ 確実性に注意、ML 入力詳細は推定 ★)**

特許 US20210165506A1 (発行番号 US11301059B2) の明確な記述:
> "Origin reset function: pressing the button returns the cursor to the center, redefining the current orientation as the 'origin'."

(「**原点リセット機能：ボタンでカーソルを中央に戻し、現在の向きを『原点』として再定義する**」)

この特許文から確実に言えるのは:
- **「ボタン押下で原点を再定義する」概念が特許化されている** こと
- 「向き (orientation)」を再定義しているので、内部状態の何らかの基準姿勢を更新している

ただし、**まだ確認しきれていない点**:
- **その「原点」が ML モデルへの入力前処理 (= q_ref 取得 → 相対 quat 入力) なのか**、それとも単に画面表示用のカーソル原点リセットだけで、ML 入力は加速度生波形 or 絶対 quat なのか
- 機械学習モデルの具体的種別 (CNN / RNN / kNN / Random Forest / DTW)
- 公開特許文だけでは内部実装は確定できない (バックグラウンドで追加調査中)

**バックグラウンド調査結果 (2026-05、深掘り完了)**:

特許 US11301059B2 の原文を逐条確認した結果:
> "The system is operable to enable the **cursor position** to be reset to the origin in response to a reset input via gesture input means"

**確認できた事実** (★ 当初推定と異なる):
- **"origin reset" は主に「画面 UI のカーソル位置」のリセット** であり、ML モデルへの入力前処理として相対 quaternion を計算する記述は特許文中になし
- **ML 入力ベクトル形式は特許に明示なし**
- Hackaday 記事 (Jennifer Wang による Kano ライク実装の解説、2018) によれば、**scikit-learn ベースの古典 ML (kNN / SVM 相当の浅いモデル)** で、**入力は「加速度 + ジャイロの生時系列データ」**
- **深層学習 (CNN/RNN) の証拠なし**、特許 Claim にも "neural network" の語はなく "classifier" とのみ
- **9 軸 IMU をフル使用** (3 軸加速度 + 3 軸ジャイロ + 3 軸磁気) 、200Hz サンプリング

→ **Kano の実際の方式は「絶対加速度+ジャイロ生波形 + 古典 ML (kNN/SVM)」と推定の方が確度高** (画面表示の原点リセットは確定だが、ML 入力レベルでは絶対波形をそのまま投入している可能性が高い)

**この発見の含意**:
- 当初の plan の「Kano = 相対方式 + ML」記述は **画面表示レイヤでは正しいが、判定アルゴリズムレベルでは「絶対波形 + ML」が実態に近い**
- つまり、相対方式 (= q_ref で前処理) は「業界標準」というほどではなく、**Burst Motion Phase 5.39 で導入する相対 Quaternion 軌跡判定は、業界で広く実装されていない領域**
- 一方、WB Magic Caster Wand は **DTW + テンプレート照合**で、テンプレートが「開始時点からの時系列軌跡」のため**事実上の相対方式** (これは plan の元評価通り)
- → Kano (絶対+ML)、WB (相対+DTW)、Burst Motion Phase 5.39 (相対+FSM) はそれぞれ異なる象限を占めている

#### 6 象限マトリクス 再修正版 (バックグラウンド調査結果反映)

| 判定アーキテクチャ \ 姿勢表現 | 絶対 (起動基準 / 生波形) | 相対 (トリガ基準) |
|------|------|------|
| **FSM (状態遷移)** | Burst Motion 旧 SF サンプル、Hogwarts-Legacy-Wand 8 方向 (Kano ワンドのデジタル方向出力を使用、Kano 内部は別) | **Burst Motion Phase 5.39 新モード** ★ 業界で空白だった象限 |
| **DTW** | 学術論文の理論実装 | **WB Magic Caster Wand**、flooxo/MagicWand-Gesture-recognition |
| **ML (kNN/SVM/RF/HMM/CNN)** | **Kano Coding Wand** (推定、scikit-learn ベース、生波形入力)、TFLite magic_wand | (報告例少、相対化のメリット小 = ML が学習で吸収するため不要との見方) |

→ Burst Motion Phase 5.39 は **「FSM × 相対」という業界未開拓の象限** を埋める = **特許リスクが相対的に低い**位置取り (Kano の絶対+ML、WB の相対+DTW のいずれとも明確に異なる)

**Warner Bros Magic Caster Wand: 相対 (静止検出基準) + DTW**

公式仕様と逆工学から:
> "Gestures start and end with stillness (gravity-only acceleration). DTW computes similarity to stored templates during motion."

(「**ジェスチャは静止 (重力のみの加速度) で開始・終了。動作中に DTW で保存テンプレートとの相似度を計算**」)

つまり WB は:
- **静止検出 (= 重力ベクトルのみが見える状態) でジェスチャ開始時点を確定** → これが暗黙の q_ref
- DTW テンプレートは「開始時点からの時系列軌跡」として記録 → やはり相対方式
- 9 軸 IMU を持つが、ジェスチャ単位で開始時点を取り直すなら絶対方位への依存はない
- 判定エンジン: DTW (50+ 呪文、テンプレート照合)

→ **WB = 相対方式 + DTW**

**両社が「相対」を選んだ理由 (技術的合理性)**

| 理由 | 補足 |
|------|------|
| 屋内磁気擾乱で 9 軸 IMU でも絶対 Yaw が信用できない | 鉄筋・家電・PC モニタの近傍で 30°+ 誤差 |
| ジェスチャは「振りの軌跡形状」が本質 | 絶対方位 (北はどっち) はジェスチャ判定に無関係 |
| 持ち方への非依存性 | どの向きで持っても同じジェスチャが成立 → UX 良好 |
| 製品互換性 | 同じ呪文を全ユーザー / 全ハードで成立させる |

→ **ユーザー定義の「相対クォータニオン」方式は業界標準と完全一致**。Burst Motion Phase 5.39 で相対モードを採用することは、Kano / WB と同じ設計哲学に立つことを意味する。

#### 判定アーキテクチャ × 姿勢表現の 6 象限マトリクス

ユーザーの定義を反映した完全な比較:

| 判定アーキテクチャ \ 姿勢表現 | 絶対 (Mahony 起動基準) | 相対 (トリガ基準) |
|------|------|------|
| **FSM (状態遷移)** | Burst Motion 旧 SF サンプル (基本姿勢 R+90/P0 ハードコード)、Hogwarts-Legacy-Wand 8 方向 | **Burst Motion Phase 5.39 新モード (推奨)**、未調査の OSS 例 |
| **DTW** | 学術論文の理論実装 | **WB Magic Caster Wand**、flooxo/MagicWand-Gesture-recognition |
| **ML (CNN/RF/HMM)** | TFLite magic_wand (加速度生波形) | **Kano Coding Wand** (ボタン押下を原点) |

→ **Burst Motion Phase 5.39 は「左下 (FSM × 相対)」を埋める** — 既存業界実装の中で最も実装コストが低く (FSM)、最も実用的 (相対方式)、最もホワイトボックス (FSM 由来)、最もユーザー追加しやすい (FSM × ルールベース) ポジションを取る。

#### Kano (ML) と Burst Motion (FSM) の哲学的差異

| 観点 | Kano | Burst Motion |
|------|------|--------------|
| 呪文追加 | 開発者が学習データ収集して再リリース | ユーザーが Web UI で姿勢キャプチャして即追加 |
| 認識精度 | 高 (ML の長所) | 中-高 (FSM + 中間 waypoint で十分) |
| 透明性 | ブラックボックス | 完全に可視 (Web UI で waypoint 表示) |
| 計算リソース | nRF52832 + クラウド学習バックエンド前提 | M5StickC 単体で完結、$25 BOM |
| カスタマイズ | 不可 (公式呪文のみ) | 完全自由 (任意の姿勢列で任意のキー出力) |

→ **Burst Motion は OSS / ユーザーカスタマイズ前提**なので、FSM のホワイトボックス性が決定的に重要。

#### Sources

- [US20210165506A1 - Kano Wand Patent (Alex Klein, 2022)](https://patents.google.com/patent/US20210165506A1/en)
- [Magic Wand Learns Spells Through Machine Learning And An IMU (Hackaday)](https://hackaday.com/2018/12/07/magic-wand-learns-spells-through-machine-learning-and-an-imu/)
- [Warner Bros. swiped our Harry Potter wand IP, says Kano (TechCrunch)](https://techcrunch.com/2023/01/27/warner-bros-swiped-our-harry-potter-wand-ip-says-kano/)
- [Harry Potter Kano Coding Kit employs nRF52832 (Nordic Semiconductor)](https://www.nordicsemi.com/Nordic-news/2018/10/Harry-Potter-Kano-Coding-Kit-employs-nRF52832-to-wirelessly-connect-magic-wand-with-app)
- [flooxo/MagicWand-Gesture-recognition (ESP32 DTW 実装)](https://github.com/flooxo/MagicWand-Gesture-recognition)
- [Thats-so-Mo/Hogwarts-Legacy-Wand (8 方向 FSM)](https://github.com/Thats-so-Mo/Hogwarts-Legacy-Wand)
- [petewarden/magic_wand (TFLite CNN)](https://github.com/petewarden/magic_wand)
- [MDPI Sensors 2016 - Quaternion-Based Gesture Recognition (96% 精度)](https://www.mdpi.com/1424-8220/16/5/605)
- [LSM6DSV16X MLC AN5804 (STMicroelectronics)](https://www.st.com/resource/en/application_note/an5804-lsm6dsv16x-machine-learning-core-stmicroelectronics.pdf)
- [Gesture Recognition Based on TFLite (Espressif Blog 2026)](https://developer.espressif.com/blog/2026/04/gesture-recognition-based-on-tflite/)

---

### 近傍判定アルゴリズム (★ 最重要 設計判断 ★)

ユーザーからの本質的な問い: **「絶対 Quat / 相対 Quat の場合、登録した姿勢の "近くになった" ことをどう判定するのか」**

クォータニオンは 4 次元値であり、「だいたい同じ姿勢」を判定するには数学的に妥当な距離測度が必要。3 つの候補:

#### 候補 A: Euler 軸別 tol (推奨デフォルト、Phase 5.38 と整合)

事前に **「target との差クォータニオン」を計算** してから Euler 抽出:
```
q_delta = quat_conj(q_target) ⊗ q_current   # 「target からの回転差」
(droll, dpitch, dyaw) = quaternion_to_ZYX_euler(q_delta)
match = |droll|  <= roll_tol  AND
        |dpitch| <= pitch_tol AND
        |dyaw|   <= yaw_tol
```

絶対 / 相対の使い分け:
- **絶対モード**: `q_target` = rule.posture.quat、`q_current` = sensor.quat
- **相対モード**: `q_target` = rule.posture.quat (ref 基準で記録)、`q_current` = `quat_conj(rule.q_ref) ⊗ sensor.quat` (現在姿勢を ref 基準に変換)

利点:
- Phase 5.38 の判定軸 ON/OFF (`euler_tol[i] = 180` で除外) と**完全互換** — 既存実装そのまま流用
- ユーザー UI 直感的 (「Roll 許容 ±20°、Pitch 許容 ±15°、Yaw 無視」)
- 軸別の tol 指定で「Roll だけ厳しく、Pitch は緩く」等の細かい制御
- 計算コスト: quat 積 (16 乗算) + 共役 (符号反転 3) + Euler 抽出 (asin/atan2 各 1) → 1 姿勢判定で 50-100 ns @ 240MHz ESP32

欠点:
- q_delta の Pitch が ±90° 近傍に来るとジンバルロックで Roll/Yaw が縮退 (絶対 Euler と同じ問題、ただし**「target との差」が 90° 級になるケースは稀**)

#### 候補 B: Quat 内積 cone 判定 (シンプル、軽量、軸別不可)

```
dot = q_target.w * q_current.w + q_target.x * q_current.x
    + q_target.y * q_current.y + q_target.z * q_current.z
match = |dot| >= cos(tol_angle / 2)    # |dot| で二重被覆対処 (q と -q が同じ回転)
```

数学的背景:
- 正規化クォータニオン 2 つの内積 = `cos(θ/2)` (θ = 2 姿勢間の最短測地距離角度)
- 閾値の例: tol=30° → cos(15°)≈0.966, tol=60° → cos(30°)≈0.866, tol=90° → cos(45°)≈0.707
- `|dot|` で絶対値を取るのは「q と -q は同じ回転を表す」二重被覆問題への対処

利点:
- **計算最速** (4 乗算 + 3 加算 + abs 1 回、Euler 比 10 倍速)
- **ジンバルロック完全無関係** — 4 次元での測地距離なので特異点なし
- 単一閾値で表現できる (UI シンプル)
- 学術論文での標準手法 (Markov chain ベース手法等で 96% 精度実証 — MDPI Sensors 2016)

欠点:
- **軸別判定が不可能** — 「Yaw だけ無視」「Roll だけ厳しく」が表現できない (内積は全軸を等重み)
- Phase 5.38 の判定軸 ON/OFF UI と整合しない (Yaw OFF 要求が反映できない)
- ユーザー UI 直感的でない (「許容 30°」の意味が「3 軸合計の測地距離」と説明必要)

#### 候補 C: 主軸方向ベクトル cone + twist tol (将来拡張)

クォータニオンを「forward vector (主軸の指す方向)」と「twist (主軸周りの捩じれ)」に分解:
```
forward_target = q_target ⊗ (0,0,1) ⊗ quat_conj(q_target)   # target が指す方向ベクトル
forward_current = q_current ⊗ (0,0,1) ⊗ quat_conj(q_current)
cone_dot = forward_target · forward_current
match_forward = cone_dot >= cos(cone_half_angle)

# twist 判定 (オプション): forward 軸周りの回転角を抽出して別 tol で比較
```

利点:
- 「**杖先端がどこを向いているか**」だけ判定可、捩じれは無視 (Wand UX に自然)
- ジンバルロック無関係 (ベクトル空間判定)
- forward と twist を別 tol にできる (一種の軸別判定)

欠点:
- 実装複雑 (主軸の選択 = どの軸を forward とするか、デバイス向きで変わる)
- MVP では過剰

#### 結論: MVP は A + B の 2 系統、判定方式を rule.judge_by で切替

| judge_by | アルゴリズム | tol 表現 | デフォルト |
|----------|-----------|---------|----------|
| `"euler"` (default) | 候補 A | `euler_tol[3]` (Roll/Pitch/Yaw 軸別 deg) | ✅ MVP デフォルト、Phase 5.38 と完全互換 |
| `"quat"` | 候補 B | `quat_tol_angle` (deg、単一値 → 内部で `cos(θ/2)` 閾値に変換) | オプション、シンプル判定用 |

**Web UI の判定セレクタ (既存)** で `Euler 角 (Roll/Pitch tol、推奨)` と `Quaternion 内積` を切替可能 — UI は変わらず、内部ロジックだけ拡張。

**相対モード (`posture_basis = relative`)** では、A も B も判定前に `q_current` を `quat_conj(q_ref) ⊗ q_current` で前処理 (ref 基準に変換)。target はもともと「ref からの相対」として rule に保存されている。

#### 計算コスト一覧 (1 姿勢判定あたり、ESP32 PICO D4 @ 240MHz、100Hz tick)

| 演算 | 時間 | 100Hz × N rule での負荷 |
|------|------|------------------------|
| 候補 A (相対 Euler): quat 積 + Euler 抽出 | ~80 ns | N=30 rule で 0.024 ms / tick (実用負荷以下) |
| 候補 B (相対 Quat dot): quat 積 + 内積 | ~30 ns | N=30 rule で 0.009 ms / tick |
| Phase 5.38 (絶対 Euler、現状) | ~10 ns | N=30 rule で 0.003 ms / tick |

→ **どちらも 100Hz tick の 1% も使わない、性能的に問題なし**

#### Mahony フィルタの Yaw ドリフトとの整合性

相対モードの利点が最大化されるのは **「ref を取得する瞬間に Yaw drift が完全リセットされる」** こと。つまり:
- 起動 30 分後 (Yaw 累積 ~10° drift) でも、state[0] enter で q_ref を取った瞬間に「以降の rel quat は drift ゼロからカウント」
- 1 モーション (~3 秒) 内の追加 drift は 0.05° 以下 → 判定 tol 15° に対して無視可能

これは「**絶対座標が信用できなくなった IMU でも、相対判定なら永遠に同じ精度で動作する**」ことを意味し、Wand 用途では決定的な優位。

---

### 設計サマリ

**新 ui_mode**: `"hold_with_waypoints"`

**内部表現** (既存 SEQUENCE 機構を再利用、FW 配列拡張なし):

| state | 役割 | match_condition | on_enter | on_exit |
|-------|------|-----------------|----------|---------|
| state[0] | 開始 | posture + button + accel + gyro + **静止検出** (AND/OR、ユーザー組合せ) | **AT_PRESS** (key) + 相対モード時 q_ref 取得 | AT_NONE |
| state[1..N-2] | 中間 (0-2 個) | posture が主、accel/gyro/button オプション | AT_NONE | AT_NONE |
| state[N-1] | 終了 | posture + 他オプション + **静止検出** (オプション) | **AT_RELEASE** (key) | AT_NONE |

#### 開始/終了トリガとして「静止検出」を追加 (ユーザー要件、WB 方式互換)

ユーザー方針: **「ボタントリガではなく、静止検出トリガも暗に入れる」**

WB Magic Caster Wand が採用していた「静止 (重力のみ) でジェスチャ開始・終了」を、Burst Motion でも各 state の Condition オプションに追加する。これにより:
- ボタン無しの杖タイプデバイスでも、自然な「振りはじめ・振りおわり」を検出可能
- Kano の「ボタン押下」、WB の「静止検出」、ユーザー独自の「姿勢条件」、いずれも統一的に表現できる

**静止検出 (stillness) の定義**:
```
stillness_detected = (|accel - 1g| < accel_threshold)
                  AND (|gyro| < gyro_threshold)
                  AND (上記が duration_ms 連続で成立)
```

**実装**: 既存 `Condition` 構造体にフィールド追加:
```cpp
struct Condition {
    // ... 既存 (posture, button, accel_abs_threshold, gyro_abs_threshold)
    bool     stillness_required;    // 静止検出を AND 条件に含めるか
    uint16_t stillness_window_ms;   // 連続静止が必要な時間 (default 200ms)
    uint8_t  stillness_accel_th_mg; // |accel - 1g| の許容差 (mg、default 100)
    uint8_t  stillness_gyro_th_dps; // |gyro| の許容差 (dps、default 5)
};
```

**FW 評価** (TriggerEngine.cpp の matchCondition 内に追加):
```cpp
if (cond.stillness_required) {
    static uint32_t still_since_ms = 0;  // rule 毎に持つべき、ここでは説明用
    float a_dev = abs(sqrt(ax*ax + ay*ay + az*az) - 9.81f);
    float g_dev = sqrt(gx*gx + gy*gy + gz*gz);
    bool still_now = (a_dev < cond.stillness_accel_th_mg * 0.00981f)
                  && (g_dev < cond.stillness_gyro_th_dps);
    if (still_now) {
        if (still_since_ms == 0) still_since_ms = now;
        if (now - still_since_ms < cond.stillness_window_ms) return false;
    } else {
        still_since_ms = 0;
        return false;
    }
}
```

**ユースケースマトリクス**: state[0] の Condition 組合せで、Kano / WB / Hybrid を表現可能

| 開始トリガ | Condition の組合せ | UX |
|----------|------------------|----|
| **ボタン押下** (Kano 方式) | `button=Btn3, button_state=pressed` | Btn3 押下時に q_ref 取得、振り開始 |
| **静止検出** (WB 方式) | `stillness_required=true, stillness_window_ms=300` | 静止 300ms で q_ref 取得、振り開始 |
| **姿勢一致** | `posture={Roll=0,tol=15}` | 水平にした瞬間 q_ref 取得 |
| **静止 + 姿勢** (推奨 hybrid) | `stillness_required + posture` | 静止しつつ特定姿勢の時のみ起動 (誤起動最小) |
| **ボタン + 静止** | `button + stillness_required` | 確実性重視 (誤起動ほぼゼロ) |

→ FW は「Condition の AND 評価」のままで、ユーザーが UI で**フィールドを組合せるだけで全方式を実現**できる (1 つの汎用エンジンで Kano 方式も WB 方式も両対応)。

**Web UI 追加**:
- 既存「姿勢条件」「ボタン条件」「加速度条件」セクションと同じ並びに「**静止検出 (任意)**」セクションを追加 (折り畳み可)
  ```
  ☑ 静止検出 を開始条件に含める
    accel 許容: [100] mg   gyro 許容: [5] dps   連続: [200] ms
  ```
- 既存の hold_start_end / hold_with_waypoints / oneshot / sequence の全モードで利用可
- 終了側 (state[N-1]) にも同じセクションを表示 (hold_with_waypoints / hold_start_end のみ)

- `loop = false` (一方通行、最終 state 到達後 idle 復帰、cooldown_ms 適用)
- `states_count`: 2-4 (中間 0 個 = 既存 hold_start_end と完全等価、中間 2 個で states[4] フル使用)
- 各 state の `max_dwell_ms` (タイムアウト) / `min_dwell_ms` (誤検出防止滞在最小時間) は新モードで活用

**判定基準切替** (新フィールド `r.posture_basis`):

| basis | 動作 | 用途 |
|-------|------|------|
| `absolute` (default) | 各 state の posture.euler と現在 sensor.euler を直接 tol 比較 (現状互換)。Phase 5.38 の Roll/Pitch/Yaw 軸別 ON/OFF と組合せて、ジンバルロック / Yaw ドリフトを部分回避 | 既存サンプル (波動拳の R+90/P0 規約等)、絶対座標が意味を持つ用途 |
| `relative` | state[0] enter の瞬間に `q_ref = q_current` を保存。各 state の posture.euler は **「ref からの相対 Euler オフセット」** として解釈し、`q_rel = q_ref* ⊗ q_current` から Euler 抽出して tol 比較 | Yaw ドリフト無関係、持ち方自由、Kano ワンド方式、ハリポタ呪文の純ジェスチャ判定 |

**相対モードの利点**:
- **Yaw ドリフトに完全免疫** — ref を毎回取り直すため累積ドリフトを引きずらない
- **絶対姿勢非依存** — どの向きで持っても同じジェスチャが成立 (LCD 左持ち / LCD 右持ち / 横持ち どれでも OK)
- **ジンバルロック軽減** — 開始姿勢を「ref」とすれば、相対 Euler の Pitch 領域が ±90° に達しにくくなる (絶対 Pitch=-80° で開始しても、相対は 0° スタート)

**相対モードの欠点**:
- state[0] enter の瞬間に IMU ノイズが乗っていると q_ref が不正確 → **state[0] enter 後 10ms 程度の移動平均で q_ref を確定** すれば緩和可能
- 絶対座標が意味を持つ用途 (「常に水平を保つ」「机に対して垂直」等) には不適 → absolute モード使用

### データモデル変更

```cpp
// types.hpp
struct Rule {
  // ... 既存フィールド
  uint8_t posture_basis;   // 0=ABS_EULER (default), 1=REL_QUAT (新規)

  // runtime (RAM のみ、永続化不要)
  float q_ref[4];          // posture_basis=REL_QUAT 時、state[0] enter で取得
  bool  q_ref_valid;       // q_ref が確定済か
};

enum PostureBasis : uint8_t {
  PB_ABSOLUTE_EULER = 0,
  PB_RELATIVE_QUAT  = 1,
};
```

### FW 変更

| ファイル | 変更内容 |
|---------|---------|
| `src/core/types.hpp` | `Rule::posture_basis` (uint8_t)、`Rule::q_ref[4]`、`Rule::q_ref_valid` (bool) 追加。`PostureBasis` enum 追加 |
| `src/core/TriggerEngine.cpp` | `evaluateRule()` 内: state[0] match で transition 時、`posture_basis == PB_RELATIVE_QUAT` なら `q_ref = sensor.quat`、`q_ref_valid = true` を保存。state リセット時に `q_ref_valid = false`。新ヘルパー `matchPostureRelative()` を追加: `q_rel = quat_conj(q_ref) * sensor.quat` を計算 → ZYX Euler 抽出 → posture.euler との差を tol 比較 (既存 Phase 5.38 の軸別 ON/OFF と整合) |
| `src/core/TriggerEngine.cpp` | `matchCondition()`: posture 判定部分で `posture_basis` を見て `matchPostureAbsolute()` / `matchPostureRelative()` を分岐呼出し |
| `src/core/Profile.hpp` | `serializeRule()` / `deserializeRule()` に `posture_basis` を追加。デフォルト 0 (absolute) で後方互換 |
| `src/main_v2.cpp` | rule.add ハンドラに `ui_mode == "hold_with_waypoints"` 分岐を追加 (~80 行)。受け付ける JSON: `r.start_posture`, `r.mid_postures[]` (0-2 要素), `r.end_posture`, `r.posture_basis` ("absolute" or "relative")、`r.key`, `r.end_key` (オプション)。states[] への自動展開: state[0]=start (AT_PRESS), state[1..N-2]=mid (AT_NONE), state[N-1]=end (AT_RELEASE) |

**FW 拡張は最小**: TriggerEngine の状態遷移ロジック自体は既存 SEQUENCE 機構をそのまま再利用、追加するのは「相対 posture 判定関数」と「q_ref のスナップショット処理」のみ。

### Web 変更

| 箇所 | 変更内容 |
|------|---------|
| `app.js:91` 周辺 | `ruleMode` UI ドロップダウンに `<option value="hold_with_waypoints">HOLD with Waypoints (開始→中間→終了)</option>` 追加 |
| `app.js:109-119` 周辺 | 新 state: `midPostures` (配列、最大 2)、`postureBasis` ('absolute' \| 'relative'、default 'absolute') |
| `app.js:876-893` (capture 周辺) | `captureMidPosture(index)`, `clearMidPosture(index)`, `addMidPostureSlot()`, `removeMidPostureSlot()` 関数追加。`buildPostureTol()` (Phase 5.38) を全姿勢キャプチャで流用 (判定軸選択は rule 全体で統一) |
| `app.js:970-983` (handleAddRule) | `ruleMode === 'hold_with_waypoints'` 分岐: `r.ui_mode = 'hold_with_waypoints'`, `r.posture_basis = postureBasis`, `r.start_posture = {...}`, `r.mid_postures = midPostures.map(...)`, `r.end_posture = {...}`, `r.key`, `r.end_key` |
| `app.js:2076-2170` (姿勢条件 UI) | `ruleMode === 'hold_with_waypoints'` 分岐で中間姿勢キャプチャブロックを追加。「📷 中間姿勢 1」「📷 中間姿勢 2」+ 「+ 中間追加」「- 中間削除」ボタン。各姿勢に R/P/Y 表示 (Phase 5.38 の軸別 line-through 描画流用) |
| `app.js` 姿勢条件 UI | `postureBasis` セレクタ追加: 「判定基準: ○ 絶対 Euler  ○ 相対 Quaternion」。相対選択時は説明バナー「state[0] (開始条件) 成立時に q_ref を自動取得、以降は ref からの相対姿勢で判定」を表示 |
| `app.js:2019-2033` (rule.list バッジ表示) | 既存 SEQUENCE バッジ (Phase 5.34) でそのまま `r.states[]` を描画 (変更不要)。相対モードのバッジには `🔄 rel` チップを追加 |
| `app.js:2046-2050` (PitchRollGrid setSequences) | 絶対モードは既存 setSequences でそのまま描画。相対モードは描画スキップ (Phase 5.39.1 で対応)、テキストで waypoint 内容のみ表示 |

### サンプルプロファイル追加

`Web/hidconfig/profiles/wingardium_hold.json` (新規) — Wingardium Leviosa を相対モード hold_with_waypoints で実装する例:

```json
{
  "schema": "burst_motion_sample_v1",
  "id": "wingardium_hold",
  "title": "🪄 Wingardium Leviosa (相対 Hold + 中間 2 姿勢)",
  "description": "Btn3 押下で q_ref 取得 → 中間 (右 +30°) → 終了 (右下 +60°、下 30°) で 'wingardium leviosa' を release-fire。Yaw ドリフト無関係、持ち方自由。",
  "hardware_required": ["m5stickc"],
  "rules": [
    {
      "name": "wingardium_leviosa_hold",
      "ui_mode": "hold_with_waypoints",
      "posture_basis": "relative",
      "start_posture": { "euler": [0, 0, 0], "euler_tol": [180, 180, 180] },
      "mid_postures": [
        { "euler": [30, 0, 0],   "euler_tol": [20, 20, 180] },
        { "euler": [60, -30, 0], "euler_tol": [20, 20, 180] }
      ],
      "end_posture": { "euler": [0, -30, 0], "euler_tol": [20, 20, 180] },
      "button_idx": 3, "button_state": 0,
      "type_text": "wingardium leviosa\n",
      "cooldown_ms": 2000
    }
  ]
}
```

### 検証手順

1. **回帰**: 中間 0 個で組んだ hold_with_waypoints が既存 hold_start_end と完全に同じ動作になること
2. **絶対モード**: 「右傾→中間 水平→上傾→release」を組み、途中で別経路を通ると不成立になること (中間 waypoint の効果)
3. **相対モード**: 任意の向きで Btn3 押下 → 「上 30°→右 45°→下 30°」相対振り → release。**M5C をどの向きで持っても成立** すること
4. **ドリフト耐性試験**: デバイス起動後 30 分 idle (Mahony Yaw drift 蓄積) → 相対モードで同じジェスチャが成立すること、絶対モードでは Yaw=180° で組んだルールが微妙にずれていること
5. **ジンバルロック試験**: 絶対 Pitch=-80° から開始 → 絶対モードでは Phase 5.38 警告が出るが、相対モードでは ref=-80° スタートで rel Pitch が 0° から始まるため警告なしで動作すること
6. **加速度併用**: 中間 state に `accel_abs_threshold: 2.0` を追加し、振りが弱いと不成立になること (姿勢 + 加速度の AND 評価が中間でも機能すること)

### Profile schema_version

- **schema_version 据え置き** (`posture_basis` 省略時 absolute で後方互換、`ui_mode == "hold_with_waypoints"` は既存 enum に追加するだけ)
- 既存 hold_start_end / oneshot / hold_start_only / sequence のサンプルプロファイルは無変更で動作継続

### リスク・要検討

1. **q_ref のノイズ感度**: state[0] enter 瞬間の IMU ノイズが q_ref に乗ると軌跡判定が全体的にズレる。対策: state[0] enter 後 5-10 サンプル (50-100ms) の移動平均で q_ref を確定する。MVP では単一スナップショット、Phase 5.39.1 で改善。
2. **相対モードの可視化**: 2D PitchRollGrid は絶対座標前提なので、相対モード rule は描画スキップで MVP。Phase 5.39.1 で「ref からのオフセット軌跡」を別 grid に描画する案あり。
3. **States[4] 上限**: 中間 3 個以上が必要になれば states[8] への配列拡張 (Phase 5.40)。RAM 消費は rule あたり +~200B。現時点では 2 個までで十分。
4. **複数 rule の q_ref 競合**: 各 rule が独立 q_ref を持つので競合なし。ただし大量の相対モード rule が並列に存在すると state[0] enter 検出のたびに q_ref 計算負荷 (1 rule あたり数 µs、N=10 でも 1 tick 50µs 以下なので問題なし)。
5. **既存 PostureBasis enum との衝突**: 現 FW に PostureBasis 概念は存在しないため衝突なし。新規追加。
6. **rule.list で states[] が肥大化**: 中間 2 個入りで states_count=4、JSON 1 行あたり ~800B → NUS / Serial 帯域には影響軽微 (rule 数 30 件で 24KB、初回 list 取得時のみ)。

### Phase 5.39.1 以降の拡張余地

- q_ref のノイズ平均化 (5-10 サンプル)
- 2D PitchRollGrid に「相対モード rule の ref からのオフセット軌跡」表示モード追加
- 中間 state に on_enter=AT_FIRE_ONCE を許可 (中間通過時に効果音 / LED フラッシュ)
- 各 state 個別の判定軸 ON/OFF (現在は rule 全体で統一)
- States[4] → States[8] 拡張で中間 3-6 個対応 (Phase 5.40 候補)

### Web UI 複雑度抑制ガイドライン (ユーザー要件)

既存 TriggerEditor は既に「許容±」「判定」「判定軸」「開始姿勢」「終了姿勢」「ボタン」「修飾」「キー」など 7-8 要素が並ぶ密度。中間姿勢と判定基準を**雑に追加すると認知負荷が破綻** するため、以下の方針で実装する。

#### 採用する単純化ルール

| 要素 | 方針 | 理由 |
|------|------|------|
| **モード切替 (ui_mode)** | 既存ドロップダウンに `HOLD with Waypoints (開始→中間→終了)` の 1 オプション追加のみ | UI 構造は変えない、ユーザーは「既存モード + 新 1 つ」として認識 |
| **中間姿勢キャプチャ** | **デフォルト 0 個 (= hold_start_end と完全等価)**。 開始姿勢の下に `[+ 中間姿勢を追加]` テキストボタンを 1 行だけ常時表示。クリックで 1 個追加、最大 2 個まで。各中間に `×` 削除ボタン | 既存ユーザーには見た目変化なし、必要なユーザーだけ展開、`hold_with_waypoints` モード選択時のみ表示 |
| **判定基準 (絶対/相対)** | **`▼ 詳細設定` 折り畳み (details/summary) 内に格納**。デフォルト「絶対 Euler」。相対選択時のみ警告バナー 1 行表示: 「state[0] 成立時に基準姿勢を自動取得、以降は ref からの相対で判定」 | 普通の使い方では存在を意識せず、Wand 用途の上級ユーザーだけ展開 |
| **判定軸 ON/OFF** | Phase 5.38 既存 UI そのまま流用、**全姿勢 (開始/中間/終了) で統一適用**。中間ごとに個別軸選択は持たせない | 軸選択を per-state にすると要素が爆発、Phase 5.39.1 で必要なら拡張 |
| **加速度/ジャイロ閾値** | 既存通り rule 全体に 1 セット。中間 state 個別の accel/gyro は MVP では持たない | rule あたり設定数を増やさない、必要なら「accel ピーク = 中間通過判定」を Phase 5.39.1 で |
| **rule.list バッジ** | 既存 SEQUENCE バッジ (Phase 5.34) 路線そのまま。相対モードは末尾に小さく `🔄 rel` チップ 1 個だけ | 視認性を維持、特殊モードは小さなアイコンで示すだけ |
| **デフォルト値** | 中間姿勢追加時の tol は `postureTol` を流用、Yaw は `postureUseYaw` (default OFF) を継承 | ユーザーが新規入力する項目を最小化 |
| **説明テキスト** | 中間姿勢ボタンに `title=` ツールチップ 1 行のみ、本文には書かない | 画面を埋めない、必要な人がホバーで読む |

#### UI レイアウト (HOLD with Waypoints モード選択時)

```
┌─ 姿勢条件 (任意) ─────────────────────────────────┐
│  許容 ±[15]°  判定: [Euler 角 ▼]                  │
│  判定軸: ☑Roll ☑Pitch ☐Yaw   → OFF 軸の tol=180   │
│                                                    │
│  📷 開始姿勢   🟦R:0 P:0 Y:0  ×                   │
│  📷 中間姿勢 1 🟪R:30 P:0 Y:0 ×    ← 0 個ならこの行ごとなし
│  [+ 中間姿勢を追加]  ← 0-1 個入っているときだけ表示、2 個で消える
│  📷 終了姿勢   🟧R:60 P:-30 Y:0 ×                 │
│                                                    │
│  ▼ 詳細設定                                        │
│    判定基準: (●) 絶対 Euler  (○) 相対 Quaternion │
│    [相対選択時のみ表示] 💡 state[0] 成立時に基準姿勢│
│                       を自動取得、以降は相対で判定 │
└────────────────────────────────────────────────────┘
```

#### UI 追加要素のカウント比較

| 要素 | 既存 hold_start_end | 新 hold_with_waypoints (中間 0 個) | 中間 2 個 + 相対モード |
|------|---------------------|------------------------------------|------------------------|
| 行数 (姿勢条件ブロック内) | 3 行 (許容/判定軸/開始+終了) | **3 行** (同上 + + ボタン 1 つ) | 5 行 (+ 中間 2 + 詳細展開 2) |
| 入力要素数 | 8 (許容/判定/軸×3/開始/終了/×) | **9** (+ 中間追加ボタン) | 14 (+ 中間×4 + ラジオ 2) |
| 折り畳み | なし | なし | 詳細設定のみ折り畳み |

→ **デフォルト構成 (中間 0 個 + 絶対モード) では既存とほぼ同じ密度**。複雑化はユーザーが明示的に + ボタンや詳細設定を展開したときだけ発生する。

#### 段階的開示の徹底

1. **Step 1**: ユーザーが ui_mode を `hold_with_waypoints` に切替 → 既存 hold_start_end UI が表示される (見た目変化最小)
2. **Step 2**: [+ 中間姿勢を追加] ボタンを押すと 1 個増える (ユーザーの能動的選択)
3. **Step 3**: ▼ 詳細設定を開かないと判定基準は見えない (上級者だけ)
4. **Step 4**: 相対モードを選んだ瞬間に 1 行の説明バナーが現れる (選んだ時だけ、それ以外は非表示)

これにより「**初学者は hold_start_end と同じ感覚で使え、上級者は中間と相対モードを段階的に開拓できる**」UX を実現。

#### 実装で守るルール

- 新規追加する `useState` は **2 個まで** (`midPostures` 配列、`postureBasis` 文字列)
- 新規追加する関数は **4 個まで** (`captureMidPosture`, `clearMidPosture`, `addMidPostureSlot`, `removeMidPostureSlot`)
- JSX 内の条件分岐は `ruleMode === 'hold_with_waypoints' && (...)` の 1 ブロックに集約、姿勢条件以外のセクションには波及させない
- 既存 hold_start_end 動作・UI を**完全に温存** (回帰テスト必須)
- 文字情報の追加は ツールチップ + 1 行バナー (相対選択時) のみ、長文の説明は plan ドキュメントに留める

---

### 関連特許情報 (★ 法的リスク評価、調査 1 次結果、深掘り継続中 ★)

#### Kano Computing 関連特許

| 特許番号 | 種別 | タイトル | 状態 | 出願日 | 公開日 | 発行日 | 満了 (推定) | 譲受人 | 発明者 | URL |
|---------|------|---------|------|--------|--------|--------|-------------|--------|--------|-----|
| **US20210165506A1** = **US11301059B2** | 米国 (出願→発行) | Wand-based gesture recognition system with origin reset | **Active (有効)** | 2019-07-24 | 2021-06-03 | **2022-04-12** | **2039-07-24** | Kano Computing Limited | Alex Klein, Peter Griffith | https://patents.google.com/patent/US20210165506A1/en |
| **US20220100280A1** | 米国 (出願公開) | Gesture recognition device with minimal wand form factor | 出願公開後審査中 | (要確認) | 2022 年 | (未発行 or 不明) | (要確認) | Kano Computing Limited | (要確認) | https://patents.google.com/patent/US20220100280A1/en |
| **GB202003891A** | 英国 | Touch sensor + IMU wand with on-screen cursor trail | **Granted (2021-04)** | 2020 年 | 2021 年 | 2021-04 | 2040 年頃 | Kano Computing Limited | (要確認) | https://www.gov.uk/search-for-patent |

**重要なクレーム要素 (Kano US11301059B2)**:
1. IMU (3 軸加速度 + 3 軸ジャイロ、オプション 3 軸磁気)
2. 単一ボタン (押下時間長短で動作区別)
3. **Origin reset 機能**: ボタン押下で「現在の向きを原点として再定義」
4. デュアルモード: Configuration mode (ジェスチャー記録) / Regular mode (認識)
5. 計算は controller 側ではなく **computing device 側** (= ホスト PC / スマホ)
6. 物理形状: 杖型 (elongated wand)

→ **Kano 特許のキーは "origin reset + 杖型 + 単一ボタン + computing device 側計算"**。Burst Motion はこの組合せを**意図的に部分回避**することで特許リスクを軽減可能。

#### 隣接特許

| 特許番号 | 譲受人 | 内容 | リスク |
|---------|--------|------|--------|
| **US20220100280A1** | Spin Master (推定、WB 系) | IMU (加速度+ジャイロ+磁気) でジェスチャー認識 IoT デバイス | 中 |
| **US11221730** | HTC | Vive コントローラ IMU 親指ジェスチャー検出 | 低 (Wand と用途違) |
| **US20150220153A1** | (不明) | FSM gesture recognition (状態遷移ジェスチャー認識) | 要確認 |
| Nintendo Wii MotionPlus 特許群 (多数) | Nintendo | InvenSense IDG-600/650 + 振り動作判定 | 低 (棒状コントローラの先行技術として豊富、防衛特許化) |
| Sony PS Move 特許群 | Sony | IMU + 光学位置追跡コントローラ | 低 (Wand と用途違) |

#### Kano の倒産と特許の譲渡 (2023 年経緯)

- **2023 年 1 月**: Kano が WB Magic Caster Wand を「特許侵害」と告発 (TechCrunch)
- **2023 年 4 月**: WB Magic Caster Wand 販売停止 (公式は「需要対応」、業界は知財紛争影響と推測)
- **2023 年 6 月**: Kano Computing Limited が UK Administration (英国管財人手続き = 倒産プロセス) 入り
- **訴訟記録**: 公式な訴訟記録は確認できず、両社の和解 or Kano 倒産で事実上停止と推測
- **現在の特許所有者**: Kano 倒産後の特許譲渡先は未確定 (清算人手続き or 第三者買収の可能性)
- **影響**: Kano 自身は訴訟継続困難でも、特許自体は満了まで有効 (2039 年)、買収者が権利行使する可能性あり

#### Burst Motion の特許リスク評価

**OSS / 非商用 (個人・ハッカープロジェクト) として**:
- リスク**低**。Hogwarts-Legacy-Wand 等の OSS は活動継続中、小規模ゆえ訴訟対象外傾向
- ただし特許は OSS でも有効 (Defensive Patent License や prior art disclosure の検討余地)

**OSS + 商用販売 ($25 量産品) として**:
- リスク**中**。特に米国市場で Kano 系特許 (US11301059B2) との衝突懸念
- 回避策:
  1. **「Origin reset 機能」の具体実装を Kano と差別化** — 例: ボタン押下ではなく「静止検出」をデフォルトトリガに、ボタン押下は補助手段に
  2. **計算は controller 側で実行** — Kano は "computing device 側" がクレーム要素、Burst Motion は M5StickC 内で完結なので回避可能
  3. **9 軸 IMU を採用しない** — Burst Motion は 6 軸 (MPU6886 / LSM6DSV16X / BMI270) 主体、Kano の "9 軸" クレーム要素を踏まない
  4. **杖型を強調しない** — 「motion controller」として汎用化 (Wand プロファイルは一機能、本質は HID 変換)
  5. **Configuration mode / Regular mode の二相設計を避ける** — Web UI で随時編集できる「設定モードと使用モードが融合」した設計

**地域別リスク**:
- **米国**: 高 (ソフトウェア特許に許容的、Kano 特許の権利行使範囲が広い)
- **欧州**: 中 (EPC で技術効果の実証必要、IMU ジェスチャー認識は狭く認められる)
- **日本**: 低-中 (応用発明は許可されるが、Kano 日本出願は未確認)

**推奨事項**:
1. 商用化前に弁理士による FTO 調査 (Freedom To Operate)
2. Kano US11301059B2 のクレーム文を逐条チェックして非侵害論を整理
3. Burst Motion の「**FSM + 中間 waypoint + 相対 quat + 静止検出**」は Kano 単一ボタン方式と意図的に差別化されているので、適切に文書化すれば防御可能
4. OSS GPL v3 で defensive publication 効果を狙う (将来の他社特許出願を阻止)
5. 量産時は IP 賠償保険 (IP indemnity insurance) の購入を検討

**Sources (1 次調査)**:
- [US20210165506A1 - Google Patents](https://patents.google.com/patent/US20210165506A1/en)
- [US20220100280A1 - Google Patents](https://patents.google.com/patent/US20220100280A1/en)
- [Patents Assigned to Kano Computing Limited - Justia](https://patents.justia.com/assignee/kano-computing-limited)
- [Warner Bros. swiped our Harry Potter wand IP, says Kano (TechCrunch)](https://techcrunch.com/2023/01/27/warner-bros-swiped-our-harry-potter-wand-ip-says-kano/)
- [Warner Bros. Suspends Sales of Magic Caster Wand (MuggleNet)](https://www.mugglenet.com/2023/04/warner-bros-suspends-sales-of-magic-caster-wand-possibly-due-to-dispute-with-kano/)
- [KANO COMPUTING LIMITED insolvency - GOV.UK](https://find-and-update.company-information.service.gov.uk/company/08375450/insolvency)
- [Stanford: Software Patent Eligibility Overview](https://law.stanford.edu/publications/no-72-software-patent-eligibility-and-patentability-an-overview-of-the-developments-in-japan-europe-and-the-united-states-and-an-analysis-of-their-impact-on-patenting-trends/)
- [Hogwarts-Legacy-Wand (OSS、特許リスク回避例)](https://github.com/Thats-so-Mo/Hogwarts-Legacy-Wand)

**バックグラウンド調査結果 (2026-05 完了)**:

| 項目 | 確認結果 | 含意 |
|------|---------|------|
| Kano ML モデル種別 | **scikit-learn ベースの古典 ML (kNN / SVM 相当)** ; Hackaday 記事による | Burst Motion FSM 方式とは別系統、特許衝突リスクは別軸 |
| ML 入力ベクトル | **加速度 + ジャイロの生時系列 (200Hz)**、ボタン押下による相対化は **画面カーソルレベルのみ** で ML 入力にはおそらく適用なし | Kano = "相対 + ML" の推定は誤り、実態は "絶対波形 + ML" が近い |
| 倒産後の特許譲渡 | 2023-06 Kano UK Administration → 資産は **Ashdust LLP** に譲渡。**特許自体は Kano Computing Limited 名義のまま** (USPTO 譲渡記録なし) | 特許権者不明確、買収者が権利行使する可能性あり |
| Kano vs WB 訴訟ステータス | 2023-01 cease & desist → WB "claims without merit"。**2024-2026 の判決情報なし** | 和解 or 継続中、Burst Motion は当面影響少だが特許自体は 2039 年まで有効 |
| 追加 Kano 特許 | US20170303371A1 (ワンドジェスチャー基本、Published)、US9699866B2 (加速度ベース照明制御、2017 expired)、GB202003891A (タッチ + IMU、UK Granted 2021) | 範囲広い、複数クレームで防衛 |
| Spin Master 特許 | **US20220100280A1** は実際は **Warner Bros. 名義** (Kano ではない)、minimal wand form factor | Kano vs WB の知財係争の焦点候補 |
| センサーレート | Kano = 200Hz、Burst Motion = 100Hz | Kano は 2 倍高速、判定精度向上 |
| Sources 追加 | [Hackaday: Magic Wand Learns Spells](https://hackaday.com/2018/12/07/magic-wand-learns-spells-through-machine-learning-and-an-imu/)、[Fast Company: WB vs Kano](https://www.fastcompany.com/90841372/makers-of-harry-potter-smart-wand-claim-warner-bros-took-their-ip)、[Nordic Semi: nRF52832 + Kano](https://www.nordicsemi.com/Nordic-news/2018/10/Harry-Potter-Kano-Coding-Kit-employs-nRF52832-to-wirelessly-connect-magic-wand-with-app) | — |

→ **Burst Motion Phase 5.39 (FSM + 相対 quat + 静止検出) は Kano (絶対波形 + 古典 ML) と判定アルゴリズムが全く異なる**ため、特許侵害の直接リスクは低い。"origin reset" 概念だけが Kano クレームと部分一致する可能性あり、これは 「相対 quat 取得トリガ = button (推奨) vs 静止検出 (推奨) vs 姿勢一致 (推奨)」 と複数選択肢を残すことで意図的に **Kano 単一ボタン方式と差別化** する設計とする。

---

### Burst Motion rule モード体系 整理 (Phase 5.39 時点、★ 重要 ★)

Burst Motion の rule は **4 つの直交する軸** で定義される (「モード」と呼ばれるのは主に軸 1 + 軸 2)。

#### 軸 1: `ui_mode` (動作モード、5 種類)

| ui_mode | 動作 | state 数 / loop | 用途例 |
|---------|------|----------------|--------|
| `oneshot` | 開始条件成立で 1 回発火 (cooldown 付き) | 1 / false | 1 動作 1 キー、波動拳 macro 発火 |
| `hold_start_only` | 開始条件成立中 press、離脱で自動 release | 1 / true | 「右傾けてる間 D 押下」 |
| `hold_start_end` | 開始で press、終了 (別条件) で release | 2 / true | 「右傾→Shift 押下、左傾→Shift 解除」 |
| `sequence` | 1-4 状態の状態機械、終端で fire (`directions[]` ショートハンド) | 1-4 / false | Wingardium DR→R→UR→D で text 入力 (Phase 5.33) |
| **`hold_with_waypoints`** (★ Phase 5.39) | **開始 → 中間 0-2 → 終了** の状態機械、開始で press、終了で release | 2-4 / false | Wingardium Hold (中間 waypoint 通過必須) |

#### 軸 2: `posture_basis` (姿勢の解釈基準、Phase 5.39 で追加、2 種類)

| posture_basis | 動作 | サンプル |
|---------------|------|---------|
| `absolute` (default、PB_ABSOLUTE_EULER) | 姿勢は **Mahony 起動基準座標**で解釈、target.euler は絶対 Euler 値 (Roll/Pitch/Yaw deg) | 既存全サンプル、wingardium_hold_euler |
| **`relative`** (★ Phase 5.39、PB_RELATIVE_QUAT) | 姿勢は **state[0] enter 時の q_ref (= q_current snapshot) からの相対オフセット**、target.euler は ref からのオフセット値 | wingardium_hold_relative |

#### 直交性 — 5 ui_mode × 2 posture_basis = 10 通り (うち実装は absolute 全 5 + relative hold_with_waypoints)

| ui_mode \ posture_basis | absolute | relative |
|-------------------------|----------|----------|
| oneshot | ✅ 既存 | ✅ 技術的可能、未サンプル |
| hold_start_only | ✅ 既存 | ✅ 技術的可能、未サンプル |
| hold_start_end | ✅ 既存 | ✅ 技術的可能、未サンプル |
| sequence | ✅ 既存 (Hogwarts ハリポタ全サンプル) | ✅ 技術的可能、未サンプル |
| **hold_with_waypoints** | ✅ Phase 5.39 サンプル (wingardium_hold_euler) | ✅ Phase 5.39 サンプル (wingardium_hold_relative) |

#### 軸 3: `judge_by` (姿勢比較アルゴリズム、posture 内設定)

| judge_by | アルゴリズム | tol 表現 |
|----------|------------|----------|
| `euler` (default) | Roll/Pitch/Yaw 軸別差分の絶対値比較 (直方体内判定) | `euler_tol[3]` Roll/Pitch/Yaw 軸別 |
| `quat` | Quaternion 内積 cone 判定 \|dot\| ≥ cos(θ/2) | `quat_dot_min` 単一値 |

#### 軸 4: state[0] 開始トリガ条件 (Condition 内、AND 評価)

各 state の Condition は以下を AND で組合せ可能:
- posture (Euler 軸別 tol or Quat dot)
- button (idx + state = 押 / 離 / どちらでも)
- accel_abs_threshold (合成加速度の閾値)
- gyro_abs_threshold (合成ジャイロの閾値)
- **stillness_required** (★ Phase 5.39): 静止検出 (重力のみの加速度 + 微小ジャイロが window_ms 連続)

これらの組合せで Kano (button-driven)、WB (stillness-driven)、姿勢一致駆動などすべて表現可。

---

## Phase 5.39.2 — UI 洗練 (rule 選択フォーカス + 軌跡可視化 + ★ 相対 3D ビュア新設 + タブ式 UI ★)

### Context

Phase 5.39 で hold_with_waypoints + 相対 Quat 判定を実装したが、**ユーザーが「ある rule の軌跡 (辿るべき道) を視覚的に理解する」UI が不足**。

特に **相対モード rule** には根本的な課題:
- target.euler が q_ref からのオフセットなので、絶対座標 3D ビュアでは描画できない (q_ref が動的)
- 既存 IMUViewer は「ワールド座標固定、M5C モデルが回転」する絶対視点
- **ユーザー視点 = M5C 視点 (= 一人称視点) のほうが「どっちにどれだけ振ればいいか」が直感的**

ユーザー要望 (2026-05-14):
1. **相対モード rule では「M5C が今向いている方向を中心基準とした 3D ビュア」が必要** (一人称視点)
2. Web UI に **タブを新設** して「絶対 / 相対 / 2D」を切替えるのがわかりやすい
3. M5C モデルの描画は **デフォルト中央固定/回転無し**、ただし **「q_ref に定着 (q_ref 確定後)」も切替可能** にする
4. 2D 相対表示は後で検討 (Phase 5.39.3 持ち越し)

### 全体構成 (タブ式 UI、3 タブ Phase 5.39.2 + 1 タブ Phase 5.39.3)

LiveView の 3D/2D 表示エリアに **タブバー** を新設。**rule の設定によって有用なタブが変わる** ので、複数タブを残してユーザーが切替えられるようにする。

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ ビュー: [ 🌐 絶対 3D ●][ 📐 2D Roll-Pitch ][ 👁 相対 3D ][ 📊 2D 相対 Quat (5.39.3)]│
│ ────────────────────────────────────────────────────────────────────────── │
│                                                                            │
│                  (選択中のタブの canvas、他は描画停止)                       │
│                                                                            │
└──────────────────────────────────────────────────────────────────────────────┘
```

#### 各タブで有用な rule モード

| rule 設定 | 🌐 絶対 3D | 📐 2D Roll-Pitch | 👁 相対 3D | 📊 2D 相対 Quat (5.39.3) |
|----------|----------|----------------|----------|----------------------|
| **Euler モード** (`absolute` + `judge_by=euler`、e.g. SF サンプル、wingardium_hold_euler) | ✅ メイン | ✅ メイン | — | — |
| **絶対 Quat** (`absolute` + `judge_by=quat`) | ✅ メイン | △ (Pitch/Roll 投影だけ参考) | — | — |
| **相対 Quat** (`posture_basis=relative`) | ✅ サブ (自分の姿勢確認用) | △ (絶対座標なので相対 rule の waypoint は描けない) | ✅ メイン | ✅ メイン (5.39.3) |

タブ動作:
- デフォルト: **🌐 絶対 3D**
- **自動推奨ハイライト** (タブ名にドット表示、自動切替はしない):
  - rule 選択時、その rule で有用なタブ (= 表内「✅ メイン」のタブ) に小ドット ● 表示
  - 例: 相対モード rule を選択 → 「👁 相対 3D」「📊 2D 相対 Quat」にドット
- **非選択タブの canvas は描画停止** (RAF 停止、CPU 節約)

#### Phase 5.39.2 で実装する 3 タブ

1. **🌐 絶対 3D** — 既存 IMUViewer を拡張 (selected フォーカス + 絶対軌跡 + trail)
2. **📐 2D Roll-Pitch** — 既存 PitchRollGrid を拡張 (selected フォーカス、現状描画維持)
3. **👁 相対 3D** — 新規 RelativeIMUViewer (一人称視点、相対 rule 軌跡可視化)

#### Phase 5.39.3 で追加する 1 タブ

4. **📊 2D 相対 Quat** — 新規 RelativePitchRollGrid (現在姿勢 = 中心 (0,0)、waypoint は相対オフセット位置でプロット)

### 描画方針一覧

| ビュア | 球面 (固定基準) | M5C モデル | ワールド (waypoint/軸/軌跡) | rule 描画 |
|--------|---------------|-----------|----------------------------|----------|
| **🌐 絶対 3D (既存 IMUViewer 拡張)** | scene 固定 | quaternion で回転 | scene 固定 | absolute rule の target.quat を絶対座標で球面接続 |
| **👁 相対 3D (新規 RelativeIMUViewer)** | scene 固定 | **デフォルト identity 固定** (中央) / オプション q_ref 定着 | **q_current⁻¹ で逆回転** (= 一人称視点) | relative rule の rel_quat を相対座標で配置 |
| **📐 2D Roll-Pitch (既存 PitchRollGrid)** | — | — | — | absolute rule の Roll/Pitch を 2D グリッドに配置 (現状通り) |

### 採用方針: ハイブリッド軌跡描画

| 状況 | 表示タブ | 軌跡描画 | 線スタイル |
|------|---------|---------|----------|
| 絶対モード rule、rule 選択中 | 🌐 絶対 3D | target.quat を球面 Slerp 接続 (常時) | 濃い実線、紫 |
| **相対モード rule、選択中 + ボタン押下前** | **👁 相対 3D** | rel_quat を相対座標で配置 (M5C 中央固定、ワールド側で表示) | **薄い破線、紫 (プレビュー)** |
| **相対モード rule、ボタン押下後 (state[0] enter 済)** | **👁 相対 3D** | FW 受信の実 q_ref で確定描画 | **濃い実線、紫** |
| state リセット | 👁 相対 3D | プレビューに戻る | 薄い破線 |
| rule 未選択 | 任意 | 軌跡なし、既存 dots のみ淡色 | — |

### 実装項目

#### A. タブ式 UI (app.js)

- 新 state: `viewerTab` (default `'absolute'`) — `'absolute'` | `'relative'` | `'2d'`
- 既存 LiveView 上に `<div role="tablist">` でタブバー追加
- 各タブの canvas は別個の `<canvas>` 要素、表示中以外は `display:none` + RAF 停止
- 「相対」タブには「自動推奨」ドットを `selectedRule?.posture_basis === 'relative'` で表示

#### B. rule 選択フォーカス (app.js + 全ビュア)

- 新 state: `selectedRuleId` (default -1 = 未選択)
- rule.list 行 onClick でトグル選択、選択行ハイライト
- PitchRollGrid: `setSelectedRuleId(id)`、非選択 rule は globalAlpha=0.18
- IMUViewer (絶対): `setSelectedRuleId(id)`、非選択は淡色
- RelativeIMUViewer (新規): 同様

#### C. 既存 IMUViewer (絶対 3D) の拡張

- 新メソッド `setTargetTrajectory(quats)` — 絶対 quat 配列を 32 段階 Slerp で接続、紫実線で球面上に描画
- 既存 referenceDots は selectedRuleId のものだけ濃く

#### D. 新規 RelativeIMUViewer (相対 3D)

新規ファイル: `Web/hidconfig/src/lib/RelativeIMUViewer.js`

scene 階層:
```
scene
├── 球面ワイヤー (固定、既存と同じ見た目)
├── M5C モデル (中央配置、回転は m5cMode で切替)
└── worldGroup (★新規グループ)
    ├── 軸ヘルパー
    ├── 相対 waypoint dots (rel_quat 適用済の球面上点)
    ├── 相対 waypoint 接続線
    └── 過去軌跡 trail (世界に固定された動き履歴)
```

メソッド:
- `setQuaternion(qw,qx,qy,qz)` — sensor 更新時に **worldGroup.quaternion = quat_conj(q_current)** (= ワールドを逆回転、M5C 視点で固定)
- `setSelectedRule(rule)` — relative rule なら rel_quats を waypoint dots に配置、相対接続線も描画
- `setM5CMode(mode)` — `'fixed'` (default、identity) | `'qref_anchor'` (q_ref で定着)
- `setQRef(qref, valid)` — q_ref 確定通知。`valid=true` で relative_fixed モード、`qref_anchor` 表示時に M5C モデルを `q_current * quat_conj(q_ref)` 風に向ける
- `addTrailPoint(...)`, `clearTrail()` — 軌跡を worldGroup 内に追加 (M5C 視点では動いて見える)
- `setTrajectoryMode(mode)` — `'preview'` (薄い破線、現在姿勢を仮 q_ref とする) | `'fixed'` (濃い実線、FW q_ref 固定)

#### E. M5C モデルの 2 モード切替 (相対ビュア内、UI チェックボックス)

| m5cMode | 動作 | UX 説明 |
|---------|------|--------|
| `'fixed'` (default、推奨) | M5C モデル = identity (中央で動かない) | **完全一人称視点**、ワールドの動きだけ見える |
| `'qref_anchor'` (オプション) | q_ref 確定後: M5C モデルを `q_current * quat_conj(q_ref)` で回転 (=「ref からの相対回転」を M5C が示す)、未確定時は identity | 「ref からどれだけ動いたか」が M5C 自身の向きで分かる、三人称視点的 |

切替 UI: 相対タブ canvas の下に小さなチェックボックス「☐ M5C を q_ref に定着」(default OFF = fixed)

#### F. 過去軌跡 trail

- IMUViewer (絶対): 世界固定座標で trail を描画 (現在姿勢の球面上点履歴)
- RelativeIMUViewer (相対): worldGroup 内に trail を描画 → ユーザーが動くと trail が反対方向に流れる = M5C 視点で「どこから来たか」見える
- 共通: trigger.hit `phase='enter'` で `clearTrail()`、3 秒 fade out
- メソッド `addTrailPoint(...)`、`clearTrail()`、`setTrailDurationMs(ms)`

#### G. FW 改修 (相対モード q_ref 受信用)

- `trigger.hit` event の payload に新フィールド `q_ref: [w, x, y, z]` を追加
- 該当箇所: `M5C_MPU6886_cpp/src/core/TriggerEngine.cpp` の `evaluateRule()` 内、state[0] enter 時に hit event を発火する箇所 (rule.q_ref を JSON に dump)
- 5 行程度

#### H. app.js 連携

- `selectedRuleId` 変更時、対応 rule の posture_basis を見て:
  - absolute → 絶対 IMUViewer に `setTargetTrajectory(abs_quats)`、相対 ビュア側は空
  - relative → 相対 RelativeIMUViewer に `setSelectedRule(rule)` + 「相対」タブ推奨ハイライト
- `trigger.hit` event の `phase='enter'`:
  - relative モード rule で q_ref を受信 → `relativeViewer.setQRef(qref, true)` + `setTrajectoryMode('fixed')`
  - 両ビュアの trail を clear
- `trigger.release`:
  - relative モード → `setTrajectoryMode('preview')` (再度プレビュー)
- onSensor:
  - 両ビュア (表示中のものだけ) に `addTrailPoint(...)`
  - 相対ビュアの worldGroup を毎フレーム逆回転 (`setQuaternion`)

### 実装対象ファイル (絶対パス)

- `C:\Users\thefu\Documents\M5C_Serial_Unity\M5C_MPU6886_cpp\Web\hidconfig\src\app.js` (タブ + state + 連携)
- `C:\Users\thefu\Documents\M5C_Serial_Unity\M5C_MPU6886_cpp\Web\hidconfig\src\lib\IMUViewer.js` (selected フォーカス + 絶対軌跡 + trail)
- **新規**: `C:\Users\thefu\Documents\M5C_Serial_Unity\M5C_MPU6886_cpp\Web\hidconfig\src\lib\RelativeIMUViewer.js` (相対 3D ビュア)
- `C:\Users\thefu\Documents\M5C_Serial_Unity\M5C_MPU6886_cpp\Web\hidconfig\src\lib\PitchRollGrid.js` (selected フォーカス)
- `C:\Users\thefu\Documents\M5C_Serial_Unity\M5C_MPU6886_cpp\src\core\TriggerEngine.cpp` (trigger.hit に q_ref 追加、5 行)

### FW 側での絶対/相対の区別 (★ 現状実装の確認 ★)

ユーザー質問: 「M5C 側のアクションルールの判定や登録が、絶対 Quat と相対 Quat で区別されるようになってるか？」

#### ✅ 完全に区別されている (Phase 5.39 で実装完了)

**1. データ型 (types.hpp)**:
```cpp
enum PostureBasis : uint8_t {
    PB_ABSOLUTE_EULER = 0,   // 絶対 (Mahony 起動基準)
    PB_RELATIVE_QUAT  = 1,   // 相対 (state[0] enter で q_ref snapshot)
};

struct ActionRule {
    uint8_t posture_basis;   // ★ 絶対/相対の区別フィールド
    float   q_ref[4];        // 相対モード時の基準姿勢 (runtime のみ)
    bool    q_ref_valid;
};
```

**2. 判定 (TriggerEngine.cpp:561-565)**:
```cpp
if (rule != nullptr && rule->posture_basis == PB_RELATIVE_QUAT && rule->q_ref_valid) {
    r = evalPostureRelative(cond.posture, s, rule->q_ref);  // 相対判定
} else {
    r = evalPosture(cond.posture, s);                       // 絶対判定
}
```

**3. 登録・永続化 (Profile.hpp, main_v2.cpp の rule.add ハンドラ)**:
- `rule.add` で `"posture_basis": "absolute"|"relative"` を受信 → ActionRule.posture_basis に保存
- `rule.list` レスポンスに `posture_basis` フィールド出力 (各 rule ごと)
- `profile.save` / `profile.load` JSON にも `posture_basis` 含む (省略時 absolute、後方互換)
- LittleFS で永続化、電源切断後も保持

**4. 検証済み (Python smoke test、`c:/tmp/bm_phase539_test.py`)**:
```
=== rule.add (Wingardium Hold Euler) ===
  ack ok=True id=0 posture_basis='absolute' states=4

=== rule.add (Wingardium Hold Relative) ===
  ack ok=True id=1 posture_basis='relative' states=4

=== rule.list ===
  - id=0 name='wingardium_hold_euler' posture_basis='absolute' states_count=4
  - id=1 name='wingardium_hold_relative' posture_basis='relative' states_count=4
```

#### `posture_basis` と `judge_by` は独立 (2 軸)

| posture_basis | judge_by | 判定アルゴリズム | 用途 |
|---|---|---|---|
| `absolute` | `euler` (default) | sensor.euler vs target.euler、軸別 tol | 現状サンプル (SF / wingardium_hold_euler) |
| `absolute` | `quat` | **絶対 Quat 内積** \|dot(sensor.quat, target.quat)\| ≥ cos(tol/2) | 絶対座標で全方向 cone 判定 |
| `relative` | `euler` (default) | q_rel から Euler 抽出 + 軸別 tol | wingardium_hold_relative |
| `relative` | `quat` | **相対 Quat 内積** \|dot(q_rel, target_rel_quat)\| ≥ cos(tol/2) | 完全ジンバルロック回避、持ち方自由 |

→ FW 側で **絶対 Quat と相対 Quat は posture_basis フィールドで完全に区別**。両方のモードで `judge_by="quat"` (Quat 内積判定) は利用可能。

#### 単一 cone tol (judge_by="quat") の数値例 (★ Pitch=0 近傍の安全領域での比較、ジンバルロック領域は別途下記)

`judge_by="quat"` は target との **全方向の最短回転角度 (測地距離)** が tol 以内で成立する球状判定。以下は **すべて Pitch ≈ 0 の安全領域** での Euler 偏差と Quat 測地距離の近似関係:

| シナリオ (Pitch=0 安全領域) | 各軸の振り | 測地距離 | judge_by="quat" tol=30° | judge_by="euler" R±30,P±15,Y±180 |
|---|---|---|---|---|
| A: Roll だけ 30° 振り | R=30,P=0,Y=0 | ≈ 30° | ✅ 境界 | ✅ Roll 範囲内 |
| B: Pitch だけ 30° 振り | R=0,P=30,Y=0 | ≈ 30° | ✅ 境界 | ❌ Pitch±15 超 |
| C: Roll+Pitch 各 21° | R=21,P=21,Y=0 | ≈ 30° (合算) | ✅ 境界 | ❌ Pitch±15 超 |
| D: 各軸 10° | R=10,P=10,Y=10 | ≈ 17.3° | ✅ 余裕 | ✅ 余裕 |
| E: Yaw だけ 100° | R=0,P=0,Y=100 | ≈ 100° | ❌ 30° 超 | ✅ Yaw±180 内 (drift 許容) |

→ 安全領域では **球状 (cone) vs 直方体 (Euler 軸別)** という幾何的違いがあり、UI の表現力で差がある。

#### ★ ジンバルロック領域 (Pitch ≈ ±65° 超) での挙動 (★ 決定的な違い ★)

ユーザー指摘: 「Pitch=65 度以上ではジンバルロックにより Yaw や Roll が一気に変動してしまう」 → **完全に正しい**

**シナリオ F**: 静止状態で M5C を Pitch=85° (LCD ほぼ真上向き) に保持し、Roll を 0° → 30° → 0° と微小振動

| 観点 | 安全領域 (Pitch=10°) | **ジンバルロック領域 (Pitch=85°)** |
|---|---|---|
| Mahony Euler Roll 出力 | 安定: 0° → 30° → 0° | **±180° 範囲で振動** (ノイズで yaw と縮退、`atan2` 分母 ≈ 0 で数値不安定) |
| Mahony Euler Yaw 出力 | 安定: 0° で固定 | **同じく振動** (yaw も roll と連動して跳ねる) |
| 物理姿勢 (quat) | 微小変化 | **微小変化** (quat 自体は安定、4 次元では特異点なし) |
| **judge_by="euler" 判定** | ✅ 安定動作 | **❌ 誤判定多発** (Roll/Yaw 値が暴れる) |
| **judge_by="quat" 判定** | ✅ 安定動作 | **✅ 安定動作** (quat 内積は 4 次元演算、Euler 縮退の影響なし) |

**ジンバルロック領域での Quat 内積判定の本当の利点**:
- 数学的安定性: Pitch ≈ ±90° でも quat 演算は数値不安定なし
- **物理姿勢の一意性**: Pitch=89° で Roll が見かけ上 ±180° 跳ねても、quat 内積は同じ物理姿勢と判定
- 誤判定なし: Euler 縮退による「擬似的な大変動」に惑わされない

**Phase 5.39.2 設計判断の根拠**:
- **Pitch ±65° 以内のジェスチャ (Wingardium swish & flick 等)** → judge_by="euler" で十分、UI 直感性最良
- **Pitch ±65° 超のジェスチャ (大きな前後振りなど)** → q_rel の Pitch 警告 + judge_by="quat" 推奨切替
- 相対モードでは絶対 Pitch ではなく **q_rel の Pitch** が縮退条件 → 「振り始め基準で大きく振らない限り」安全領域に留まる
- これが「相対モード = ジンバルロック軽減」の本質 (完全回避ではなく、領域に入りにくくなる)

→ **判定値の安定性差が決定的**。これが judge_by の選択の真の意味。

#### ★ 杖用途の実運用 (ユーザー環境前提、★ 重要)

**ユーザー指摘 (2026-05-14)**: 「Wingardium はもちろん、そもそも杖は **Pitch = -90° 付近で開始 (杖が水平)** することが多い (M5C Display 左持ちの場合)。実装される IMU の向きや Mahony の計算方法等によって厳密に決定はできないため、あくまでも今私が使っている環境・状態での話です」

#### 杖の自然な構え方と Mahony Euler の関係

| 杖の物理姿勢 | M5StickC の状態 | Mahony 絶対 Euler (ユーザー環境推定値) |
|---|---|---|
| **杖を水平に構える (開始姿勢、★ 杖の通常の構え)** | LCD 左、USB-C 側を前方水平に向ける | Roll ≈ +90°、**Pitch ≈ -90°**、Yaw 任意 |
| 杖を上に振り上げる | USB-C 側が上向き | Pitch 変化 (-90° → 縮退して跳ねる) |
| 杖を下に振り下げる | USB-C 側が下向き | Pitch 変化 (-90° → 縮退で跳ねる、または ±180° wrap) |
| 杖を真上に向ける (Lumos 等) | USB-C 側が真上 | Pitch ≈ 0°、Roll/Yaw が安定 (縮退から脱出) |
| 杖を斜めに | 振りの組合せ | 不安定 (ジンバルロック領域跨ぐ) |

→ **杖の「開始姿勢」(= 水平構え) が既にジンバルロック領域そのもの**。ここから振り始めると、絶対 Euler はノイズに極めて敏感。

#### ただし絶対モードでもジンバルロック回避は可能 (★ ユーザー追加指摘)

「杖を持つときに IMU や M5C の姿勢、もしくは Mahony の基準姿勢を変えれば、杖の水平方向を Roll 方向にできるため、Euler の Pitch のジンバルロックに制約されない領域で利用できる場合もある」

**3 つのアプローチ**:

| アプローチ | 内容 | 影響範囲 | 適用容易さ |
|---|---|---|---|
| **a) 保持姿勢を変える** | LCD 上向き横持ち / 縦持ちで USB-C 側を前方に向け、杖水平振りを Yaw 回転にマッピング、Pitch ≈ 0° 維持 | ユーザーの持ち方変更のみ | 即可、HW/FW 変更不要 |
| **b) IMU 物理向きの調整** | HW 設計時に IMU を 90° 回転して実装、結果として「杖水平振り = Roll 軸」になる | ハードウェア設計 (PCB 起こし直し) | HW 量産前のみ可 |
| **c) ★ Mahony 基準姿勢のキャリブレーション** | 「杖を水平に構えた状態」を **基準姿勢 (q=identity 相当)** として記録 → 以降の Mahony 出力は「基準からの差分」→ 絶対 Pitch ≈ 0° から始まる | FW 設定 (NVS 永続化)、ソフトウェアのみ | 既存 `q.init` / qref 機能で実現可、Web UI 強化のみで対応 |

**アプローチ c は既存 FW 機能で実現可能** ([本 plan の "ユーザー初期姿勢オフセット (実行時、NVS 保存)" セクション参照](#ユーザー初期姿勢オフセット実行時nvs保存)):

```jsonc
{"cmd":"calibrate.qref.set", "name":"wand_horizontal", "source":"current"}  // 杖水平で実行
{"cmd":"calibrate.qref.apply", "name":"wand_horizontal"}                     // これを基準姿勢に
```

→ 以降の Mahony Euler 出力は「杖水平を 0° とした座標系」、Pitch=0 から始まる。

#### ★ ジンバルロック耐性と Yaw drift 耐性は別問題 (★ ユーザー追加指摘で訂正)

ユーザー追加指摘: 「これは絶対 Quat の話ではない。絶対 Quat は Yaw のドリフト誤差が生じる」

前述のアプローチ a/b/c (保持姿勢変更 / IMU 向き調整 / Mahony 基準姿勢キャリブ) は **Euler のジンバルロック回避策**であり、**Yaw drift には効かない**。両者は独立した問題:

| 問題 | 原因 | 影響範囲 | 解決手段 |
|---|---|---|---|
| **ジンバルロック** | Mahony ZYX Euler 抽出で Pitch ≈ ±90° 時に Roll/Yaw が縮退 | Euler 出力のみ (quat 自体は無関係) | 保持姿勢変更 / IMU 向き調整 / Mahony 基準姿勢設定 / judge_by="quat" |
| **Yaw drift** | 磁気センサなしで Mahony Yaw が積分のみ、0.1-1°/分の累積 | **Mahony quat 出力全体** (絶対座標の Yaw 成分が時間とともにずれる) | **q_ref を取り直す (= 相対モード) のみ** |

#### モード × 耐性の完全マトリクス

| モード | judge_by | ジンバルロック耐性 | Yaw drift 耐性 |
|---|---|---|---|
| 絶対 (default) + Euler | euler | ❌ Pitch ±65° 超で縮退 | ❌ 起動からの累積 drift |
| **絶対 + Euler + 基準姿勢キャリブ (アプローチ c)** | euler | ✅ Pitch=0 から始められる | ❌ **Yaw drift は引き続き累積** (基準設定後の経過時間に比例) |
| **絶対 + Quat** | quat | ✅ 完全回避 | ❌ **Yaw drift の影響あり** (★ ユーザー指摘) |
| 相対 + Euler (Phase 5.39 サンプル) | euler | ⚠ q_rel の Pitch ±65° 超で縮退 (振り幅次第) | ✅ state[0] enter で毎回 reset |
| **相対 + Quat** | quat | ✅ 完全回避 | ✅ state[0] enter で毎回 reset (★ 最強組合せ) |

★ **絶対 Quat (`absolute`+`quat`) の限界 (重要、ユーザー指摘で明確化)**:
- 4 次元演算なので Euler 縮退は無関係 → **ジンバルロック完全回避**
- **しかし Mahony quat 自体に Yaw drift が乗っている** → target_quat (登録時の絶対 quat) と sensor.quat (現在の絶対 quat) の **Yaw 基準が時間とともにずれる**
- 内積判定値が時間経過で低下、誤動作の原因
- 起動 30 分後には 5-10° ずれる可能性、長時間使用で誤判定多発
- → 「絶対 Quat = ジンバルロックを解決」だが「Yaw drift は別問題、解決していない」

#### 「絶対 vs 相対」の二項対立を超えた構造 (訂正版)

| モード | q_ref の確定タイミング | q_ref の保持先 | ジンバルロック | Yaw drift |
|---|---|---|---|---|
| 絶対モード (default) | デバイス起動時 / Mahony 初期化時 | Mahony フィルタ内部 (固定) | ❌ (judge_by=euler) / ✅ (judge_by=quat) | ❌ |
| **絶対モード + 基準姿勢キャリブ (アプローチ c)** | ユーザーが Web UI で明示設定 | NVS 永続化 (デバイス固有) | ✅ (杖水平を基準にすれば Pitch=0) | ❌ **drift は止められない** |
| **相対モード (rule 単位、Phase 5.39)** | rule の state[0] enter | RAM (毎回更新) | ✅ q_rel が中央近傍に留まる + judge_by=quat ならさらに完全 | ✅ **毎回 reset で drift 完全回避** |

→ **「アプローチ c は相対モードと等価」という前回の記述は誤り**。両者はジンバルロック耐性は同等だが、**Yaw drift 耐性は相対モードのみ**。

#### 結論: 相対モードの本当の本領

相対モードの 2 つの優位:
1. **ジンバルロック軽減** (q_rel が中央近傍に留まるため、judge_by=euler でも安全領域内)
2. ★ **Yaw drift 完全回避** ★ (state[0] enter で毎回基準を取り直すため、累積 drift が無関係になる)

→ **2 番目こそが相対モードの本質的価値**。1 番目だけならアプローチ c で代替可能だが、2 番目は相対モードでしか実現できない。

#### サンプル `wingardium_hold_relative.json` の最適化提案

現在 `judge_by="euler"` (default)。Phase 5.39.2 後の実機検証次第で `judge_by="quat"` に変更を検討:

| 設定 | ジンバルロック | Yaw drift | UI 直感性 |
|---|---|---|---|
| `relative` + `euler` (現状) | ⚠ q_rel Pitch ±65° 超で警告 | ✅ | ★★★ 軸別 tol |
| **`relative` + `quat`** (検討) | ✅ 完全回避 | ✅ | ★★ 単一 cone tol |

→ どちらも実用範囲、ユーザーがジェスチャ振り幅次第で選べる。Phase 5.39.2 では default を `euler` 維持、上級者向けに UI で `quat` 切替可能。

#### ★ 絶対 Quat の実用化: 動的 Yaw drift 補正機能 (Phase 5.39.3 候補)

ユーザー追加指摘: 「絶対 Quat を使う場合、Yaw ドリフト誤差や動いているときの積分誤差による Yaw 誤差を、静止時またはボタン押すなどにより、適宜補正し、ユーザーが向いている方向と Yaw=0° 等、つまりユーザーの杖の基準 Yaw 向きと M5C 内が保持する Yaw 基準値を合わせる必要がある」

#### 設計案: Quaternion ベースの動的補正 (★ ユーザー指摘で訂正)

**ユーザー指摘**: 「補正トリガー時に `yaw_offset = current_mahony_yaw` ではなく、Quaternion で補正するか、リセットする必要があるはず」 → **完全に正しい**。Euler 差分 `current_mahony_yaw` はジンバルロック領域 (Pitch ≈ ±90°) で縮退するため信頼できない。Quaternion 演算で補正すべき。

#### 数学: Twist-Swing 分解で Yaw 成分を抽出

quat `q = (qw, qx, qy, qz)` の Z 軸 (Yaw) 周り成分:
```
q_yaw = normalize((qw, 0, 0, qz))    // Z 軸 twist 成分 (Yaw 回転のみ)
q_tilt = q ⊗ quat_conj(q_yaw)         // Roll/Pitch 成分 (重力で安定)
```

これは Mahony Euler 抽出と異なり **Pitch ≈ ±90° でも数学的に安定** (asin/atan2 不要、quat 演算のみ)。

#### 3 つの実装案

| 案 | 補正方式 | 内部状態への影響 | 計算負荷 | 推奨度 |
|---|---|---|---|---|
| **B: 判定時 quat 演算で適用** | `q_yaw_corr` (quat) を NVS 保存、判定時に `effective_quat = q_yaw_corr ⊗ sensor.quat` | Mahony 無傷、副作用なし | 判定毎に quat 積 1 回 (~30 ns、ESP32 で無視可) | ★★★ 推奨 |
| A: Mahony 内部 quat の Yaw 成分のみリセット | Mahony 内部 `q_mahony = q_tilt` に置換、Yaw 履歴消去 | Mahony 補正履歴の Yaw 成分失う、Roll/Pitch は維持 | 一度きり (補正トリガー時のみ) | 中 |
| C: Mahony 内部 quat 全置換 (= 強制 Yaw=0) | `q_mahony` を強制的に q_tilt に書換 | 過去履歴喪失、再収束に少時間 | 一度きり | 低 (副作用大) |

#### 案 B (推奨) の実装案

```cpp
// FW 内、NVS 永続化対象
static float yaw_corr[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // identity (= 補正なし)

// 補正トリガー時 (yaw.calibrate コマンド or 静止検出時)
void yawCalibrate(const float sensor_quat[4]) {
    // Z 軸 twist 成分を抽出 (Twist-Swing decomposition)
    float qw = sensor_quat[0], qz = sensor_quat[3];
    float norm = sqrtf(qw * qw + qz * qz);
    if (norm < 1e-6f) {
        // 異常系: q が垂直 Pitch 領域でも twist 抽出は理論上可能だが安全のため identity
        yaw_corr[0] = 1; yaw_corr[1] = 0; yaw_corr[2] = 0; yaw_corr[3] = 0;
        return;
    }
    // 現在の Yaw quat (Z 軸 twist 成分)
    float q_yaw_current[4] = { qw / norm, 0.0f, 0.0f, qz / norm };
    // 補正 quat = 現在 Yaw quat の逆 (これを乗じれば Yaw=0 相当)
    quatConjugate(q_yaw_current, yaw_corr);
    // NVS に永続化 (オプション、`nvs_set_blob("yaw_corr", yaw_corr, 16)`)
}

// 判定時 (judge_by="quat" + absolute + Yaw 補正有効時)
float effective_quat[4];
quatMultiply(yaw_corr, sensor_quat, effective_quat);
// effective_quat = 「補正時を Yaw=0°」とした座標系の quat
// 以降 target_quat との内積判定: |dot(target_quat, effective_quat)| ≥ cos(tol/2)
```

#### 案 B の利点

- **Pitch ≈ ±90° でも数学的に安定** (Twist-Swing は quat 演算のみ、特異点なし)
- **Mahony 内部状態は無傷** (Roll/Pitch の安定性に影響しない、過去履歴も維持)
- **NVS 永続化可能** (yaw_corr 16B を blob 保存、再起動後も補正値を保持)
- **judge_by="quat" との組合せが自然** (両方とも 4 次元 quat 演算で完結)
- 判定負荷: quat 積 1 回 (~30 ns @ ESP32 240MHz)、100Hz × N=30 rule でも 0.1% 未満

| 項目 | 内容 |
|---|---|
| FW 内 state | `yaw_corr[4]` (float、NVS 永続化または RAM)、default identity (1,0,0,0) |
| 補正計算 | 完全に quat 演算: `q_yaw_current = twist_z(sensor.quat)`、`yaw_corr = quat_conj(q_yaw_current)` |
| 判定時の適用 | `effective_quat = yaw_corr ⊗ sensor.quat` (judge_by="quat" 時)、**Euler 抽出を一切経由しない** |
| 補正されない成分 | q_tilt (swing 成分) — Twist-Swing 分解で抽出される「重力方向に対する傾斜」を表す純 quat、Mahony の加速度補正で安定維持される (= Yaw drift の影響を受けない quat 成分) |

#### Twist-Swing 分解 (Swing-Twist Decomposition) の詳細

3D 回転 quaternion を **指定軸周りの捻り (twist)** と **その軸の傾き (swing)** に分けて表す数学手法。

##### 直感的イメージ (Z 軸 twist の場合)

```
   元の状態          Twist (Yaw)          Swing (傾き)
                 Z 軸周りに回す         Z 軸が別方向を向く

       │                │                       \
       │                │              ───→      \
       │     →          │   ↻                    \
       │                │                         \
   ━━━━━━━━━        ━━━━━━━━━              ━━━━━━━━━

   identity        Yaw 回転だけ           Roll/Pitch 的傾き
```

任意の quat `q` は **「q_swing で先に傾けて、q_twist で捻る」** に分解可能:
```
q = q_swing ⊗ q_twist     (合成順、conventions が複数あり)
```

##### 厳密な定義 (Z 軸 twist)

```
q = (qw, qx, qy, qz)
norm = sqrt(qw² + qz²)
q_twist = (qw / norm, 0, 0, qz / norm)   // Z 軸 twist (純 quat、純 Yaw 回転)
q_swing = q ⊗ quat_conj(q_twist)          // swing 成分 (純 quat、Z 軸を別方向に向ける回転)
```

検証: `q_swing ⊗ q_twist = q` (元の q を再構成)

##### 各成分の物理的意味

| quat 成分 | 表す回転 | 物理 | drift 性質 (Mahony 6 軸 IMU) |
|---|---|---|---|
| `q_twist` (Z 軸 twist) | Z 軸周り回転だけ | 水平面内の旋回 (= Yaw 相当) | **drift する** (磁気センサなし) |
| `q_swing` (swing) | Z 軸そのものを別方向に向ける | 重力に対する傾斜 (Roll/Pitch 的成分) | **安定** (Mahony の加速度補正で校正) |

##### Euler 抽出より優れる理由

| 観点 | Euler 抽出 (ZYX) | Twist-Swing 分解 |
|---|---|---|
| 計算式 | `pitch = asin(...)`、`roll/yaw = atan2(...)` | `q_twist = normalize((qw, 0, 0, qz))` |
| Pitch ≈ ±90° での動作 | **縮退** (asin 頭打ち、atan2 分母 0、Roll/Yaw が暴れる) | **数値安定** (norm=0 は q が垂直の極限のみ) |
| 必要関数 | asin、atan2 (各 ±90° で特異点) | sqrt、除算 (特異点なし) |
| 計算負荷 | ~80 ns/姿勢 | ~30 ns/姿勢 |

##### Burst Motion 文脈での意義 (Yaw drift 補正の理論基盤)

Mahony 6 軸 IMU では:
- 加速度センサで重力ベクトルを観測 → 鉛直軸 (Z) を補正基準にする
- **q_swing (鉛直軸からの傾き)** は加速度補正で安定維持
- **q_twist (Z 軸周り回転)** は磁気センサなしで drift する

→ Twist-Swing 分解で q_twist を取り出して逆乗算すれば、**Yaw drift だけを Quaternion 演算で除去** できる:
```
q_yaw_corr = quat_conj(twist_z(sensor.quat))   // 補正トリガー時に保存
effective_quat = q_yaw_corr ⊗ sensor.quat      // 判定時に適用
// → effective_quat の twist 成分は 0、swing 成分は元のまま
```

これがジンバルロック領域でも安定動作する理由 — Euler 抽出による「Yaw 値」ではなく、quat 演算で直接 q_twist (Yaw 成分) を取り出して打ち消すため。

##### 参考実装 (FW C++)

```cpp
void twistSwingDecomposeZ(const float q[4], float q_twist[4], float q_swing[4]) {
    float qw = q[0], qz = q[3];
    float norm = sqrtf(qw * qw + qz * qz);
    if (norm < 1e-6f) {
        // q が垂直 (Pitch ≈ ±90°、qw=qz=0) の極限、稀
        q_twist[0] = 1; q_twist[1] = 0; q_twist[2] = 0; q_twist[3] = 0;
        for (int i = 0; i < 4; i++) q_swing[i] = q[i];
        return;
    }
    q_twist[0] = qw / norm;
    q_twist[1] = 0;
    q_twist[2] = 0;
    q_twist[3] = qz / norm;
    float q_twist_conj[4];
    quatConjugate(q_twist, q_twist_conj);
    quatMultiply(q, q_twist_conj, q_swing);
}
```

##### 参考文献

- Verth & Bishop, "Essential Mathematics for Games and Interactive Applications", 2008
- Ken Shoemake, "Animating Rotation with Quaternion Curves", SIGGRAPH 1985
- Unity / Unreal Engine のジョイント制限実装 (Swing-twist hinge) で標準採用

---

#### Phase 5.39.3 でさらに議論すべき技術論点 (★ 設計詳細詰め)

Phase 5.39.3 (Yaw drift 補正機能) を本格実装する前に整理すべき論点:

##### 論点 1: 複数 rule 間の yaw_corr 共有
- yaw_corr はデバイス全体で 1 つ (NVS 永続化) → 全 rule に同時適用
- rule ごとに異なる Yaw 基準を持たせるなら、相対モード (rule.q_ref で個別管理) を使うべき
- 絶対 Quat + Yaw 補正は「セッション全体で同じ Yaw 基準を使う」前提
- 各 rule に `apply_yaw_corr: true/false` フラグを追加するか?

##### 論点 2: rule 登録時の Yaw 補正値の扱い
- Web UI で「現在の姿勢を target に登録」するとき、登録時の Yaw drift も補正済の値で記録すべきか、生の値か
- 案 A: 補正済 (effective_quat) で記録 → 登録時と判定時の補正基準が一致して整合的
- 案 B: 生 (sensor.quat) で記録 → 登録時の絶対座標が保持される、yaw_corr が変わると判定基準も変わる
- → **案 A 推奨** (補正済で記録、yaw_corr 変更後も登録時の物理姿勢が成立)

##### 論点 3: 補正トリガーの優先順位 / 競合
- 自動補正 (静止検出) と手動補正 (ボタン) の同時発生
- 補正中の rule 評価との干渉 (state 維持中に補正されると q_ref との不整合)
- → 補正は「全 rule が idle 状態のときだけ」許可する制約を入れる

##### 論点 4: 補正瞬間の判定値の不連続性
- 補正前後で effective_quat が突然変化 → 内積判定値が跳ねる → 微妙な閾値で誤動作
- 滑らかな移行 (Slerp 補間で数 100ms かけて yaw_corr 更新) が必要か?
- → MVP では一括更新、Phase 5.39.4 で滑らか化を検討

##### 論点 5: NVS 永続化 vs RAM のみ
- 案 A: NVS 永続化 → 再起動後も同じ yaw_corr で動作、ユーザーが意図した基準を維持
- 案 B: RAM のみ → 毎起動で identity (補正なし) にリセット、ユーザーが毎回補正
- → **案 A 推奨** (NVS 永続化、ただし「起動時に yaw_corr を identity にリセット」オプションも UI で提供)

##### 論点 6: 量産版 IMU (LSM6DSV16X / BMI270) との整合
- LSM6DSV16X は IC 内 SFLP (Sensor Fusion Low-Power) で quat 直出力可
- BMI270 は生データのみ → MCU 側 Mahony 同様
- どちらも Twist-Swing 分解 + yaw_corr 機構はそのまま流用可能 (quat 演算は IMU 機種非依存)
- → 量産版でも同じコードベースで動作、Phase 6+ HW 移行で問題なし

##### 論点 7: Mahony 内部 q_ref (= 既存 calibrate.qref.*) との関係
- 既存 `calibrate.qref.set` は **quat 全体 (Roll/Pitch/Yaw 全成分)** を再基準化、NVS 永続化
- 新 `yaw.calibrate` は **q_twist (Yaw 成分のみ)** を再基準化
- 両者は補完関係: qref で重力アライメント + 杖水平に固定、yaw で動的 drift 補正
- UI で両機能を分けて提示 (混同しないように)

##### 論点 8: Web 側可視化 (Phase 5.39.2 タブとの統合)
- 「絶対 + Yaw 補正」モードの 3D ビュア表現
- yaw_corr の値を 3D 球面に小さなマーカーで表示 (基準方向の可視化)
- 補正前後で waypoint の位置が変わるので、ユーザーが理解しやすいアニメーション

#### 補正トリガー (4 種類、組合せ可)

| トリガー | タイミング | 用途 |
|---|---|---|
| 起動時自動 | デバイス起動直後 | yaw=0 で再スタート、起動時方向が基準 |
| **静止検出時自動** (WB 方式互換) | accel ≈ 1g & gyro ≈ 0 が N 秒継続 | ユーザーが向いた方向で自動補正、無意識 UX |
| **ボタン押下時明示** | 専用キー長押し / Web UI ボタン | 「今を基準にする」明示 |
| rule 評価中の state[0] enter | hold_with_waypoints の state[0] enter 時、`yaw_calibrate_on_state0: true` フラグあり | ジェスチャ開始で Yaw 整列、相対モードに近い動作 |

#### 実装案 (FW)

```cpp
// src/core/TriggerEngine.cpp 拡張
static float yaw_offset = 0.0f;   // NVS 永続化 (`nvs_set_blob("yaw_offset", ...)`)

void yawCalibrate(float current_yaw) {
    yaw_offset = current_yaw;
}

// 判定時 (judge_by=quat + absolute、Yaw 補正有効時)
// quat 補正方式:
float q_yaw_corr[4];
quatFromAxisAngle({0, 0, 1}, -yaw_offset * DEG2RAD, q_yaw_corr);
float effective_quat[4];
quatMultiply(q_yaw_corr, sensor.quat, effective_quat);
// 以降 effective_quat と target_quat の内積判定
```

新 JSON コマンド:
```jsonc
{"cmd":"yaw.calibrate"}                                  // 現在の Yaw を 0 に補正
{"cmd":"yaw.calibrate", "trigger":"auto_stillness", "window_ms":1000}  // 自動補正設定
{"cmd":"yaw.get"}                                        // 現在の yaw_offset を取得
{"cmd":"yaw.reset"}                                      // yaw_offset = 0 に戻す
```

#### 「絶対 Quat + Yaw 補正」をモード分類に組込み

絶対 / 相対の二項対立を超えた **第 3 のモード**:

| モード | q_ref 確定 | Yaw drift 対策 | 用途 |
|---|---|---|---|
| 絶対 (default) | 起動時 | ❌ 補正なし | 起動直後 / Yaw 不要な用途 |
| **絶対 + Yaw 補正 (★ 新規 Phase 5.39.3)** | yaw_offset を NVS 永続化 + 動的補正 | ✅ 補正トリガーで都度リセット | Yaw を使うが頻繁な補正で精度維持 (5-10 分セッション) |
| 相対モード (Phase 5.39) | rule の state[0] enter | ✅ q_ref で都度リセット | ジェスチャごとの基準切替 |

#### 「絶対 + Yaw 補正」と「相対モード」の使い分け

| 特性 | 絶対 + Yaw 補正 | 相対モード |
|---|---|---|
| 補正単位 | デバイス全体 (1 つの yaw_offset) | rule ごと (rule.q_ref) |
| 補正タイミング | 明示 / 静止 / 起動時 | rule の state[0] enter で自動 |
| 補正範囲 | **Yaw のみ** (Roll/Pitch は無補正) | quat 全体 (Roll/Pitch/Yaw 全て) |
| 永続化 | yaw_offset を NVS 保持 (再起動後も保持) | RAM のみ |
| 用途 | 絶対座標で位置を保ちたいが Yaw drift 嫌 (例: 「常に北を向くカーソル」、絶対 quat rule を多数登録するセッション) | ジェスチャの「形」だけ判定 (例: Wingardium 振り) |

#### 既存 FW 機能との関連

- 既存 `calibrate.qref.set` / `calibrate.qref.apply` は **quat 全体を再設定** (Roll/Pitch/Yaw 全て)、起動時の Mahony 基準姿勢全体を変える (アプローチ c)
- 新 `yaw.calibrate` は **Yaw のみ補正** (Roll/Pitch は重力で安定維持、無補正)
- 両者は補完関係、共存可能。`qref.set` が一度きりの基準設定、`yaw.calibrate` が動的な Yaw drift 補正

#### Phase 配置

- **Phase 5.39.2 (今、実装中)**: スコープ外 (相対モード rule の UI 洗練に集中)
- **Phase 5.39.3 (次候補)**: Yaw drift 補正機能を追加
  - FW: `yaw.calibrate` コマンド、yaw_offset NVS 永続化、静止検出自動補正、quat 演算で実装
  - Web: 「📍 Yaw を 0 に補正」ボタン (Live View タブに常時表示)、自動補正 ON/OFF トグル、現在の yaw_offset 表示

→ この機能の追加により、**絶対 Quat (`absolute`+`quat`) + Yaw 補正** が「持ち方は固定だが Yaw drift は許容したくない」用途に対する有力な選択肢となる。相対モードに頼らず絶対座標を維持しつつ Yaw drift を克服。

#### 設計上の含意 (Phase 5.39.2 への反映)

| 観点 | 影響 |
|------|------|
| `wingardium_hold_euler.json` (絶対モード) の評価修正 | **基準姿勢を「杖水平」にキャリブレーションすれば実用可能**。前述の「実運用困難」評価は厳しすぎた |
| `wingardium_hold_relative.json` (相対モード) の評価 | **毎回 q_ref が自動取得されるため、基準姿勢キャリブレーション不要**。気軽に使える |
| Web UI で目立たせる機能 | **「📍 現在の姿勢を基準にする (絶対モード rule の Mahony 基準)」ボタンを Live View タブに常時表示** (既存 `calibrate.qref.set` の UX 強化) |
| サンプルプロファイル説明 | 「このサンプルは基準姿勢を**杖水平**にキャリブレーションしてから使うか、相対モード版を使ってください」のヒント表示 |
| 既存 `q.init` / `calibrate.qref.*` FW 機能 | 既に実装済み (本 plan 「ユーザー初期姿勢オフセット」セクション参照)、Web UI からの呼出 UX 強化のみで対応可能 |

→ Phase 5.39.2 のサンプルは「絶対モード版 + 相対モード版」両方を残す価値あり (前者は「基準姿勢キャリブレーション」を覚えたいユーザー向け、後者は気軽に使いたいユーザー向け)。

**前提の限定性**:
- M5StickC LCD 左持ち縦持ち (基本姿勢 R+90/P0)
- M5StickC 内蔵 MPU6886 の axes 配置
- Mahony フィルタの ZYX intrinsic Euler 抽出
- 杖として振る用途
- → 他の機種 / IMU 取付方向 / フィルタ計算方法では異なる可能性あり

**杖用途で絶対モードの限界が顕在化**:
- 杖先端 (USB-C 側) を前 / 下 / 上に振る → 絶対 Pitch が -90° 付近に到達することが多い
- これは Mahony ZYX 抽出で絶対 Euler の Roll/Yaw が縮退する領域
- → 絶対モード rule (`posture_basis=absolute` + `judge_by=euler`) は **杖振り用途では実用困難**

**相対モードが杖用途の本命となる理由**:
- q_ref を「振り始め (杖を構えた姿勢、絶対 Pitch ≈ -90° でも OK)」に取れば、**q_rel の Pitch は 0 近傍から始まる**
- 杖振り中の q_rel は ±60° 程度に収まりやすい (安全領域)
- judge_by="euler" でもジンバルロック領域に入らずに判定可能
- → **杖用途は相対モード rule が本命**、絶対モード rule はサンプル比較用途に留める

**設計上の含意**:
- `wingardium_hold_relative.json` (相対モード) は **ユーザー環境での実用 default サンプル**
- `wingardium_hold_euler.json` (絶対モード) は「絶対モードの限界を示す比較対象」として残す
- Web UI の Library タブで、杖用途サンプルには「相対モード推奨」のヒント表示を検討 (Phase 5.39.3+)
- 他の用途 (FPS WASD / プレゼン / メディアリモコン) は基本姿勢で操作するため絶対モードで十分

---

### 相対モードの判定方式 詳細 (★ ジンバルロックとの関係を明確化 ★)

ユーザー指摘: 「相対 Euler という表示だが、相対クォータニオンで実際は計算されるのか？相対 Euler はジンバルロックのために使えないはずである」

#### 内部計算の正確な順序

[TriggerEngine.cpp:446-477](M5C_MPU6886_cpp/src/core/TriggerEngine.cpp#L446-L477) の実装:

```
Step 1: q_rel = quat_conj(q_ref) ⊗ q_current     ← 完全にクォータニオン演算、4 次元、ジンバルロック無関係
Step 2a (judge_by="quat"):   |dot(q_rel, target_quat)| ≥ cos(tol/2)   ← ジンバルロック完全回避
Step 2b (judge_by="euler"):  (Δr, Δp, Δy) = quatToZYXEuler(q_rel)     ← ここで Pitch ≈ ±90° なら縮退
                              各軸 |Δi - target.euler[i]| < tol[i]
```

→ **内部の主計算は完全に相対クォータニオン**。Euler は judge_by="euler" 時の最終比較段だけで使用。

#### 「相対 Euler はジンバルロックのため使えない」は半分正しい

| 観点 | 状況 | 結論 |
|------|------|------|
| q_rel 計算自体 | quat 積で 4 次元演算 | **ジンバルロック無関係** |
| q_rel → Euler 抽出 | q_rel の Pitch が ±90° 近傍 | **縮退、Roll/Yaw が不安定** |
| **相対モードでの軽減効果** | q_rel は「振り始めからの差分」なので、通常ジェスチャ (±60° 振り) では q_rel Pitch も ±60° 以内 → ジンバルロック領域に入りにくい | △ 軽減はする、完全回避ではない |
| **大振り** (例: Pitch ±70° 超のジェスチャ) | q_rel の Pitch も大きくなり Euler 抽出が縮退 | judge_by="quat" を使うべき |

#### 絶対モードとの比較

| モード | ジンバルロック発生条件 |
|--------|---------------------|
| 絶対 Euler (`absolute`+`euler`) | **絶対 Pitch ≈ ±90°** (M5C を真上/真下に向ける) — M5StickC LCD 左持ち基本姿勢 R+90/P0 から大きく Pitch 振ると簡単に到達 |
| **相対 Euler** (`relative`+`euler`) | **q_rel の Pitch ≈ ±90°** (振り始め基準から ±90° 回す) — 通常のジェスチャでは到達しにくい |
| 絶対 Quat (`absolute`+`quat`) | 発生しない (= 完全回避) |
| **相対 Quat** (`relative`+`quat`) | **発生しない (= 完全回避)** ★ |

→ **「相対 Quat (`relative`+`quat`)」が最も堅牢**。ただし軸別 tol が指定できないため、UI 設定の柔軟性は下がる。

#### Wingardium サンプル値の検証

`wingardium_hold_relative.json` の waypoint:
- start_posture: `[0, 0, 0]` (q_rel = identity、Pitch 0°)
- mid_postures[0]: `[30, 0, 0]` (q_rel Pitch 0°、安全)
- mid_postures[1]: `[60, -30, 0]` (q_rel Pitch 30°、安全)
- end_posture: `[0, -30, 0]` (q_rel Pitch 30°、安全)

→ q_rel の Pitch が ±60° 以内に収まるため、現状の `judge_by="euler"` でも問題なし。

#### Phase 5.39.2 で UI に追加する警告

相対モード rule の waypoint キャプチャ時、**q_rel の Pitch が ±65° を超える場合に警告**:

```
⚠ この waypoint は q_rel の Pitch=78° で、相対 Euler 判定でもジンバルロック領域です。
   判定基準を「Quaternion 内積」に切替えるか、振り方を ±65° 以内に収めてください。
```

Phase 5.38 の絶対モード警告と同じ仕組み (postureUsePitch + postureJudgeBy 判定) を相対モードにも適用。

#### UI 表示の正確な表記

「相対 Euler」という表記は誤解を招くため、UI では以下のように明示:

| UI 要素 | 表記 |
|---------|------|
| 判定基準セレクタ | 「**相対 Quaternion (内部処理、最終比較は Euler 軸別 or Quat dot)**」 |
| waypoint 値表示 (judge_by=euler) | 「Δ R+30 Δ P-15 Δ Y0 ° (q_rel から抽出)」 |
| waypoint 値表示 (judge_by=quat) | 「q_rel cone ≥ 0.95 (≈ ±18°)」 |
| 説明バナー | 「相対モード: q_ref を基準として q_rel = q_ref⁻¹ ⊗ q_current を計算 (クォータニオン演算、4 次元)。最終比較は Euler 軸別 (UI 直感) または Quat 内積 (ジンバルロック完全回避) を選択可。」 |

#### Phase 5.39.3 以降の検討

**forward vector + twist 分解判定** (plan の候補 C):
- クォータニオンを「主軸方向ベクトル (杖先端の向き)」と「主軸周りの捩じれ (twist)」に分解
- 主軸 cone tol (= 杖先端がどこを向くか) + twist tol (捩じれ許容) で判定
- 軸別 tol と Quat dot の利点を兼ね備える、ただし実装複雑
- 需要次第で Phase 5.39.3+ 検討

---

### データ保持と判定の役割分担 (★ FW 単独動作の設計原則 ★)

ユーザー質問: 「相対モード rule の登録姿勢はどちらが保持するのか？判定は M5C と Web 側のどちらで行うのか？」

#### 役割分担表

| 項目 | 保持先 / 実行場所 |
|------|------------------|
| rule 定義 (ui_mode, posture_basis, start/mid/end_posture.euler, key, button_idx 等) | **FW: LittleFS 永続化 + RAM `ActionRule` 配列** |
| `q_ref` (実行時の基準姿勢、相対モード時 state[0] enter で snapshot) | **FW: RAM のみ `ActionRule::q_ref[4]`、永続化なし** |
| **判定アルゴリズム** (q_rel 計算 → Euler 抽出 → tol 比較) | **FW: 100Hz tick で完全実行** (TriggerEngine.cpp の `evalPostureRelative`) |
| HID 出力 (キー press/release) | **FW: BLE HID / USB HID** |
| 登録時の `qRefCapture` (Web UI で姿勢キャプチャ作業用) | **Web: 一時 useRef、rule.add 送信後は破棄** |
| 軌跡描画用 q_ref (Phase 5.39.2 で trigger.hit event 受信予定) | **Web: FW から受信、可視化のみ (判定には使わない)** |

#### 設計原則: FW 単独動作

**判定は FW 側で完結** — Web 接続が切れていても M5C 単体で動作:
- M5C はスタンドアロン BLE HID デバイス
- PC / スマホは **設定時にだけ** Web Serial / Web Bluetooth で接続
- ゲーム中は M5C と PC が BLE HID で直接通信、Web 不要
- Web の役割は「設定」「可視化」「デバッグ」のみ、判定ロジックは持たない

#### 登録時と実行時の対称性

| 段階 | q_ref の決定者 | データの流れ |
|------|--------------|------------|
| **登録時** (Web で姿勢キャプチャ) | Web 側 `qRefCaptureRef.current` | ユーザーが M5C を構える → Web が現在 sensor.quat を保存 → 各 waypoint の sensor.quat を `quat_conj(qRefCapture) ⊗ q_current` で相対 Euler 化 → `rule.add` JSON で `target.euler = 相対値` を FW に送信 → FW が LittleFS に保存 |
| **実行時** (FW 評価) | FW 側 `rule.q_ref` | ユーザーが M5C を構えて Btn3 押下 → FW が state[0] enter で `rule.q_ref = sensor.quat` snapshot → 各 tick で `q_rel = quat_conj(rule.q_ref) ⊗ sensor.quat` を計算 → target.euler と tol 比較 → 全 waypoint 通過で AT_RELEASE |

**重要**: 登録時の `qRefCapture` (Web) と実行時の `rule.q_ref` (FW) は **別の量** だが、どちらも「振り始めの基準姿勢」を意味する。両者が違っていても、振りの「形」(相対 Euler 軌跡) が一致すれば成立 → これが相対モード rule の本質。

#### Web ↔ FW 通信プロトコル (相対モード関連)

| 方向 | コマンド / イベント | データ | タイミング |
|------|------------------|--------|----------|
| Web → FW | `rule.add` | `target.euler` (相対値、登録時に Web 計算済) | rule 登録時 |
| Web → FW | `profile.save` | rule リスト全体 (target.euler は相対値で保存) | プロファイル保存時 |
| FW → Web | `trigger.hit` (phase=enter) | **`q_ref[4]`** (実行時の snapshot、Phase 5.39.2 で追加) | state[0] enter 時 |
| FW → Web | `rule.list` | 各 rule の posture_basis / states[].posture.euler (相対値) | Web の rule 一覧取得時 |

→ Phase 5.39.2 で追加する `trigger.hit` の q_ref は **可視化用のみ**、FW の判定ロジックには影響しない。Web が切断されていても FW は普通に動く。

### 相対モード rule の登録フロー (★ ユーザー指摘の運用課題への解答 ★)

**問題**: 相対モード rule の `target.euler` は「q_ref からの相対オフセット」だが、ユーザーが「R+30°、P-15°」のような相対値を直接入力するのは直感的でない。どうやってあらかじめ登録するか？

**解決方針**: **Web UI 側で q_ref_capture を自動保持し、ユーザーは姿勢を順次キャプチャするだけ**。相対値計算は Web が自動で行う。

#### キャプチャフロー (UI 設計)

```
[ HOLD with Waypoints モードに切替 ]
[ 詳細設定 → 判定基準: 相対 Quaternion を選択 ]

★ Web UI のキャプチャ手順 (相対モード時、ガイド表示あり):
  Step 1: 杖を「振り始め」の姿勢に構えて
          [ 📷 開始姿勢 ]
          → 現在 sensor.quat を Web 側 useState/ref `qRefCapture` に保存
          → startPosture.euler は [0, 0, 0] に設定 (ref からゼロオフセット)

  Step 2: 杖を「中間 1」の姿勢に動かして
          [ 📷 中間姿勢 1 ]
          → 現在 sensor.quat と qRefCapture から相対 Euler を計算
          → midPostures[0].euler = quatToEuler(quatConj(qRefCapture) ⊗ sensor.quat)
          → 表示: 「Δ R+30°, Δ P-15°, Δ Y0°」(相対値表示)

  Step 3-4: 中間 2、終了姿勢も同様

  [ rule.add 送信 ]
  → start_posture.euler = [0,0,0]、mid_postures[i].euler = 相対 Euler、end_posture.euler = 相対 Euler
  → FW では target.euler が「ref 相対値」として保存され、実行時の q_ref と組合せて判定
```

#### FW 動作との対応 (登録時 vs 実行時)

| タイミング | q_ref の決定者 | 用途 |
|----------|-------------|------|
| **登録時 (Web UI でキャプチャ)** | Web 側 `qRefCapture` (開始姿勢キャプチャ時に保存) | 各 waypoint の相対 Euler を計算して rule.add 送信 |
| **実行時 (FW で評価)** | FW 側 `rule.q_ref` (state[0] enter で sensor.quat を snapshot) | 各 tick で `q_rel = quat_conj(rule.q_ref) ⊗ sensor.quat` を計算、target.euler と比較 |

**対称性の保証**:
- 登録時の「振り始め姿勢」(qRefCapture) と実行時の「state[0] enter 時姿勢」(rule.q_ref) が**異なっていても、ジェスチャの「形」が同じなら成立**
- ユーザーは持ち方や向きを自由に変えてよい、Yaw drift 影響なし
- これが相対モード rule の本質的優位

#### Web UI の表示差別化

| モード | start_posture 表示 | mid_postures 表示 | 内部値 |
|--------|-----------------|------------------|--------|
| 絶対 (`absolute`) | `R:120 P:0 Y:0` | `R:120 P:30 Y:0` | 絶対 Euler (Mahony 起動基準) |
| **相対** (`relative`) | `📍 ref (基準点)` | `Δ R+30 Δ P-15 Δ Y0` | 相対 Euler (q_ref_capture からの差) |

UI:
- 開始姿勢キャプチャボタンの表記: 絶対モード = `📷 開始姿勢`、**相対モード = `📷 開始姿勢 (基準点として記録)`**
- 中間姿勢のラベル: 絶対 = `R:120 P:30`、**相対 = `Δ R+30 P-15`** (符号付き、ref からの差を明示)
- 「相対モード時の説明バナー」を強化:
  ```
  💡 相対モード: 開始姿勢を基準点 (q_ref) として記録、以降の姿勢は基準からの相対回転で判定。
     持ち方を変えても同じジェスチャが成立。Yaw ドリフト無関係。
  ```

#### 実装変更項目 (app.js + plan 内 captureXxxPosture 関数)

新 useRef: `const qRefCaptureRef = useRef(null);  // [qw,qx,qy,qz] 相対モード時の開始姿勢キャプチャ`

```js
const captureStartPosture = () => {
  if (!sensor) { alert('Stream ON で'); return; }
  if (postureBasis === 'relative') {
    // 相対モード: 現在 quat を qRefCapture として保存、start_posture.euler は [0,0,0]
    qRefCaptureRef.current = [sensor.qw, sensor.qx, sensor.qy, sensor.qz];
    setStartPosture({
      euler: [0, 0, 0],
      euler_tol: buildPostureTol(),
      quat: [1, 0, 0, 0],   // identity
    });
  } else {
    // 絶対モード (現状通り)
    setStartPosture({
      euler: [sensor.roll, sensor.pitch, sensor.yaw],
      euler_tol: buildPostureTol(),
      quat: [sensor.qw, sensor.qx, sensor.qy, sensor.qz],
    });
  }
};

const captureMidPosture = (index) => {
  if (!sensor) { alert('Stream ON で'); return; }
  let euler, quat;
  if (postureBasis === 'relative' && qRefCaptureRef.current) {
    // 相対 Euler を計算: rel_quat = quat_conj(qRefCapture) ⊗ sensor.quat
    const qRel = quatMultiply(quatConj(qRefCaptureRef.current),
                              [sensor.qw, sensor.qx, sensor.qy, sensor.qz]);
    euler = quatToZYXEuler(qRel);   // deg、相対値
    quat = qRel;
  } else {
    euler = [sensor.roll, sensor.pitch, sensor.yaw];
    quat = [sensor.qw, sensor.qx, sensor.qy, sensor.qz];
  }
  setMidPostures(prev => {
    const copy = [...prev];
    copy[index] = { euler, euler_tol: buildPostureTol(), quat };
    return copy;
  });
};

// captureEndPosture も同様
```

#### サンプルプロファイル更新

`wingardium_hold_relative.json` の各 waypoint:
- `start_posture.euler: [0, 0, 0]` (基準点、現状の値と整合 ✅)
- `mid_postures[].euler` は ref からの相対値で記述 (現状の値 [30,0,0] / [60,-30,0] はそれっぽいが、実機で振ってキャプチャした実値で更新する想定)
- end_posture も同様

ユーザーが Web UI で「Wingardium Leviosa の swish & flick」を実際に振ってキャプチャすると、自分の振り方に合った相対 Euler 値が自動で保存される。

#### Phase 5.39.2 で実装する関連項目 (追加)

実装項目に以下を追加:

I. **相対モード キャプチャの自動変換 (app.js)**
- `qRefCaptureRef` 追加
- `captureStartPosture` / `captureMidPosture` / `captureEndPosture` に postureBasis 分岐
- quat ヘルパー関数 (quatConj, quatMultiply, quatToZYXEuler) を共通モジュール化 (Web 側、IMUViewer.js のものを流用 or 独立)

J. **UI 表示の差別化**
- 相対モード時のキャプチャボタンラベル変更
- 相対モード時の waypoint 値表示を「Δ R±... Δ P±...」形式に
- 相対モード説明バナーの強化

### 検証手順

1. **Euler モード rule (wingardium_hold_euler) を選択** → 「🌐 絶対 3D」「📐 2D Roll-Pitch」両タブにハイライトドット ● 表示
2. 「絶対 3D」タブで紫実線軌跡 (waypoint 4 個 Slerp 接続)、「2D Roll-Pitch」タブで番号付き waypoint と矩形 tol 表示が両方確認できる
3. **相対 rule (wingardium_hold_relative) を選択** → 「🌐 絶対 3D」「👁 相対 3D」両タブにハイライトドット ●、絶対 3D は自分の姿勢確認用、相対 3D が rule 軌跡可視化メイン
4. 「相対 3D」タブで、M5C を動かすと **ワールド (waypoint 群、軸、軌跡) が逆回転して見える** = 一人称視点、M5C モデルは中央固定
5. Btn3 押下前 → waypoint 軌跡が **薄い破線** (現在姿勢を仮 q_ref として配置)、M5C を動かすと waypoint がリアルタイムに動く
6. Btn3 押下 → **濃い実線**に切替、ボタン離すまで固定 (FW から受信した実 q_ref で固定配置)
7. M5C モデル下の「☐ q_ref に定着」を ON → ボタン押下後、M5C モデルが ref からの相対回転を示す (三人称視点モード)
8. 振り終わって過去 trail (赤線) が球面に 3 秒残る (両 3D ビュアとも、絶対 = 世界固定、相対 = 世界が動く)
9. 「📐 2D Roll-Pitch」タブで既存挙動が変わらないことを確認 (回帰)、相対モード rule を選んでも waypoint は描画されない (絶対座標で計算できないため、Phase 5.39.3 の 2D 相対 Quat 待ち)

### Phase 5.39.3 以降に持ち越し

- **2D 相対 Roll-Pitch ビュア** (現在 Roll/Pitch を中心 (0,0)、waypoint が相対オフセット位置)
- D 案 (判定進捗バー、状態プログレス可視化)
- 設定 UI (trail 持続時間スライダ等)
- 相対モード rule の waypoint 個別キャプチャ時の「相対値」表示 (例: 「ref から R+30°、P-15°」のような UI フィードバック)
- 相対ビュアでの「絶対モード rule」表示 (今は posture_basis に応じて自動振分け、両方表示する場面は想定外)

---

### 本セッションでのユーザー要求事項のまとめ (★ 確認用 ★)

| # | ユーザー要求 | plan 反映箇所 |
|---|------------|--------------|
| 1 | Hold Start End モードに中間姿勢を追加 | 設計サマリ / FW 変更 / Web 変更 |
| 2 | 中間 0-2 個可変 (現状 States[4] 範囲内) | 設計サマリ |
| 3 | 絶対 Euler / 相対 Quat の両方併存 | データモデル / 判定基準切替 |
| 4 | 相対モードの ref 起点 = state[0] match (button or posture or 静止で自由に組める) | 開始/終了トリガ表 |
| 5 | Web UI 複雑化を抑制 | UI 複雑度抑制ガイドライン |
| 6 | 状態遷移 vs ML vs 各方式の比較 | 各判定アーキテクチャの詳細比較・解説 |
| 7 | Kano / WB の方式調査 | Kano/WB 再評価 + 6 象限マトリクス |
| 8 | 「絶対」「相対」の定義明確化 (地磁気とは別) | 用語の定義 |
| 9 | 近傍判定アルゴリズムの考察 | 近傍判定アルゴリズム |
| 10 | Kano の方式を再検証 (確証 vs 推定) | Kano = 「相対 + ML」推定、確証は要追加調査と明記 |
| 11 | 静止検出トリガを暗に追加 | Condition::stillness_required 新規追加 |
| 12 | 特許内容・期限・URL | 関連特許情報セクション |
| 13 | 特許調査をバックグラウンドで継続 | バックグラウンド Explore agent 起動済 |
| 14 | 現状で資料化 (= 本 plan ファイル) | 本セクションで完結 |

---

## Phase 5.39 設計議論セッション総括 (2026-05-14)

ユーザーの要求: 「今までの議論は全て資料に残すこと」 → 本セッションの議論を時系列で整理し、確定事項・経緯・未解決事項をまとめる。

### 議論の時系列

| # | ユーザーの問い | 結果 / 確定事項 | plan の該当箇所 |
|---|---|---|---|
| 1 | Hold Start End モードに中間姿勢を追加した新モードが必要、可能か？ | 既存 SEQUENCE 機構 (state[4]) 流用で実装可能 → Phase 5.39 として実装、サンプル 2 種 (Wingardium Hold Euler / Relative) 追加・デプロイ済 | Phase 5.39 セクション |
| 2 | Yaw drift と Pitch ジンバルロック対策で Euler / 絶対 quat / 相対 quat を検討 | 3 方式の比較・特性整理、相対 Quat 採用 (state[0] match で q_ref 自動取得) | 用語の定義、近傍判定アルゴリズム |
| 3 | 状態遷移と機械学習どちらがよいか、Kano は何を使っていたか？ | FSM / DTW / HMM / ML 浅層 / ML 深層 / HW MLC の 6 軸比較、Burst Motion は FSM 採用継続 | 判定アーキテクチャの選択 + 各判定アーキテクチャの詳細比較・解説 |
| 4 | 絶対 Quat と相対 Quat の定義明確化 (地磁気とは別) | 「絶対 = Mahony 起動基準」「相対 = トリガ基準」と明確化、地磁気は使わない (6 軸 IMU 前提) | 用語の定義セクション |
| 5 | Kano / WB の方式を「絶対 / 相対」で再評価、本当に相対モードを使っているのか？ | バックグラウンド特許深掘り調査 → Kano は実は「絶対波形 + 古典 ML」、画面表示のみ相対。WB は「相対 + DTW」 | Kano/WB 再評価 (バックグラウンド調査結果反映) |
| 6 | 近傍判定で「登録した姿勢の近くになった」をどう判定するか？ 距離か？ | 候補 A (Euler 軸別 tol)、候補 B (Quat 内積 cone)、候補 C (forward+twist) を比較、A をデフォルト、B も実装済 | 近傍判定アルゴリズム |
| 7 | Web UI を複雑にせず実装するには? | UI 複雑度抑制ガイドライン (新 useState 2 個まで、新規関数 4 個まで、デフォルト中間 0 個、判定基準は折り畳み details) | Web UI 複雑度抑制ガイドライン |
| 8 | rule 選択時に 3D / 2D で軌跡を視覚化できるか? | Phase 5.39.2 として rule 選択フォーカス + 軌跡描画機能を新規追加 (タブ式 UI: 絶対 3D / 2D Roll-Pitch / 相対 3D) | Phase 5.39.2 セクション |
| 9 | 相対モード rule の目標軌跡は、ボタン押下時の自身の quat が分からないと確定しないはず | ハイブリッド方式採用: 押下前 = 現在姿勢を仮 q_ref として薄い破線プレビュー、押下後 = FW 受信実 q_ref で濃い実線固定 | Phase 5.39.2 採用方針表 |
| 10 | 絶対モードと相対モード rule の 2 つしかないのか? 今どのようなモードがあるか? | 4 軸の直交モード体系 (ui_mode × posture_basis × judge_by × Condition 組合せ) を整理、5 × 2 = 10 通り | Burst Motion rule モード体系 整理 |
| 11 | 相対モード rule では中心基準の 3D ビュアが必要 | 新規 RelativeIMUViewer (一人称視点、M5C 中央固定、ワールド逆回転) を Phase 5.39.2 で実装、タブ式 UI 採用 | Phase 5.39.2 全体構成 |
| 12 | Euler モードでは絶対 3D + 2D Roll-Pitch 両方必要、相対モードでは絶対 3D + 相対 3D + 2D 相対 Quat | タブ構成を 4 つに整理 (絶対 3D / 2D Roll-Pitch / 相対 3D を Phase 5.39.2、2D 相対 Quat を Phase 5.39.3) | Phase 5.39.2 タブ構成 |
| 13 | 相対モード rule の登録姿勢はどう保持するか? 判定は M5C と Web どちらで? | FW 完全完結 (LittleFS 永続化 + RAM ActionRule + 100Hz TriggerEngine)、Web は登録・可視化のみ | データ保持と判定の役割分担 |
| 14 | 相対モードはどうやって相対 quat 変化を判定するか? あらかじめ登録する必要があるが | Web 側で qRefCapture を一時保持、開始姿勢キャプチャ時に sensor.quat 保存、各 waypoint で相対 Euler 自動計算 | 相対モード rule の登録フロー |
| 15 | M5C 側の判定や登録が絶対 Quat と相対 Quat で区別されるか? | ✅ 完全に区別 (types.hpp の PostureBasis enum、matchCondition の分岐、Profile JSON 入出力、Python smoke test で実証) | FW 側での絶対/相対の区別 |
| 16 | judge_by="quat" とは絶対 quat か相対 quat か? UI 直感性を持たせる必要 | posture_basis に従って自動切替、α (Euler 抽出 + 警告、推奨) / β (Quat cone、現状実装) / γ (軸別 Quat cone、Phase 5.39.3+) の 3 案、α 採用 | 相対モードの判定方式 詳細 |
| 17 | 単一 cone tol とは? Euler 角で判定しているように見える | Pitch ≈ 0 安全領域の数値例だった、ジンバルロック領域の議論を欠いていたことを訂正、quat 内積判定の真の利点を明示 | 単一 cone tol (judge_by="quat") の数値例 |
| 18 | 杖は Pitch=-90° 付近で開始 (杖水平) することが多い、ユーザー環境前提 | アプローチ a (保持姿勢) / b (IMU 向き) / c (Mahony 基準姿勢キャリブ) で絶対モードでもジンバルロック回避可、相対モードは「Yaw drift も同時解決」が本領 | 杖用途の実運用 |
| 19 | これは絶対 Quat の話ではない、絶対 Quat は Yaw drift がある | Phase 5.39.3 候補として動的 Yaw drift 補正機能 (yaw_corr quat + 補正トリガー 4 種) を追加 | Phase 5.39.3 候補 |
| 20 | Yaw drift 補正は yaw_offset = current_yaw でなく Quaternion で補正すべき | 案 B (yaw_corr quat を NVS 保存、判定時に quat 演算で適用) 採用、Twist-Swing 分解で実装 | 動的 Yaw drift 補正機能 (Phase 5.39.3 候補) |
| 21 | なぜ絶対 Quat の話で Euler の Roll/Pitch が出てくる? 単純化のためか? | はい、語彙混在は単純化のためでした、訂正 (q_tilt = swing、q_yaw = twist という quat 用語に統一) | 厳密な quat 用語での記述 |
| 22 | Twist-Swing 分解とは? | 3D 回転を Z 軸周り twist と swing に分解する数学手法、Mahony Euler 抽出より数値安定、Yaw drift 補正の理論基盤 | Twist-Swing 分解 (Swing-Twist Decomposition) の詳細 |

### 確定した設計判断 (本セッションで決定)

| 項目 | 確定内容 | 実装状態 |
|---|---|---|
| Phase 5.39 (Hold with Waypoints + 中間 0-2 個 + 相対 Quat 判定) | FW types.hpp + TriggerEngine + Profile + main_v2 拡張、Web app.js + サンプル 2 種、deploy 済 | ✅ 実装完了、Python smoke test 実証済 |
| Phase 5.39.2 タブ式 UI | 4 タブ構成 (絶対 3D / 2D Roll-Pitch / 相対 3D / 2D 相対 Quat (5.39.3)) | plan 確定、未実装 |
| Phase 5.39.2 相対 3D ビュア | M5C 中央固定 (default) + ワールド逆回転、q_ref 定着オプション切替可 | plan 確定、未実装 |
| Phase 5.39.2 軌跡描画 | 絶対モード = 常時実線、相対モード = 押下前は薄い破線プレビュー、押下後は実線固定 | plan 確定、未実装 |
| Phase 5.39.2 相対モード rule 登録フロー | Web の qRefCapture で開始姿勢を一時保存、各 waypoint で相対 Euler 自動計算 | plan 確定、未実装 |
| Phase 5.39.2 FW 改修 | `trigger.hit` event に `q_ref[4]` 追加 (5 行) | plan 確定、未実装 |
| Phase 5.39.3 動的 Yaw drift 補正 | yaw_corr quat (NVS 永続化) + Twist-Swing 分解で q_twist 補正、補正トリガー 4 種 | plan 確定、Phase 5.39.2 完了後に実装 |
| サンプルプロファイル位置付け | wingardium_hold_euler (絶対モード、基準姿勢キャリブ前提) と wingardium_hold_relative (相対モード、気軽に使える) は両立価値あり | サンプル両方デプロイ済 |

### 訂正された前回の誤り (議論で明確化)

| 私の誤り | ユーザーの指摘で訂正 | 訂正後の正しい認識 |
|---|---|---|
| 「Kano = 相対方式 + ML」と確証していた | バックグラウンド調査で「画面 origin reset は確定だが ML 入力レベルでは絶対波形を使っている可能性が高い」と判明 | Kano は「絶対波形 + 古典 ML (kNN/SVM)」が実態に近い、相対化は画面表示のみ |
| 「judge_by="quat" 数値例」を Pitch=0 領域だけで示した | ジンバルロック領域での挙動を含めていなかった | Pitch ≈ ±65° 超で Euler 縮退、Quat 内積は安定 (数値例追加) |
| 「絶対モード + 基準姿勢キャリブ ≈ 相対モード」と書いた | Yaw drift 耐性は相対モードのみ、アプローチ c では drift 止められない | 訂正、両者は別物 (ジンバルロック耐性は同等、Yaw drift 耐性は相対モードのみ) |
| 「yaw_offset = current_mahony_yaw」(Euler 差分) で補正 | Euler 差分はジンバルロック領域で破綻、Quaternion で補正すべき | Twist-Swing 分解で q_twist を quat 演算で打ち消す方式に訂正 |
| 「Roll/Pitch は触らない」と Euler 用語使用 | 絶対 Quaternion の議論で Euler 用語を持ち出すのは混在 | quat 用語に統一 (q_tilt = swing、q_twist = Yaw 成分) |

### 未解決の論点 (Phase 5.39.3 以降で議論継続)

Phase 5.39.3 (Yaw drift 補正実装) で詰めるべき設計詳細 (本 plan の Phase 5.39.3 セクションに記載):
- 論点 1: 複数 rule 間の yaw_corr 共有
- 論点 2: rule 登録時の Yaw 補正値の扱い (補正済 / 生)
- 論点 3: 補正トリガーの優先順位 / 競合
- 論点 4: 補正瞬間の判定値の不連続性
- 論点 5: NVS 永続化 vs RAM のみ
- 論点 6: 量産版 IMU との整合
- 論点 7: 既存 calibrate.qref.* との関係
- 論点 8: Web 側可視化 (3D ビュア統合)

Phase 5.39.2 で未実装の項目:
- タブ式 UI (絶対 3D / 2D Roll-Pitch / 相対 3D)
- 新規 RelativeIMUViewer.js (一人称視点ビュア)
- rule 選択フォーカス + 軌跡描画
- 相対モード rule の登録フロー (qRefCapture)
- FW 側 trigger.hit に q_ref 追加

Phase 5.39.4+ で持ち越し:
- 2D 相対 Quat ビュア
- 判定進捗バー
- q_ref ノイズ平均化 (state[0] enter 後 5-10 サンプル平均)
- 設定 UI (trail 持続時間スライダ等)
- 軸別 Quat cone 判定 (γ 案)
- forward + twist 分解判定 (候補 C)

### 本セッションで参照した外部資料

- US20210165506A1 / US11301059B2 (Kano Wand 特許、active until 2039)
- US20220100280A1 (WB / Spin Master、minimal wand form factor)
- Hackaday: Magic Wand Learns Spells Through Machine Learning
- TechCrunch / MuggleNet: Kano vs WB 知財紛争
- Verth & Bishop: Essential Mathematics for Games (Twist-Swing 分解)
- MDPI Sensors 2016: Quaternion-Based Gesture Recognition (96% 精度)
- ST Microelectronics AN5804: LSM6DSV16X MLC

### 議論の本質的価値

本セッションを通じて確認された設計原則:
1. **FW 単独動作** — Web 接続が切れても M5C は HID デバイスとして動作 (判定は完全に FW、Web は設定・可視化のみ)
2. **絶対 vs 相対の二項対立を超える** — 「絶対 + Yaw 補正」という第 3 のモードの存在を明示
3. **ジンバルロックと Yaw drift は別問題** — 解決手段も別、組合せで両方を回避するのが堅牢
4. **quat 演算で完結** — Euler 抽出を経由しない判定 (judge_by="quat") + 補正 (Twist-Swing) で数値安定性を確保
5. **Burst Motion ≠ Kano / WB** — FSM ベース、ユーザーカスタマイズ、OSS 透明性、相対モードで Yaw drift 完全回避 — 業界未開拓の象限
6. **Web UI 複雑度抑制** — 段階的開示 (デフォルト最小、+ ボタン / 詳細設定 折り畳みで上級者向け機能を解放)

---

## 追加 Todo (ユーザー指示、2026-05-14 末尾追加)

### 1. 計画資料のレポジトリ保存 (Git 管理化、消失防止)

**現状の問題**: 本 plan ファイルは `C:\Users\thefu\.claude\plans\` (ローカル) にあり、Git 管理外。クラッシュ / OS 再インストール / Claude メモリリセット時に消失リスク。

**対応**:
- Plan モード退出後、本 plan ファイルを `M5C_MPU6886_cpp/docs/design/burst-motion-design-plan.md` にコピー
- レポジトリ内に Git 管理対象として保存 → コミット / push で永続化
- 以降の議論で plan を更新するときは、両方を同期 (または symlink)

**実装手順**:
```bash
mkdir -p M5C_MPU6886_cpp/docs/design
cp ~/.claude/plans/m5c-mpu6886-cpp-https-findradio-jp-moti-dynamic-beacon.md \
   M5C_MPU6886_cpp/docs/design/burst-motion-design-plan.md
git add M5C_MPU6886_cpp/docs/design/burst-motion-design-plan.md
git commit -m "Add Burst Motion design plan to docs/"
```

### 2. private repo 化 + Web Serial の HTTPS 要件検討

**ユーザー要望**: 「localhost のサーバでも Web アプリを使えるか？ HTTPS でないと Web Serial などが使えないか？ レポジトリを private にしてコードを秘匿化したい」

**Web Serial / Web Bluetooth の secure context 要件**:

| 環境 | Web Serial / Web Bluetooth 動作 | 補足 |
|---|---|---|
| `https://...` (TLS) | ✅ 動作 | secure context 確定 |
| `http://localhost` / `http://127.0.0.1` | ✅ 動作 | **localhost は HTTPS なしでも secure context 扱い** (Chromium 仕様) |
| `http://...` (LAN 等の他ホスト) | ❌ 不動作 | secure context 非該当 |
| `file://...` (ローカルファイル) | ❌ 不動作 | secure context 非該当 |

→ **localhost で動かす場合は HTTPS 不要**。ローカル開発 / 自家用なら `python -m http.server` で十分。

**選択肢**:

| 選択肢 | コスト | Web Serial 対応 | private 性 | 推奨度 |
|---|---|---|---|---|
| **A. ローカル運用 (localhost)** | 無料、HTTPS 不要 | ✅ | 完全 private (公開なし) | 自家用には最適 |
| **B. GitHub Private Repo + Cloudflare Pages** | 月 $0 (CF Pages は無料、GitHub Pro は $4/月 or organization 単位) | ✅ HTTPS | コードは private、デプロイ先 URL は知っている人だけアクセス | 量産前のステージング向け |
| **C. GitHub Private Repo + GitHub Pages** | GitHub Pro ($4/月) or organization 必須 | ✅ HTTPS | 同上 | A/B どちらもダメな場合 |
| **D. 自己署名 HTTPS + 自宅 LAN** | 無料、ただし証明書設定が手間 | ✅ HTTPS (自己署名警告あり) | LAN 内のみ | 複数人で使うが公開しない用途 |

**推奨**: **A (localhost) を Burst Motion 開発の default ローカル運用**、量産前ステージング時は B (CF Pages + Private Repo) に移行。

**実装手順 (案 A、即時可能)**:
```bash
cd M5C_MPU6886_cpp/Web/hidconfig
python -m http.server 8000
# Chrome で http://localhost:8000 にアクセス → Web Serial 動作確認
```

**実装手順 (案 B、Cloudflare Pages 連携)**:
1. GitHub repo を private に設定 (Settings → Visibility)
2. GitHub Pages を無効化 (現状 public 公開を停止)
3. Cloudflare 登録、Pages プロジェクト作成、GitHub private repo を連携 (CF アプリ承認)
4. Build settings: `Web/hidconfig` を root にして、output 同じ
5. デプロイ → `<random-id>.pages.dev` の URL (知る人だけアクセス)、custom domain も設定可

**注意**:
- Cloudflare Pages は private repo に対応 (Vercel / Netlify も同様)
- GitHub Pages の private mode は GitHub Pro / GitHub Team 以上で有効
- いずれにせよ Web Serial / Web Bluetooth 動作には影響なし (HTTPS であれば OK)

**追加検討**: esp-web-tools のフラッシュ機能も HTTPS 必須なので、localhost / HTTPS どちらかで動かす必要あり (今回方針と整合)。

### 3. Memory への保存 (将来セッション参照用)

本セッションで決まった重要な方針:
- **計画資料はレポジトリ内 `docs/design/` に Git 管理化** (memory に保存推奨)
- **localhost 開発 OK + 量産前ステージングは Cloudflare Pages + private repo** (memory に保存推奨)
- **Twist-Swing 分解 + yaw_corr quat は Yaw drift 補正の標準アプローチ** (memory に保存推奨)
- **judge_by="euler" + 相対モードが Wand 用途の MVP デフォルト** (memory に保存推奨)

Plan モード退出後、これらを memory に保存する。


