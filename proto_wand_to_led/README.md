# proto_wand_to_led

HP 杖プロトタイプ Phase 0: 振り検出 → BLE Advertising Beacon → LED 5 秒点灯

**Phase 0 動作確認済 (2026-05-20)**: M5StickC ↔ XIAO nRF52840 間で SHAKE / LUMOS / NOX トリガが BLE 経由で伝達、LED 5 秒点灯 + Nox 即消灯まで確認。RSSI -39〜-43 dBm。

## 概要

- **杖 (送信側)**: M5StickC + MPU6886 + NimBLE。重力基準ジェスチャ判定 (上振り=LUMOS / 下振り=NOX / 強い振り=SHAKE)
- **LED (受信側)**: XIAO nRF52840 (Sense) + 内蔵 RGB LED + 外部 LED (D0~D10) + Bluefruit (BLE scan)
- **通信**: BLE Advertising Beacon (connectionless broadcast)、payload 7 バイト
- **動作**: 杖を振る → BLE adv 500ms burst → 受信機が scan match → LED 制御
- **LED 挙動**: 内蔵青=ハートビート / 内蔵赤+外部=トリガ点灯 / LUMOS=全点灯(NOXまで) / NOX=全消灯
- **1:N + ターゲット指定**: payload に `target_id` (2B, 1-65534)。`TARGET_ALL=0xFFFF` で全機、特定 ID で 1 機のみ
- **機体設定 (USB Serial)**: 同じ FW を全機に焼き、各機を Serial で個別設定 (DEVICE_ID, 外部ピン)。InternalFS に永続化

### Serial コマンド

**杖 (M5StickC, COM ポート)** — 疑似トリガ送信:
- `t`=SHAKE / `l`=LUMOS / `n`=NOX / `i`=INCENDIO / `a`=AGUAMENTI (全機宛て)
- 数字を前置で宛先指定: `2l`=ID2 に LUMOS / `1n`=ID1 に NOX / `0` で全機に戻す

**LED (XIAO nRF52840, COM ポート)** — 機体設定:
- `show` : 現在の設定表示
- `id=<n>` : DEVICE_ID 設定 (1-65534)
- `pins=D0,D2,G1.11` : 外部 LED ピン設定 (下記「ピン記法」参照)
- `save` : InternalFS に保存 (再起動後も保持)
- `default` : デフォルトに戻す
- `test` : 配線診断。設定ピンを 1 本ずつ 1 秒 → 全部 2 秒点灯 (杖なしで外部 LED 配線を確認)

### ピン記法 (外部 LED)

| 記法 | 意味 | 例 |
|------|------|-----|
| **D0〜D10** | XIAO シルク印刷の digital ピン | `D0`=P0.02, `D6`=P1.11 |
| **G\<port\>.\<pin\>** | nRF52840 生 GPIO | `G1.11`=P1.11, `G0.4`=P0.04 |

内部的には全て**絶対 nRF GPIO 番号 (port×32+pin)** に変換し、`nrf_gpio_*` で直接駆動 (active-high)。
配線: `pin ─[330Ω]─▶|─ GND`。nRF52840 GPIO 標準ドライブで 1 ピン 1 LED (~4mA) は直結可。

## ディレクトリ構成

```
proto_wand_to_led/
├── README.md            (このファイル)
├── docs/
│   ├── design.md        (詳細設計)
│   ├── roadmap.md       (ロードマップ)
│   ├── phone_web_constraints.md  (杖→スマホ Web アプリの制約と現実的な構成)
│   └── patent_landscape.md       (杖デバイスの特許調査: FTO 見立て / 自社出願余地)
├── shared/
│   ├── beacon_protocol.h  (ManufacturerData フォーマット共通定義)
│   ├── wand_common.h      (杖共通定数: SLEEP_AFTER_SEC 等)
│   └── wand_gesture.h     (★ジェスチャ判定コア: 重力基準フリック + Wingardium。M5/XIAO で共用)
├── wand_m5stickc/       (杖: M5StickC + MPU6886、NimBLE。PlatformIO プロジェクト)
│   ├── platformio.ini
│   └── src/{main.cpp, device_config.h}
├── wand_xiao_nrf52840/  (杖: XIAO nRF52840 Sense + LSM6DS3TR-C、Bluefruit。★移植中→README 参照)
│   ├── README.md
│   ├── platformio.ini
│   └── src/{main.cpp, device_config.h}
└── led_xiao_nrf52840/   (LED 受信: XIAO nRF52840、Bluefruit scan。PlatformIO プロジェクト)
    ├── platformio.ini
    └── src/main.cpp
```

**ジェスチャ判定の共通化**: `wand_m5stickc` と `wand_xiao_nrf52840` は判定ロジックを
`shared/wand_gesture.h` の `wand_gesture::Detector` として共有する。各機の main.cpp は
プラットフォーム依存部 (BLE 送信 / IMU 読み / LED / ボタン / 設定永続化) のみを実装し、
`Detector` に BLE 送信関数と LED フラッシュ関数を**関数ポインタで注入**する。
軸マウント (`WAND_FORWARD/RIGHT`) と閾値は `device_config.h` / `det.cfg` で機体ごとに設定。

## ビルド・書き込み手順

### 杖 (M5StickC) 側

```bash
cd wand_m5stickc
pio run -t upload
pio device monitor
```

USB-C で M5StickC を接続。書き込み後 Serial (115200) に `|a|=1.00g` のような表示が出る。

### LED (XIAO nRF52840) 側

```bash
cd led_xiao_nrf52840
pio run -t upload
pio device monitor
```

XIAO nRF52840 を USB-C で接続。

**初回書き込み or アプリがクラッシュした時**: リセットボタン (側面の銀ボタン) を**素早く 2 連打**して DFU モードに入れる (`D:\ XIAO-SENSE` ドライブが見えたら成功)。その後 `--upload-port COMxx` を DFU ポートに向けて upload。
正常動作している FW なら 2 回目以降は 1200bps トリックで自動的に DFU 入場するので物理操作不要。

## 動作確認

1. 両機を電源 ON
2. 受信側シリアル: `[BLE] scanning, waiting for wand (company_id=0xFFFF) ...` を確認
3. M5StickC を強く振る (|a| > 2.5g)、または USB Serial に `t` を送る
4. 杖シリアル: `[BLE] adv start seq=N trig=0x01 strength=...`
5. LED 側: 内蔵 LED が 5 秒間点灯 → 自動消灯
6. 受信側シリアル: `[RX] seq=N trig=0x01 ... => LED ON 5s (SHAKE)` → 5s後 `[LED] auto OFF`

## トラブルシューティング

| 症状 | 確認事項 |
|------|---------|
| 杖の Serial に IMU 値が出ない | M5StickC の電源、`pio device monitor -p COMxx -b 115200` でログ確認 |
| 杖は反応するが LED 無反応 | XIAO nRF52840 のシリアルログで `[RX]` が出るか。RSSI が -85dBm 以下なら距離を縮める |
| 距離が 1m 未満 | M5StickC を木の壁・人体から離す、`NimBLEDevice::setPower` を上げる |
| 連続点灯のたびに点滅する | seq の単調増加で重複抑止されているか確認 |
| XIAO が「不明な USB デバイス」 | アプリ FW がクラッシュ。DFU モード (リセット 2 連打) で再書き込み |

## デバッグで判明した重要事項 (再現・移植時の注意)

Phase 0 立ち上げで詰まったポイント。同じ HW を使う際は必ず確認:

### 1. XIAO nRF52840 の USB CDC Serial

- **`Serial` シンボル自体が Adafruit TinyUSB ライブラリに依存**。`platformio.ini` の `lib_deps` に `adafruit/Adafruit TinyUSB Library` と、main.cpp に `#include <Adafruit_TinyUSB.h>` の両方が必須。無いとリンクエラー (`undefined reference to 'Serial'`)。
- **`TinyUSBDevice.begin(0)` を `Serial.begin()` より先に明示呼び出し**する。これが無いと Bluefruit と併用した時に USB enumeration に失敗し「不明な USB デバイス (デバイス記述子要求の失敗)」になる。
  ```cpp
  if (!TinyUSBDevice.isInitialized()) { TinyUSBDevice.begin(0); }
  Serial.begin(115200);
  ```
- **`SdFat - Adafruit Fork` を `lib_ignore`** する。TinyUSB の MSC が引き込む SdFat の `File` 型が Adafruit_LittleFS の `File` と衝突してビルドエラーになる。

### 2. ARM Cortex-M4 の unaligned access

- BLE adv の ManufacturerData を `struct __attribute__((packed))` にポインタキャストして `uint16_t` フィールドを直接読むと **Hard Fault でクラッシュ** (USB ごと死ぬ)。`memcpy` でローカルにコピーしてから読む:
  ```cpp
  wand_beacon::Payload payload;
  memcpy(&payload, buf, sizeof(payload));
  if (payload.company_id == ...) { ... }
  ```

### 3. LED ピン

- `LED_RED` / `LED_GREEN` / `LED_BLUE` は使わず **`LED_BUILTIN` のみ**使用。RGB ピンが想定外のパッド (USB 関連等) に割り当たると pinMode で USB を壊すリスクを避けるため。active-low (LOW で点灯)。

### 4. PowerShell から Serial を読む際の DTR/RTS (チップで真逆)

ホスト PC から `[System.IO.Ports.SerialPort]` で読む場合、デバイスごとに設定が逆:

| デバイス | DtrEnable | RtsEnable | 理由 |
|---------|-----------|-----------|------|
| **M5StickC (ESP32)** | **`$false`** | **`$false`** | DTR/RTS が EN(リセット)/IO0(boot) に繋がっており、true だと連続リセット状態で何も出力しない |
| **XIAO nRF52840** | **`$true`** | **`$true`** | USB CDC が host 接続 (DTR) を検出して初めて出力する |

`pio device monitor` は ESP32 のリセットシーケンスを正しく扱うので、手動 SerialPort より楽。

### 5. nRF52 ハードウェア I2C (TWIM) はタイムアウトが無くハングする ★XIAO 杖移植で判明

- Adafruit nRF52 core の `Wire`/`Wire1` (`Wire_nRF52.cpp`) は `while(!_p_twim->EVENTS_STOPPED);` 等
  **タイムアウト無しの待ちループ**を持つ。I2C スレーブが ACK を返さない/バスが不安定だと**無限ループでハング**する。
- 症状: `setup()` 内の最初の I2C 読み (WHO_AM_I 等) で停止 → `loop()` に到達せず**シリアルに何も出ない**。
  USB CDC は別 FreeRTOS タスクで生き続けるので**ポートは見えたまま**=「動いてるのに無言」で紛らわしい。
- 切り分け: `setup()` 各所に `Serial.println("step..."); Serial.flush();` を仕込み、最後に出た行の直後が原因。
- **対策: 該当 I2C を bit-bang(ソフト I2C) に切替**。クロックを自前で刻むのでハングしない
  (実装例: `wand_xiao_nrf52840/src/main.cpp` の `imu::bb` 名前空間)。
- 補足: `Wire` コンストラクタは `g_ADigitalPinMap[]` で Arduino ピン→絶対 GPIO 変換するので、
  Wire1 のピン指定自体は正しい。ハングは純粋に TWIM のタイムアウト欠如が原因。

### 6. ハングした FW は 1200bps タッチ DFU に応答しない → RESET 2 連打が確実

- 通常は `pio run -t upload` が 1200bps タッチで自動 DFU 入場するが、**チップ上の FW が
  ハングしていると応答せず**、`adafruit-nrfutil` が "No data received... Not in DFU mode" で失敗する。
- **確実: RESET 2 連打 → `XIAO-SENSE` ドライブ → `firmware.uf2` をコピー**。
  hex→uf2: `python <framework-arduinoadafruitnrf52>/tools/uf2conv/uf2conv.py firmware.hex -c -f 0xADA52840 -o firmware.uf2`

## 次のステップ (Phase 1+)

詳細は [docs/design.md](docs/design.md) 参照。
