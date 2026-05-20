# proto_wand_to_led - Design (Phase 0)

詳細設計の正本は [プラン](file:///C:/Users/thefu/.claude/plans/m5c-hp-m5c-imu-esp32-imu-nrf52840-esp32-iridescent-cascade.md) を参照。本ドキュメントは Phase 0 実装の運用ガイド。

## 設計サマリ

| 項目 | 設定 |
|------|------|
| 通信 | BLE Advertising Beacon (connectionless broadcast) |
| 受信側 wake | Phase 0: **100% wake (連続スキャン)** |
| adv interval | 20-30ms (BLE 仕様最小、`setMinInterval(0x20)`) |
| adv burst 時間 | 振り検出後 500ms 継続 |
| クールダウン | 1 秒 (杖側) |
| 想定レイテンシ | ~40-80ms |
| 重複抑止 | seq 番号で同一振りからの複数 adv を 1 点灯にまとめる |

## ペイロード仕様

`shared/beacon_protocol.h` の `wand_beacon::Payload` 構造体 (7 バイト):

| field | 型 | 値域 | 役割 |
|-------|-----|-----|------|
| company_id | uint16_t | 0xFFFF | Bluetooth SIG 未登録の test ID。受信フィルタキー |
| seq | uint8_t | 0-255 wrap | 振り 1 回 = +1、受信側で重複抑止 |
| trigger_id | uint8_t | 0x01..0x2F | TRIG_* 定数 (呪文の種類) |
| strength | uint8_t | 0-255 | 振り強度 (将来 LED 輝度反映) |
| target_id | uint16_t | 1-65534 / 0xFFFF | 宛先 DEVICE_ID。`TARGET_ALL=0xFFFF` で全機 |

受信側 (各機) は `target_id == TARGET_ALL` または `target_id == 自機 DEVICE_ID` の時のみ反応。

### 現在実装済の trigger_id と LED 挙動

| 定数 | 値 | ジェスチャ | LED 挙動 (内蔵赤 + 外部) |
|------|-----|-----------|------------------------|
| TRIG_SHAKE | 0x01 | 強い振り (上下成分小) | 赤+外部 ON 5 秒 |
| TRIG_LUMOS | 0x10 | 上振り | **全 LED ON (NOX まで持続)** |
| TRIG_NOX | 0x11 | 下振り | **全 LED 即 OFF** |
| TRIG_INCENDIO | 0x20 | 前突き (未実装) | 赤+外部 ON 3 秒 |
| TRIG_AGUAMENTI | 0x21 | 下流し (未実装) | 赤+外部 ON 5 秒 |

杖側は重力基準ジェスチャ判定で SHAKE / LUMOS / NOX を物理的に発射。INCENDIO / AGUAMENTI は Serial コマンド (`i`/`a`) で疑似発射可能、物理判定は Phase 2 で実装。

## ジェスチャ判定アルゴリズム (重力基準フリック検出)

**正式名称**: 重力基準フリック検出 (Gravity-Referenced Flick Detection) = 重力補償 + 鉛直射影 + 閾値判定。

### 軸依存性
**固定軸 (X/Y/Z) には非依存。重力 (鉛直) 基準。** 動的推定した重力ベクトルを「上」とするため、杖をどの向きで持っても「鉛直に対する上振り/下振り」が成立する。完全な水平動作は上下成分が小さく SHAKE になる。

### 重力推定 (静止ゲート付き EMA ローパス)
```
a_mag = |accel|
if ( |a_mag - 1g| < still_band ) {        // ほぼ静止の時だけ更新 (フリック混入防止)
  grav += grav_alpha × (accel − grav)     // 1次 IIR ローパス
  静止サンプルが READY_STILL_SAMPLES(25,≈0.25s) 連続 → ready=true
}
// 振り中は grav 凍結、ready 維持
```
- カットオフ ≈ grav_alpha × 100Hz / 2π。alpha=0.05 で時定数 ≈ 0.2s
- 静止ゲートにより alpha を上げても (速い追従) フリックの加速度が重力に混入しない

### 判定 (毎サンプル)
```
u       = grav / |grav|              # 上方向の単位ベクトル
linear  = accel − grav               # 重力補償 (動き成分)
lmag    = |linear|
up_proj = linear · u                 # 鉛直方向への射影 (正=上, 負=下)
ratio   = |up_proj| / lmag           # 動きが鉛直に沿う度合い (0-1)

if lmag < flick_threshold_g            → 無反応 (動きが弱い)
elif (now - last) <= cooldown_ms       → 無反応 (クールダウン)
elif ratio >= updown_ratio and up_proj>0 → LUMOS (上振り)
elif ratio >= updown_ratio and up_proj<0 → NOX   (下振り)
else                                     → SHAKE (横/斜め)
```

### 調整パラメータ (実行時可変 + ESP32 NVS 保存)
| パラメータ | デフォルト | コマンド | 意味 |
|-----------|----------|---------|------|
| `flick_threshold_g` | 1.2g | `gth=` | フリック強度の下限 (linear accel) |
| `updown_ratio` | 0.6 | `gratio=` | 上下フリック判定の鉛直比 (0-1) |
| `cooldown_ms` | 1000 | `gcool=` | トリガ間の最小間隔 |
| `grav_alpha` | 0.05 | `galpha=` | 重力 EMA 係数 (大=速い追従) |
| `still_band` | 0.15g | `gband=` | 静止判定幅 (重力更新ゲート) |

`gshow` 一覧表示 / `gsave` NVS 永続化 / `gdefault` リセット。

### M5StickC 内蔵 LED フィードバック (GPIO 10)
A ボタン (GPIO 37) で ON/OFF トグル、**デフォルト OFF**。有効時: 静止=消灯 / 収束中=点滅 / ジェスチャ検出時=75ms フラッシュ。

### 強度 (strength)
`strength = clamp((lmag − flick_threshold_g) × 91, 0, 255)`。payload に載せ、将来 LED 輝度に反映予定。

## 機体設定システム (InternalFS + USB Serial)

同一 FW を全受信機に焼き、各機を Serial で個別設定 → InternalFS に永続化。再コンパイル不要。

- `DEVICE_ID` (uint16_t, 1-65534): この機の宛先 ID
- 外部 LED ピン配列 (最大 12 本): D 記法 or G 記法で指定 → 絶対 nRF GPIO 番号で保存
- コマンド: `show` / `id=<n>` / `pins=D0,D2,G1.11` / `save` / `default`

### 外部 LED 駆動 (nrf_gpio 直接制御)

XIAO nRF52840 の Adafruit/Seeed core は `digitalWrite()` がピンマップ経由のため、絶対 GPIO 番号 (G 記法の P1.11=43 等) では使えない。よって外部 LED は **`nrf_gpio_cfg_output` / `nrf_gpio_pin_set/clear`** で直接駆動。D 記法も `XIAO_D_TO_NRF[]` で物理 nRF ピンに変換してから同 API で駆動 (D/G 統一)。内蔵 RGB LED は Arduino `digitalWrite` のまま。

## 動作確認手順 (Phase 0)

### 1. 杖 (M5StickC) 書き込み

```powershell
cd c:\Users\thefu\Documents\M5C_Serial_Unity\proto_wand_to_led\wand_m5stickc
pio run -t upload
pio device monitor -b 115200
```

期待ログ:
```
=== Wand Beacon (Phase 0) ===
Threshold: |a| > 2.5g, Adv burst: 500ms, Cooldown: 1s
[IMU] MPU6886 OK
[BLE] init OK, waiting for shake...
|a|=1.00g
|a|=1.01g
```

### 2. LED (XIAO nRF52840) 書き込み

```powershell
cd c:\Users\thefu\Documents\M5C_Serial_Unity\proto_wand_to_led\led_xiao_nrf52840
pio run -t upload
pio device monitor -b 115200
```

初回書き込み時、XIAO nRF52840 はリセットボタン (側面の小さなボタン) を**素早く 2 回**押してブートローダーモードにする必要があることがあります。

期待ログ:
```
=== LED Receiver (Phase 0, 100% wake scan) ===
[LED] pin=XX active-low
[BLE] scan started, waiting for wand...
```

起動時、内蔵 LED が一瞬 (200ms) 光るのが確認できる。

### 3. 結合動作確認

1. 両機を電源 ON のまま近距離 (1m 以内) に置く
2. M5StickC を **強く振る** (|a| > 2.5g、目安は素早いスナップ動作)
3. 杖側ログ:
   ```
   *** TRIGGER |a|=3.42g strength=93 ***
   [BLE] adv start seq=1 trig=0x01 strength=93
   [BLE] adv stopped
   ```
4. LED 側ログ:
   ```
   [RX] seq=1 trig=0x01 strength=93 rssi=-45 → LED ON 5s (SHAKE)
   ```
5. LED 内蔵 LED が **5 秒間** 点灯し、自動消灯
6. LED 側ログ: `[LED] auto OFF`

### 4. 連続トリガ動作

- 1 秒以上の間隔で繰り返し振る → seq が +1 ずつ増えて再点灯
- 1 秒未満で連続振り → クールダウン中につき adv なし (杖側 cooldown)
- 同じ振り中の複数 adv → seq 同じなので受信側で重複点灯抑止

### 5. 距離試験

- 1m / 3m / 5m / 10m と離して動作確認
- RSSI 値をシリアルログで記録 → 想定範囲を決定
- 一般に -85dBm 程度で限界。-90dBm で `filterRssi(-90)` の制限にひっかかり無視される

## トラブルシューティング

| 症状 | 確認事項 | 対処 |
|------|---------|------|
| 杖の Serial に何も出ない | M5StickC 電源、USB-C ケーブル | `pio device list` でポート確認。PowerShell 直読みなら DTR/RTS=false (下記参照) |
| `[FATAL] MPU6886 init failed` | I2C 配線 (内蔵接続のはず) | ボードを `m5stick-c-plus` などに間違えていないか |
| 杖 BLE init で異常 | NimBLE library version | `lib_deps` に `h2zero/NimBLE-Arduino @ ~1.4.1` がある確認 |
| LED 側 `scanning` 出るが受信なし | 距離・障害物・送信側動作 | 杖側 Serial で `[BLE] adv start` が出ているか |
| LED が「不明な USB デバイス」になる | TinyUSBDevice.begin 漏れ / unaligned access | 下記「Phase 0 デバッグ知見」を参照 |
| LED が点きっぱなし | 5 秒タイマが動作していない | LED 側 Serial で `[LED] auto OFF` ログ出ているか |
| 振りごとに 2 回点滅 | seq の重複抑止が効いていない | 受信側 `last_seq` 比較を確認 |

## Phase 0 デバッグ知見 (重要・移植時必読)

立ち上げで詰まった点と解決策。同じ HW 構成を再現する際は必ず確認すること。

### XIAO nRF52840 (受信側) の USB CDC
- `Serial` シンボルは Adafruit TinyUSB ライブラリ依存 → `lib_deps` に `adafruit/Adafruit TinyUSB Library`、main.cpp に `#include <Adafruit_TinyUSB.h>` 両方必須
- **`TinyUSBDevice.begin(0)` を `Serial.begin()` より先に明示呼び出し**。無いと Bluefruit 併用時に USB enumerate 失敗 (「不明な USB デバイス (デバイス記述子要求の失敗)」)
- `SdFat - Adafruit Fork` を `lib_ignore`。`File` 型が Adafruit_LittleFS と衝突しビルド不可

### ARM Cortex-M4 unaligned access
- `struct __attribute__((packed))` を `(Payload*)buf` キャストして `uint16_t` を直接読むと **Hard Fault → USB ごとクラッシュ**
- `memcpy(&payload, buf, sizeof(payload))` でローカルにコピーしてから読む

### LED ピン
- `LED_RED/GREEN/BLUE` は使わず `LED_BUILTIN` のみ (RGB ピンが USB 関連パッドと衝突するリスク回避)。active-low

### PowerShell から Serial を読む際の DTR/RTS (チップで真逆)
| デバイス | DtrEnable | RtsEnable | 理由 |
|---------|-----------|-----------|------|
| M5StickC (ESP32) | `$false` | `$false` | DTR/RTS が EN(reset)/IO0(boot) に直結。true だと連続リセットで無出力 |
| XIAO nRF52840 | `$true` | `$true` | USB CDC が host 接続 (DTR) を検出して出力開始 |

`pio device monitor` を使えば ESP32 のリセットシーケンスを正しく扱うので手動 SerialPort より確実。

## Phase 1 への発展ポイント

Phase 0 完了後、以下を順に試す:

1. **受信側 duty cycle scan**: `Bluefruit.Scanner.setInterval(160, 48)` で 30% duty → 平均電流 ~2mA
2. **大容量電池**: XIAO nRF52840 に 18650 LiPo を JST で接続 → 60 日級稼働
3. **杖側 wake-on-motion**: XIAO nRF52840 Sense + LSM6DS3 で System OFF + INT wake
4. **Lumos / Nox 追加**: pitch 推定を追加し、上振り/下振りで trigger_id を分岐

## ファイル一覧

| パス | 役割 |
|------|------|
| `shared/beacon_protocol.h` | ペイロード struct + trigger_id 定数 (杖と LED 共通) |
| `wand_m5stickc/platformio.ini` | M5StickC PIO 設定 |
| `wand_m5stickc/src/main.cpp` | 杖 FW: IMU + NimBLE adv |
| `led_xiao_nrf52840/platformio.ini` | XIAO nRF52840 PIO 設定 |
| `led_xiao_nrf52840/src/main.cpp` | LED FW: Bluefruit scan + LED 制御 |
