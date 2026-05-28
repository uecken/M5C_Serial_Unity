# proto_m5stickc_ws2812b — 設計詳細

M5StickC 単体で WS2812B を駆動する実験の設計根拠。運用・配線は
[../README.md](../README.md) と [wiring.md](wiring.md)、実機検証は [measurement.md](measurement.md) を参照。

## 1. 目的とスコープ

| 項目 | 内容 |
|------|------|
| 目的 | M5StickC (ESP32) で WS2812B(既定 **60個**)を駆動。Serial/Button + **BLE 魔法受信**で点灯・色・効果を切替 |
| 核心の問い | 「M5StickC の 5V でこの電流を賄えるか」 + 「杖の魔法ビーコンを受けて点灯を変えられるか」 |
| 結論 | 多数個フル輝度は 5V を**賄えない**(外部5V必須)。FastLED 電流上限で安全化。BLE は `shared/beacon_protocol.h` を共用し scan 受信で実現 |
| スコープ | 60個化 / シリアル+NVS永続化 / BLE Advertising 受信(spell→色・効果) / WiFi は将来用フック(stub) |
| 非スコープ(現状) | WiFi 実装本体、deep sleep / 低電力(連続 scan のため) |

## 2. 電流モデル

### 2.1 WS2812B 実機の電流(データシート/実測ベース)

WS2812B は定電流 PWM 調光。各色チャンネルは ON 時ほぼ定電流で、輝度(8bit値)は
PWM デューティで決まる。したがって**平均電流 ≈ デューティに比例 + IC 待機電流**。

| 項目 | 1個あたり (5V, フル) | 根拠 |
|------|--------------------|------|
| R / G / B 各チャンネル | 約 20 mA | データシート定格 IF |
| 白 (R+G+B) | 約 60 mA | 20×3 (予算用 worst-case) |
| **黄 (R+G, B=0)** | **約 40 mA** | 20×2 |
| 待機 (制御IC, データ idle) | 約 0.7〜1 mA | 常時 |

輝度 `b`(0–255)での黄1個の概算:

```
I_yellow(b) ≈ 0.7mA + (b/255) × 40mA       [mA/個]
I_total(N,b) ≈ N × I_yellow(b)              [mA]  (配線損失は別途)
```

### 2.2 20個 黄色の電流(実機概算)

| 輝度 b | デューティ | 20個 合計(実機概算) |
|:---:|:---:|:---:|
| 255 (100%) | 1.00 | **約 814 mA** |
| 204 (80%) | 0.80 | 約 654 mA |
| 128 (50%) | 0.50 | 約 414 mA |
| 77 (30%) | 0.30 | 約 254 mA |
| 26 (10%) | 0.10 | 約 94 mA |

## 3. M5StickC の 5V 供給能力と判定

| 給電方法 | 供給可能 | 20個フル黄(814mA) | 備考 |
|---------|---------|:----:|------|
| 内蔵バッテリ(~120mAh)+AXP192昇圧 | 実用 数百mA・電圧降下大 | ❌ | 約8分で空。昇圧の電流制限/ブラウンアウト。既定では AXP192 EXTEN 昇圧の enable が必要 |
| PC USB (USB2.0, 500mA) | 500 mA | ❌ | ポート保護で落ちる |
| 5V/2A 充電器・モバイルバッテリ経由 | 2 A | △ | Grove コネクタ/線 ~1A 定格に近く発熱。常用非推奨 |
| 外部5V電源 → ストリップ直結 | 電源次第 | ✅ | GND だけ M5StickC と共通。推奨 |

> M5StickC の Grove 5V は、USB 給電時は VBUS パススルー、バッテリ単体時は AXP192 の
> 昇圧出力(EXTEN 制御)。後者は小容量・電流制限ありで高電流に不向き。本 FW は
> AXP192 を触らず、USB または外部5V前提とする(電池運用が要るなら別途 EXTEN enable 実装)。

### 賄うための3方針(本 FW の対応)
- **(A) 輝度を下げる**(推奨・配線最小):上表のとおり 30% で ~250mA。`b=` または既定の電流上限で自動。
- **(B) 個数を減らす**:フル黄で ~10〜12個(400〜480mA)が USB 500mA 内。`n=` で設定。
- **(C) 外部5V電源**:GND 共通化し `p=1200` 等で上限を上げてフル輝度。

## 4. FastLED 電力管理の仕組みと実機との差(重要)

本 FW は `FastLED.setMaxPowerInVoltsAndMilliamps(5, max_ma)` で電流に上限をかける。
FastLED は CRGB 配列から消費電力を見積もり、上限超過時は `show()` で**実効輝度を自動的に下げて**収める。

### 4.1 FastLED の内部電流モデル(`power_mgt.cpp`)

```
R = 16mA, G = 11mA, B = 15mA, Dark = 1mA  (各LED, 5V, フル)
MCU baseline = 25mA  (1回だけ加算)
total ≈ 25 + Σ_LED [ (r·16 + g·11 + b·15)/255 + 1 ]   [mA @5V]
```

→ FastLED モデルの**黄1個フル = 16+11+1 = 28mA**(実機概算 40mA より小さい)。
20個黄フル = 25 + 20×27 + 20×1 ≈ **585mA**(実機概算 814mA に対し約 0.72 倍)。

### 4.2 ⚠ モデル差のインパクト

FastLED の係数は「複数ストリップの平均実測」由来で、データシート worst-case(20mA/ch)より小さい。
つまり **FastLED の上限値は実機よりやや甘い(楽観的)**。

| | 黄1個フル | 比 |
|---|:---:|:---:|
| FastLED モデル | 28 mA | 1.00 |
| 実機概算(worst) | 40 mA | 約 1.43 |

→ `max_ma=450` と設定しても、FastLED は自モデルで450mAに収めるため、**実機の実電流は ~1.4倍(≈640mA)に達しうる**。
ハードな5V予算を厳守したい場合は次のいずれか:
1. FastLED の上限を**実機係数で割り戻して低めに**設定(例: 真に500mA以内なら `p=350` 程度)
2. [measurement.md](measurement.md) の手順で**実測し** `max_ma` を調整
3. 外部5V電源にして予算自体を広げる

本 FW のシリアル出力 `est=...mA(@full)` は**実機概算(40mA/個基準・保守的)**、`cap=...mA` は
FastLED に渡した上限、`eff_bri` は上限を満たすため実際に使われる輝度。両者を併記して差を可視化している。

## 5. ファームウェア構成(多源 command-source アーキテクチャ)

単一 `src/main.cpp`。**3 つのパラメータ源(Serial / BLE受信 / WiFi将来)が同じ状態 API に合流**する。

```
 [Serial cmd]──┐
 [BLE scan ]──┼─▶ cfg::*(num_leds/brightness/color/max_ma/device_id) ─┐
 [WiFi stub]──┘     dispatch::spell(seq,trig,strength,target)          ├─▶ fx (効果) ─▶ render/commit ─▶ FastLED.show()
                                                                       ┘
```

| 名前空間 | 役割 |
|---------|------|
| `cfg` | パラメータ + **NVS 永続化**(Preferences `ledcfg`)。`load/save/set_defaults/print`。既定 `num_leds=60`, `NUM_LEDS_MAX=128` |
| `fx` | **非ブロッキング効果エンジン**。base(OFF/SOLID/LUMOS持続)+ transient(時限/連続 spell 効果)。`poll()` でアニメ進行、`commit()` で電流上限適用+show |
| `dispatch` | spell 振り分け。**重複抑止(seq)** + **宛先フィルタ(target_id)**(Wingardium は連続反映)。Serial/BLE/WiFi 共通入口 |
| `ble_rx` | NimBLE 1.4.3 scan。`onResult` で `getManufacturerData()`→memcpy→`Payload` 検証し、**FreeRTOS キュー**へ。`loop()` で排出して `dispatch::spell` |
| `net` | WiFi フック。`#ifdef ENABLE_WIFI` 時のみ実体(今は stub) |
| `cmd` | Serial 行パーサ(`spell <x>` でローカル発火 = BLE と同経路) |

要点:
- **点灯数の可変**: `addLeds` は `NUM_LEDS_MAX` 固定、`leds[0..num_leds-1]` のみ色、残りは Black。
- **電流上限**: `fx::commit()` 毎に `setMaxPowerInVoltsAndMilliamps(5, cfg::max_ma)` を適用。`calculate_max_brightness_for_power_vmA()` で実効輝度を算出しログ表示。
- **BLE コールバックの分離**: `onResult` は BLE host タスク文脈で走るため **FastLED を触らず**キューに積むだけ。描画は `loop()` 側に一本化(RMT と BT の競合・並行 show を避ける)。
- **spell→色/効果**: LUMOS=暖白持続 / NOX=消灯 / INCENDIO=橙flicker3s / AGUAMENTI=青fade5s / PATRONUM=白波3s / SHAKE=一瞬250ms / WINGARDIUM=strength→輝度。時限後は base へ復帰。

### 5.1 FastLED + NimBLE(BT)共存
FastLED は ESP32 で RMT(RMT4)で WS2812B を駆動。BT コントローラの割込が RMT の補充を阻害すると `show()` 時に
チラつきが出ることがある(既知・断続的)。緩和策(本 FW で採用 or 余地):
- `show()` を**状態変化時/効果 tick(33〜50ms)に限定**(毎ループ無条件 show しない)。
- **passive scan**(scan-request TX を出さない)。チラつくなら **duty を下げる**(`setWindow<setInterval`)。
- `-D FASTLED_RMT_MAX_CHANNELS=1`(1ストリップ)。必要なら `-D FASTLED_ESP32_FLASH_LOCK=1`。
- **NVS 書込みは `save` 時のみ**(効果中のフラッシュ書込みはチラつき源)。

## 6. 設計判断と代替案

| 判断 | 採用 | 代替 | 理由 |
|------|------|------|------|
| LED ライブラリ | FastLED | Adafruit NeoPixel | `setMaxPowerInVoltsAndMilliamps` で電流を物理キャップでき、本実験の主目的(電流管理)に直結 |
| データ GPIO | G26 (底面ヘッダ) | G32/G33 (Grove) | ユーザー選択。ESP32 RMT は任意の出力ピン可。G36(入力専用)/G0(起動ストラップ)は不可 |
| トリガ | 内蔵 Button A | 外部ボタン | 配線不要、即試せる |
| 既定電流上限 | 450 mA | フル開放 | 単一 USB の安全側の目安。外部電源時のみ上げる運用 |
| AXP192 制御 | しない | EXTEN 昇圧 enable | USB/外部5V前提なら不要。電池運用は本実験のスコープ外 |
| 色指定 | `CRGB::Yellow`(255,255,0) | (255,200,0) 等 | 純色黄を基準。暖色寄りは `c=255,200,0` で再現可 |
| BLE プロトコル | `shared/beacon_protocol.h` を**共用** | 独自定義/コピー | 杖(送信)と受信で単一ソース。重複を避け相互運用を保証(クロスプロジェクト相対 include) |
| BLE ライブラリ | NimBLE-Arduino 1.4.x (scan) | Bluedroid BLE | 杖の ESP32 側と同じ。`led_xiao` の受信ロジックを ESP32 へ移植 |
| 受信→描画の連携 | onResult はキューに積むだけ | コールバックで直接描画 | BLE host タスクと loop の並行 show を避け RMT/BT 競合を低減 |
| 多源対応 | command-source 抽象(同じ setter/dispatch) | 源ごとに別ロジック | WiFi 追加が「同じ API を呼ぶだけ」で済む(§5) |

## 7. 既知の注意点

- **3.3V データ問題**: WS2812B DIN しきい値 ≈ 0.7×VDD。直結は短距離で動くことが多いが非保証 → [wiring.md](wiring.md) のレベルシフト参照。
- **突入電流**: 全点灯の瞬間に大電流。1000µF と 330Ω で緩和。電源が弱いと先頭で電圧降下し色化け。
- **Grove コネクタ電流**: HY2.0/細線は ~1A 定格。高電流常用は発熱・電圧降下の原因 → 外部給電へ。
- **FastLED モデルの楽観性**: §4.2。安全予算厳守は実測 or 係数割戻し。
