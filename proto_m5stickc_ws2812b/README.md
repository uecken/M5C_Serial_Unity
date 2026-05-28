# proto_m5stickc_ws2812b

M5StickC (ESP32) で WS2812B を駆動する **「魔法受信 LED」**。

- **Button A / Serial** で点灯・色・個数・電流上限を操作(**NVS 永続化**)。既定 **60 個**。
- **BLE Advertising 受信**: `proto_wand_to_led` の杖が出す魔法ビーコン(spell)を **scan 受信**し、
  **魔法ごとに色/エフェクトを切り替える**(`shared/beacon_protocol.h` を杖と共用)。
- **WiFi** は将来用フック(`env:m5stick-c-wifi` の `-D ENABLE_WIFI`)。今は stub。

パラメータ源(シリアル / BLE / WiFi)は、すべて同じ `cfg::` / `dispatch::spell()` API に合流する
**command-source 抽象**で実装。新しい源(WiFi 等)は同じ setter を呼ぶだけで追加できる。

> 電流の要点: WS2812B 1個=黄フル ~40mA。**60個フル ≈ 2.4A** は M5StickC の 5V では賄えない。
> FastLED `setMaxPowerInVoltsAndMilliamps()` で上限をかけ超過時は輝度を自動低減。
> 多数個をフル輝度にするには**外部5V電源 + GND共通**が必須。

## ドキュメント (docs/)

| ファイル | 内容 |
|---------|------|
| [docs/design.md](docs/design.md) | 設計詳細: 電流モデル / FastLED 電力管理 / 多源アーキテクチャ / BLE受信 / 設計判断 |
| [docs/wiring.md](docs/wiring.md) | 配線・部品表(BOM) / レベルシフト回路 / 組立手順 / トラブルシュート |
| [docs/measurement.md](docs/measurement.md) | 「電流が足りるか」を実機で検証する手順と記入用テーブル |
| [micropython/](micropython/) | MicroPython 版(Thonny)。**単体点灯のみ**(BLE受信は C++ 版) |

## 1. 電源・電流(最重要)

WS2812B 1個あたりの消費電流(フル輝度):

| 色 | 点灯チャンネル | 電流/個 |
|----|--------------|--------|
| 白 | R+G+B | 約 60 mA |
| **黄** | **R+G** (B=0) | **約 40 mA** |
| 各IC待機 | — | 約 1 mA |

→ **60個 黄色フル輝度 ≈ 40mA × 60 ≈ 2.4 A**(白なら ~3.6A)。M5StickC の USB/Grove 5V では不可。

| 給電方法 | 供給可能 | 60個フル | 備考 |
|---------|---------|:----:|------|
| 内蔵バッテリ + AXP192昇圧 | 数百mA | ❌ | 即ブラウンアウト |
| PC USB (500mA) | 500 mA | ❌ | 落ちる |
| 5V/2A 充電器経由 | 2 A | ❌ | 2.4A に届かず・Grove発熱 |
| **外部5V電源(3A以上)→ ストリップ直結** | 電源次第 | ✅ | GND だけ M5StickC と共通化。**必須** |

### 安全装置(FastLED 電流上限)
FW は `setMaxPowerInVoltsAndMilliamps(5V, max_ma)` で消費電流に上限をかけ、超える場合は
**実効輝度を自動で下げて**収める。`max_ma` は Button B / Serial `p=` で変更(プリセット 150/300/450/1200/2500mA)。
→ USB 1本でも 60個を「安全に(暗く)」点灯でき、外部5V電源時は `p=2500` 等でフル輝度に。
**上限は保護であって電源容量の代わりではない**(明るく光らせるには十分な外部電源が要る)。詳細は [docs/design.md](docs/design.md)。

## 2. 配線 / GPIO

データは **底面 8 ピンヘッダの G26** を使用(`DATA_PIN=26`)。電源(5V/GND)は底面ヘッダ
または Grove ポートから取得できる(GND は本体内で共通なので混在可)。ピン位置は本体底面のシルクで確認すること。

```
M5StickC (底面ヘッダ)     WS2812B ストリップ
  G26 ──[330Ω]─────────▶ DIN   (データは短く)
  GND ───────┬──────────  GND   ★必ず共通 (ヘッダ or Grove)
  5V  ───────┼──────────  5V    (USB給電。ヘッダ or Grove の 5V)
             │
          [1000µF] ← 5V-GND間・先頭LED付近 (突入電流吸収)

■ 外部5V電源を使う場合 (60個フル輝度・推奨):
  外部5V(+) ────────────  5V
  外部5V(GND)─┬─────────  GND
  M5StickC GND ┘ ★外部電源とGNDを共通に (データ基準を合わせるため必須)
   ※このとき M5StickC 側の 5V はストリップにつながない
```

### レベルシフト(3.3V データの注意)
WS2812B の DIN High しきい値は約 **0.7×VDD = 3.5V**。ESP32 は 3.3V 出力でわずかに下回る。
短距離なら直結で動くことが多いが**非保証**。ちらつき/先頭LEDの色化けが出たら:
1. **74AHCT125 / 74HCT245** 等のレベルシフタ(5V電源)をデータ線に入れる ← 最も確実
2. ストリップの **5V線に 1N4007 を1〜2本直列**で VDD を落とす(詳細 [docs/wiring.md](docs/wiring.md))

### 使用 GPIO まとめ

| 用途 | ピン | 備考 |
|-----|-----|------|
| WS2812B DATA | **G26** (底面ヘッダ) | 330Ω 任意。代替: G32 / G33 (Grove)。G36/G0 は不可 |
| トリガボタン | **G37** (内蔵 Button A) | active-low, 配線不要 |
| 上限切替ボタン | G39 (内蔵 Button B) | active-low, 配線不要 |
| 状態表示 | G10 (内蔵赤LED) | active-low |

## 3. 操作 (Serial / Button)

| 操作 | 動作 |
|-----|------|
| **Button A** | 手動点灯(`color`)/消灯トグル |
| Button B | 電流上限プリセット巡回 150→300→450→1200→2500 mA |
| 内蔵赤LED | 点灯/エフェクト中 = 点灯表示 |

### Serial コマンド (115200)
- **LED**: `on`/`off` | `n=<個数>`(1〜128) | `b=<0-255 輝度>` | `p=<mA上限>` | `c=r,g,b` | `y/w/r/g/b`(色)
- **設定**: `show` | `save`(NVS保存) | `load` | `default` | `id=<1-65534>`(BLE宛先) | `ble=0|1`(scan ON/OFF)
- **魔法テスト**(電波なしでローカル発火 = BLE受信と同じ経路):
  `spell l`(lumos) | `spell nox` | `spell i`(incendio) | `spell a`(aguamenti) | `spell e`(patronum) | `spell t`(shake) | `spell w <0-255>`(wingardium)
- `help`

設定は `save` で NVS に永続化(再起動後も保持)。`show` で現在値、各操作後に `eff_bri`(電流上限で実際に使う輝度)を表示。

## 4. BLE 魔法受信(spell → 色/エフェクト)

杖(`proto_wand_to_led/wand_m5stickc` 等)が出す Advertising ビーコンを **NimBLE で連続 scan** し、
`shared/beacon_protocol.h` の `trigger_id` を解釈して点灯を切り替える。**宛先フィルタ**(`target_id` が
`TARGET_ALL` か自機 `id`)・**重複抑止**(`seq`、ただし Wingardium は連続反映)は受信側 `led_xiao_nrf52840` を踏襲。

| 魔法 (trigger_id) | エフェクト | 時間 |
|---|---|---|
| LUMOS (0x10) | 暖白色 全点灯(持続) | NOX まで |
| NOX (0x11) | 消灯 | 即時 |
| INCENDIO (0x20) | オレンジ揺らぎ(flicker) | 3 秒 |
| AGUAMENTI (0x21) | 青フェードアウト | 5 秒 |
| EXPECTO PATRONUM (0x23) | 白い波がストリップを流れる | 3 秒 |
| SHAKE (0x01) | 一瞬の白フラッシュ(魔法失敗) | 250 ms |
| WINGARDIUM (0x22) | strength → 輝度(連続制御) | 連続(~1s 無音で終了) |

時限エフェクト終了後は **直前の手動状態(OFF / 手動点灯)** に戻る。`ble=0` で受信停止、`id=<n>` で宛先 ID 設定。
**動作確認**: 杖 FW を別の M5StickC に焼き、杖のシリアル `l`/`n`/`i`/`a`/`e` で魔法を飛ばす(`2 l` で ID2 宛て)。

## 5. WiFi(将来)

`env:m5stick-c-wifi`(`-D ENABLE_WIFI`)に `net` フックを用意(今は stub)。実装時は HTTP/MQTT 等を受けて
`cfg::` の setter と `dispatch::spell()` を呼ぶだけで、シリアル/BLE と同じ command-source に合流する。
※ ESP32 は WiFi+BT 同時動作が可能だが単一無線の共存制約があるため、本格運用時は scan duty 調整等が要る(design.md)。

## 6. ビルド・書き込み

```bash
cd proto_m5stickc_ws2812b
pio run -e m5stick-c -t upload      # 通常版 (BLE受信あり)
pio run -e m5stick-c-wifi           # WiFiフック有効版 (今は stub のビルド確認用)
pio device monitor
```

## 7. 推奨セットアップ早見

- **杖の魔法で 60個を光らせたい**: 外部5V電源(3A+)+ GND共通 → `p=2500`、`save`。杖から spell を飛ばす。
- **机上で動作確認(USB1本)**: 既定のまま。`spell i` 等で各エフェクトを確認(暗いが安全に動く)。
- **配線を増やさず明るさ妥協**: `n=30 b=80` 程度で PC USB でも安定。
