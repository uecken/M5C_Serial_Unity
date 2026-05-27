# led_xiao_esp32c3_ws2812b — 空中の魔法演出 LED 受信機

杖の BLE Advertising（`company_id=0xFFFF`）を受信し、**WS2812B（NeoPixel）で空中の魔法演出**を行う受信機。
**ケース外設置・WS2812B は外部 5V 給電**（在ケース内の `led_xiao_nrf52840/` とは別ユニット）。

- **MCU**: Seeed XIAO ESP32-C3（BLE = NimBLE / WS2812B = RMT ハードウェアタイミング）
- **プロトコル**: `shared/beacon_protocol.h` を流用（杖の改造不要、1:N broadcast なので在ケース受信機と同時受信可）
- BLE は **passive 連続スキャン**（adv だけ受信、低レイテンシ）

## 配線

```
XIAO ESP32-C3 GPIO10(D10) ──[330Ω]──▶ WS2812B DIN
外部5V電源 (+) ───────────────────────▶ WS2812B 5V
外部5V電源 (GND) ──┬───────────────────▶ WS2812B GND
                   └──────────────────▶ XIAO GND   ★共通GND必須
```

- ⚠ **共通 GND**: 外部5V電源の GND と XIAO の GND を必ず接続（最頻出の不点灯原因）。
- ⚠ **データのレベルシフト**: XIAO の DIN 出力は 3.3V。WS2812B は 5V ロジック想定 → 確実性のため **74AHCT125 等で 3.3V→5V** 推奨（短距離・少数なら 3.3V でも動くことあり）。SK6812 や 3.3V 駆動の WS2812B なら不要。
- 電源容量: 画素数 × 最大 ~60mA（全白）で見積（例 30連 ≈ 1.8A）。データ先頭に ~330Ω 直列、電源に 1000µF 程度の電解コンデンサ推奨。

## 設定（`src/main.cpp` 冒頭の #define だけ変更）

| define | 既定 | 意味 |
|---|---|---|
| `NUM_PIXELS` | 30 | WS2812B の画素数 |
| `LED_DATA_PIN` | 10 | データピン（XIAO ESP32-C3 D10=GPIO10） |
| `MAX_BRIGHTNESS` | 160 | 上限輝度 0-255（眩しさ/発熱抑制） |
| `DEVICE_ID` | 1 | 宛先ID（`TARGET_ALL` か一致で反応） |

## 呪文 → 演出

| trigger | 演出 |
|---|---|
| LUMOS (0x10) | 全画素 暖色白（NOX まで持続） |
| NOX (0x11) | 全消灯 |
| SHAKE (0x01) | 一瞬の白フラッシュ |
| INCENDIO (0x20) | 橙の炎ゆらぎ 3秒 |
| AGUAMENTI (0x21) | 青が流れる波 5秒 |
| EXPECTO PATRONUM (0x23) | 銀青が中心から放射する波 6秒 |
| WINGARDIUM (0x22) | **ピッチ(strength)で光の塊を上下に浮遊**（縦配置なら Web の羽の物理版）。連続制御・~1.5s 途絶で解除 |

## ビルド・書き込み

```bash
cd led_xiao_esp32c3_ws2812b
pio run -e seeed_xiao_esp32c3 -t upload
pio device monitor
```

起動後シリアルに `=== WS2812B aerial receiver ===` と `[BLE] scanning ...`、受信時に `[RX] seq=.. trig=0x..` が出る。

## メモ
- WS2812B は ESP32 の RMT でタイミング生成するため `show()` 中も BLE スキャンを阻害しにくい（nRF52840 の bit-bang より安定）。
- 縦に並べると Wingardium の「浮遊」が映える。リング/マトリクスにする場合は `render()` の各 case を配置に合わせて調整。
