# proto_m5stickc_ws2812b / micropython

C++ (PlatformIO/FastLED) 版と同じ動作を **MicroPython** で実装したもの。
**Thonny で書き込み・実行できる**。配線・電流の考え方は [../README.md](../README.md) と
[../docs/](../docs/) と共通(データピンも同じ **G26**)。

## Thonny でできるか → できる

M5StickC は ESP32 なので MicroPython が動き、Thonny から REPL 接続・スクリプト転送・実行ができる。
WS2812B は MicroPython 標準の `neopixel` モジュールで駆動する(追加ライブラリ不要)。

> ⚠ **C++ 版と MicroPython 版は排他**。MicroPython ファームを焼くと、いま入っている
> PlatformIO の C++ ファーム(`pio run -t upload` で書き込んだもの)は**上書きで消える**。
> C++ 版に戻すには再度 `pio run -t upload` する。

## セットアップ手順 (Thonny)

### 1. Thonny をインストール
<https://thonny.org/> から入手(Python 同梱)。

### 2. MicroPython ファームウェアを M5StickC に書き込む
M5StickC のポートは本環境では **COM23**(FTDI 認識)。

- **Thonny 内蔵ツールで書く場合**:
  `ツール → オプション → インタプリタ` で「MicroPython (ESP32)」を選び、
  右下の **「ファームウェアをインストールまたは更新」** から esptool 経由で書き込む。
  ターゲット = ESP32、ポート = COM23、ファームウェア = 下記 .bin。
- **ファームウェア**: <https://micropython.org/download/ESP32_GENERIC/> の最新 `.bin`(generic ESP32)。
  ※ M5Stack の UIFlow ファームでも `machine`/`neopixel` は使えるが、本スクリプトは generic ESP32 で十分。
- **手動で書く場合**(任意):
  ```bash
  esptool.py --port COM23 erase_flash
  esptool.py --port COM23 --baud 1500000 write_flash 0x1000 ESP32_GENERIC-XXXXXXXX.bin
  ```

### 3. インタプリタを設定
`ツール → オプション → インタプリタ` で「MicroPython (ESP32)」、ポート **COM23** を選ぶ。
下部シェルに `>>>`(REPL)が出れば接続成功。

### 4. main.py をデバイスへ転送
- Thonny で [main.py](main.py) を開く。
- `ファイル → 名前を付けて保存` → 保存先に **「MicroPython デバイス」** を選び、ファイル名を **`main.py`** にする。
  (`main.py` はブート時に自動実行される)
- すぐ動かすなら **F5(実行)** でも可。

### 5. 動かす
配線([../docs/wiring.md](../docs/wiring.md))を済ませ、**Button A** を押すと黄色点灯/消灯。
Button B で電流上限プリセット巡回。シェルに `[LED] ON n=20 bri=255 -> eff=... est=...mA` が出る。

## 操作・パラメータ
C++ 版と同じ。`main.py` 冒頭の定数で調整:
- `DATA_PIN = 26` / `NUM_LEDS = 20` / `COLOR = (255,255,0)` / `brightness = 255`
- `MA_PRESETS` … Button B で巡回する電流上限。既定 450mA。

## 注意
- **電流上限は自前計算**: FastLED の自動上限が無いため、保守的な実機モデル(黄1個=40mA)から
  `max_ma` を満たす実効輝度を計算して輝度を絞っている(C++ 版の `setMaxPowerInVoltsAndMilliamps` 相当)。
- **色がずれる個体**: `COLOR` の並びを入れ替える(neopixel の既定は WS2812 の GRB を内部処理し、
  通常は `(R,G,B)` で正しい)。
- **多数個・フル輝度は外部5V**: 電源・GND 共通の考え方は C++ 版と同一([../docs/design.md](../docs/design.md))。
- REPL でインタラクティブに試す例:
  ```python
  import main
  main.brightness = 80      # 暗め(省電流)
  main.on = True
  main.render()
  ```
