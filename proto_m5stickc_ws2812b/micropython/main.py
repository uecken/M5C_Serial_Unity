# proto_m5stickc_ws2812b / micropython / main.py
# M5StickC (ESP32) + MicroPython で WS2812B を駆動する版。Thonny で実行/転送できる。
#   Button A (GPIO37) 押下 → 全点灯(黄)/消灯 トグル
#   Button B (GPIO39) 押下 → 電流上限プリセット巡回 (150/300/450/1200 mA)
#   内蔵赤 LED (GPIO10, active-low) で点灯状態を表示
#
# ★ FastLED のような自動電流上限が無いので、ここでは「保守的な実機モデル
#   (黄1個=40mA)」から上限 max_ma を満たす実効輝度を計算して輝度を下げる。
#   配線・電流の根拠は ../README.md / ../docs/ を参照。データピンは C++ 版と同じ G26。

import machine
import neopixel
import time

# ---- ハード設定 (M5StickC) ----
DATA_PIN        = 26    # WS2812B DIN ← 底面ヘッダ G26 (Grove G32/G33 でも可)
NUM_LEDS        = 40    # 点灯数
BTN_A_PIN       = 37    # 前面 M5 ボタン (active-low, input-only)
BTN_B_PIN       = 39    # 側面ボタン   (active-low, input-only)
BUILTIN_LED_PIN = 10    # 内蔵赤 LED   (active-low)

# ---- 表示パラメータ ----
COLOR      = (255, 255, 0)   # 黄 (R, G, B)。色がずれる個体は順序を入れ替える
brightness = 255             # 目標輝度 0-255 (電流上限で実効値は下がりうる)

# ---- 電流モデル (保守的・実機寄り) ----
PER_LED_FULL_MA = 40         # 黄1個フル輝度の概算電流
IDLE_MA         = 1          # 制御IC待機/個

# ---- 電流上限プリセット [mA] (Button B で巡回) ----
MA_PRESETS = (150, 300, 450, 1200)   # 1200 = 外部5V電源使用時
ma_index   = 2                       # 既定 450mA
max_ma     = MA_PRESETS[ma_index]

np  = neopixel.NeoPixel(machine.Pin(DATA_PIN), NUM_LEDS)
btn_a = machine.Pin(BTN_A_PIN, machine.Pin.IN)   # M5StickC 外部プルアップ
btn_b = machine.Pin(BTN_B_PIN, machine.Pin.IN)
led   = machine.Pin(BUILTIN_LED_PIN, machine.Pin.OUT)
led.value(1)   # OFF (active-low)

on = False


def effective_brightness():
    """max_ma を満たすよう輝度を制限して返す (実機モデル基準)。"""
    budget = max_ma - NUM_LEDS * IDLE_MA
    if budget <= 0:
        return 0
    b_max = budget * 255 // (NUM_LEDS * PER_LED_FULL_MA)
    return min(brightness, max(0, b_max))


def estimate_ma(eff):
    return NUM_LEDS * (IDLE_MA + eff * PER_LED_FULL_MA // 255)


def render():
    eff = effective_brightness()
    if on:
        col = (COLOR[0] * eff // 255, COLOR[1] * eff // 255, COLOR[2] * eff // 255)
    else:
        col = (0, 0, 0)
    for i in range(NUM_LEDS):
        np[i] = col
    np.write()
    led.value(0 if on else 1)   # 内蔵LEDで状態表示 (active-low)
    capped = "  (capped)" if eff < brightness else ""
    print("[LED] {} n={} bri={} -> eff={} cap={}mA est={}mA{}".format(
        "ON " if on else "OFF", NUM_LEDS, brightness, eff, max_ma, estimate_ma(eff), capped))


def main():
    global on, ma_index, max_ma
    print("=== M5StickC -> WS2812B (MicroPython) ===")
    print("Btn A: 点灯/消灯トグル  | Btn B: 電流上限プリセット巡回")
    render()

    last_a = 1
    last_b = 1
    while True:
        va = btn_a.value()
        if va == 0 and last_a == 1:        # 立下り = 押下
            on = not on
            render()
            time.sleep_ms(200)             # デバウンス
        last_a = va

        vb = btn_b.value()
        if vb == 0 and last_b == 1:
            ma_index = (ma_index + 1) % len(MA_PRESETS)
            max_ma = MA_PRESETS[ma_index]
            print("[CAP] max current -> {}mA".format(max_ma))
            render()
            time.sleep_ms(200)
        last_b = vb

        time.sleep_ms(10)


if __name__ == "__main__":
    main()
