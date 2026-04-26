// Burst Motion - hal/esp32/M5StickCDisplay.hpp
// M5StickC 内蔵 ST7735S 0.96" LCD (80×160) ドライバ
// M5 lib 非依存、SPI 直接制御
//
// M5StickC LCD pin map:
//   MOSI = GPIO 15
//   CLK  = GPIO 13
//   CS   = GPIO 5
//   DC   = GPIO 23
//   RST  = GPIO 18
//   BL   = AXP192 LDO2 (要 AXP192 経由電源 ON)
#pragma once
#include <Arduino.h>
#include <SPI.h>

namespace BurstMotion {

class M5StickCDisplay {
public:
    static constexpr int PIN_MOSI = 15;
    static constexpr int PIN_CLK  = 13;
    static constexpr int PIN_CS   = 5;
    static constexpr int PIN_DC   = 23;
    static constexpr int PIN_RST  = 18;

    static constexpr uint16_t WIDTH = 80;
    static constexpr uint16_t HEIGHT = 160;
    static constexpr uint16_t COL_OFFSET = 26;  // ST7735S 80x160 panel offset
    static constexpr uint16_t ROW_OFFSET = 1;

    // 16-bit RGB565 colors
    static constexpr uint16_t BLACK   = 0x0000;
    static constexpr uint16_t WHITE   = 0xFFFF;
    static constexpr uint16_t RED     = 0xF800;
    static constexpr uint16_t GREEN   = 0x07E0;
    static constexpr uint16_t BLUE    = 0x001F;
    static constexpr uint16_t YELLOW  = 0xFFE0;
    static constexpr uint16_t CYAN    = 0x07FF;
    static constexpr uint16_t MAGENTA = 0xF81F;
    static constexpr uint16_t ORANGE  = 0xFD20;
    static constexpr uint16_t GRAY    = 0x8410;

    bool begin() {
        pinMode(PIN_CS, OUTPUT);
        pinMode(PIN_DC, OUTPUT);
        pinMode(PIN_RST, OUTPUT);
        digitalWrite(PIN_CS, HIGH);
        SPI.begin(PIN_CLK, -1, PIN_MOSI, PIN_CS);
        SPI.setFrequency(20000000);
        SPI.setDataMode(SPI_MODE0);

        // hardware reset
        digitalWrite(PIN_RST, HIGH); delay(50);
        digitalWrite(PIN_RST, LOW);  delay(50);
        digitalWrite(PIN_RST, HIGH); delay(150);

        // ST7735S init (M5StickC 互換)
        cmd(0x01); delay(150);  // SWRESET
        cmd(0x11); delay(255);  // SLPOUT

        cmd(0xB1); data(0x05); data(0x3C); data(0x3C);  // FRMCTR1
        cmd(0xB2); data(0x05); data(0x3C); data(0x3C);  // FRMCTR2
        cmd(0xB3); data(0x05); data(0x3C); data(0x3C); data(0x05); data(0x3C); data(0x3C);  // FRMCTR3
        cmd(0xB4); data(0x03);  // INVCTR
        cmd(0xC0); data(0x62); data(0x02); data(0x04);  // PWCTR1
        cmd(0xC1); data(0xC0);  // PWCTR2
        cmd(0xC2); data(0x0D); data(0x00);
        cmd(0xC3); data(0x8D); data(0x6A);
        cmd(0xC4); data(0x8D); data(0xEE);
        cmd(0xC5); data(0x0E);  // VMCTR1
        cmd(0x36); data(0xC8);  // MADCTL: BGR、回転 (LCD 上面が +Y)
        cmd(0x3A); data(0x05);  // COLMOD: 16-bit/pixel

        // Gamma (省略可、デフォルトで動く)
        cmd(0x21);              // INVON (反転)
        delay(10);
        cmd(0x29); delay(100);  // DISPON

        fillScreen(BLACK);
        return true;
    }

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
        cmd(0x2A);
        data16(x + COL_OFFSET);
        data16(x + w - 1 + COL_OFFSET);
        cmd(0x2B);
        data16(y + ROW_OFFSET);
        data16(y + h - 1 + ROW_OFFSET);
        cmd(0x2C);
    }

    void fillScreen(uint16_t color) {
        fillRect(0, 0, WIDTH, HEIGHT, color);
    }

    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
        if (x >= WIDTH || y >= HEIGHT) return;
        if (x + w > WIDTH) w = WIDTH - x;
        if (y + h > HEIGHT) h = HEIGHT - y;
        setAddrWindow(x, y, w, h);
        digitalWrite(PIN_DC, HIGH);
        digitalWrite(PIN_CS, LOW);
        uint8_t hi = color >> 8, lo = color & 0xFF;
        for (uint32_t n = (uint32_t)w * h; n > 0; n--) {
            SPI.transfer(hi);
            SPI.transfer(lo);
        }
        digitalWrite(PIN_CS, HIGH);
    }

    void drawPixel(uint16_t x, uint16_t y, uint16_t color) {
        if (x >= WIDTH || y >= HEIGHT) return;
        setAddrWindow(x, y, 1, 1);
        digitalWrite(PIN_DC, HIGH);
        digitalWrite(PIN_CS, LOW);
        SPI.transfer(color >> 8);
        SPI.transfer(color & 0xFF);
        digitalWrite(PIN_CS, HIGH);
    }

    // 5x7 ASCII font, optimized: 1 文字 = 1 setAddrWindow + 35 pixel burst
    // scale=1 限定で高速化、scale>1 はフォールバック
    void drawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale = 1) {
        if (c < 32 || c > 126) c = '?';
        const uint8_t* glyph = FONT_5x7[c - 32];
        if (scale == 1) {
            // 高速版: row-major で 5x7 を burst write
            // 1 文字あたり setAddrWindow 1 回 + 35 pixel = 70 byte SPI burst のみ
            if (x + 5 > WIDTH || y + 7 > HEIGHT) return;
            setAddrWindow(x, y, 5, 7);
            digitalWrite(PIN_DC, HIGH);
            digitalWrite(PIN_CS, LOW);
            uint8_t fg_hi = fg >> 8, fg_lo = fg & 0xFF;
            uint8_t bg_hi = bg >> 8, bg_lo = bg & 0xFF;
            for (int row = 0; row < 7; row++) {
                for (int col = 0; col < 5; col++) {
                    if (glyph[col] & (1 << row)) {
                        SPI.transfer(fg_hi);
                        SPI.transfer(fg_lo);
                    } else {
                        SPI.transfer(bg_hi);
                        SPI.transfer(bg_lo);
                    }
                }
            }
            digitalWrite(PIN_CS, HIGH);
        } else {
            for (int8_t col = 0; col < 5; col++) {
                uint8_t bits = glyph[col];
                for (int8_t row = 0; row < 7; row++) {
                    uint16_t color = (bits & (1 << row)) ? fg : bg;
                    fillRect(x + col*scale, y + row*scale, scale, scale, color);
                }
            }
        }
    }

    // 文字列描画 (scale=1 高速 burst 版)
    void drawString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, uint8_t scale = 1) {
        uint16_t cx = x;
        while (*str) {
            if (*str == '\n') {
                cx = x;
                y += 8 * scale;
            } else {
                drawChar(cx, y, *str, fg, bg, scale);
                // 文字間 1 列の bg
                if (scale == 1 && cx + 5 < WIDTH) {
                    fillRect(cx + 5, y, 1, 7, bg);
                }
                cx += 6 * scale;
            }
            str++;
        }
    }

private:
    void cmd(uint8_t c) {
        digitalWrite(PIN_DC, LOW);
        digitalWrite(PIN_CS, LOW);
        SPI.transfer(c);
        digitalWrite(PIN_CS, HIGH);
    }
    void data(uint8_t d) {
        digitalWrite(PIN_DC, HIGH);
        digitalWrite(PIN_CS, LOW);
        SPI.transfer(d);
        digitalWrite(PIN_CS, HIGH);
    }
    void data16(uint16_t d) {
        digitalWrite(PIN_DC, HIGH);
        digitalWrite(PIN_CS, LOW);
        SPI.transfer(d >> 8);
        SPI.transfer(d & 0xFF);
        digitalWrite(PIN_CS, HIGH);
    }

    // 5x7 ASCII フォント (' '=0x20 から '~'=0x7e、95 文字)
    // 各文字 5 列、各列 8 bit (LSB が上端)
    // 簡略版: 数字 + アルファベット + 主要記号のみ実装、それ以外は空
    static constexpr uint8_t FONT_5x7[95][5] = {
        {0,0,0,0,0},                    // ' '
        {0,0,0x5F,0,0},                  // !
        {0,7,0,7,0},                     // "
        {0x14,0x7F,0x14,0x7F,0x14},      // #
        {0x24,0x2A,0x7F,0x2A,0x12},      // $
        {0x23,0x13,0x08,0x64,0x62},      // %
        {0x36,0x49,0x55,0x22,0x50},      // &
        {0,5,3,0,0},                     // '
        {0,0x1C,0x22,0x41,0},            // (
        {0,0x41,0x22,0x1C,0},            // )
        {0x14,0x08,0x3E,0x08,0x14},      // *
        {0x08,0x08,0x3E,0x08,0x08},      // +
        {0,0x50,0x30,0,0},               // ,
        {0x08,0x08,0x08,0x08,0x08},      // -
        {0,0x60,0x60,0,0},               // .
        {0x20,0x10,0x08,0x04,0x02},      // /
        {0x3E,0x51,0x49,0x45,0x3E},      // 0
        {0,0x42,0x7F,0x40,0},            // 1
        {0x42,0x61,0x51,0x49,0x46},      // 2
        {0x21,0x41,0x45,0x4B,0x31},      // 3
        {0x18,0x14,0x12,0x7F,0x10},      // 4
        {0x27,0x45,0x45,0x45,0x39},      // 5
        {0x3C,0x4A,0x49,0x49,0x30},      // 6
        {0x01,0x71,0x09,0x05,0x03},      // 7
        {0x36,0x49,0x49,0x49,0x36},      // 8
        {0x06,0x49,0x49,0x29,0x1E},      // 9
        {0,0x36,0x36,0,0},               // :
        {0,0x56,0x36,0,0},               // ;
        {0x08,0x14,0x22,0x41,0},         // <
        {0x14,0x14,0x14,0x14,0x14},      // =
        {0,0x41,0x22,0x14,0x08},         // >
        {0x02,0x01,0x51,0x09,0x06},      // ?
        {0x32,0x49,0x79,0x41,0x3E},      // @
        {0x7E,0x11,0x11,0x11,0x7E},      // A
        {0x7F,0x49,0x49,0x49,0x36},      // B
        {0x3E,0x41,0x41,0x41,0x22},      // C
        {0x7F,0x41,0x41,0x22,0x1C},      // D
        {0x7F,0x49,0x49,0x49,0x41},      // E
        {0x7F,0x09,0x09,0x09,0x01},      // F
        {0x3E,0x41,0x41,0x49,0x7A},      // G
        {0x7F,0x08,0x08,0x08,0x7F},      // H
        {0,0x41,0x7F,0x41,0},            // I
        {0x20,0x40,0x40,0x40,0x3F},      // J
        {0x7F,0x08,0x14,0x22,0x41},      // K
        {0x7F,0x40,0x40,0x40,0x40},      // L
        {0x7F,0x02,0x0C,0x02,0x7F},      // M
        {0x7F,0x04,0x08,0x10,0x7F},      // N
        {0x3E,0x41,0x41,0x41,0x3E},      // O
        {0x7F,0x09,0x09,0x09,0x06},      // P
        {0x3E,0x41,0x51,0x21,0x5E},      // Q
        {0x7F,0x09,0x19,0x29,0x46},      // R
        {0x46,0x49,0x49,0x49,0x31},      // S
        {0x01,0x01,0x7F,0x01,0x01},      // T
        {0x3F,0x40,0x40,0x40,0x3F},      // U
        {0x1F,0x20,0x40,0x20,0x1F},      // V
        {0x3F,0x40,0x38,0x40,0x3F},      // W
        {0x63,0x14,0x08,0x14,0x63},      // X
        {0x07,0x08,0x70,0x08,0x07},      // Y
        {0x61,0x51,0x49,0x45,0x43},      // Z
        {0,0x7F,0x41,0x41,0},            // [
        {0x02,0x04,0x08,0x10,0x20},      // (back)slash
        {0,0x41,0x41,0x7F,0},            // ]
        {0x04,0x02,0x01,0x02,0x04},      // ^
        {0x40,0x40,0x40,0x40,0x40},      // _
        {0,1,2,4,0},                     // `
        {0x20,0x54,0x54,0x54,0x78},      // a
        {0x7F,0x48,0x44,0x44,0x38},      // b
        {0x38,0x44,0x44,0x44,0x20},      // c
        {0x38,0x44,0x44,0x48,0x7F},      // d
        {0x38,0x54,0x54,0x54,0x18},      // e
        {0x08,0x7E,0x09,0x01,0x02},      // f
        {0x0C,0x52,0x52,0x52,0x3E},      // g
        {0x7F,0x08,0x04,0x04,0x78},      // h
        {0,0x44,0x7D,0x40,0},            // i
        {0x20,0x40,0x44,0x3D,0},         // j
        {0x7F,0x10,0x28,0x44,0},         // k
        {0,0x41,0x7F,0x40,0},            // l
        {0x7C,0x04,0x18,0x04,0x78},      // m
        {0x7C,0x08,0x04,0x04,0x78},      // n
        {0x38,0x44,0x44,0x44,0x38},      // o
        {0x7C,0x14,0x14,0x14,0x08},      // p
        {0x08,0x14,0x14,0x18,0x7C},      // q
        {0x7C,0x08,0x04,0x04,0x08},      // r
        {0x48,0x54,0x54,0x54,0x20},      // s
        {0x04,0x3F,0x44,0x40,0x20},      // t
        {0x3C,0x40,0x40,0x20,0x7C},      // u
        {0x1C,0x20,0x40,0x20,0x1C},      // v
        {0x3C,0x40,0x30,0x40,0x3C},      // w
        {0x44,0x28,0x10,0x28,0x44},      // x
        {0x0C,0x50,0x50,0x50,0x3C},      // y
        {0x44,0x64,0x54,0x4C,0x44},      // z
        {0,0x08,0x36,0x41,0},            // {
        {0,0,0x7F,0,0},                  // |
        {0,0x41,0x36,0x08,0},            // }
        {0x08,0x04,0x08,0x10,0x08},      // ~
    };
};

}  // namespace BurstMotion
