# 2026-04-25 Stream 周期停止問題の調査と修正

## 報告
ユーザー: 「ディスプレイ更新があるためか、Stream送信が周期的に止まる。」

## 原因分析

### 旧実装の問題
1. **`drawChar` が pixel 毎に `setAddrWindow`** 発行
   - 1 文字 35 pixel × 4 SPI コマンド = ~140 SPI トランザクション
   - 10 行 × 10 文字 = 14,000 SPI トランザクション/cycle
   - 推定 50-100ms+ blocking
2. **`updateLcd()` が 500ms 毎に全 10+ 行を一気に更新**
   - 1 cycle で全描画 → loop() が 50-100ms ブロック
   - その間 sensor stream が送信できず gap 発生

## 修正内容

### `M5StickCDisplay::drawChar()` バーストモード
```cpp
// 1 文字 = 1 setAddrWindow + 35 pixel burst write
setAddrWindow(x, y, 5, 7);
digitalWrite(PIN_DC, HIGH);
digitalWrite(PIN_CS, LOW);
for (int row = 0; row < 7; row++)
    for (int col = 0; col < 5; col++)
        SPI.transfer(...);  // 直接 burst
digitalWrite(PIN_CS, HIGH);
```
- SPI overhead を ~80% 削減

### `updateLcd()` 行分散更新
- 旧: 500ms 毎に全 10 行を一気に
- 新: **50ms 毎に 1 行のみ** (Stream OFF 時) / **100ms 毎に 1 行** (Stream ON 時)
- 全 12 行で 600ms / 1.2s cycle、各 tick の blocking は ~5ms

### 差分描画
- 各行の前回文字列を `s_lcd_prev[12][32]` に保持
- 同じ文字列なら描画スキップ → 大幅な負荷削減
- 値変化時のみ fillRect + drawString

### `loop()` 順序改善
```cpp
// 1. Serial.process()      ← 最優先 (応答性)
// 2. updateSensor() (100Hz)
// 3. streamSensorIfDue()
// 4. Serial.process() 再   ← LCD 描画前にもう一度
// 5. updateLcd()           ← 1 行のみ ~5ms blocking
// 6. handlePowerButton()
// delay(1)
```

## 結果 (50Hz Stream、6 秒間 ~290 サンプル測定)

### 改善幅
| 指標 | 修正前 (推定) | 第 1 段階 | **最終 (Stream-aware)** |
|------|--------------|-----------|------------------------|
| avg gap | 30-50ms | 21.1ms | **20.6ms** (理想 20ms) |
| p50 | - | 20ms | **20ms** |
| p95 | 80-100ms+ | 36ms | **20ms** ← 最適化 1.8x |
| p99 | - | 43ms | **43ms** |
| max | 150-200ms | 44ms | **43ms** |
| **30ms 超** | ~30% | 8.2% | **3.8%** |
| **50ms 超** | 多発 | 0% | **0% ✅** |
| **100ms 超** | あり | 0% | **0% ✅** |

### LCD 干渉の周期パターン
- 旧: 500ms 周期で 50-100ms+ の gap
- 新: 100-200ms 周期で 30-43ms の小さな gap (LCD 1 行更新分)、p99 でも 50ms 未満

## Web UI: Stream Debug パネル追加

### 機能
- `sensor.t` (FW timestamp) 同士の差を計算 (ブラウザ受信遅延を排除)
- 直近 100 サンプルのヒストグラム表示
- 統計: count / avg / min / p95 / max
- 色分けバー: 🟢 <25ms / 🟡 25-50ms / 🟠 50-100ms / 🔴 >100ms
- Stream ON 中のみ自動表示
- Clear ボタンで再計測可能

### URL
https://uecken.github.io/M5C_Serial_Unity/

ユーザーは Stream ON で実時間 gap を確認、問題が再発したら即座に視覚化できる。

## 残課題

- **44ms max gap**: LCD 1 行 fillRect (80px × 8px = 640pixel = ~10ms) + drawString (10 char × 35 px) が原因。SPI DMA を使えば更に削減可能だが、複雑度増。今回は実用範囲内なので保留。
- **将来**: 16-bit framebuffer をメインメモリに保持して差分のみ flush する方式が理想。NimBLE と RAM を食うので Phase 4 で検討。
