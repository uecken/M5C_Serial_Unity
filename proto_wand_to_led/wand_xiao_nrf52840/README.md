# wand_xiao_nrf52840 (杖: XIAO nRF52840 Sense 版)

M5StickC 版の杖 (`wand_m5stickc`) を **XIAO nRF52840 Sense + 内蔵 LSM6DS3TR-C** へ移植したもの。
ジェスチャ判定ロジックは `../shared/wand_gesture.h` を M5 版と**共用** (重複なし)。
ワイヤフォーマットは `../shared/beacon_protocol.h` で、受信側 `led_xiao_nrf52840` とそのまま相互運用できる。

> **状態 (2026-05-23): ビルド成功・実機 IMU 検証は保留中**。
> ソフトウェア (BLE Advertising / ボタン / ジェスチャ共通コア) は実装完了。
> 内蔵 IMU(LSM6DS3TR-C) のハードウェア I2C が nRF52 でハングする問題を回避するため
> **bit-bang(ソフト) I2C に切り替え済み**だが、WHO_AM_I 応答の実機確認は未完 (下記)。

## このポートの方針 (確定事項)

1. **外付ボタン**: XIAO にユーザーボタンが無い (RESET のみ) → タクトスイッチ 1 個を `BUTTON_B_PIN`(既定 **D0**) と GND の間に配線 (`INPUT_PULLUP`, 押下=LOW)。M5 の B ボタン相当:
   - **長押し ≥1s** = モーション ⇄ 手動モード切替 (赤 LED: モーション=2回点滅 / 手動=3回点滅)
   - **一瞬押し** = モーション時:手元 LED フィードバック ON/OFF / 手動時:LUMOS↔NOX 交互送信
2. **省電力はスコープ外**: まず always-on で動作確認。LSM6DS3 wake-on-motion + nRF52840 System OFF は今後。起動時は常にモーションモード。
3. **共通コア**: ジェスチャ判定は `shared/wand_gesture.h` の `wand_gesture::Detector`。BLE 送信と LED フラッシュを関数ポインタで注入する。

## ハードウェア (XIAO nRF52840 Sense, variant=Seeed_XIAO_nRF52840_Sense)

| 機能 | Arduino マクロ | 絶対 nRF GPIO | 備考 |
|------|---------------|--------------|------|
| 内蔵 IMU | LSM6DS3TR-C | — | I2C addr **0x6A**, WHO_AM_I(0x0F)=0x6A, 出力はリトルエンディアン |
| IMU SDA / SCL | `PIN_WIRE1_SDA`=D17 / `PIN_WIRE1_SCL`=D16 | P0.07 / P0.27 | 本来 Wire1。ただし現状は bit-bang で手動駆動 |
| IMU 電源 EN | `PIN_LSM6DS3TR_C_POWER`=D15 | P1.08 | HIGH で給電 (begin で両極性を試行) |
| IMU INT1 | `PIN_LSM6DS3TR_C_INT1`=D18 | P0.11 | 今回未使用 (将来 WOM) |
| 内蔵赤 LED | `LED_BUILTIN`=`LED_RED`=D11 | P0.26 | active-low (LOW=点灯) |
| 外付ボタン | `BUTTON_B_PIN`=**D0** | P0.02 | 要配線 (タクト → GND, INPUT_PULLUP) |

加速度換算: ±8g レンジ → 0.244 mg/LSB → `g = raw / 4098.0` (`IMU_ACC_LSB_PER_G`)。

## ビルド・書き込み

```bash
pio run -e seeed_xiao_nrf52840_sense
pio run -e seeed_xiao_nrf52840_sense -t upload --upload-port COMxx
```

**書き込みは RESET ダブルタップ → UF2 が最も確実** (下記「重要な落とし穴」参照)。

## 重要な落とし穴 (このポートで判明、再開時の注意)

### A. nRF52 ハードウェア I2C (TWIM) が IMU バスでハングする ★最重要
- Adafruit nRF52 core の `Wire`/`Wire1` は **タイムアウトが無い** (`Wire_nRF52.cpp` の
  `while(!_p_twim->EVENTS_STOPPED);` 等)。IMU が ACK を返さない/バスが不安定だと**無限ループでハング**する。
- 症状: `setup()` が `imu::begin()` の WHO_AM_I 読みで停止 → `loop()` に到達せず、シリアルに何も出ない
  (USB CDC は FreeRTOS タスクで生きているのでポートは見える=紛らわしい)。
- **対策: IMU アクセスを全て bit-bang(ソフト I2C) に変更** (`imu::bb` 名前空間)。クロックを自前で
  刻むのでハングしない。Wire1 ピン (SDA=P0.07/D17, SCL=P0.27/D16) を `pinMode`/`digitalWrite` で手動駆動。
- ピン変換は OK だった: `Wire` コンストラクタは `g_ADigitalPinMap[]` で Arduino→絶対 GPIO 変換するので
  Wire1 のピン指定自体は正しい。ハングは純粋に TWIM のタイムアウト欠如が原因。

### B. 書き込み (DFU) の信頼性
- 通常時は 1200bps タッチで自動 DFU 入場するが、**チップ上の FW がハングしていると 1200bps タッチに
  応答せず DFU に入れない** (`adafruit-nrfutil` が "No data received... Not in DFU mode" で失敗)。
- **確実な方法: RESET ボタンを素早く2連打 → `XIAO-SENSE` ドライブ出現 → `firmware.uf2` をコピー**。
  - hex→uf2 変換: `python <framework>/tools/uf2conv/uf2conv.py firmware.hex -c -f 0xADA52840 -o firmware.uf2`
- COM 番号が頻繁に変わる / 書き込み直後はポートが一時ロックされるので、検出→リトライで開く。

### C. その他 (led_xiao_nrf52840 と共通)
- `TinyUSBDevice.begin(0)` を `Serial.begin()` より先に。`lib_ignore = SdFat - Adafruit Fork`。
- PowerShell SerialPort で読むときは `DtrEnable=$true`(XIAO は host 接続検出で出力開始)。

## 残作業 (再開時)

1. RESET ダブルタップで UF2 書き込み → シリアルで `[IMU] bitbang WHO_AM_I: power_HIGH=0x.. power_LOW=0x..` を確認。
   - `0x6A` が出る極性で IMU 応答 OK。両方 NACK なら配線/個体を疑う (`idle bus` の SDA/SCL が 1=High か確認)。
2. 静止で `|a|≈1.00g`、各軸に傾けて ±1g を確認 → `device_config.h` の `WAND_FORWARD/RIGHT/UP` を実装向きに合わせる。
3. 受信側 `led_xiao_nrf52840` を別の XIAO に焼き、上振り=LUMOS / 下振り=NOX で LED 連動を確認。
4. 動作確認後、デバッグ用の `setup()` 内 20s シリアル待ち・step マーカー・`idle_levels` を簡素化。
5. (将来) bit-bang が安定したら速度面で十分か評価。不足なら TWIM をタイムアウト付き自前ドライバ化 or リカバリ実装。
