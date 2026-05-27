# 呪文 ↔ モーション ↔ LED 効果 一覧 (proto_wand_to_led)

最終更新: 2026-05-20 / 対象 FW: `wand_m5stickc` (杖) + `led_xiao_nrf52840` (受信)

杖 (M5StickC + MPU6886) が**重力基準フリック検出**でモーションを自動判定し、
BLE Advertising Beacon (`trigger_id`) を 1:N broadcast。受信側 (XIAO nRF52840 / スマホ) が反応する。

---

## 呪文一覧表

| 呪文 | trig | モーション | 判定条件 (杖側) | LED 効果 (受信側) | 持続時間 | 宛先 |
|------|------|-----------|----------------|------------------|---------|------|
| **LUMOS** | 0x10 | 上フリック **＋ 下記以外の全 Motion (暫定フォールバック)** | `ratio_up≥0.75 & up>0`、または NOX/EXPECTO/INCENDIO に当てはまらない `lmag≥1.5g` の振り | 全 LED 点灯 (赤+緑+青+外部) | **永続 (NOX まで)** | 全機 |
| **NOX** | 0x11 | 下フリック | `ratio_up≥0.75 & up<0` | 全 LED 即消灯 → 青ハートビート再開 | 即時 | 全機 |
| **EXPECTO PATRONUM** | 0x23 | **前突き (杖先端 +Y)** | `ratio_fwd≥0.75 & fwd>0` | **光の波動アニメ**: 外部 LED を内→外へ点灯本数を増やして拡散→全消灯を繰り返す。青が波に同期点滅、最大拡散の瞬間に緑も足して明フラッシュ | **3 秒** (点滅) | 全機 |
| **INCENDIO** | 0x20 | **横振り (±X どちらも)** | `ratio_right≥0.75` | 赤+外部 点灯 (オレンジ風) | 3 秒 | 全機 |
| **AGUAMENTI** | 0x21 | (下流し・**ジェスチャ未実装**) | シリアル `a` のみ | 赤+外部 点灯 (青風) | 5 秒 | 全機 |
| **WINGARDIUM LEVIOSA** | 0x22 | 杖先端を上 ~45°以上に向け **0.8 秒静止保持** で活性化 → 浮遊モード | `pitch_sin>0.70 & lmag<0.30` を 0.8s | **LED 受信機は無反応** (スマホ用)。`strength`=ピッチを連続送信 | 浮遊 8 秒 / 50ms (20Hz) 間隔送信 | 全機 |
| (SHAKE) | 0x01 | ジェスチャでは送出しない (曖昧な振りは LUMOS にフォールバック) | シリアル `t` のみ | 赤+外部 一瞬点灯 (魔法失敗の演出) | 250ms | 全機 |

> **持続時間の定義** (`shared/beacon_protocol.h`):
> `LED_DURATION_SHAKE_MS=250` / `LED_DURATION_INCENDIO_MS=3000` /
> `LED_DURATION_AGUAMENTI_MS=5000` / `LED_DURATION_PATRONUM_MS=3000`。
> LUMOS は時限ではなく `all_on()` 永続 (NOX の `all_off()` で消灯)。

---

## モーション判定アルゴリズム — Mahony は**未使用**

姿勢推定 (Mahony / Madgwick / DMP) は使っていない。**加速度のみ**の軽量な重力基準フリック検出:

1. **重力推定** = 生加速度の EMA ローパス。`grav += alpha*(accel - grav)`、`alpha=0.02`。
   **静止ゲート**付き: `||accel|-1g| < still_band(0.15g)` の時だけ更新 (振りの混入を防ぐ)。
   → ジャイロは読んでいない (`read_accel_g` は accel 3 軸のみ。WOM 時は `PWR_MGMT_2` で gyro 無効)。
2. **「上」方向** = 実測重力の単位ベクトル `u = grav/|grav|`。
3. **linear accel** = `生 − 重力推定`。これを各方向へ射影:
   - 鉛直 (上下): `up_proj = linear · u` … 重力フレーム
   - 前後: `fwd_proj = linear を WAND_FORWARD(+Y) に射影` … 機体軸
   - 左右: `right_proj = linear を WAND_RIGHT(+X) に射影` … 機体軸
4. **割合判定**: `ratio_* = |*_proj| / |linear|` (= その方向への揃い具合 `|cosθ|`)。
   `ratio ≥ updown_ratio(0.75)` で「その軸方向の明確な振り」と判定。
5. **WINGARDIUM のピッチ** = 重力単位ベクトルを WAND_FORWARD に射影した `pitch_sin` (−1..+1)。
   `strength = (pitch_sin+1)*127.5` → 0(真下)/128(水平)/255(真上)。

- 3 軸 (上下=重力フレーム / 前後・左右=機体軸) が直交するので、明確な単軸振りは競合しない。
- どの軸も 0.75 未満の**斜め振り等は【暫定】LUMOS にフォールバック** (魔法失敗で無反応にせず、とにかく点灯させる方針)。共通クールダウン 1 秒。
- ⚠️ **Yaw (方位) は加速度だけでは出ない**。軌跡認識 (Phase 3+) では Mahony + ジャイロ + (必要なら磁気) を導入予定。それまで上下/前後/左右の単発フリックのみ。

### 軸割当 (`wand_m5stickc/src/device_config.h`、機体ごとに #define で変更)
- `WAND_FORWARD = +Y` (杖先端) … 前突き = Expecto Patronum
- `WAND_RIGHT   = +X` (右) … 横振り = Incendio
- `WAND_UP      = +Z` (上、初期重力整合チェック用)
- 基本姿勢: M5StickC を LCD 左向きの縦持ち、先端 = +Y。

---

## 対応デバイス (現状 2026-05-20)

| 役割 | デバイス | env / 備考 |
|------|---------|-----------|
| 杖 (送信) | M5StickC + MPU6886 (ESP32-PICO-D4) | `m5stick-c` = Button A wake / `m5stick-c-wom` = MPU6886 WOM 加速度 wake。15s 静止で deep sleep |
| LED 受信 | XIAO nRF52840 (無印・IMU なし) + Bluefruit | 連続スキャン (100% wake)。内蔵 RGB + 外部 LED (D0~D10, active-high) |
| スマホ受信 | Web Bluetooth (`requestLEScan`) | Wingardium の羽浮遊など。Android/PC Chrome のみ・iOS 不可 ([phone_web_constraints.md](phone_web_constraints.md)) |

通信: BLE Adv Beacon (connectionless 1:N broadcast)、`company_id=0xFFFF`、7B payload
`{company_id, seq, trigger_id, strength, target_id}`。adv interval 20ms 固定。
振り検出後 500ms burst (Wingardium 中は 50ms ごとに再 emit で実質連続)。

### 杖シリアルコマンド (手動送信)
`t`=SHAKE / `l`=LUMOS / `n`=NOX / `i`=INCENDIO / `a`=AGUAMENTI / `e`=EXPECTO / `w`=WINGARDIUM 浮遊強制。
`<id> <cmd>` で特定機宛て (例 `2 l`)。閾値: `gth=/gratio=/gcool=/galpha=/gband=/wom=/gsave/gdefault/gshow`。

---

## 受信側 LED 効果の実装メモ
- **LUMOS** `all_on()`: 内蔵 RGB 全点灯 + 外部 ON、`lumos_mode=true` でハートビート抑止、NOX まで持続。
- **NOX** `all_off()`: 全消灯、`lumos_mode=false`、青ハートビート (5s 周期) 再開。
- **INCENDIO / AGUAMENTI / SHAKE** `red_on_for(ms)`: 内蔵赤 + 外部を時限点灯、`poll()` が時間で自動消灯。
- **EXPECTO PATRONUM** `patronum_start(3000)`: `poll()` が `PATRONUM_TICK_MS(70ms)` ごとに外部 LED の点灯本数を 0→1→…→全→0 と循環させ「波の拡散」を表現 (最優先・3 秒)。
- 内蔵青はアイドル時 5 秒に 1 回 100ms の**ハートビート**。
