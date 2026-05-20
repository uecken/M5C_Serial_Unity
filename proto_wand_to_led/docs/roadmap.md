# proto_wand_to_led ロードマップ (Phase 0 以降)

正本プラン: `C:\Users\thefu\.claude\plans\m5c-hp-m5c-imu-esp32-imu-nrf52840-esp32-iridescent-cascade.md`

杖 (IMU 付き無線センサ) を振ると、別の電池駆動 LED デバイス (複数台) が反応する。
最終目標は HP 呪文の軌跡認識 + 1:N broadcast。通信は **BLE Advertising Beacon** を共通基盤とし、Phase 進行で MCU / 通信を差し替える。

---

## Phase 0: 動作確認 (✅ 完了 2026-05-20)

**目的**: 通信プロトコル・ジェスチャ検出・LED 制御の成立確認。電池持ちは問わない。

| 項目 | 内容 |
|------|------|
| 杖 | M5StickC (ESP32 + MPU6886) + NimBLE |
| LED | XIAO nRF52840 (Sense) + Bluefruit、両側 100% wake |
| 通信 | BLE Adv Beacon (connectionless broadcast)、payload 7B |
| ジェスチャ | 上振り=LUMOS / 下振り=NOX / 強い振り=SHAKE (重力基準) |
| LED 挙動 | 内蔵青=ハートビート / 内蔵赤+外部=トリガ点灯 / LUMOS=全点灯持続 / NOX=全消灯 |
| 外部 LED | D0~D10 任意ピン (D/G 記法)、active-high source 駆動 |
| ターゲット | target_id 2B (1-65534)、TARGET_ALL=0xFFFF |
| 設定保存 | InternalFS + USB Serial コマンド (id= / pins= / save / show) |

**達成済**: SHAKE/LUMOS/NOX、target_id 個別指定、外部ピン設定、RSSI -39〜-44dBm 安定。

### Phase 0 で確立した実装ノウハウ (移植時必読)
- nRF52840: `TinyUSBDevice.begin(0)` を Serial より先に / `lib_ignore = SdFat`
- payload は memcpy (ARM unaligned access fault 回避)
- PowerShell Serial: ESP32 は DTR/RTS=false、nRF52 は DTR/RTS=true
- 詳細は [design.md](design.md)

---

## Phase 1: 屋内ショーケース向け省電力化

**目的**: 受信機をショーケース内で月単位稼働させる。

| 項目 | 変更内容 |
|------|---------|
| 受信側 scan | 連続 → **duty cycle 30% (`setInterval(160,48)`)** → ~2mA |
| 受信側電池 | 18650 LiPo (3000mAh) → **約 60 日** / 21700 (5000mAh) → 100 日 |
| 杖側 | M5StickC のまま (日常充電) or XIAO nRF52840 Sense + wake-on-motion |
| レイテンシ | 平均 ~50ms / 最大 ~100ms 維持 |

**タスク**:
1. LED `setInterval` を duty cycle 化、電流実測
2. (任意) 杖を XIAO nRF52840 Sense へ移行、LSM6DS3 wake-on-motion + System OFF で数週間稼働
3. 設定の BLE 経由書き換え対応 (USB 不要化の第一歩)
4. **OTA DFU 有効化** — スマホ (nRF Connect) から無線 FW 更新、ショーケースを開けず更新可能に

---

## Phase 2: 屋外対応 (BLE + LoRa デュアル)

**目的**: 屋外 100m〜1km の運用を追加。屋内は BLE、屋外は LoRa。

| 項目 | 内容 |
|------|------|
| HW | 杖・LED 両方 **XIAO nRF52840 + Wio-SX1262 キット** |
| 通信 | BLE Adv (屋内 ~10m) + **LoRa SF7/BW500/920MHz** (屋外 100m-1km) 並列 |
| レイテンシ | 屋内 ~50ms / 屋外 100-300ms |
| 呪文追加 | INCENDIO (前突き→オレンジ) / AGUAMENTI (下流し→青) |

**タスク**:
1. RadioLib で SX1262 制御、LoRa TX/RX 実装
2. 杖: 振り検出 → BLE adv + LoRa TX 並列発射、seq で重複抑止
3. LED: BLE scan + LoRa RX 両待受、先着で点灯
4. INCENDIO / AGUAMENTI のジェスチャ検出ロジック追加

### wake-on-radio の優位性 (Phase 2-3 で LoRa を使う理由)
- BLE 内蔵 MCU は「無線と CPU が同居」→ scan duty cycle で擬似省電力 (~1-2mA)
- LoRa (SX1262) は「無線チップが独立」→ **MCU を System OFF (~1.5μA) にして、SX1262 が DIO1 割り込みで叩き起こす**真の wake-on-radio
- → 受信側を数ヶ月〜年単位で電池駆動できる

---

## Phase 3: LoRa 専用 + wake-on-LoRa (屋外長期設置)

**目的**: 屋外ガーデン/イベント設置で電池交換ほぼ不要に。

| 項目 | 内容 |
|------|------|
| 受信側 | SX1262 **RxDutyCycle (100ms ON / 900ms OFF)**、MCU 完全 deep sleep |
| wake | SX1262 が電波検知 → DIO1 割り込みで nRF52840 wake |
| 杖側 | wake-on-motion + LoRa TX one-shot → System OFF |
| 電池持ち | 受信側 **4 ヶ月〜1 年** (18650)、杖側 数ヶ月 |
| レイテンシ | 最大 ~1-1.5s (sleepPeriod + preamble) ← 屋外用途で妥協 |

**タスク**:
1. SX1262 RxDutyCycle + Long Preamble 実装
2. nRF52840 System OFF + GPIO 割り込み wake
3. レイテンシ vs 電池持ちのチューニング

---

## Phase 4: カスタム PCB 化 (小型化・量産)

**目的**: XIAO 開発ボードから専用基板へ。杖を細身に、LED をカード状に。

| モジュール | サイズ | 用途 |
|-----------|-------|------|
| Raytac MDBT50Q (nRF52840) | 10×10mm | 杖・LED 小型化の第一選択 |
| MDBT53V (nRF5340) | 10×10mm | 高性能 + 省電力 (Matter 対応も) |
| ESP32-C3/H2-MINI-1 | 13×16mm | ESP32 路線 |

**タスク**:
1. 回路設計 (MCU + IMU + SX1262 + 電源 + LED ドライバ)
2. 電池選定 (薄型 LiPo / LIR2032 / 18650)、ショーケース形状に合わせる
3. アンテナ最適化で TX power を下げ電池持ち改善
4. BLE Adv + LoRa プロトコルは Phase 1-3 と完全互換 → FW 流用

---

## 軌跡認識 (Phase 2-4 と並行、本命機能)

最終目標の HP 呪文軌跡認識。現状は単純な上下/突き/流しのフリック判定だが:

| 段階 | 内容 |
|------|------|
| 現状 | 重力基準の方向 + 強度判定 (LUMOS/NOX/INCENDIO/AGUAMENTI/SHAKE) |
| 発展 1 | ジャイロ統合で回転ジェスチャ (円描き=Expecto Patronum 等) |
| 発展 2 | Mahony 姿勢推定 + 加速度積分で 3D 軌跡化 |
| 発展 3 | テンプレートマッチング / 軽量 ML で呪文分類 |
| 発展 4 | 既存 Burst Motion FW (M5C_MPU6886_cpp) の TriggerEngine / 軌跡資産と統合 |

trigger_id 体系 (0x01 基本 / 0x10-0x1F Phase1 / 0x20-0x2F Phase2 / 0x30+ 軌跡) は拡張余地を確保済み。

### 軸割当の設定 (前突き・左右払い、姿勢推定不要)

方向検出を機体・持ち方に依存させないため、軸を符号付きで設定値にする (NVS 保存):

| 設定名 | 例 | 意味 |
|--------|-----|------|
| `tip=` | `+Y` | 杖**先端 (前方向)** の body 軸。前突き (INCENDIO) 検出に使用。M5StickC の杖持ちでは **+Y** |
| `right=` | `+X` | **右方向**の body 軸。左右払い検出に使用 (符号で左右反転を吸収) |
| 上方向 | (重力から自動) | 設定不要。重力ベクトルから算出 |

→ 「右手系/左手系」トグルは作らず、`tip` / `right` の符号で逆持ち・ミラーを吸収。
→ これらは**瞬時の重力 + body 軸射影 (無積分)** なので Yaw ドリフトの影響を受けない。

### Yaw ドリフト対策 (ジャイロ統合・軌跡認識を入れる時の確定要件)

加速度+ジャイロのみ (MPU6886 = 磁気センサ無し) では **Pitch/Roll は重力で無ドリフトだが Yaw は累積誤差**。
ジャイロ積分を使う発展 1-2 では以下を**必須**とする:

1. **ジャイロバイアス校正**: 起動時に静止して角速度オフセットを測定・減算 (静止検出を利用)
2. **ZUPT (Zero-velocity Update)**: 静止検出時に速度・積分をリセットし誤差累積を断つ
3. **ジェスチャ単位リセット**: 呪文の開始/終了で積分をクリア。各ジェスチャは数百ms〜2s と短いので、その間のドリフトは小さく実用域

→ 本案件は**絶対方位ではなく相対ジェスチャ形状**で判定するので、上記 3 つで Yaw ドリフトは管理可能。
→ 絶対方位が必要になったら磁気センサ内蔵 IMU (ICM-20948 / LSM6DSV16X+BMM150 等) へ。

---

## 設定・更新の運用フロー (USB 再書き込みを避ける)

| 操作 | 方法 | USB 要否 |
|------|------|---------|
| 初回 FW 焼き + DEVICE_ID 初期設定 | USB Serial (`id=` / `pins=` / `save`) | 要 (1 回) |
| 運用中の設定変更 | BLE 設定コマンド (Phase 1 で実装) | 不要 |
| 運用中の FW 更新 | BLE OTA DFU (nRF Connect アプリ) | 不要 |

→ ショーケースを開けるのは初回組立時のみ。以降は無線で設定・更新。

---

## Agent チーム (実装体制)

| Agent | 担当 |
|-------|------|
| Main (Claude) | shared/beacon_protocol.h、統合、Phase 判断 |
| 杖 FW | wand_*/ (NimBLE adv、IMU、ジェスチャ) |
| LED FW | led_*/ (Bluefruit scan、LED、config、duty cycle) |
| (Phase 2+) LoRa FW | Wio-SX1262 + RadioLib |
| (Phase 4) HW | カスタム PCB、BOM、電池選定 |
