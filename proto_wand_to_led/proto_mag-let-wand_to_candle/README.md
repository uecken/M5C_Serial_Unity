# proto_mag-let-wand_to_candle

魔法の杖 → ろうそく「光が移る」演出の **ろうそく(受信)側** プロトタイプ。
**XIAO nRF52840 が杖(ADMGLW)の可視光をフォトトランジスタで検出し、ゆらぎ点灯を自己保持** する。

> 📄 **全体の検討資料・部品比較・価格・電池寿命は [../magic_wand_candle_system_gpt.md](../magic_wand_candle_system_gpt.md) を参照。**
> 本フォルダはその「本命=XIAO nRF52840・光学式」を実装に落とした最小プロト。

---

## 演出フロー

1. 部屋の隠し磁石に杖先をタッチ → **杖 ADMGLW が灯る**
2. 灯った杖をろうそくに近づける → ろうそくの**フォトTRが光を検出** → **ゆらぎ点灯(自己保持)**
3. さらに近づけると ろうそく**内蔵の磁石**が 杖 ADMGLW を**トグルOFF** → 杖が消え「光が移った」ように見える
4. ろうそくは点灯を保持。**今は30秒で自動消灯**(本番3分) → System OFF（or リードSWで手動リセット）。起動時/sleep中は LED OFF

> ⚠ センサを「手前」・磁石を「奥」に空間配置すると「先にろうそく点灯→次に杖消灯」の順になる。
> ⚠ ろうそく内蔵の常設磁石は自分のリードSWを誤作動させる → リセットは**自動消灯タイマ**を基本に。

---

## なぜ MCU なのか — 「フォトTR直列LED」が失敗する理由

実測で「フォトTR と LED を直列直結」では杖を **~5mm 以内**に近づけないと点かなかった。
原因は **①増幅が無い（LED電流=光電流で数十µA〜）②照度は距離の逆二乗で減衰 ③3V電源では白色LEDのVfで余裕が無い**。
→ **フォトTRには「検出(D2 の Low/High)」だけさせ、LED は MCU が D0/D1 を HIGH にして電池から駆動** する。
増幅・しきい値・自己保持・ゆらぎ・自動消灯を **全部ソフトで** 実現でき、アナログのコンパレータ/D-FF/MOSFETラッチが不要。

---

## 回路（最小）

```
 3V3 ──[470kΩ プルアップ(外付)]── D2 ──[フォトTR C–E]── GND
        暗=High / 光でフォトTR導通→ D2=Low      ← 「光 = Low」で検出・wake する
        (感度不足なら アンプ内蔵フォトIC S9648(IRカット)/S7183 に差し替え)

  D0 ┐
  D1 ┴─[各 直列R ~100〜220Ω]── 電球色LED ── GND          ← 点灯(光/wake)時に D0・D1 を HIGH で駆動(2ピン)
                                                          起動時/sleep中は OFF / USE_FLICKER=1 でPWMゆらぎ

  D3 ──── リードSW ──── GND   (INPUT_PULLUP, 任意=手動リセット)

 電源: LIR2032H を XIAO の BAT+/BAT- へ (XIAO が充電も兼ねる, 週次充電運用に好適)
       ※CR2032(非充電)は充電回路付き BAT に繋がない。バラ運用/カスタムPCB時のみ別途給電
```

| 機能 | ピン | nRF | 備考 |
|---|---|---|---|
| 光センサ(検出/wake) | **D2** | P0.28 | 外付け470kΩプルアップ。**光=Low**。GPIO SENSE=Low で System OFF から wake |
| LED駆動 | **D0 / D1** | P0.02 / P0.03 | **点灯時 HIGH**(2ピンでソース能力UP)・**起動時/sleep中は OFF**。USE_FLICKER=1 でPWM |
| リードSW(任意) | D3 | P0.29 | 手動リセット |

---

## ファームウェア (`src/main.cpp`)

2モードを `#define LOWPOWER_SYSTEMOFF` で切替:

| モード | 値 | 動作 | 待機消費 | 用途 |
|---|---|---|---|---|
| **簡易・デモ** | `0` | loop() で D2 をポーリング → 検出でゆらぎ点灯 | 大(数mA) | **手元の XIAO で最速に動かす** (USB給電/動作確認) |
| **低消費・本番** | `1` | **D2(Low) を GPIO SENSE で wake 源**に → **System OFF** | **~1.5µA** | コイン電池で長期(≥1週間) |

- **検出**: D2 の **Low(=光)** が `CONFIRM_MS`(既定100ms) 継続 → 「杖の光」と確定（チラ点灯誤検出を防ぐ）
- **点灯**: **D0/D1 を HIGH** で駆動（既定 solid HIGH、`USE_FLICKER=1` で PWMゆらぎ平均~0.5mA）。`AUTO_OFF_MS`(**今は30秒**/本番3分) で自動消灯 → System OFF。**起動時・sleep中は D0/D1 OFF**
- **本番モード**: nRF52 の System OFF 復帰は「**リセット**」なので、wake 時に `setup()` が再実行され、
  `RESETREAS.OFF`(GPIO wake) を判定 → D2再確認 → ショー → 再び `nrf_gpio_cfg_sense_input(D2, SENSE=LOW)` 武装して System OFF

> ⚠ **wake 極性**: 外付け **470kΩ プルアップ**前提 → **暗=High / 光=Low → SENSE=LOW で wake**。
> ⚠ 感度は 470kΩ とフォトTR で決まる。外乱光が問題なら値調整 or コンパレータ(MCP6541)追加 or S9648/S7183 へ。
> ⚠ System OFF 経路は **未実機検証**。まず `LOWPOWER_SYSTEMOFF 0` で挙動確認 → 後で本番モードへ。
> ⚠ BLE(Bluefruit) を足す場合、System OFF は `NRF_POWER->SYSTEMOFF` でなく `sd_power_system_off()` を使う。

### ビルド / 書き込み (PlatformIO)
```
# 簡易・デモモード (LOWPOWER_SYSTEMOFF=0, ADCポーリング) — まずこれで動作確認
pio run -e seeed_xiao_nrf52840 -t upload          # XIAO を BOOT 2回押しで UF2/bootloader に
pio device monitor -b 115200                      # ADC値・状態ログでしきい値キャリブレーション

# 低消費・本番モード (LOWPOWER_SYSTEMOFF=1, D2 GPIO-sense Low→System OFF wake)
pio run -e seeed_xiao_nrf52840_lowpower -t upload # build_flags で mode 1 を有効化
```
> 検証済み(2026-05-24): 両 env `pio run` 成功＋**XIAO へ書込み確認済**。今は点灯30秒→System OFF（本番は `AUTO_OFF_MS` を3分へ）。実機 wake/しきい値は 470k 配線後に要キャリブレーション。
（既存 [../led_xiao_nrf52840/](../led_xiao_nrf52840/) と同じ board/lib 構成。BLE で複数ろうそく連動させる場合は
そちらの Bluefruit コードが土台に流用可。）

---

## nRF52 を使わない方式（資料の比較より）

コスト/手元環境に応じて選べる。詳細・価格・電池収支は [../magic_wand_candle_system_gpt.md](../magic_wand_candle_system_gpt.md)。

| 方式 | 構成 | 1台概算 | 向き/注意 |
|---|---|---|---|
| **T2 無MCU・堅牢** | フォトTR → コンパレータ(LM393/MCP6541, ヒステリシス) → D-FF(TC4013)自己保持 → Pch MOSFET(AO3401A) → ゆらぎLED | ~¥400〜600 | 書込環境不要。LED連続点灯ゆえ「店時間だけ点灯＋夜off」運用で CR2032 1週間 |
| **T1 無MCU・最安** | フォトTR → MOSFET直接セルフラッチ → ゆらぎLED | ~¥300〜450 | 最安だがしきい値が曖昧→明るい店舗で誤点灯しやすい |
| **磁気式(完成品)** | btoshop **MMGLFL** 磁気スイッチ付ゆらぎLED(¥660, ラッチ式) を流用 | ¥660〜 | 「光で移る」感は弱いが実績豊富・即入手。保険 |

> 「光トリガで自己保持する LED 完成品」は事実上存在しない（光控モジュールは"暗→ON"の逆動作で保持もしない）。
> → 光学式は自作が前提。磁気式なら完成品/脱出ゲーム用プロップが豊富。

---

## センサ候補（要点）

| センサ | アンプ | 特徴 | 評価 |
|---|---|---|---|
| TEPT4400 / NJL7502L | なし | フォトTR。**直結NG**・要増幅(=MCUのADCで受ける) | ○ |
| **S9648 / S9648-100** | 内蔵 | 視感度・**IRカット内蔵で外乱光に強い**・電流アンプ内蔵 | **◎本命** |
| S7183 | 内蔵(×1300) | 光電流を大増幅。小受光面で距離を稼げる(650nm) | ◎ |

---

## ステータス / TODO
- [ ] 簡易モードで点灯デモ → 感度(470kΩ)/`CONFIRM_MS` を実機キャリブレーション（D2 の Low しきい確認）
- [ ] 室内灯ON下での誤点灯確認（適応しきい値の調整）
- [ ] S9648/S7183 を入れて検出距離・外乱光耐性を比較
- [ ] 本番モード(D2 GPIO-sense + System OFF)の実機検証・µA計測（wake極性=Low）
- [ ] LIR2032H 実装＋連続点灯持続時間の実測（目標 ≥1週間）
- [ ] (発展) BLE で複数ろうそく連動・呪文連動（`../led_xiao_nrf52840/` 流用）

> 本コードはプロトタイプ。価格・型番は発注前に再確認のこと。
