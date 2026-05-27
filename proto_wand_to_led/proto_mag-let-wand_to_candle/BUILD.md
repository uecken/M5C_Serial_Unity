# BUILD — 他PCで git clone → XIAO に書き込む手順

`proto_mag-let-wand_to_candle`（ろうそく受信機 FW）を、別PCでクローンしてビルド・書き込みする手順。
**XIAO nRF52840 / ESP32-C3 / ESP32-C6** に同一コードで対応（`#ifdef` でプラットフォーム分岐）。

> 環境は **PlatformIO**。設計・配線・部品比較は [README.md](README.md) と [../magic_wand_candle_system_gpt.md](../magic_wand_candle_system_gpt.md)。

---

## 1. 前提ソフトのインストール

- **Git**
- **PlatformIO**（どちらか）
  - VS Code 拡張「PlatformIO IDE」（推奨・GUIで Build/Upload/Monitor）、または
  - PlatformIO Core (CLI): `pip install -U platformio` → `pio --version`
- **USB ドライバ**: XIAO は概ね OS 標準 CDC で認識。Windows でポートが出ない時のみベンダドライバ。

> 初回ビルドで PlatformIO が **各ボードの platform / ツールチェーン / ライブラリを自動DL**します（数百MB・要ネット）。
> nRF52840 = Seeed nRF platform + GCC ARM、ESP32 = espressif32 platform + RISC-V toolchain + arduino-esp32。

## 2. クローン

```bash
git clone <このリポジトリ>            # 例: git clone https://github.com/uecken/M5C_Serial_Unity.git
cd M5C_Serial_Unity/proto_wand_to_led/proto_mag-let-wand_to_candle
```

## 3. ビルド対象（env 一覧）

| ボード | env（簡易=USB観測） | env（本番=deep sleep + D2 wake） |
|---|---|---|
| XIAO **nRF52840** | `seeed_xiao_nrf52840` | `seeed_xiao_nrf52840_lowpower` |
| XIAO **ESP32-C3** | `seeed_xiao_esp32c3` | `seeed_xiao_esp32c3_lowpower` |
| XIAO **ESP32-C6** | `seeed_xiao_esp32c6` | `seeed_xiao_esp32c6_lowpower` |

- **簡易**: deep sleep しない。USB 給電のまま `D2`/光/ADC をログ表示 → しきい値（470kプルアップ値）を調整する用。**まずこちらで動作確認推奨**。
- **_lowpower**: `D2=LOW`（光）で deep sleep から wake → 点灯30秒 → 再び deep sleep。**コイン電池運用はこちら**。

## 4. ビルド & 書き込み

```bash
# 例: nRF52840 を簡易モードで
pio run -e seeed_xiao_nrf52840                 # ビルドのみ
pio run -e seeed_xiao_nrf52840 -t upload       # ビルド＋書込み
pio device monitor -b 115200                   # シリアルログ

# ESP32-C3 / C6 も同様に env を差し替えるだけ
pio run -e seeed_xiao_esp32c3 -t upload
pio run -e seeed_xiao_esp32c6_lowpower -t upload
```

VS Code 拡張なら、左下の env を選んで **✓Build / →Upload / 🔌Monitor** ボタン。

### 書き込み時のブートローダ
- **nRF52840**: 通常は自動（pio が 1200bps タッチでリセット）。失敗時は **RESET を素早く2回押し**て UF2 ブートローダ（`XIAO-SENSE` 等のドライブが出る）に入れてから upload。
- **ESP32-C3/C6**: 通常は自動。失敗時は **BOOT を押しながら RESET**（または BOOT 長押し）でダウンロードモードに。

## 5. 動作確認

- 簡易モード: `pio device monitor -b 115200` → `D2=1 light=0 adc=...`（暗）/ 光を当てると `D2=0 light=1` → 点灯ログ。
- 配線: `D2`=フォトTR＋外付け **470kΩ プルアップ**（光=LOW）、`D0`/`D1`=LED（直列R 100〜220Ω）、電源は 3V3 か BAT(LIR2032H)。

---

## 6. トラブルシュート

| 症状 | 対処 |
|---|---|
| `could not open port ... PermissionError`（アクセス拒否） | その COM を開いている**シリアルモニタを閉じて**再 upload |
| ポートが自動検出されない | `pio run -e <env> -t upload --upload-port COMxx` / monitor は `--port COMxx` |
| **C6 で board が見つからない** | `pio pkg update` で espressif32 platform を最新化（C6 は arduino-esp32 3.x / platform 6.6+ 必須） |
| ESP32 install が `idf_tools.py ... MSys/Mingw is not supported` で失敗（↑ toolchain 破損の発端） | **git-bash/MSYS から `pio` を実行しない**（idf_tools が MSYS を弾く）。**PowerShell / cmd / VS Code(PlatformIO)** などネイティブシェルでビルド・インストールする。確実に通したい C3 は `platform = espressif32@6.5.0`（arduino-esp32 2.0.x・toolchain 同梱で idf_tools 不要）にピン留めでも回避可（C6 は 3.x 必須で不可） |
| ESP32 で `'riscv32-esp-elf-g++' は…認識されていません` / `ModuleNotFoundError: No module named 'esptool'` | ESP32 ツールチェーン/esptool の破損・未完インストール。修復: ① `~/.platformio/packages/toolchain-riscv32-esp` を削除 → 再ビルドで再DL ② penv の esptool 再インストール `~/.platformio/penv/Scripts/python -m pip install --force-reinstall esptool` ③ Windows は **開発者モード ON / 長パス有効化**（"Restart required after enabling" = symlink/長パス対策, 一度 PC/PIO 再起動）。**クリーンなPCでは通常そのままビルド可** |
| 書込み後すぐ「消えた」(USB 切れる) | **_lowpower は正常**：30秒後に deep sleep → USB 切断。再書込みは起動30秒の窓内 or ブートローダで |
| deep sleep で wake しない/即wake | `D2` の **470kプルアップ未配線**だとフローティングで不定 → 配線する。極性は**光=LOW**（`ESP_GPIO_WAKEUP_GPIO_LOW` / nRF `SENSE_LOW`） |

## 7. deep sleep wake の注意（ESP32-C3/C6）

- wake 対象ピンが限られる: **C3=GPIO0..5**, **C6=LP_GPIO0..7**。XIAO の **D2 は C3=GPIO4 / C6=GPIO2** で対象内（OK）。別ピンに変える時は要確認。
- wake は **リセット扱い**（nRF の System OFF と同様）→ 復帰時に `setup()` が再実行。`platWokeByLight()` が wake 要因を判定。
- 待機電流の目安（チップ値・基板で増）: **nRF52840 ~1.5µA < C3 ~5µA < C6 ~7µA**。長期コイン電池運用は nRF52840 が有利。
- うまく行かない場合のフォールバック: ESP32 は **light sleep + `gpio_wakeup_enable()`**（RAM保持・実装容易・消費は増）も可。

---

## 関連
- [README.md](README.md) — 回路/動作/方式比較
- [../magic_wand_candle_system_gpt.md](../magic_wand_candle_system_gpt.md) — 設計資料（部品・価格・電池収支・既製品調査）
