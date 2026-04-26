# 2026-04-25 Phase 5.1: Hardware 別ボタン定義 + GPIO 読取り

## 背景
ユーザー指摘 (2 件):
1. **アクションルールのトリガー条件にボタンを含めているが、Web UI に登録欄がない**
2. **Clear All 押下後、Web GUI から即時に消えない**

ユーザー要望:
> 利用するボタンは、利用するハードウェアに依存するので、Hardware 選択時に、使えるボタン番号 (と GPIO) を提示して、Web GUI で選択可能にすること

## 調査結果
- 新 FW v2 (`main_v2.cpp`) では `g_sensor_state.buttons_bitmap` への代入が**一切なく**、常に 0 のまま
- → `rule.add` で `button_idx` を指定しても TriggerEngine は常に「押下されていない」と判定
- → ボタン条件付きルールは動作不能だった
- 旧 FW (`IMU_BLEorSerial_tester.cpp`) は `M5.BtnA` / `M5.BtnB` を使うが、新 FW は M5 ライブラリ非依存方針
- M5StickC のボタン: BtnA=GPIO 37 (LCD 下、M5 ロゴ)、BtnB=GPIO 39 (右側面)、いずれも active low + 外付け 10kΩ pull-up

## 実装

### 1. FW: ButtonsGpio HAL 新規 (`src/hal/esp32/ButtonsGpio.hpp`)
- GPIO 37/39 を `pinMode(INPUT)` (RTC ピンは内部 PU 不可、外付け PU で十分)
- 5ms チャタリング除去付き `update(now_ms)` → bitmap (bit0=BtnA, bit1=BtnB) を返す
- `main_v2.cpp` の `updateSensor()` 内で 100Hz で更新
- sensor stream に `"btn"` フィールド追加 (Web UI で押下状態が見える)

### 2. FW: rule.add の button_state 反映
- 従来 `state` は固定 0 (pressed) だったが、`r["button_state"]` から読むよう修正
- 0=押下中、1=解放中、2=どちらでも (TriggerEngine が既に対応済み)
- `button_idx` 範囲チェック (1-15) も追加

### 3. Web: hardware_profiles.json (5 機種)

| 機種 | MCU | ボタン |
|------|-----|--------|
| M5StickC | ESP32 PICO D4 | BtnA (GPIO 37), BtnB (GPIO 39) |
| M5StickC Plus | ESP32 PICO D4 | + BtnC (GPIO 35、上面、IR LED 兼用) |
| M5StickC Plus2 | ESP32 PICO V3-02 | + BtnC (GPIO 35) |
| M5Atom S3 | ESP32-S3 | Btn (GPIO 41、LCD タッチ風) |
| Motion Burst (自作) | ESP32-S3 | 仮 GPIO 0/1 (PCB 設計中) |

各エントリは `idx`, `name`, `gpio`, `location`, `active_low`, `default_label` を持ち、Web UI が選択肢として表示。

### 4. Web UI: Hardware 選択 + ボタンセクション
- `hardwareDefs` state に `hardware_profiles.json` の内容
- `selectedHardware` state を `localStorage` で永続化
- 接続デバイスの `device.info.board` が来たら自動選択
- 「アクションルール」セクション上部に**Hardware 選択ドロップダウン**追加:
  - 機種一覧 + 利用可能ボタン一覧表示 (`BtnA (idx=1, GPIO 37), BtnB (idx=2, GPIO 39)`)
  - 接続中デバイスと一致なら ✓ 緑、違うなら ⚠ 黄表示
- 姿勢条件の下に**ボタン条件ブロック**追加:
  - チェックボックス「ボタン条件 (任意)」
  - ドロップダウン「ボタン: idx=1 BtnA (GPIO 37, LCD 下 / M5 ロゴ)」
  - ドロップダウン「状態: 押下中 / 解放中 / どちらでも」

### 5. Web UI: Clear All 楽観的更新
- 従来は `rule.clear` 送信 → ack 受信 → `rule.list` 自動送信 → 応答 → setRuleList 反映 の往復で 50-100ms 遅延
- 修正後: クリック直後に `setRuleList([])`, `setRuleReferences([])`, `setClosestRuleIdx(-1)` を即時実行 → UI 即時クリア
- 楽観的更新後、FW へ `rule.clear` 送信 → 通常通り FW 側も同期 → 後続の `rule.list` 応答で確認

### 6. サンプルプロファイル更新
- `street_fighter.json` の各ルールに `"button_idx": 1, "button_state": 0` を追加 (BtnA 押下中限定)
- `index.json` のタイトルを「🥊 ストリートファイター 風 (要 BtnA)」に変更
- `hardware_required: ["m5stickc", "m5stickc_plus", "m5stickc_2"]` フィールド追加 (将来の互換チェック用)

## 検証方法
1. 接続後、device.info で `board: "m5stickc"` が来れば自動で M5StickC 選択
2. アクションルールセクション上部に「⚙ Hardware: M5StickC | 利用可能ボタン: BtnA (idx=1, GPIO 37), BtnB (idx=2, GPIO 39)」表示
3. ルール作成時にボタン条件をチェック → 例「BtnA 押下中 + 右傾け → 'p'」を登録
4. Stream ON で sensor 受信、`"btn":1` が BtnA 押下時に流れる (まだ Web UI には可視化していない、必要なら追加)
5. ストリートファイター サンプルを「適用」→ BtnA を押しながら右に傾けると 'p' が発火

## ビルド結果
```
RAM:   11.9% (39152 / 327680 bytes)        +16 B
Flash: 54.5% (715153 / 1310720 bytes)      +1240 B
```

## 公開
https://uecken.github.io/M5C_Serial_Unity/  
v20260425-221451

## ファイル変更
- 新規: `src/hal/esp32/ButtonsGpio.hpp`
- 新規: `Web/hidconfig/hardware_profiles.json`
- 改訂: `src/main_v2.cpp` (ButtonsGpio 統合 + sensor stream に btn + rule.add の button_state)
- 改訂: `Web/hidconfig/src/app.js` (Hardware 選択 / ボタン条件 UI / Clear All 楽観的更新)
- 改訂: `Web/hidconfig/profiles/street_fighter.json` (button_idx 追加)
- 改訂: `Web/hidconfig/profiles/index.json`
- 新規: `docs/changelog/2026-04-25-phase5-1-hardware-buttons.md`
