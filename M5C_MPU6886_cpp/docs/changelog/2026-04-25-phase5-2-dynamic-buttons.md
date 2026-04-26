# 2026-04-25 Phase 5.2: ボタン GPIO 動的構成 (Serial 経由設定 + NVS 永続化)

## 背景
ユーザー指摘:
> もともと G26, G36, G0 を利用していた。シリアル経由でもボタンごとの GPIO を設定で追記可能にしたほうがよいか?

## 調査結果
レガシー `MotionController.hpp:185` で **3 ボタン (G0/G36/G26)** を使用していた:

```cpp
uint8_t button[3] = {0, 36, 26};
```

各 GPIO の **active_low/high と pull mode が異なる** (重要):
- **GPIO 0**: `INPUT_PULLUP`、active_low (BOOT モード兼用、SW で GND 落とし)
- **GPIO 36**: `INPUT` (input only ピン、内部 PU 不可)、active_high (3.3V 入力で押下判定)
- **GPIO 26**: `INPUT_PULLUP`、active_high (半 PD 状態、3.3V ショートで判定)

Phase 5.1 で私がハードコードした GPIO 37/39 はレガシー資産と非互換だった → 修正必須。

## 設計判断: シリアル経由設定可能化 (YES)

理由:
1. 各 GPIO で active_low/high が機種依存、ハードコードは非汎用
2. 量産 HW (Motion Burst)、M5Atom S3 等で全く違う GPIO になる
3. `hardware_profiles.json` (Web 側) と FW 側のずれを実行時に揃えられる
4. NVS 永続化で起動時に復元 → FW 焼き直し不要
5. 将来 MCP23017 等 I2C エクスパンダ対応への布石になる

## 実装

### 1. ButtonsGpio.hpp 動的構成化
- `ButtonConfig { gpio, active_low, pull_mode }` を `std::vector` で保持
- `pull_mode`: 0=INPUT, 1=PULLUP, 2=PULLDOWN
- GPIO 34/35/36/39 (RTC input only) は内部 PU 不可なので強制 INPUT
- `defaultM5StickC()` でレガシー 3 ボタン構成を返す
- `begin(fallback)`: NVS Preferences から構成ロード、なければ fallback
- `setConfig(cfg)`: 構成変更 + NVS 書込み (atomic、Preferences 経由)
- `clearNvs()`: factory reset 用
- 5ms チャタリング除去は維持

### 2. main_v2.cpp 新コマンド (3 種)

#### `hw.buttons.get`
```jsonc
→ {"cmd":"hw.buttons.get"}
← {"type":"hw.buttons", "buttons":[
    {"idx":1, "gpio":0,  "active_low":true,  "pull_mode":1},
    {"idx":2, "gpio":36, "active_low":false, "pull_mode":0},
    {"idx":3, "gpio":26, "active_low":false, "pull_mode":1}
   ], "bitmap":0}
```

#### `hw.buttons.set` (NVS 永続化)
```jsonc
→ {"cmd":"hw.buttons.set", "buttons":[
    {"gpio":0,  "active_low":true,  "pull_mode":1},
    {"gpio":36, "active_low":false, "pull_mode":0},
    {"gpio":26, "active_low":false, "pull_mode":1}
   ]}
← {"type":"ack", "cmd":"hw.buttons.set", "ok":true, "count":3}
```

#### `hw.buttons.reset` (デフォルト復帰)
```jsonc
→ {"cmd":"hw.buttons.reset"}
← {"type":"ack", "cmd":"hw.buttons.reset", "ok":true, "count":3}
```

### 3. setup() でレガシーデフォルト適用
```cpp
g_buttons.begin(ButtonsGpio::defaultM5StickC());
```
NVS に保存済みがあればそれを優先、なければ G0/G36/G26 のデフォルト。

### 4. hardware_profiles.json レガシー反映
- M5StickC / Plus / Plus2 すべて 3 ボタン (G0/G36/G26) に統一
- 各エントリに `active_low`, `pull_mode`, `location` (詳細説明) を追加
- 例: `Btn1 (G0)` 「GPIO 0 (BOOT 兼用、SW で GND 落とし)」

### 5. Web UI 機能追加

#### Hardware セクション拡張
- ドロップダウン下に **ボタン構成テーブル**: idx / 名前 / GPIO / 論理 (active_low/high) / PU / 場所
- **「📤 FW に適用 (hw.buttons.set + NVS 保存)」ボタン**
- **同期状態バッジ**: 「✓ FW 構成と一致」「⚠ FW 構成と異なる (適用が必要)」「FW 構成 未取得」
- **FW 現在構成の表示**: `idx1=G0(L,PU) / idx2=G36(H,-) / idx3=G26(H,PU)` 形式

#### 接続時自動取得
- `connected` 変化時に `hw.buttons.get` を自動送信 (800ms 遅延)
- `type:hw.buttons` 受信ハンドラで `fwButtons` state 更新
- 接続中デバイスの構成と Web 側選択 hardware の一致状況を表示

### 各 GPIO の物理的事情まとめ

| 名前 | GPIO | 内部 PU 可能? | active_low | pull_mode | 押下検出方法 |
|------|------|--------------|------------|-----------|-------------|
| Btn1 (G0)  | 0   | ✓ | true  | PU (1) | SW で GND に落とす |
| Btn2 (G36) | 36  | ✗ (input only) | false | INPUT (0) | 3.3V を入力 |
| Btn3 (G26) | 26  | ✓ | false | PU (1) | 3.3V でショート (半 PD) |

## 動作フロー (典型ケース)

1. ユーザーが新 FW を焼き込み → `g_buttons.begin(default)` でレガシー G0/G36/G26 が即動作
2. Web UI 接続 → 自動で `hw.buttons.get` → 「✓ FW 構成と一致」表示
3. ユーザーが Hardware ドロップダウンを変更 (例: M5Atom S3) → 「⚠ FW 構成と異なる (適用が必要)」
4. 「📤 FW に適用」ボタン → `hw.buttons.set` 送信 → FW が NVS に保存 → 再起動なしで即反映
5. 次回起動時、NVS から自動ロード

## ビルド結果
```
RAM:   12.0% (39320 / 327680 bytes)        +168 B (Preferences + std::vector)
Flash: 54.6% (716329 / 1310720 bytes)      +1176 B
```

## 公開
https://uecken.github.io/M5C_Serial_Unity/  
v20260425-222851

## 残課題
- Web UI にボタン構成の**直接編集機能** (現状は hardware_profiles.json 編集後 PR が必要)。`hw.buttons.set` 経由で個別 GPIO 編集 UI を追加すれば完結
- 6 点キャリブ NVS 永続化 (Phase 5 残課題)
- ストリートファイター サンプルの button_idx は G0 の Btn1 を指す → ユーザーが Btn3 (G26) を使いたければサンプル編集 or 別サンプル追加
- ButtonsGpio が ESP32 系のみ。nRF52840 移植時は `hal/nrf52/` 系に同等実装が必要 (Phase 6+)

## ファイル変更
- 大幅改訂: `src/hal/esp32/ButtonsGpio.hpp` (動的構成、Preferences 永続化、defaultM5StickC)
- 改訂: `src/main_v2.cpp` (`hw.buttons.{get,set,reset}` コマンド追加、setup() で fallback 渡し)
- 改訂: `Web/hidconfig/hardware_profiles.json` (M5StickC/Plus/Plus2 を G0/G36/G26 に修正)
- 改訂: `Web/hidconfig/src/app.js` (Hardware セクション拡張: 構成テーブル + FW 適用ボタン + 同期状態バッジ)
- 新規: `docs/changelog/2026-04-25-phase5-2-dynamic-buttons.md`
