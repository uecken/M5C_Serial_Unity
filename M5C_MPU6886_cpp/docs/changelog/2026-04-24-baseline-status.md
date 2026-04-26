# 2026-04-24 ベースライン状況調査

## 実施内容

1. platformio.ini の platform を `espressif32@6.6.0` にピン留め (pioarduino fork 回避)
2. 共通 `[env]` セクションに monitor_speed=921600 設定
3. 既存 FW `pio run -e m5stick-c` のビルド試行

## 発見

### Platform 問題 (解決)
- 当初の `platform = espressif32` (自動で v55+ の pioarduino fork を導入)
- `platform_packages` override で arduino-esp32 v2.0.14 を指定していたが、builder 側が pioarduino フォーマットを要求 → `missing SConscript pioarduino-build.py` エラー
- **解決**: `platform = espressif32@6.6.0` で arduino-esp32 v2.0.14 ネイティブ builder を使用

### 既存 FW コード構文エラー (未解決、スコープ外)
- `src/MotionController.hpp:261` の `void begin() {` 内のスコープが閉じていない
- `src/IMU_BLEorSerial_tester.cpp:1845-1911` 付近で「function-definition is not allowed here」「expected '}' at end of input」エラー連発
- git log `9aabaef cannot compile M5C` の本質的原因
- **対応方針**: 既存 FW 修復は時間コスト大。**Phase 1 新 FW で代替**し、MVP 完了後に必要なら migrate ツールで旧 pk3 データを吸い出す

## M5C Serial 接続状況
- COM8 に M5C 接続確認（ユーザー報告）
- VID:PID = 0403:6001 (FTDI、M5StickC の USB-Serial は FTDI モデルの可能性)
- Baud 115200 で SERIAL,ON / SHOW コマンド送信 → 応答なし
- **推定**: 現在の M5C には未完成 FW が flash されている、または boot loop 状態

## 次のアクション
1. **Phase 1 新 FW スケルトン** (`src/core/` + `src/main_v2.cpp`) を作成
2. `[env:m5stick-c-v2]` で build 検証
3. アップロードして Serial 疎通確認 → これが新しいベースライン
