# 2026-04-25 BLE NUS (Nordic UART Service) 追加

## 背景

ユーザー報告: 「BLE Serial 接続すると failed to open serial port が出る」

### 原因
- 現 FW は **Classic Bluetooth SPP** (BluetoothSerial.h) を持っていない
- Windows BT ペアリング → 仮想 COM 割当て → Web Serial で open は不可
- BleCombo (NimBLE) は **BLE HID 専用**、SPP は対応外

### 解決方針
**BLE NUS (Nordic UART Service)** を FW に追加し、**Web Bluetooth API** で接続する。
これは元計画 Phase 2 の項目で前倒し実装。

## 実装内容

### FW 側
- 新規: `src/transport/BleNusServer.hpp` — NimBLECharacteristicCallbacks 継承
  - Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e` (Nordic 標準)
  - TX: `6e400003-...` (notify、FW→Web)
  - RX: `6e400002-...` (write、Web→FW)
  - JSON Lines プロトコルを USB Serial と完全互換で実装
  - 200 byte chunk で MTU 制約に対応
- `main_v2.cpp` 更新:
  - `g_ble_nus` グローバル追加
  - `ble.start` コマンドで BLE HID + BLE NUS を**同時起動** (NimBLE 単一サーバーに同居)
  - `device.info` 応答に `ble_nus_started` / `ble_nus_connected` 追加
  - sensor stream は USB Serial と BLE NUS 両方に流す
- ビルド成果: RAM 11.7%、Flash 48.1% (NUS 追加で +0.3% / +4KB)

### Web 側
- 新規: `Web/hidconfig/src/lib/BleClient.js` — Web Bluetooth API ラッパ
  - `SerialClient` と同一 API (connect/send/request/event)
  - notification → buffer → JSON 行抽出 → イベント
  - 20 byte chunk で write
- `app.js` 更新:
  - **USB Serial / BLE NUS のタブ切替 UI**
  - 接続中はトランスポート表示チップ
  - Web Serial / Web Bluetooth の対応検出
  - `device.info` パネルに NUS 状態追加

### 検証結果
USB Serial 経由で `ble.start` 発行:
```json
→ {"cmd":"ble.start"}
← {"type":"ack","cmd":"ble.start","ok":true,"ble_hid":true,"ble_nus":true}
← {"type":"device.info","ble_nus_started":true,"ble_nus_connected":false,...}
```
**両方の BLE service が同時稼働**することを確認。

## ユーザーへの使い方

1. USB 接続して Web UI を開く (`https://192.168.0.111:8443/` 等)
2. 「📡 USB Serial」タブで接続 → ping → 「BLE Start」ボタン
3. USB ケーブルを抜いても BLE は継続稼働
4. 別端末/同じ PC の Chrome で `https://...:8443/` を開く
5. 「📶 BLE」タブ → 「🔌 BLE NUS で接続」 → "Burst Motion" を選択
6. 同じ JSON Lines プロトコルで全機能使える

## 注意点

| OS / ブラウザ | Web Serial | Web Bluetooth |
|--------------|-----------|---------------|
| Chrome Desktop (Win/Mac/Linux) | ✅ | ✅ |
| Edge Desktop | ✅ | ✅ |
| Chrome Android 148+ | ✅ (USB OTG) | ✅ |
| Firefox | ❌ | ❌ |
| Safari (iOS/Mac) | ❌ | ❌ |

iPhone は **Web Bluetooth/Web Serial どちらも非対応**、Bluefy 等のサードパーティブラウザを使うか PC/Android を使う。

## 後続タスク

- Phase 2 残: BLE HID 入力テスト (実際にメモ帳に入力できるか)
- Phase 2 残: ActionRule 登録/読込 + LittleFS 永続化
- Phase 2 残: 3D Three.js viewer 統合
- Phase 2 残: キャリブレーションウィザード
