# Burst Motion — Web 設定アプリ (GitHub Pages 公開版)

## 公開 URL
- **https://uecken.github.io/M5C_Serial_Unity/**

## 概要
M5StickC + Burst Motion ファームウェア向け Web 設定 UI。
USB Serial / BLE NUS で Controller に接続し、リアルタイムセンサー表示・HID テスト・
アクションルール登録を行う。

## 動作要件
- **Chrome / Edge** Desktop または **Chrome 148+** Android (Web Serial API 対応)
- HTTPS 必須（GitHub Pages は自動で HTTPS）
- Burst Motion FW v2 が書き込まれた M5StickC

## 機能
- 📡 Web Serial 自動再接続（初回手動 → 以降自動）
- 📶 Web Bluetooth (BLE NUS) 接続
- 🎨 Three.js 3D 姿勢ビュー
- 🧪 HID 直接テスト (キー / テキスト / マウス)
- 🎯 簡易アクションルール登録 (加速度 → キー)
- 📊 リアルタイムセンサー (Accel / Gyro / Euler / Quaternion)
- 📜 通信ログ (TX/RX 色分け)

## 使い方
1. このページを Chrome で開く
2. M5C を USB で PC に接続 (Burst Motion FW v2 書込済)
3. 「📡 USB」タブ → 「USB Serial で接続」 → デバイス選択
4. 「BLE Start」 でデバイス内 BLE HID/NUS を開始
5. 「Stream ON」 で IMU データを表示

## ソース
- リポジトリ: https://github.com/uecken/M5C_Serial_Unity
- Web アプリソース (main branch): `M5C_MPU6886_cpp/Web/hidconfig/`
- 本ブランチ (gh-pages) は配信用、main から自動生成

## Build / FW
- ビルド: PlatformIO `pio run -e m5stick-c-v2`
- アップロード: `pio run -e m5stick-c-v2 -t upload`
- baud: 115200

## License
TBD (MVP 期間中)
