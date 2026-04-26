# 2026-04-25 Phase 2.2: 全部入り完成

## 完了した機能 (実機検証済み)

### FW (`m5stick-c-v2`)

#### 起動シーケンス
```
fs_init       → LittleFS マウント (auto-format)
pre_imu_init  → boot 通知
axp_lcd       → AXP192 + LCD 初期化 (axp_ok:true, lcd_ok:true)
post_imu_init → MPU6886 I2C 通信、imu_ok:true
auto_calibrated → 500ms × 50 サンプルで gyro_bias 自動測定
profile_loaded → active_profile があれば自動ロード (rule_count 復元)
```

#### 機能
| 機能 | 動作確認 |
|------|---------|
| **MPU6886 IMU** | 100Hz、Mahony 姿勢推定、Quaternion + Euler 出力 |
| **gyro bias 自動キャリブ** | 起動時 + `calibrate.simple` コマンドで再校正 |
| **BLE HID** (BleCombo + scan response) | "Burst Motion" 名前広告、Windows でペア可 |
| **BLE NUS** (Nordic UART Service) | HID と同居、Web Bluetooth で接続可 |
| **JSON Lines プロトコル** | 全コマンド + イベント |
| **TriggerEngine** | 統一状態機械、ONESHOT/HOLD_*/SEQUENCE |
| **Profile (LittleFS)** | save/load/list/delete + active_profile 起動時自動ロード |
| **AXP192 PMIC** | バッテリ電圧/残量/充電状態 (98%, 4.13V, charging:true 確認済) |
| **M5StickC LCD** (ST7735S 80×160) | BLE/Battery/Rules/Uptime/Euler/Accel/Profile 表示 |
| **HID test コマンド** | press/release/fire/text/mouse_move/mouse_click |
| **rule.add/list/clear** | アクションルール登録 |
| **watch.set** | trigger.hit イベント通知 |

#### 主要コマンド
```jsonc
ping / device.info / sensor.stream / output.set / ble.start
test.hid / calibrate.simple / watch.set
rule.add / rule.list / rule.clear / rule.remove
profile.list / profile.save / profile.load / profile.delete / profile.active
```

### Web UI (`Web/hidconfig/`)

#### 構成 (Preact + htm + Tailwind CDN、no-build)
```
index.html
src/
├── app.js                  # メイン App
└── lib/
    ├── SerialClient.js     # Web Serial + JSON Lines + auto-reconnect
    ├── BleClient.js        # Web Bluetooth + BLE NUS
    └── IMUViewer.js        # Three.js 3D 姿勢ビュー
firmware/
└── m5stickc-v2/
    ├── manifest.json
    ├── bootloader.bin
    ├── partitions.bin
    ├── boot_app0.bin
    └── firmware.bin
```

#### パネル
1. **🔧 デバイス**: ping/info/BLE Start/Calibrate/Stream + バッテリ表示
2. **📊 センサー / 🎨 3D 姿勢**: リアルタイム値 + Three.js M5StickC モデルが回転
3. **🧪 HID 直接テスト**:
   - 送信遅延 (デフォルト 1000ms、メモ帳にフォーカス移動可能)
   - キー / テキスト / マウス 4 方向 / クリック (マウスは即実行)
4. **🎯 アクションルール**:
   - 一覧表示 (id/name/states/loop) + 発火フラッシュ
   - 加速度しきい値 + キーで Add
   - watch トグル
5. **📁 プロファイル (LittleFS)**:
   - 名前付き保存 / Load / Delete / アクティブ表示
6. **⚡ FW 書込み (esp-web-tools)**:
   - ブラウザから直接 ESP32 に FW フラッシュ可能
7. **📜 通信ログ**: TX/RX 色分け

#### 接続方法
- **USB Serial** (auto-reconnect): `navigator.serial.getPorts()` で記憶ポート取得、初回後は自動
- **BLE NUS** (Web Bluetooth): "Burst Motion" 選択
- **VID/PID フィルタ**: FTDI / SiLabs / CH340 / ESP32 native のみ chooser に表示
- **localStorage**: 最後のトランスポート記憶

### GitHub Pages 公開
- **URL**: https://uecken.github.io/M5C_Serial_Unity/
- **ブランチ**: `gh-pages` (orphan)
- **デプロイスクリプト**: `Web/hidconfig/deploy_ghpages.py`
  - Web/hidconfig → gh-pages worktree 同期 → commit → push
  - cert.pem / key.pem / serve_https.py は除外

## メトリクス

### Build (m5stick-c-v2)
- RAM: 11.8% (38616 bytes / 320KB)
- Flash: 54.0% (707KB / 1.31MB)

### 動作確認結果
- ✅ Boot シーケンス全 stage 通過
- ✅ Auto-calibration: gyro_bias [-3.1, -14.3, 0.3] dps 検出
- ✅ Yaw drift 解消: -15°/s → -0.006°/s (実用ゼロ)
- ✅ BLE 名前広告: "Burst Motion" Windows BT 設定で発見可能
- ✅ Profile round trip: rule.add×2 → profile.save → rule.clear → profile.load 完全復元
- ✅ Profile auto-load: 再起動時 test1 から rule_count:2 自動復元
- ✅ Battery: 98%, 4.13V, charging:true 正しく報告
- ✅ LCD: 全項目表示 (BLE/Bat/Rules/Time/Euler/Profile)
- ✅ Web UI 14/14 + E2E mock 8/8 + 実機 11/11 + Full E2E 3/3 全テスト pass
- ✅ GitHub Pages HTTP 200

## 残作業

### 短期
- [ ] **ユーザーが Burst Motion を Windows BT 設定でペアリング**
- [ ] **ペア後、Notepad で test.hid → 'a' が入力されるか確認** (1秒遅延付き)
- [ ] **shake (≥2.5g) → 'a' rule で実際にメモ帳に出力されるか**

### 中期 (Phase 3+)
- [ ] **HOLD_START_END 用の終了条件 UI** (現状 ONESHOT/HOLD_START_ONLY のみ)
- [ ] **複合トリガー** (button + posture の AND 表現 UI)
- [ ] **multiple key + modifiers** (Ctrl+Shift+A 等)
- [ ] **6点フルキャリブレーション** (現状簡易のみ)
- [ ] **DeepSleep + 3g wake-on-motion** (MPU6886 では 1g 上限なので将来 LSM6DSV16X 等で)
- [ ] **電源ボタン処理** (AXP192 PEK 検出 → mode 切替)

### 長期 (Phase 4+)
- [ ] M5Atom S3 / XIAO ESP32-S3 USB HID 対応
- [ ] BLE HID Gamepad 拡張 (BleCombo は Keyboard+Mouse のみ)
- [ ] OTA FW 更新 (BLE / WiFi)
- [ ] Adapter FW (RP2040 W / ESP32-S3 経由 USB HID for Switch/PS4)

## 重要ファイル一覧 (Phase 2.2 で追加・変更)

### FW
- `src/core/types.hpp` ActionRule/Condition/State 型
- `src/core/MahonyFilter.hpp` 姿勢推定
- `src/core/TriggerEngine.{hpp,cpp}` 状態機械 evaluator
- `src/core/Profile.hpp` LittleFS JSON SerDe
- `src/hal/esp32/ImuMpu6886.hpp` MPU6886 直接 I2C
- `src/hal/esp32/Axp192.hpp` AXP192 PMIC
- `src/hal/esp32/M5StickCDisplay.hpp` ST7735S LCD
- `src/hid/{IHidSink,BleHidSink}.hpp` HID 抽象 + BleCombo wrapper
- `src/transport/{SerialJsonLine,BleNusServer}.hpp` JSON Lines transport
- `src/main_v2.cpp` エントリポイント

### Web
- `Web/hidconfig/index.html`
- `Web/hidconfig/src/app.js`
- `Web/hidconfig/src/lib/SerialClient.js`
- `Web/hidconfig/src/lib/BleClient.js`
- `Web/hidconfig/src/lib/IMUViewer.js`
- `Web/hidconfig/serve_https.py` ローカル HTTPS サーバー
- `Web/hidconfig/deploy_ghpages.py` gh-pages 自動デプロイ
- `Web/hidconfig/firmware/m5stickc-v2/{manifest,*.bin}`

### Test
- `test/web/test_web_app.py` UI 描画 (14/14)
- `test/web/test_serial_e2e.py` JSON SerDe (8/8)
- `test/web/test_real_device.py` FW 実機 (11/11)
- `test/web/test_full_e2e.py` 実機 → ブラウザ注入 (3/3)
- `test/web/test_3d_viewer.py` Three.js (canvas + WebGL)
- `test/web/test_auto_reconnect.py` 自動接続 (8/8)
