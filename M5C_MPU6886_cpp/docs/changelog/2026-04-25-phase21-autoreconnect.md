# 2026-04-25 Phase 2.1: 自動再接続 + HID テスト + キャリブレーション

## 背景
ユーザー要望: 「PC は COM8 とわかっていれば自動接続できるように」

## 実装内容

### 1. Web Serial 自動再接続 (`SerialClient.js`)

**標準パターン**: 初回 `requestPort()` でユーザー認可取得、次回以降は `getPorts()` で記憶ポート取得 → ユーザー操作なしで `open()`。

新 API:
- `getAuthorizedPorts()`: 認可済みポート一覧を取得 (chooser 出さない)
- `autoConnect(baudRate)`: 認可済みポートが 1 つでもあれば自動接続。なければ `false`
- `connect(baudRate, useFilter=true)`: VID/PID フィルタ付き chooser 表示
- `forgetAllPorts()`: 全ポート許可解除 (デバッグ用)

**USB VID/PID フィルタ** (M5C 関連の chip のみを chooser に表示):
- `0x0403` FTDI (FT231)
- `0x10C4` SiLabs (CP210x、M5StickC 標準)
- `0x1A86` QinHeng (CH340/CH9102、M5StickC 新型)
- `0x303A` Espressif native (ESP32-S3 内蔵 USB)

### 2. Web UI (`app.js`)

#### 起動時自動接続フロー
```
1. ページ読込
2. localStorage から transport (USB/BLE) 復元
3. localStorage から autoConnect (true/false) 復元
4. transport=USB かつ autoConnect=true なら:
   getPorts() → 認可済みあれば自動接続 + ping
5. なければ「USB Serial で接続」ボタン待機
```

#### 新パネル追加
- **🧪 HID 直接テスト**: Press 'a' / Type "Hello" / Mouse 50,0 / Click
- **🎯 簡易アクションルール**: 加速度しきい値 + キー → Add Rule (ONESHOT/HOLD_START_ONLY)
- **デバイス**パネルに **Calibrate** ボタン追加
- ヘッダ近くに「📌 認可済みポート N 個」表示 + 自動接続トグル + 許可解除リンク

#### localStorage キー
- `burst_motion_transport`: 'usb' | 'ble'
- `burst_motion_autoconnect`: 'true' | 'false'

### 3. FW 新コマンド (`main_v2.cpp`)

| コマンド | 用途 |
|---------|------|
| `test.hid {action,key,text,dx,dy,button}` | HID 直接送信（PRESS/RELEASE/FIRE/TEXT/MOUSE_MOVE/MOUSE_CLICK） |
| `calibrate.simple {duration_ms}` | 1 秒静止 → gyro bias 自動測定 → Mahony reset |
| `rule.add {r:{...}}` | ActionRule 登録（ONESHOT/HOLD_START_ONLY/HOLD_START_END） |
| `rule.list` | 登録済みルール一覧 |
| `rule.clear` | 全削除 |

### 4. 起動時自動キャリブレーション (重要)

`main_v2.cpp::setup()` に追加:
```cpp
// 起動時自動 gyro bias キャリブレーション (静止前提、500ms × 50 サンプル)
if (g_imu_ok) {
    delay(200);
    double sx=0, sy=0, sz=0;
    for (int i = 0; i < 50; i++) {
        float a[3], gv[3];
        if (g_imu.read(a, gv)) { sx+=gv[0]; sy+=gv[1]; sz+=gv[2]; }
        delay(10);
    }
    g_gyro_bias_rad[0] = sx / 50;
    g_gyro_bias_rad[1] = sy / 50;
    g_gyro_bias_rad[2] = sz / 50;
}
```

`updateSensor()` で gyro 読取直後に bias を減算:
```cpp
g_imu.read(accel, gyro_rad);
gyro_rad[0] -= g_gyro_bias_rad[0];
gyro_rad[1] -= g_gyro_bias_rad[1];
gyro_rad[2] -= g_gyro_bias_rad[2];
g_mahony.update(...);
```

## 検証結果

### Yaw drift の劇的改善
| 項目 | Before | After |
|------|--------|-------|
| gyro_y 表示 (静止時) | **-15.0 °/s** | **-0.38 °/s** |
| yaw drift 速度 | 数秒で +173° へ | **-0.006 °/s** (実質ゼロ) |

### コマンド動作確認
すべて成功:
```
✅ test.hid (BLE 未起動) → err: ble_not_started
✅ ble.start → ble_hid:true, ble_nus:true
✅ test.hid (after ble.start) → ack
✅ calibrate.simple → samples:50, gyro_bias_dps:[-3.6, -15.9, 0.2]
✅ rule.add (shake_a, oneshot, accel 2.5g, key 'a') → id:1, rule_count:1
✅ rule.list → [{id:1, name:"shake_a", states_count:1, loop:false}]
```

### Web UI 機能確認 (8/8 PASS)
- ✅ 自動接続トグル
- ✅ USB / BLE タブ
- ✅ HID Press / Type / Mouse 50,0 / Click ボタン
- ✅ Add Rule
- ✅ Calibrate

### Build メトリクス
- RAM 11.7% (38448 bytes)
- Flash 49.2% (645KB) — 新コマンド + 自動キャリブで +6KB

## 残課題

- BLE HID actual key input テスト（PC でペアリング → メモ帳でキー出力確認）
- ActionRule fire の実機検証（shake → BLE HID 'a' 送信）
- 3D Three.js viewer 統合
- LCD ディスプレイ (M5StickC 1.14" LCD) 表示
