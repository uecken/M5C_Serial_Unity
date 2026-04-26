# JSON Lines プロトコル仕様

Burst Motion Controller ↔ Web アプリ間の通信プロトコル。
USB Serial (115200) または BLE NUS 経由。

## 基本ルール

- **1 行 = 1 JSON オブジェクト**、改行 (`\n`) 区切り
- UTF-8 エンコード
- Web → FW: `"cmd"` フィールド必須
- FW → Web: `"type"` フィールド必須
- 未知のコマンドは `{"type":"err","cmd":"unknown","err":"unknown_cmd"}` 応答

## コマンド一覧（Web → FW）

### 接続確認
```json
{"cmd":"ping"}
→ {"type":"pong","fw":"1.0.0","board":"m5stickc","uptime":12345}
```

### プロファイル管理
```json
{"cmd":"profile.load", "name":"sf2"}
→ {"type":"ack","cmd":"profile.load","ok":true}

{"cmd":"profile.save", "name":"my-profile", "data":{...}}
→ {"type":"ack","cmd":"profile.save","ok":true}

{"cmd":"profile.list"}
→ {"type":"profile.list","names":["default","sf2","fps","accessibility"]}

{"cmd":"profile.delete", "name":"old"}
→ {"type":"ack","cmd":"profile.delete","ok":true}

{"cmd":"profile.active", "name":"sf2"}
→ {"type":"ack","cmd":"profile.active","ok":true}
```

### アクションルール管理
```json
{"cmd":"rule.add", "rule":{...}}
→ {"type":"ack","cmd":"rule.add","ok":true,"id":42}

{"cmd":"rule.list"}
→ {"type":"rule.list","rules":[{...},{...}]}

{"cmd":"rule.remove", "id":42}
→ {"type":"ack","cmd":"rule.remove","ok":true}

{"cmd":"rule.clear"}
→ {"type":"ack","cmd":"rule.clear","ok":true}
```

### 出力切替
```json
{"cmd":"output.set", "target":"ble"}
→ {"type":"ack","cmd":"output.set","ok":true}

// target: "ble" | "usb" | "both" | "none"
// usb は ESP32-S3 のみ、M5StickC では err 応答
```

### センサーデータストリーム
```json
{"cmd":"sensor.stream", "rate_hz":50}
→ {"type":"ack","cmd":"sensor.stream","ok":true}
→ {"type":"sensor","t":12345,"ax":0.1,"ay":0.2,"az":0.98,"gx":0,"gy":0,"gz":0,"pitch":1,"roll":2,"yaw":3,"qw":1,"qx":0,"qy":0,"qz":0} 継続

{"cmd":"sensor.stream", "rate_hz":0}  // 停止
→ {"type":"ack","cmd":"sensor.stream","ok":true}
```

### キャリブレーション
```json
{"cmd":"calibrate.simple", "duration_ms":10000}
→ {"type":"ack","cmd":"calibrate.simple","ok":true}
→ {"type":"calibration.progress","done":0.5} 継続
→ {"type":"calibration.done","accel_bias":[...],"gyro_bias":[...]}

{"cmd":"calibrate.full.start"}
→ {"type":"calibration.step","step":1,"instruction":"LCD 面を上に向けて静置"}

{"cmd":"calibrate.full.capture", "step":1}
→ {"type":"calibration.step","step":2,"instruction":"LCD 面を下に向けて静置"}
...
```

### q_ref 管理
```json
{"cmd":"qref.set", "name":"horizontal", "source":"current"}
→ {"type":"ack","ok":true}

{"cmd":"qref.apply", "name":"horizontal"}
→ {"type":"ack","ok":true}

{"cmd":"qref.list"}
→ {"type":"qref.list","presets":[{"name":"horizontal","quat":[...]},...]}
```

### 出力テスト (トリガー評価無視して直接出力)
```json
{"cmd":"test.hid", "type":"keyboard", "keys":["A"]}
{"cmd":"test.hid", "type":"mouse", "dx":10, "dy":0}
{"cmd":"test.hid", "type":"gamepad", "buttons":1}
```

### 監視・デバッグ
```json
{"cmd":"watch.set", "enabled":true}
→ トリガー一致時に trigger.hit イベントを受信

{"cmd":"device.info"}
→ {"type":"device.info","fw":"1.0.0","board":"m5stickc","imu":"mpu6886","ble_mac":"...","battery":85}
```

### システム
```json
{"cmd":"reboot"}
→ {"type":"ack","ok":true}  (送信後すぐ再起動)

{"cmd":"factory_reset"}
→ 要確認: {"cmd":"factory_reset","confirm":"yes"}
→ {"type":"ack","ok":true}
```

## イベント（FW → Web、能動送信）

### センサーデータ
```json
{"type":"sensor","t":12345,"ax":0.1,"ay":0.2,"az":0.98,"gx":0,"gy":0,"gz":0,"pitch":1,"roll":2,"yaw":3,"qw":1,"qx":0,"qy":0,"qz":0}
```
- `t`: millis() 以降の経過 ms
- `ax/ay/az`: 加速度 [m/s²]
- `gx/gy/gz`: ジャイロ [°/s]
- `pitch/roll/yaw`: Euler [°]
- `qw/qx/qy/qz`: Quaternion (user frame)

### トリガー発火通知 (watch 有効時)
```json
{"type":"trigger.hit","t":12346,"id":42,"rule_name":"波動拳","phase":"start","action":"fire"}
{"type":"trigger.release","t":12400,"id":42,"phase":"end"}
```

### ステータス (10 秒毎、周期送信)
```json
{"type":"status","uptime":12345,"ble":true,"usb":false,"output":"ble","profile":"sf2","battery":85,"fw":"1.0.0"}
```

### ログ / エラー
```json
{"type":"log","level":"info","msg":"BLE connected to Windows PC"}
{"type":"err","cmd":"profile.load","err":"not_found","detail":"no profile named 'xxx'"}
```

## 接続フロー

```
1. ユーザーが Web UI で Connect ボタン押下
2. navigator.serial.requestPort() で COM 選択
3. port.open({baudRate: 115200})
4. {"cmd":"ping"} を送信
5. FW が {"type":"pong",...} で応答 → 接続確立
6. {"cmd":"device.info"} で能力問い合わせ
7. {"cmd":"sensor.stream","rate_hz":50} で可視化開始
```

## エラーハンドリング

- **応答タイムアウト**: 3 秒、リトライ 3 回まで
- **JSON パースエラー**: `{"type":"err","err":"parse_error"}` で通知、以降の行は正常処理
- **行長制限**: 2048 文字（ArduinoJson バッファ）、超過時は `err":"line_too_long"`
- **FW 未対応コマンド**: `{"type":"err","cmd":"xxx","err":"unknown_cmd"}`

## BLE NUS 経由の場合

- Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` (Nordic UART Service 互換)
- TX Characteristic: `6E400003-...`
- RX Characteristic: `6E400002-...`
- MTU 23 で分割される場合あり、受信側で `\n` までバッファリング

## 関連

- [プロファイル JSON スキーマ](profile-schema.md)
- [アーキテクチャ概要](../architecture/overview.md)
