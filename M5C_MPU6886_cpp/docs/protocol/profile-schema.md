# プロファイル JSON スキーマ

Burst Motion のプロファイル（アクションルール集）のデータ構造。

## 全体構造

```json
{
  "schema_version": 2,
  "id": "sf2-hadoken-v1",
  "game": {
    "id": "sf2",
    "title": "Street Fighter II",
    "platform": ["PC", "Arcade"]
  },
  "profile": {
    "name": "Street Fighter II (P1 右利き)",
    "description": "波動拳/昇竜拳/竜巻旋風脚の 3 大必殺技",
    "author": "uecken",
    "version": "1.0.0",
    "created_at": "2026-04-24",
    "tags": ["fighting", "sf2", "beginner"]
  },
  "hardware": {
    "controllers": ["m5stickc", "m5stickc_plus", "m5atom_s3"],
    "adapter": null,
    "output_paths": ["ble_hid_direct"],
    "button_count_min": 3,
    "required_features": ["ble_hid_keyboard"],
    "target_consoles": ["pc", "android_usb"]
  },
  "calibration_context": {
    "qref_name": "horizontal",
    "qref_quat": [1.0, 0.0, 0.0, 0.0],
    "axis_frame": "standard_m5stickc"
  },
  "rules": [
    { ... アクションルール ... }
  ]
}
```

## ActionRule の構造（統一状態機械モデル）

```json
{
  "id": 1,
  "name": "波動拳",
  "ui_mode": "oneshot",
  "states": [
    {
      "match_condition": {
        "logic_op": "and",
        "posture": {
          "euler": [0, 0, 45],
          "euler_tol": [10, 10, 180],
          "quat": [0.92, 0, 0, 0.38],
          "quat_dot_min": 0.95,
          "judge_by": "euler"
        },
        "accel_abs": { "threshold": 3.0 }
      },
      "min_dwell_ms": 0,
      "max_dwell_ms": 0,
      "on_enter": {
        "action": "fire_macro",
        "keys": ["DOWN", "RIGHT", "DOWN", "RIGHT", "P"],
        "interval_ms": 30
      },
      "on_exit": { "action": "none" }
    }
  ],
  "loop": false,
  "priority": 10,
  "cooldown_ms": 500
}
```

## フィールド詳細

### ui_mode (UI 表示用プリセット)

| 値 | 内部構造 |
|----|---------|
| `oneshot` | 1 状態、`loop:false`、`on_enter:fire_macro/fire_once` |
| `hold_start_only` | 1 状態、`loop:true`、`on_enter:press`、`on_exit:release` |
| `hold_start_end` | 2 状態、`loop:true`、state1 で press、state2 で release |
| `sequence` | N 状態、`loop:false`、最終 state で fire（UI 非表示、将来用） |

### Condition (match_condition)

```json
{
  "logic_op": "and",                   // "and" | "or"、複数トリガーの結合
  "button": {
    "idx": 0,                           // 0-15、0 = 未指定 = 任意
    "state": "pressed"                  // "pressed" | "released" | "any"
  },
  "posture": {
    "euler":      [0, -45, 0],          // 常に保存
    "euler_tol":  [10, 10, 180],
    "quat":       [0.92, 0, -0.38, 0],  // 常に保存
    "quat_dot_min": 0.95,
    "judge_by":   "euler"               // "euler" | "quat"
  },
  "accel_abs": {
    "threshold": 3.0,                   // 合成加速度 [g]
    "comparison": "gte"                 // "gte" | "lte"
  },
  "accel_per_axis": {                   // 任意、詳細モード時
    "x": null,
    "y": null,
    "z": null
  },
  "gyro_abs": {
    "threshold": 100                    // 合成ジャイロ [°/s]
  }
}
```

いずれのキーも省略可。`null` や欠如は「無視」を意味する。

### Action (on_enter / on_exit)

```json
// キー押下
{ "action": "press", "key": "D" }
{ "action": "press", "key": "SHIFT", "modifiers": [] }

// キー解放
{ "action": "release", "key": "D" }

// キー発火（press→release 完結、cooldown 付き）
{ "action": "fire_once", "key": "SPACE", "duration_ms": 30 }

// マクロ発火（複数キー順次送信）
{
  "action": "fire_macro",
  "keys": ["DOWN", "RIGHT", "DOWN", "RIGHT", "P"],
  "interval_ms": 30
}

// マウス
{ "action": "mouse_move", "dx": 10, "dy": 0 }
{ "action": "mouse_click", "button": "left" }

// ゲームパッド (ESP32-S3 + LSM6DSV16X)
{ "action": "gamepad_press", "button": "A" }
{ "action": "gamepad_axis", "axis": "lx", "value": 0.5 }

// コンシューマ (音量、メディア)
{ "action": "consumer", "usage": "VOLUME_UP" }

// 何もしない
{ "action": "none" }
```

## 例: Street Fighter II プロファイル

```json
{
  "schema_version": 2,
  "id": "sf2-uecken-v1",
  "game": {"id":"sf2","title":"Street Fighter II","platform":["PC"]},
  "profile": {"name":"SF2 P1 右利き","author":"uecken","version":"1.0.0"},
  "hardware": {"controllers":["m5stickc"],"output_paths":["ble_hid_direct"]},
  "rules": [
    {
      "id":1, "name":"波動拳", "ui_mode":"oneshot", "priority":10,
      "states":[{
        "match_condition":{
          "posture":{"euler":[0,0,90],"euler_tol":[15,15,180],"judge_by":"euler"},
          "accel_abs":{"threshold":3.0,"comparison":"gte"}
        },
        "on_enter":{"action":"fire_macro","keys":["DOWN","RIGHT","DOWN","RIGHT","P"],"interval_ms":30}
      }],
      "loop":false, "cooldown_ms":500
    },
    {
      "id":2, "name":"前進", "ui_mode":"hold_start_only", "priority":5,
      "states":[{
        "match_condition":{"posture":{"euler":[0,-30,0],"euler_tol":[10,10,180]}},
        "on_enter":{"action":"press","key":"RIGHT"},
        "on_exit":{"action":"release","key":"RIGHT"}
      }],
      "loop":true
    },
    {
      "id":3, "name":"ソニックブーム", "ui_mode":"hold_start_end", "priority":8,
      "states":[
        {
          "match_condition":{"posture":{"euler":[0,30,0],"euler_tol":[10,10,180]}},
          "on_enter":{"action":"none"}
        },
        {
          "match_condition":{"posture":{"euler":[0,-30,0],"euler_tol":[10,10,180]}},
          "on_enter":{"action":"fire_macro","keys":["LEFT","RIGHT","P"],"interval_ms":20}
        }
      ],
      "loop":true
    }
  ]
}
```

## バージョニングポリシー

- `schema_version`: スキーマ破壊的変更時に +1
- 下位互換を維持する拡張は schema_version 据え置き、フィールド追加のみ
- FW は `schema_version` を見て対応版を判定、未対応なら警告

## 関連

- [JSON Lines プロトコル](json-lines.md)
- [アーキテクチャ概要](../architecture/overview.md)
