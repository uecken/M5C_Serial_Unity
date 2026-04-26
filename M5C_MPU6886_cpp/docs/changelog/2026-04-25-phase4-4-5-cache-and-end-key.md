# 2026-04-25 Phase 4.4 + 4.5: Cache busting + HOLD_START_END 別キー

## Phase 4.4: Web キャッシュ無効化 + version 自動付与

### 背景
Web/hidconfig を変更してデプロイしても、Chrome がキャッシュ済みファイルを返して
新版が即座に反映されない問題があった (GitHub Pages の `Cache-Control: max-age=600`)。
ユーザーがブラウザで動作確認できるよう、デプロイ時に必ず新 URL になる仕組みが必要。

### 実装

#### `index.html`
- no-cache メタタグ 3 種を追加 (Cache-Control / Pragma / Expires)
- `<meta name="app-version" content="__BUILD_VERSION__">` を追加
- `<script type="module" src="./src/app.js?v=__BUILD_VERSION__">` プレースホルダ化

#### `deploy_ghpages.py` の `apply_cache_busting()`
1. デプロイ実行時に `datetime.now().strftime('%Y%m%d-%H%M%S')` で VERSION 生成
2. `index.html` の `__BUILD_VERSION__` を VERSION に置換
3. `src/**/*.js` の相対 import (`from './foo.js'`) に `?v=VERSION` を正規表現で自動付与
   - 既に `?v=...` が付いていたら新バージョンに置換
4. `version.json` を生成 (`{"version": "...", "deployed_at": "..."}`)

#### Web UI フッター
- 起動時に `<meta name="app-version">` を読み込み (即時表示)
- `fetch('./version.json?nocache=...')` で詳細を取得 (deployed_at)
- フッター右に `v20260425-213454 (2026-04-25)` 形式で表示

### 効果
- ブラウザのキャッシュが効かなくなり、デプロイ後の新版が即座に反映される
- ユーザーは `Ctrl+F5` 等の強制リロード不要
- バージョン番号が UI に表示されるので、現在見ているコードがいつのものか即判定できる

## Phase 4.5: HOLD_START_END で開始/終了に別キー指定

### 背景
旧計画書の残課題:
> HOLD_START_END で開始/終了で別キー (現状同じキーの press/release のみ)

例えば「右傾けで Shift 押下 → 左傾けで Shift 解除と同時に Beep キーを発火」のような、
**終了姿勢時に追加で別キーを発火**するシナリオに対応する。

### 実装方針: 状態機械の 3 状態化

types.hpp の構造変更なし。`states[]` を 3 つ使うだけで実現:

| state | match_condition | on_enter Action |
|-------|-----------------|-----------------|
| `s0` | 開始姿勢 | `AT_PRESS` start key |
| `s1` | 終了姿勢 | `AT_RELEASE` start key |
| `s2` | **空 (常時 true)** = 即遷移 | `AT_FIRE_ONCE` end key |

`loop=true` なので s2 → s0 で開始姿勢戻り待機、循環。

s2.match_condition を全 disabled にすると `matchCondition()` が常に true を返し、
**s1 → s2 は次 tick (10ms 後) に即自動遷移** → end key が瞬時に発火される。

### FW 変更 (`src/main_v2.cpp` の rule.add)
- `r["end_key"]` が指定された場合のみ `states_count = 3` に拡張
- s2.on_enter に end_modifiers + end_key を `AT_FIRE_ONCE` で構築
- `r["end_duration_ms"]` (default 30ms) で press → release 間隔を制御
- end_posture にも quat 保存 (Closest 計算 / 検査用)

### コード抜粋
```cpp
const char* end_key = r["end_key"] | "";
if (end_key[0] && rule.states_count < 4) {
    rule.states_count = 3;
    State& s2 = rule.states[2];
    s2.match_condition.logic_op = LogicOp::OP_AND;
    // posture/button/accel/gyro 全部 disabled = 常時 true
    uint8_t end_mods = r["end_modifiers"] | 0;
    uint8_t kc = 0;
    if (end_mods & 0x01) s2.on_enter.keys[kc++] = 0x80;  // KEY_LEFT_CTRL
    if (end_mods & 0x02) s2.on_enter.keys[kc++] = 0x81;  // KEY_LEFT_SHIFT
    if (end_mods & 0x04) s2.on_enter.keys[kc++] = 0x82;
    if (end_mods & 0x08) s2.on_enter.keys[kc++] = 0x83;
    s2.on_enter.keys[kc++] = (uint8_t)end_key[0];
    s2.on_enter.keys_len = kc;
    s2.on_enter.modifiers = end_mods;
    s2.on_enter.type = ActionType::AT_FIRE_ONCE;
    s2.on_enter.duration_ms = r["end_duration_ms"] | 30;
}
```

### Web 変更 (`src/app.js`)
- `endKey`, `endModCtrl`, `endModShift`, `endModAlt`, `endModGui` state 追加
- `handleAddRule()` で `end_key` / `end_modifiers` / `end_duration_ms` を payload に含める
- HOLD_START_END 選択時のみ「+ 終了姿勢時に追加発火するキー (任意)」ブロックを表示
- 出力 HID キーブロックの見出しも HOLD_START_END 時は説明追加

### 動作シーケンス例
```
[初期状態 idle]
ユーザー: 右傾け     → s0 マッチ → AT_PRESS 'a' (a を押し続ける)
ユーザー: 左傾け     → s1 マッチ → AT_RELEASE 'a' (a を離す)
                    → 次 tick で s2 マッチ (常時 true) → AT_FIRE_ONCE 'b' (b を 30ms フラッシュ)
                    → s2 滞在 (s0 戻り待ち)
ユーザー: 右傾け復帰 → s2 → s0 遷移 → AT_PRESS 'a' (繰り返し)
```

### ビルド結果
```
RAM:   11.9% (39136 / 327680 bytes)        変化なし
Flash: 54.5% (713913 / 1310720 bytes)      +524 B
```

## 公開
https://uecken.github.io/M5C_Serial_Unity/  
v20260425-213828 で確認可。

## ファイル変更
- 改訂: `Web/hidconfig/index.html` (no-cache メタ + version プレースホルダ)
- 改訂: `Web/hidconfig/deploy_ghpages.py` (apply_cache_busting 追加、`?v=X` 自動付与)
- 改訂: `Web/hidconfig/src/app.js` (version 表示 + endKey state + UI ブロック)
- 改訂: `src/main_v2.cpp` (rule.add に end_key/end_modifiers/end_duration_ms 受け取り)
- 新規: `docs/changelog/2026-04-25-phase4-4-5-cache-and-end-key.md`
