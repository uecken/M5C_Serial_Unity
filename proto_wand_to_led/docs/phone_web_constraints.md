# 杖でスマホ Web アプリを動かす — 制約と現実的な構成

杖 (BLE Advertising Beacon) で**スマートフォンの Web アプリ**を反応させたい場合の技術制約をまとめる。
（例: 杖を振る → Lumos/Nox → スマホ画面の光球が点灯/消灯）

検証用デモ: <https://uecken.github.io/M5C_Serial_Unity/wand_receiver/>
（`M5C_MPU6886_cpp/Web/hidconfig/wand_receiver/index.html`）

---

## 結論 (TL;DR)

- **Web アプリが BLE 広告を直接スキャンする方式は、一般顧客には実用にならない。**
  - iPhone: Web Bluetooth 自体が非対応 → 不可
  - Android: `chrome://flags` の手動有効化が必要 → 一般顧客はやらない
- **現状の Web デモは「開発・社内動作確認用」と割り切る。**
- 顧客に配れるのは実質 **① ゲートウェイ方式** と **② ネイティブアプリ** の 2 つ。

---

## 1. 技術の前提: Web で BLE 広告を受信する API

| API | 用途 | 杖用途への適合 |
|-----|------|--------------|
| `navigator.bluetooth.requestLEScan()` + `advertisementreceived` | **広告 (manufacturerData) をパッシブ受信** | ◎ 杖の broadcast にそのまま合致 |
| `navigator.bluetooth.requestDevice()` + GATT 接続 | デバイスに**接続**して読み書き | ✗ 接続型・デバイス選択 UI・1:N broadcast 不可 |

杖は connectionless な BLE Advertising Beacon (1:N broadcast) なので、必要なのは前者 (`requestLEScan`)。
ところがこれが**ブラウザ側の最大の制約**になる。

---

## 2. `requestLEScan` の対応状況（核心の制約）

| 環境 | requestLEScan | 条件 |
|------|--------------|------|
| Android Chrome | △ | `chrome://flags/#enable-experimental-web-platform-features` を**手動で有効化**して再起動 |
| PC Chrome / Edge | △ | 同上フラグ |
| **iOS Safari / iOS Chrome 等** | ✗ | **Web Bluetooth 自体が非対応**（WebKit 未実装、回避不可） |
| Firefox (全 OS) | ✗ | 非対応 |

- `requestLEScan` は **2019 年から実験機能 (experimental flag) のまま**で、Chrome 安定版に正式搭載される見込みが立っていない。
- iOS は OS レベルで Web Bluetooth 非対応。Bluefy 等の WebBLE ラッパーブラウザを使えば動くが、「専用ブラウザを入れてもらう」前提で顧客向けには非現実的。

### Android 特有の追加ハードル（フラグ ON でも詰まる点）
- 「スキャン開始」時に **Bluetooth 権限 + 位置情報 (Location) 権限**の許可が必要。
- **端末の位置情報 (GPS) を ON** にしないと、権限を出しても**スキャン結果がゼロ**になる。
- → 「LED は反応するのにスマホで拾えない」の最頻出原因。

### その他の運用制約（フラグが ON でも残る）
- HTTPS (secure context) 必須。
- スキャン開始は**ユーザー操作 (ボタン) 起点**が必須。
- **フォアグラウンドのみ**。画面 OFF / 他アプリ切替で受信停止。バックグラウンド常駐不可。

### Android のスキャン制限 (実機 adb で確認 / 2026-05-20)
長時間・高頻度の連続スキャンを Android が**自動でペナルティ**にかける。Pixel 6a / Android 16 で確認:
- 症状: しばらく使うと突然**受信ゼロ**になる（「○○件で止まった」ように見える）。件数に上限がある訳ではない。
- 実体 (`adb shell dumpsys bluetooth_manager` / `adb logcat`):
  - `LE scans (started/stopped): 146/145` … リロード多発で**スキャン開始が暴走**
  - `ScanController: Skipping ScanClient(... scanModeUsed=SCAN_MODE_OPPORTUNISTIC ...) for location deny list`
  - Chrome のスキャンが要求 `LOW_LATENCY` → Android が **`OPPORTUNISTIC` に強制ダウングレード** → 他アプリのスキャンに相乗りした分しか受信できず実質ゼロ。
- **受信件数を絞っても無関係**（ペナルティ判定はスキャンの開始回数/継続時間ベース、結果数ではない）。manufacturerData フィルタは負荷軽減にはなるが解除はできない。
- **回復**: 端末の **Bluetooth を OFF→ON**（`AppScanStats` リセット）。adb なら `adb shell cmd bluetooth_manager disable; ... enable`。
- **再発防止 (wand_receiver/ 実装済み)**:
  - `pagehide` / `visibilitychange(hidden)` で `scan.stop()` → scanner 登録リーク・start 回数暴走を防止。
  - 連続スキャン抑止（前回 start から 2.5 秒は再開不可）。
  - 魔力ウォッチドッグ: 8 秒 広告が途絶えたら**「🪫 魔力切れ」**を魔法画面+ログに表示（acceptAll なので周辺機器の広告が常時届く＝無音は異常のサイン）。
- → **本番で連続スキャンが必要なら Web スキャンは不適**。§4 のゲートウェイ/ネイティブが必須という結論を補強する実証データ。

---

## 3. なぜ顧客向けに使えないのか

| 顧客の端末 | 体験 |
|-----------|------|
| iPhone ユーザー | URL を開いても**そもそも動かない** |
| Android ユーザー | `chrome://flags` 有効化 + 位置情報 ON…**普通の顧客はやらない** |

→ 「顧客がスマホで URL を開くだけ」というゼロ設定体験が成立しない。**製品としては配れない。**

---

## 4. 顧客に配れる現実的な3案

| 方式 | 顧客の手間 | iPhone | 1:N broadcast | フラグ | 向くシーン |
|------|-----------|--------|--------------|--------|-----------|
| **A. ゲートウェイ + WebSocket** | URL/QR を開くだけ | ✅ | ✅ ハブが集約 | 不要 | 固定会場・ショーケース |
| **B. ネイティブアプリ** (Flutter 等で iOS+Android) | アプリをインストール | ✅ | ✅ 各自スキャン | 不要 | 持ち帰り・どこでも使う |
| C. 標準 Web Bluetooth (requestDevice+GATT) | 毎回ペアリング選択 | ✗ | ✗ 接続型 | 不要 | 杖用途には不適 |

### A. ゲートウェイ方式（推奨・既存ハードが活きる）
- 杖はそのまま BLE 広告を broadcast。
- **常時稼働の小型ハブ (ESP32 等) が広告を受信** → Wi-Fi 経由 (WebSocket / SSE / MQTT) で各スマホへイベント配信。
- スマホは**普通のブラウザで URL を開くだけ** = フラグ不要・インストール不要・**iPhone でも動く**。
- 本プロジェクトは既に受信ハード (XIAO nRF52840 / ESP32) があり、ショーケース前提なので相性が良い。
- 留意点: ハブの電源・Wi-Fi (会場 LAN または ハブ自身を AP 化) が必要。レイテンシは BLE 受信 + LAN 配信で十分小さい。

### B. ネイティブアプリ
- スマホが直接 BLE 広告を拾えるのは**ネイティブアプリだけ** (iOS=CoreBluetooth / Android=BluetoothLeScanner、フラグ不要)。
- Flutter / React Native なら 1 コードベースで iOS+Android。
- 「顧客が杖を持ち帰り、自宅などどこでも使う」製品ならこちら。
- 留意点: アプリストア審査・配布。iOS のバックグラウンド広告スキャンは制限あり（フォアグラウンドは問題なし）。

### C. 標準 Web Bluetooth（参考・不適）
- フラグ不要だが、接続 (GATT) 型でデバイス選択 UI が出る。1:N broadcast の「パッシブな魔法」体験にならず、iOS も非対応。杖用途には向かない。

---

## 5. シーン別の推奨

| 想定シーン | 推奨構成 |
|-----------|---------|
| 固定会場 / ショーケースで来場者が自分のスマホで | **A. ゲートウェイ + WebSocket** |
| 顧客が杖を持ち帰り、どこでも自分のスマホで | **B. ネイティブアプリ** |
| 開発者 / 社内の動作確認のみ | 現状の Web スキャンデモで十分 |

---

## 6. 現状の Web デモの位置づけ

- `wand_receiver/` の Web スキャンデモは **Android Chrome / PC Chrome の開発・動作確認用**。
- 価値: 杖の broadcast 仕様 (`manufacturerData 0xFFFF`, 7 バイト payload) が正しいか、PC/Android で即検証できる。
- **顧客配布物ではない**。顧客向けに進める場合は §4 の A か B に作り替える。

---

## 参考

- Web Bluetooth requestLEScan — Chrome Platform Status: <https://chromestatus.com/feature/5346724402954240>
- Communicating with Bluetooth devices over JavaScript — Chrome for Developers: <https://developer.chrome.com/docs/capabilities/bluetooth>
- Web Bluetooth — Can I use: <https://caniuse.com/web-bluetooth>
- Web Bluetooth Scanning spec — WebBluetoothCG: <https://webbluetoothcg.github.io/web-bluetooth/scanning.html>
- ペイロード仕様: `shared/beacon_protocol.h` / [docs/design.md](./design.md)
