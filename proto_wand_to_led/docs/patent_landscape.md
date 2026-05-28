# 杖デバイスの特許ランドスケープ（調査メモ）

「IMU 付き杖 → ジェスチャ → 無線 → 光/演出」に関する特許の調査結果。当プロジェクト方式の **FTO（侵害回避）見立て**と**自社出願の余地**を整理する。

> ⚠️ **これは一般情報であり法的助言ではない。** Google Patents の法的ステータスは遅延・誤りがあり得る。
> 商用化前に必ず **弁理士による FTO 調査**＋ **USPTO 原簿 / J-PlatPat / Espacenet で年金状況・継続出願・各国ファミリーの生存**を再確認すること。
> 調査日: 2026-05-25。

---

## 0. 結論サマリ

- **当方式の本丸＝方式1（杖内 IMU → しきい値判定＝重力基準フリック検出 → BLE Advertising broadcast）は FTO リスクが低い。** 近接する特許は満了/失効しており、active な近接特許（Kano/Google）とは構成が異なる。
- **最大の制約は特許ではなく Warner Bros. の商標・著作権（HP ブランド・呪文名）**。商用は「WB ライセンス」か「独自ブランド化（呪文名リネーム）」の二択。詳細は本ファイル §5 と過去調査参照。
- **将来 ML / 本格的なジェスチャ認識に進化させる場合のみ**、Google US 12,141,381 と Kano US 11,301,059 のクレーム精読が必要。

---

## 1. 当プロジェクトの2方式

| 方式 | 内容 | 実装状況 |
|---|---|---|
| **方式1: しきい値フリック** | 杖内 IMU の linear accel を重力基準で射影し、しきい値で上振り/下振り/前突き等を**杖側で**判定 → BLE Advertising で trigger_id を broadcast | **現行実装**（wand_m5stickc / shared/wand_gesture.h） |
| **方式2: IMU 軌跡判定** | 慣性データから動きの軌跡/パターンを認識 | 未実装（将来案） |

---

## 2. 方式1（しきい値フリック）に関係する特許

「杖＋加速度しきい値→効果」は**古くから既知**。核心特許はほぼ満了/失効＝**実質パブリックドメイン領域**。

| 特許 | 権利者 | 内容 | 状態 |
|---|---|---|---|
| US 6,626,728 *Motion-sequence activated toy wand* | 玩具系 | 加速度センサで「振りの連続=呪文」→ 光演出 | **満了**（2001出願) |
| US 7,445,550 *Magical wand and interactive play* | 玩具系 | 先端 accel>閾値 かつ 根元<閾値 で発動（フリック式しきい値） | 満了間際（~2005出願) |
| US 9,699,866 *Wand gesture* | **MOJO Labs** | accel が閾値を跨ぐ→tap/swipe/tilt 検出→外部(照明等)へ command 送信 | **失効（年金未納 "Expired-Fee Related"）** |
| US 9,888,090 *Magic wand methods* | （要確認） | accel→位置・姿勢へ変換 | 要確認 |

➡️ 当方式に最も近い MOJO 9,699,866 は**失効**、玩具基礎特許は**満了**。**FTO リスク低**。同時に**新規の広い特許は取りにくい**（既知技術）。

---

## 3. 方式2（IMU 軌跡判定）に関係する特許

慣性センサで軌跡再構成→認識も**既知**。基礎は満了。ただし **active な近接特許（Kano / Google）あり**。

| 特許/技術 | 権利者 | 内容 | 状態 |
|---|---|---|---|
| US 8,229,226 *慣性センシング+軌跡再構成* | **ITRI（台湾）** | accel+gyro→積分で3D軌跡を再構成→文字/ジェスチャ認識 | **失効（2024年金未納）** |
| Wii / Wii MotionPlus（+AiLive） | Nintendo | accel+gyro でモーション/ジェスチャ認識 | 多くが満了/間際（~2006-08） |
| **US 11,301,059** *Gesture recognition system* | **Kano Computing** | §4 参照（姿勢→カーソル+テンプレ照合。**軌跡ではない**） | **有効（2022登録）** |
| US 12,141,381 *Selective gesture recognition for handheld devices* | **Google** | 杖の IMU→**機械学習**でジェスチャ（"smart wand"/"spell-like motions" 明記）、ローカル/リモート選択処理 | **有効（2024登録）** |
| US 11,449,802 *ML based gesture recognition using multiple sensors* | **Apple** | IMU+生体センサ+ML でジェスチャ（Apple Watch 系） | 有効。IMU+ML の広い先行/並走 |

---

## 4. Kano 杖（HP コーディングキット）の正確な中身 ★要注意点

**当初「軌跡判定」と誤記したが、特許実体は「軌跡」ではない。** US 11,301,059 B2（Kano Computing Ltd、Klein & Griffith、2018優先/2022登録・有効）の独立クレーム：

- **加速度の積分による空間軌跡の再構成では「ない」**。
- 杖の**姿勢（ジャイロ/磁気）を 2D カーソル**に写像（レーザーポインタ的）。カーソルは button で「原点姿勢」にリセット可能（積分軌跡と非両立）。
- 認識は**杖ではなく接続デバイス(アプリ)側**（杖は処理を持たず安価・低電力）。
- 方式は**テンプレートマッチング**（motion 出力を gesture library と比較）。**ML ではない**。
- HW: 9DoF IMU（accel+gyro+磁気）+ BLE（nRF52832）。

➡️ **Kano = 「姿勢→カーソル + アプリ側テンプレ照合」**。当方式（杖内しきい値→BLE broadcast、認識は杖側、カーソル/パネル無し）とは**構成が大きく異なり射程外の可能性が高い**。
（なお Kano は 2023 に WB の Magic Caster Wand が自社 IP を侵害と主張し係争。WB は一時販売停止。）

---

## 5. Universal（City Studios LLC）の杖系特許

テーマパーク前提。**杖内 IMU で自動判定する純粋方式は確認できず**、カメラ/姿勢/RFID が中心。

| 方式 | 内容 | 当方式との関係 |
|---|---|---|
| 再帰反射 + 赤外カメラ（US 10,134,267 ファミリー: 10,380,884 / 10,699,557 / 12,100,292 等） | 受動杖先端を IR カメラで軌跡追跡 | カメラ式＝**別物** |
| 姿勢センサ + 音声（Interactive Pepper's Ghost） | ハンドヘルド機の向き検出＋マイクで演出 | 演出装置前提で別構成 |
| RFID/ウェアラブル識別（US 10,603,564） | 来場者を RFID 識別 | 無関係 |
| 2023 新世代杖（キャラ対話） | 既存杖の改良＋キャラ応答 | 詳細センサ未確認 |

➡️ Universal は「カメラ」「姿勢＋音声」「RFID」が主。**当方式(IMU+BLE broadcast)とは重なりにくい。**

---

## 6. FTO まとめと次アクション

| 観点 | 方式1（しきい値フリック=現行） | 方式2（IMU 軌跡） |
|---|---|---|
| FTO（侵害リスク） | **低**（MOJO 9,699,866 失効＋玩具基礎満了。Kano/Google とは構成相違） | 核心は低（ITRI/Wii 満了）。**Kano/Google が active**だが、当方式が「姿勢→カーソル+テンプレ」でも「ML」でもなければ射程外 |
| 特許性（自社出願） | 低い（既知）。狙うなら *BLE Advertising 1:N broadcast + 特定呪文マッピング* 等の狭い組合せ | 低い（既知）。新規アルゴリズム部分のみ |

**注意の分岐点**: 将来 ①アプリ側で「姿勢→カーソル+テンプレ照合」をやると **Kano US 11,301,059** に、②**機械学習**でジェスチャ認識すると **Google US 12,141,381 / Apple US 11,449,802** に近づく。現行のオンデバイスしきい値方式は両者の外。

**自社出願の候補**: 光学式ろうそく「光の受け渡し」機構（`proto_light-wand_to_candle/`）は比較的ユニーク → 先行調査の上で出願性を検討する価値あり。

**次アクション候補**:
1. 主要 active 特許（Kano 11,301,059 / Google 12,141,381）の **JP ファミリーを J-PlatPat / Espacenet (INPADOC family) で確認**（日本での効力）。
2. ろうそく光学機構の**出願性メモ**（先行: 脱出ゲーム用プロップ等の調査）。
3. 商用ブランド方針（呪文名リネーム vs WB ライセンス）の確定。

---

## 出典（調査 2026-05-25）

- US 6,626,728 Motion-sequence activated toy wand — <https://patents.google.com/patent/US6626728>
- US 7,445,550 Magical wand and interactive play experience — <https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7445550>
- US 9,699,866 Wand gesture (MOJO Labs, 失効) — <https://patents.google.com/patent/US9699866B2/en>
- US 8,229,226 慣性軌跡再構成 (ITRI, 失効) — <https://patents.google.com/patent/US8229226B2/en>
- **US 11,301,059 / US20210165506A1 Gesture recognition system (Kano)** — <https://patents.google.com/patent/US20210165506A1/en>
- US 12,141,381 Selective gesture recognition for handheld devices (Google) — <https://patents.google.com/patent/US12141381B2/en>
- US 11,449,802 ML based gesture recognition using multiple sensors (Apple) — <https://patents.google.com/patent/US11449802B2/en>
- Universal passive wand tracking US 10,380,884 — <https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10380884>
- Kano vs Warner Bros. 係争 (TechCrunch 2023) — <https://techcrunch.com/2023/01/27/warner-bros-swiped-our-harry-potter-wand-ip-says-kano/>
- Wii MotionPlus / AiLive — <https://en.wikipedia.org/wiki/Wii_MotionPlus>
