# Burst Motion — 設計資料

Burst Motion（旧 Motion Controller / Motion Burst）の設計資料・実装ドキュメント。

関連: [Protopedia #1988](https://protopedia.net/prototype/1988)

## ディレクトリ構成

| ディレクトリ | 内容 |
|-------------|------|
| [architecture/](architecture/) | 全体アーキテクチャ、HAL 抽象化、Phase 構成 |
| [hardware/](hardware/) | HW 設計、BOM、IMU 選定、PCB レイアウト |
| [firmware/](firmware/) | FW 実装ガイド、モジュール仕様、ビルド方法 |
| [web/](web/) | Web アプリ設計、UI mockup、Web Serial プロトコル使用 |
| [protocol/](protocol/) | JSON Lines プロトコル仕様、プロファイル JSON スキーマ |
| [testing/](testing/) | テスト計画、互換性マトリクス、性能測定結果 |
| [changelog/](changelog/) | Phase 別の実装ログ、設計変更履歴 |

## クイックリンク

- [アーキテクチャ概要](architecture/overview.md)
- [JSON Lines プロトコル仕様](protocol/json-lines.md)
- [プロファイル JSON スキーマ](protocol/profile-schema.md)
- [Phase 1 実装ガイド](firmware/phase1-implementation.md)
- [Web アプリ仕様](web/hidconfig-app-spec.md)
- [BOM $25 内訳](hardware/bom.md)

## 計画書（全体）

包括的な設計計画は上位プランフォルダに保存:
- `C:\Users\thefu\.claude\plans\m5c-mpu6886-cpp-https-findradio-jp-moti-dynamic-beacon.md` (2837 行)

## ドキュメント作成ポリシー

1. **実装の進捗に合わせて随時更新**
2. **マークダウン統一**、図は Mermaid 優先
3. **日本語主体**、主要な技術用語は英語併記
4. **コードスニペットは動くもの**のみ掲載
5. **Phase 完了時に changelog/ にサマリを追加**
