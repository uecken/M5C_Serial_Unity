#!/bin/bash
# Burst Motion - gh-pages 自動デプロイスクリプト
# Web/hidconfig の変更を gh-pages worktree にコピーして commit + push
#
# 使い方:
#   bash deploy_ghpages.sh "コミットメッセージ"
#
# 前提:
#   - 親 repo に gh-pages worktree が登録済み
#     (../M5C_Serial_Unity-gh-pages)
#   - cert.pem / key.pem / serve_https.py は除外
set -e

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Web/hidconfig から見て: ../../.. = M5C_Serial_Unity の親 (Documents)
# その下に M5C_Serial_Unity-gh-pages がある
PARENT_DIR="$(cd "$SRC_DIR/../../../.." && pwd)"
GH_DIR="$PARENT_DIR/M5C_Serial_Unity-gh-pages"

if [ ! -d "$GH_DIR" ]; then
    echo "ERROR: gh-pages worktree not found at: $GH_DIR"
    echo "Setup once: cd ../.. && git worktree add ../M5C_Serial_Unity-gh-pages --detach && cd ../M5C_Serial_Unity-gh-pages && git checkout gh-pages"
    exit 1
fi

MSG="${1:-Update Web app}"

# rsync で同期 (cert/key/server を除外)
echo "[1/3] Syncing $SRC_DIR -> $GH_DIR"
rsync -a --delete \
    --exclude=cert.pem \
    --exclude=key.pem \
    --exclude=serve_https.py \
    --exclude=deploy_ghpages.sh \
    --exclude=.git \
    --exclude=GHPAGES_README.md \
    "$SRC_DIR/" "$GH_DIR/"

# .nojekyll 維持
touch "$GH_DIR/.nojekyll"

# GHPAGES_README.md は維持 (rsync で消えないよう、明示的に維持)
# (--exclude で守ったので OK)

cd "$GH_DIR"

if git diff --quiet HEAD; then
    echo "[2/3] No changes."
    exit 0
fi

echo "[2/3] Committing..."
git add -A
git commit -m "$MSG"

echo "[3/3] Pushing to origin gh-pages..."
git push origin gh-pages

echo ""
echo "✅ Deployed to: https://uecken.github.io/M5C_Serial_Unity/"
echo "   (反映まで数十秒〜数分)"
