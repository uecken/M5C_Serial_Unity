#!/usr/bin/env python3
"""
Burst Motion - gh-pages 自動デプロイスクリプト
Web/hidconfig の変更を gh-pages worktree にコピーして commit + push

使い方:
  python deploy_ghpages.py "コミットメッセージ"

前提:
  - 親 repo に gh-pages worktree が登録済み
    (../M5C_Serial_Unity-gh-pages)

キャッシュバスティング:
  - デプロイ時のタイムスタンプを VERSION (例: 20260425-153005) として生成
  - index.html の __BUILD_VERSION__ プレースホルダを VERSION で置換
  - すべての .js ファイルの相対 import に ?v=VERSION を自動付与
  - version.json を生成 (UI から fetch して表示)
  - これにより Chrome がキャッシュ済みファイルを返すことを防止
"""
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

SRC_DIR = Path(__file__).resolve().parent
# Web/hidconfig からみて 4 つ上 = Documents
GH_DIR = SRC_DIR.parents[3] / 'M5C_Serial_Unity-gh-pages'

EXCLUDE = {'cert.pem', 'key.pem', 'serve_https.py', 'deploy_ghpages.sh',
           'deploy_ghpages.py', 'GHPAGES_README.md', '.git'}


def apply_cache_busting(gh_dir: Path, version: str):
    """index.html の __BUILD_VERSION__ 置換 + .js の相対 import に ?v= 付与"""
    # 1. index.html プレースホルダ置換
    index_path = gh_dir / 'index.html'
    if index_path.is_file():
        txt = index_path.read_text(encoding='utf-8')
        new_txt = txt.replace('__BUILD_VERSION__', version)
        if new_txt != txt:
            index_path.write_text(new_txt, encoding='utf-8')

    # 2. すべての .js ファイル内の `from './...js'` 形式の import に ?v= を付与
    #    既に ?v=... が付いている場合は新バージョンに置換
    src_dir = gh_dir / 'src'
    if not src_dir.is_dir():
        return
    pattern = re.compile(
        r"""(from\s+['"])(\.[^'"]+?\.js)(\?v=[^'"]*)?(['"])""",
        re.MULTILINE
    )
    repl = lambda m: f'{m.group(1)}{m.group(2)}?v={version}{m.group(4)}'
    for js_file in src_dir.rglob('*.js'):
        txt = js_file.read_text(encoding='utf-8')
        new_txt = pattern.sub(repl, txt)
        if new_txt != txt:
            js_file.write_text(new_txt, encoding='utf-8')

    # 3. version.json 生成 (UI から fetch して表示)
    (gh_dir / 'version.json').write_text(
        json.dumps({
            'version': version,
            'deployed_at': datetime.datetime.now().isoformat(timespec='seconds'),
        }, ensure_ascii=False),
        encoding='utf-8'
    )


def main():
    msg = sys.argv[1] if len(sys.argv) > 1 else 'Update Web app'

    if not GH_DIR.is_dir():
        print(f'ERROR: gh-pages worktree not found at: {GH_DIR}')
        print('Setup once:')
        print('  cd ../..')
        print('  git worktree add ../M5C_Serial_Unity-gh-pages --detach')
        print('  cd ../M5C_Serial_Unity-gh-pages && git checkout gh-pages')
        sys.exit(1)

    # PIO ビルド成果物を firmware/ ディレクトリにコピー (esp-web-tools 用)
    # Phase 5.20.1: ブラウザから直接 FW 書込み可能なように最新 FW を同梱
    pio_build = SRC_DIR.parents[1] / '.pio' / 'build' / 'm5stick-c-v2'
    fw_target = SRC_DIR / 'firmware' / 'm5stickc-v2'
    if pio_build.is_dir() and fw_target.is_dir():
        copied = []
        for fname in ['firmware.bin', 'bootloader.bin', 'partitions.bin']:
            src = pio_build / fname
            if src.is_file():
                shutil.copy2(src, fw_target / fname)
                copied.append(fname)
        if copied:
            print(f'[FW] copied to firmware/m5stickc-v2/: {", ".join(copied)}')
        # manifest.json の version を build 時刻で更新
        manifest_path = fw_target / 'manifest.json'
        if manifest_path.is_file():
            try:
                m = json.loads(manifest_path.read_text(encoding='utf-8'))
                fw_path = fw_target / 'firmware.bin'
                if fw_path.is_file():
                    mtime = datetime.datetime.fromtimestamp(fw_path.stat().st_mtime)
                    m['version'] = f'2.0.0-dev (' + mtime.strftime('%Y-%m-%d %H:%M') + ')'
                    manifest_path.write_text(json.dumps(m, indent=4, ensure_ascii=False), encoding='utf-8')
                    print(f'[FW] manifest version: {m["version"]}')
            except Exception as e:
                print(f'[FW] manifest update warning: {e}')

    # 杖 FW (proto_wand_to_led/wand_m5stickc, env=m5stick-c-wom) を
    # wand_receiver/firmware/wand/ にコピー (wand_receiver ページの esp-web-tools フラッシャ用)。
    # boot_app0.bin は静的なので再コピー不要。
    wand_build = SRC_DIR.parents[2] / 'proto_wand_to_led' / 'wand_m5stickc' / '.pio' / 'build' / 'm5stick-c-wom'
    wand_target = SRC_DIR / 'wand_receiver' / 'firmware' / 'wand'
    if wand_build.is_dir() and wand_target.is_dir():
        wcopied = []
        for fname in ['firmware.bin', 'bootloader.bin', 'partitions.bin']:
            src = wand_build / fname
            if src.is_file():
                shutil.copy2(src, wand_target / fname)
                wcopied.append(fname)
        if wcopied:
            print(f'[FW] wand copied to wand_receiver/firmware/wand/: {", ".join(wcopied)}')

    print(f'[1/3] Syncing {SRC_DIR} -> {GH_DIR}')

    # gh-pages 側にあって、source 側に無いファイルを削除 (但し EXCLUDE と GHPAGES_README.md は維持)
    keep_in_gh = {'.git', '.nojekyll', 'GHPAGES_README.md', 'README.md'}
    for entry in GH_DIR.iterdir():
        if entry.name in keep_in_gh:
            continue
        rel = SRC_DIR / entry.name
        if not rel.exists():
            if entry.is_dir():
                shutil.rmtree(entry)
            else:
                entry.unlink()

    # source -> gh-pages へコピー
    for root, dirs, files in os.walk(SRC_DIR):
        # EXCLUDE に含まれるディレクトリは除外
        dirs[:] = [d for d in dirs if d not in EXCLUDE]
        rel_root = Path(root).relative_to(SRC_DIR)
        target_root = GH_DIR / rel_root
        target_root.mkdir(parents=True, exist_ok=True)
        for f in files:
            if f in EXCLUDE:
                continue
            src_file = Path(root) / f
            dst_file = target_root / f
            shutil.copy2(src_file, dst_file)

    # .nojekyll を維持
    (GH_DIR / '.nojekyll').touch()

    # キャッシュバスティング: index.html / .js / version.json を更新
    version = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    print(f'[1.5/3] Applying cache-busting (version={version})')
    apply_cache_busting(GH_DIR, version)

    print(f'[2/3] Checking diff...')
    os.chdir(GH_DIR)
    diff = subprocess.run(['git', 'diff', '--quiet', 'HEAD'], capture_output=True)
    untracked = subprocess.run(['git', 'ls-files', '--others', '--exclude-standard'],
                                capture_output=True, text=True).stdout.strip()
    if diff.returncode == 0 and not untracked:
        print('  No changes. Skipping commit.')
        return

    print(f'  Committing: "{msg}"')
    subprocess.run(['git', 'add', '-A'], check=True)
    subprocess.run(['git', 'commit', '-m', msg], check=True)

    print('[3/3] Pushing to origin gh-pages...')
    subprocess.run(['git', 'push', 'origin', 'gh-pages'], check=True)

    print()
    print('✅ Deployed to: https://uecken.github.io/M5C_Serial_Unity/')
    print('   (反映まで数十秒〜数分)')


if __name__ == '__main__':
    main()
