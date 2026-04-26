"""
Burst Motion Web 設定アプリ — 自動 UI テスト

Playwright で Chrome を起動し、Web Serial / Web Bluetooth API の存在、UI 描画、
コンソールエラーの有無を検証する。

実機の Serial 接続は user gesture を要するため自動化不可だが、
ボタンクリック後にダイアログが開く動作までは確認できる。

実行: python test_web_app.py
"""
import sys
import asyncio
import time

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

from playwright.async_api import async_playwright

URL = 'https://localhost:8443/'


async def main():
    results = []
    def record(name, ok, detail=''):
        mark = '✅' if ok else '❌'
        line = f'{mark} {name}'
        if detail:
            line += f'  — {detail}'
        results.append(line)
        print(line)

    async with async_playwright() as p:
        # Web Serial / Web Bluetooth のため Chrome (チャンネル使用) ではなく
        # 同梱 chromium で十分。HTTPS 自己署名は ignore_https_errors で許可。
        browser = await p.chromium.launch(
            headless=False,  # 画面表示。headless=True にすると一部 API 無効
            args=[
                '--enable-features=WebBluetooth,WebSerial',
                '--enable-experimental-web-platform-features',
            ],
        )
        context = await browser.new_context(
            ignore_https_errors=True,
            viewport={'width': 1280, 'height': 800},
        )
        page = await context.new_page()

        console_errors = []
        page.on('console', lambda msg: (
            console_errors.append(msg.text) if msg.type == 'error' else None
        ))
        page.on('pageerror', lambda exc: console_errors.append(f'PageError: {exc}'))

        # 1. ページ読込
        print(f'\n=== Loading {URL} ===')
        try:
            await page.goto(URL, wait_until='domcontentloaded', timeout=10000)
            record('Page loads', True)
        except Exception as e:
            record('Page loads', False, str(e))
            await browser.close()
            return

        # 2. Tailwind / Preact が読み込まれて DOM レンダリングが進むのを待つ
        try:
            await page.wait_for_selector('h1', timeout=8000)
            title = await page.text_content('h1')
            record('App title rendered', 'Burst Motion' in (title or ''), f'h1="{title}"')
        except Exception as e:
            record('App title rendered', False, str(e))

        # 3. JS API の存在確認
        api_check = await page.evaluate('''() => ({
            webSerial: 'serial' in navigator,
            webBluetooth: 'bluetooth' in navigator,
            preact: !!document.querySelector('h1'),
        })''')
        record('Web Serial API available',    api_check['webSerial'])
        record('Web Bluetooth API available', api_check['webBluetooth'])
        record('Preact rendered DOM',         api_check['preact'])

        # 4. UI 要素確認: トランスポート切替タブ
        usb_tab = await page.query_selector('button:has-text("USB Serial")')
        ble_tab = await page.query_selector('button:has-text("BLE")')
        record('USB Serial tab present', usb_tab is not None)
        record('BLE tab present',        ble_tab is not None)

        # 5. 「Controller に接続」ボタン
        connect_btn = await page.query_selector('button:has-text("接続")')
        record('Connect button present', connect_btn is not None)

        # 6. BLE タブをクリックしてみる
        if ble_tab:
            await ble_tab.click()
            await page.wait_for_timeout(300)
            connect_btn_ble = await page.query_selector('button:has-text("BLE NUS で接続")')
            record('BLE connect button after tab switch', connect_btn_ble is not None)

        # 7. USB タブに戻す
        if usb_tab:
            await usb_tab.click()
            await page.wait_for_timeout(300)
            connect_btn_usb = await page.query_selector('button:has-text("USB Serial で接続")')
            record('USB connect button after tab switch', connect_btn_usb is not None)

        # 8. スクリーンショット保存
        screenshot = 'test/web/screenshot_main.png'
        await page.screenshot(path=screenshot, full_page=True)
        record('Screenshot saved', True, screenshot)

        # 9. SerialClient.js が import できているか
        loaded_modules = await page.evaluate('''() => {
            const scripts = Array.from(document.querySelectorAll('script[type=module]'));
            return scripts.map(s => s.src);
        }''')
        record('Module script loaded', any('app.js' in s for s in loaded_modules),
               f'modules={loaded_modules}')

        # 10. console エラーの確認
        await page.wait_for_timeout(500)
        record('No console errors', len(console_errors) == 0,
               f'errors={console_errors[:3]}' if console_errors else '')

        # 11. JSON ライン処理のロジック単体テスト (SerialClient/BleClient _handleLine 相当)
        unit = await page.evaluate('''async () => {
            const m = await import('./src/lib/SerialClient.js');
            const c = new m.SerialClient();
            const got = [];
            c.addEventListener('type:pong', (ev) => got.push(ev.detail));
            // 内部メソッドを直接テスト
            c._handleLine('{"type":"pong","fw":"2.0.0-dev"}');
            return got;
        }''')
        record('SerialClient parses JSON line', len(unit) == 1 and unit[0]['type'] == 'pong',
               f'parsed={unit}')

        # まとめ
        print()
        print('=' * 50)
        passed = sum(1 for r in results if r.startswith('✅'))
        total = len(results)
        print(f' RESULT: {passed}/{total} passed')
        print('=' * 50)

        # 5 秒間ブラウザを開いたまま (目視確認用)
        print()
        print('ブラウザを 5 秒間表示します (目視確認用)...')
        await page.wait_for_timeout(5000)

        await browser.close()
        return passed == total


if __name__ == '__main__':
    ok = asyncio.run(main())
    sys.exit(0 if ok else 1)
