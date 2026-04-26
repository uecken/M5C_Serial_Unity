"""
Auto-reconnect & UI feature test (Phase 2.1)
"""
import asyncio
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

from playwright.async_api import async_playwright

URL = 'https://localhost:8443/'


async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(headless=True,
            args=['--enable-features=WebSerial,WebBluetooth'])
        ctx = await b.new_context(ignore_https_errors=True)
        page = await ctx.new_page()

        await page.goto(URL, wait_until='domcontentloaded')
        await page.wait_for_selector('h1', timeout=5000)

        # SerialClient.getAuthorizedPorts (Playwright context = 認可なし、0 件期待)
        n = await page.evaluate('''async () => {
            const m = await import('./src/lib/SerialClient.js');
            const c = new m.SerialClient();
            const ports = await c.getAuthorizedPorts();
            return ports.length;
        }''')
        print(f'getAuthorizedPorts() = {n} (test context expected 0)')

        ok = await page.evaluate('''async () => {
            const m = await import('./src/lib/SerialClient.js');
            const c = new m.SerialClient();
            return await c.autoConnect(115200);
        }''')
        print(f'autoConnect() = {ok} (expected false: no authorized ports)')

        # UI 要素
        results = []
        async def check(name, sel):
            el = await page.query_selector(sel)
            ok = el is not None
            results.append((name, ok))
            print(f'  {"✅" if ok else "❌"} {name}')

        await check('自動接続トグル',  'label:has-text("自動接続")')
        await check('USB タブ',         'button:has-text("USB")')
        await check('BLE タブ',         'button:has-text("BLE")')
        await check('HID Press button', 'button:has-text("Press")')
        await check('HID Type button',  'button:has-text("Type")')
        await check('Mouse 50,0',       'button:has-text("Mouse 50,0")')
        await check('Add Rule',         'button:has-text("Add Rule")')
        await check('Calibrate',        'button:has-text("Calibrate")')

        await page.screenshot(path='test/web/screenshot_phase21.png', full_page=True)
        print('\nScreenshot: test/web/screenshot_phase21.png')

        passed = sum(1 for _, ok in results if ok)
        total = len(results)
        print(f'\n=== UI features: {passed}/{total} ===')
        await b.close()
        return passed == total


if __name__ == '__main__':
    sys.exit(0 if asyncio.run(main()) else 1)
