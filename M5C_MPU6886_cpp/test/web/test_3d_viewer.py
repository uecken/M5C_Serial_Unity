"""
3D viewer (Three.js) test
"""
import asyncio
import sys
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

from playwright.async_api import async_playwright

URL = 'https://localhost:8443/'


async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(headless=False, args=['--enable-features=WebSerial,WebBluetooth'])
        ctx = await b.new_context(ignore_https_errors=True, viewport={'width': 1280, 'height': 900})
        page = await ctx.new_page()

        errors = []
        page.on('pageerror', lambda exc: errors.append(str(exc)))
        page.on('console', lambda msg: errors.append(f'[err] {msg.text}') if msg.type == 'error' else None)

        await page.goto(URL, wait_until='domcontentloaded', timeout=10000)
        await page.wait_for_selector('h1', timeout=8000)
        await page.wait_for_timeout(1000)

        # canvas 要素 (3D) があること
        canvas = await page.query_selector('canvas')
        print(f'  canvas present: {canvas is not None}')
        if canvas:
            box = await canvas.bounding_box()
            print(f'  canvas size: {box}')

        # Three.js が WebGL で描画していることを確認: WebGLRenderingContext を持つか
        webgl_ok = await page.evaluate('''() => {
            const canvases = document.querySelectorAll('canvas');
            for (const c of canvases) {
                try {
                    const ctx = c.getContext('webgl') || c.getContext('webgl2');
                    if (ctx) return true;
                } catch (e) {}
            }
            return false;
        }''')
        print(f'  WebGL context active: {webgl_ok}')

        # 3D Reset ボタン
        reset_btn = await page.query_selector('button:has-text("3D Reset")')
        print(f'  3D Reset button present: {reset_btn is not None}')

        # IMUViewer に sensor 注入してクォータニオンを変えてみる
        # (Preact App の useEffect で setQuaternion が呼ばれる経路は serialClient/bleClient イベント経由)
        # ここでは window レベルで viewerRef を取れないので、IMUViewer を直接 import してテスト
        import_ok = await page.evaluate('''async () => {
            try {
                const m = await import('./src/lib/IMUViewer.js');
                // サンプルで作ってみる
                const c = document.createElement('canvas');
                c.width = 200; c.height = 150;
                document.body.appendChild(c);
                const v = new m.IMUViewer(c);
                v.setQuaternion(0.707, 0.707, 0, 0);  // 90° rotate around X
                await new Promise(r => setTimeout(r, 100));
                v.setEuler(45, 0, 0);
                await new Promise(r => setTimeout(r, 100));
                v.reset();
                v.destroy();
                document.body.removeChild(c);
                return true;
            } catch (e) {
                return 'error: ' + e.message;
            }
        }''')
        print(f'  IMUViewer module test: {import_ok}')

        # スクリーンショット
        await page.screenshot(path='test/web/screenshot_3d.png', full_page=True)
        print('  Screenshot: test/web/screenshot_3d.png')

        if errors:
            print(f'\n  Errors ({len(errors)}):')
            for e in errors[:5]:
                print(f'    {e[:200]}')

        await page.wait_for_timeout(2000)
        await b.close()
        return canvas is not None and webgl_ok and import_ok is True


if __name__ == '__main__':
    sys.exit(0 if asyncio.run(main()) else 1)
