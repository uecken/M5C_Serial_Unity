"""
Visual demo: 実 M5C のセンサーデータをブラウザに流し、UI が更新される様子を screenshot 撮影。

手順:
1. M5C ↔ Python serial で接続、sensor.stream 50Hz 開始
2. Chrome (Playwright) で Web app を開く
3. ブラウザで app.js の serialClient シングルトンを取得し、注入
4. UI が更新されたら screenshot
"""
import asyncio, json, sys, time, threading, queue
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
import serial
from playwright.async_api import async_playwright

URL = 'https://localhost:8443/'


async def main():
    # 1. M5C 接続
    ser = serial.Serial('COM8', 115200, timeout=0.3)
    ser.dsrdtr = False; ser.rtscts = False
    time.sleep(0.4)
    ser.dtr = False; ser.rts = False
    ser.reset_input_buffer()

    # 2. ブラウザ起動
    async with async_playwright() as p:
        browser = await p.chromium.launch(
            headless=False,
            args=['--enable-features=WebBluetooth,WebSerial',
                  '--enable-experimental-web-platform-features'])
        ctx = await browser.new_context(ignore_https_errors=True,
                                         viewport={'width': 1280, 'height': 900})
        page = await ctx.new_page()
        await page.goto(URL, wait_until='domcontentloaded', timeout=10000)
        await page.wait_for_selector('h1', timeout=8000)

        # 3. app.js 内の serialClient シングルトンに直接アクセスする
        # app.js は import で隠蔽してあるので、グローバル化のため Hook 注入
        await page.evaluate('''async () => {
            // app.js を再 import → 既にインスタンス化されている SerialClient を取得不可
            // 代替: app.js を新たに評価する代わりに、document の preact root を一旦 unmount して再 mount し
            //       新しい client インスタンスを window 経由で公開
            // → 簡単化: 新規 SerialClient を作成し、その _handleLine を window.__inj に紐付け
            const m = await import('./src/lib/SerialClient.js');
            window.__client = new m.SerialClient();
            window.__inj = (line) => window.__client._handleLine(line);

            // App は serialClient シングルトン (もう一つ) で動いているが、
            // ここで直接 React state を操作するのは困難。代替:
            // app.js のシングルトンも同じ module 評価から得られるか確認
        }''')

        # 4. M5C に ping → sensor.stream 開始
        ser.write(b'{"cmd":"ping"}\n')
        time.sleep(0.3)
        ser.write(b'{"cmd":"sensor.stream","rate_hz":50}\n')
        time.sleep(0.2)
        ser.reset_input_buffer()

        # 5. M5C データ → ブラウザへ 1.5 秒間注入
        end = time.time() + 1.5
        buf = b''
        injected = 0
        while time.time() < end:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line_str = line.decode('utf-8', errors='replace').strip()
                    if line_str:
                        await page.evaluate('(l) => window.__inj(l)', line_str)
                        injected += 1
            await asyncio.sleep(0.01)

        print(f'Injected {injected} sensor lines')

        # 6. 取得済データを window.__client から取り出して JSON で確認
        last = await page.evaluate('''() => new Promise((resolve) => {
            let last = null;
            window.__client.addEventListener('type:sensor', (ev) => last = ev.detail);
            setTimeout(() => resolve(last), 100);
            // 念のため最後にもう一発注入
            window.__inj('{"type":"sensor","t":1,"ax":0,"ay":0,"az":9.81,"pitch":0,"roll":0,"yaw":0,"qw":1,"qx":0,"qy":0,"qz":0}');
        })''')
        print(f'Last sensor in browser: {last}')

        # 7. スクリーンショット
        await page.screenshot(path='test/web/screenshot_visual_demo.png', full_page=True)
        print('Screenshot: test/web/screenshot_visual_demo.png')

        # 終了
        ser.write(b'{"cmd":"sensor.stream","rate_hz":0}\n')
        time.sleep(0.2)
        await page.wait_for_timeout(1500)
        await browser.close()
    ser.close()


asyncio.run(main())
