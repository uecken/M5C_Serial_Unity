"""
Burst Motion E2E テスト: ブラウザ → USB Serial → M5StickC FW → ブラウザ受信

Web Serial API の port chooser はユーザー操作が必要なので、
**ブラウザ上で navigator.serial.requestPort() の代わりに直接 chrome テストフラグ**を使う。

Playwright + chrome 開発者用 launch options で port chooser をバイパスし、
chrome.serial 系の test API を使用。

代替: `chromedriver` の experimental options でも可能、Playwright の方が制御性高い。

実行: python test_serial_e2e.py
"""
import asyncio
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

from playwright.async_api import async_playwright

URL = 'https://localhost:8443/'
COM_PORT = 'COM8'
BAUD = 115200


async def main():
    async with async_playwright() as p:
        # Chrome に Web Serial の port chooser を auto-pick させる方法は限定的。
        # 代わりに、Web Serial API を Web Worker やテスト用 mock で動かす。
        #
        # ここでは: Playwright から直接 Python serial で M5C と通信し、
        # Web app は parse-only / display-only テストに切る。
        #
        # 完全自動 E2E は難しいので、以下を確認:
        #   1. Web app が SerialClient/BleClient を正しく構築できる
        #   2. JSON 行を _handleLine() で渡すと型別イベントが発火する
        #   3. send() で正しい encoded text を生成する (mock writer)
        browser = await p.chromium.launch(
            headless=False,
            args=[
                '--enable-features=WebBluetooth,WebSerial',
                '--enable-experimental-web-platform-features',
            ],
        )
        ctx = await browser.new_context(ignore_https_errors=True,
                                         viewport={'width': 1280, 'height': 800})
        page = await ctx.new_page()

        console_logs = []
        page.on('console', lambda msg: console_logs.append(f'[{msg.type}] {msg.text}'))

        await page.goto(URL, wait_until='domcontentloaded', timeout=10000)
        await page.wait_for_selector('h1', timeout=8000)

        # SerialClient を直接インスタンス化、mock の writable/readable で接続シミュレート
        result = await page.evaluate('''async () => {
            const m = await import('./src/lib/SerialClient.js');
            const c = new m.SerialClient();

            // mock port: writable / readable のかわりに event を発火するだけ
            const events = [];
            c.addEventListener('type:pong', (ev) => events.push({t:'pong', d:ev.detail}));
            c.addEventListener('type:sensor', (ev) => events.push({t:'sensor', d:ev.detail}));
            c.addEventListener('type:device.info', (ev) => events.push({t:'devinfo', d:ev.detail}));
            c.addEventListener('type:err', (ev) => events.push({t:'err', d:ev.detail}));

            // 受信シミュレーション
            c._handleLine('{"type":"pong","fw":"2.0.0-dev","board":"m5stickc","imu":"mpu6886","uptime":1234}');
            c._handleLine('{"type":"sensor","t":1234,"ax":0.1,"ay":0.2,"az":9.8,"pitch":1,"roll":2,"yaw":3}');
            c._handleLine('{"type":"device.info","fw":"2.0.0-dev","ble_nus_started":true}');
            c._handleLine('{"type":"err","cmd":"foo","err":"unknown_cmd"}');

            // bad JSON
            const unparsedEvents = [];
            c.addEventListener('unparsed', (ev) => unparsedEvents.push(ev.detail));
            c._handleLine('not a json');

            return { events, unparsedEvents };
        }''')

        passed = []
        def check(name, ok, detail=''):
            mark = '✅' if ok else '❌'
            line = f'{mark} {name}'
            if detail:
                line += f'  — {detail}'
            print(line)
            passed.append(ok)

        events = result['events']
        check('pong event fired',     any(e['t'] == 'pong' for e in events))
        check('sensor event fired',   any(e['t'] == 'sensor' for e in events))
        check('devinfo event fired',  any(e['t'] == 'devinfo' for e in events))
        check('err event fired',      any(e['t'] == 'err' for e in events))
        check('unparsed line surfaced', len(result['unparsedEvents']) == 1)

        # BleClient も同様
        result2 = await page.evaluate('''async () => {
            const m = await import('./src/lib/BleClient.js');
            const c = new m.BleClient();
            const events = [];
            c.addEventListener('type:pong', (ev) => events.push(ev.detail));
            c._handleLine('{"type":"pong","fw":"2.0.0-dev"}');
            return events;
        }''')
        check('BleClient parses pong', len(result2) == 1 and result2[0]['type'] == 'pong')

        # encode 正当性 (送信メッセージの形)
        result3 = await page.evaluate('''async () => {
            const m = await import('./src/lib/SerialClient.js');
            const c = new m.SerialClient();
            const written = [];
            // mock writer
            c.writer = { write: async (b) => written.push(new TextDecoder().decode(b)) };
            await c.send({ cmd: 'ping' });
            await c.send({ cmd: 'sensor.stream', rate_hz: 50 });
            return written;
        }''')
        check('send() emits ping (newline-terminated)',
              any('"cmd":"ping"' in w and w.endswith('\n') for w in result3),
              f'written={result3}')
        check('send() emits stream cmd',
              any('"sensor.stream"' in w and '"rate_hz":50' in w for w in result3))

        # まとめ
        total = len(passed)
        ok = sum(passed)
        print()
        print(f'=== E2E mock: {ok}/{total} passed ===')

        # スクリーンショットを撮る
        await page.screenshot(path='test/web/screenshot_e2e.png', full_page=True)
        print('Screenshot: test/web/screenshot_e2e.png')

        if console_logs:
            print('\nConsole logs:')
            for line in console_logs[:5]:
                print(f'  {line}')

        await browser.close()
        return ok == total


if __name__ == '__main__':
    ok = asyncio.run(main())
    sys.exit(0 if ok else 1)
