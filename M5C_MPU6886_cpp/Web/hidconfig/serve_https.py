#!/usr/bin/env python3
"""
Burst Motion Web 設定アプリ - LAN 用 HTTPS サーバー

スマホ等の LAN 端末から Web Serial を使うために HTTPS が必要。
自己署名証明書 (cert.pem, key.pem) を使い、0.0.0.0:8443 で待機。

使い方:
    python serve_https.py [port]

スマホの Chrome で https://<PC の LAN IP>:8443/ にアクセス。
初回は「証明書が信頼できない」警告が出るので「詳細設定 → このサイトに進む」で許可。
"""
import http.server
import ssl
import socket
import sys
import os

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8443
HOST = '0.0.0.0'
CERT = os.path.join(os.path.dirname(__file__), 'cert.pem')
KEY = os.path.join(os.path.dirname(__file__), 'key.pem')


def get_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    finally:
        s.close()


def main():
    # Windows コンソールの cp932 で UTF-8 印字を強制
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    if not os.path.exists(CERT) or not os.path.exists(KEY):
        print(f'ERROR: {CERT} または {KEY} が存在しません。先に証明書を生成してください。')
        sys.exit(1)

    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    handler = http.server.SimpleHTTPRequestHandler
    httpd = http.server.ThreadingHTTPServer((HOST, PORT), handler)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(CERT, KEY)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

    lan_ip = get_lan_ip()
    print('=' * 60)
    print(' Burst Motion Web Config — HTTPS Server')
    print('=' * 60)
    print(f'  Local:  https://localhost:{PORT}/')
    print(f'  LAN:    https://{lan_ip}:{PORT}/')
    print('')
    print('  * スマホは同一 LAN 上で上記 LAN URL を Chrome で開く')
    print('  * 自己署名証明書のため警告が出ます')
    print('    Chrome: 詳細設定 → サイトにアクセス (危険性は承知)')
    print('  * Windows ファイアウォール許可が必要な場合あり')
    print('')
    print('  Ctrl+C で停止')
    print('=' * 60)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\nstopped')


if __name__ == '__main__':
    main()
