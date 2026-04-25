# Burst Motion Web 設定アプリ — 起動方法

Phase 1 MVP。Preact + htm + Tailwind CDN、Web Serial API。
ビルド不要、`python serve_https.py` で起動。

## 動作要件

- **Chrome / Edge** Desktop または **Chrome 148+** Android (Web Serial API 対応)
- M5C を **COM8 など Serial 経由で接続** (FW v2 必要、`pio run -e m5stick-c-v2 -t upload`)
- 接続 baud は **115200**（FW 側 platformio.ini `SERIAL_BAUD=115200` と一致）

## ローカル PC からアクセス (HTTP で OK)

```bash
cd Web/hidconfig
python -m http.server 8000
# → http://localhost:8000/ を Chrome で開く
```

`localhost` は HTTP でも Web Serial が動く。

## LAN 上のスマホ等からアクセス (HTTPS 必須)

### 初回セットアップ

自己署名証明書を生成（既に `cert.pem`, `key.pem` がある場合は不要）:

```bash
cd Web/hidconfig
python -c "
from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
import datetime, ipaddress, socket

# ※ 自分の LAN IP を確認
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('8.8.8.8', 80))
LAN_IP = s.getsockname()[0]
s.close()
print(f'LAN IP: {LAN_IP}')

key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
subject = issuer = x509.Name([
    x509.NameAttribute(NameOID.COUNTRY_NAME, 'JP'),
    x509.NameAttribute(NameOID.COMMON_NAME, 'Burst Motion Dev')])
cert = (x509.CertificateBuilder()
    .subject_name(subject).issuer_name(issuer)
    .public_key(key.public_key()).serial_number(x509.random_serial_number())
    .not_valid_before(datetime.datetime.utcnow())
    .not_valid_after(datetime.datetime.utcnow() + datetime.timedelta(days=365))
    .add_extension(x509.SubjectAlternativeName([
        x509.DNSName('localhost'),
        x509.IPAddress(ipaddress.ip_address('127.0.0.1')),
        x509.IPAddress(ipaddress.ip_address(LAN_IP))]), critical=False)
    .sign(key, hashes.SHA256()))
open('cert.pem','wb').write(cert.public_bytes(serialization.Encoding.PEM))
open('key.pem','wb').write(key.private_bytes(serialization.Encoding.PEM,
    serialization.PrivateFormat.TraditionalOpenSSL, serialization.NoEncryption()))
"
```

### サーバー起動

```bash
cd Web/hidconfig
python serve_https.py 8443
```

出力:
```
============================================================
 Burst Motion Web Config — HTTPS Server
============================================================
  Local:  https://localhost:8443/
  LAN:    https://192.168.0.111:8443/
```

### スマホでの接続手順 (2 通り)

**A. USB OTG 経由 (Web Serial)**:
1. PC とスマホが**同一 LAN** であることを確認
2. スマホ Chrome で **`https://<PC の LAN IP>:8443/`** を開く
3. 「**接続がプライベートではありません**」警告 → 「詳細設定」 → 「<IP> にアクセスする (安全ではありません)」
4. アプリが表示される
5. **USB OTG ケーブル**で M5C をスマホに接続
6. 「📡 USB Serial」タブ → 「🔌 USB Serial で接続」 → デバイス選択
7. Stream ON で IMU データが見える

**B. BLE NUS 経由 (ケーブルレス、Phase 2 で追加)** 🆕
1. **事前に**: PC で USB 接続後 `{"cmd":"ble.start"}` を送って Controller の BLE HID + NUS を起動 (Web UI の「BLE Start」ボタンで OK)
2. USB ケーブルを抜く (任意、繋いだままでも BLE は別経路で機能)
3. スマホ Chrome で `https://<PC の LAN IP>:8443/` を開く
4. 「📶 BLE」タブ → 「🔌 BLE NUS で接続」
5. ブラウザのデバイス選択画面で **"Burst Motion"** を選ぶ
6. 接続後、同じ JSON Lines プロトコルで全コマンド使用可能
7. **iPhone Safari は Web Bluetooth 非対応**、Chrome Android で動作

#### BLE NUS の仕様 (技術メモ)
- Service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e` (Nordic 標準)
- TX (notify, FW→Web): `6e400003-...`
- RX (write, Web→FW): `6e400002-...`
- BLE HID と同一ペリフェラルに同居 (NimBLE 単一サーバー)
- Web Bluetooth API は HTTPS 必須、Chrome/Edge 対応

### Windows ファイアウォール

スマホから接続できない場合、Windows ファイアウォールが TCP 8443 を遮断している可能性。
**管理者として PowerShell** を開き:

```powershell
netsh advfirewall firewall add rule name="Burst Motion HTTPS 8443" dir=in action=allow protocol=TCP localport=8443 profile=private
```

または「Windows セキュリティ → ファイアウォールとネットワーク保護 → 詳細設定 → 受信の規則 → 新しい規則」で TCP 8443 を許可。

## 構成

```
Web/hidconfig/
├── index.html        # シェル、Tailwind CDN + ES Module importmap
├── serve_https.py    # HTTPS サーバー (LAN 用)
├── cert.pem          # 自己署名証明書 (.gitignore 推奨)
├── key.pem           # 秘密鍵         (.gitignore 必須)
├── README.md         # 本ファイル
├── src/
│   ├── app.js        # Preact App
│   └── lib/
│       └── SerialClient.js  # Web Serial + JSON Lines
└── profiles/         # サンプルプロファイル (空)
```

## トラブルシュート

| 症状 | 原因 | 対処 |
|------|------|------|
| 「Web Serial API is not supported」 | Firefox/Safari、または HTTP over LAN | Chrome/Edge を使う、HTTPS 化 |
| ボタン押しても COM が出ない | デバイス未接続、ドライバ不足 | デバイスマネージャで COM8 等を確認 |
| 接続後 ping 応答なし | FW 未書込 or boot loop | `pio run -e m5stick-c-v2 -t upload` 後、物理電源ボタン 6 秒で reset |
| LAN IP が変わった | DHCP 更新 | cert.pem を再生成（SAN に新 IP 含めて） |
