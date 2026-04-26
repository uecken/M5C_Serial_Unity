// Burst Motion - BleClient
// Web Bluetooth API + BLE NUS (Nordic UART Service)
// SerialClient と同じインタフェース (connect/disconnect/send/request/event)

const NUS_SERVICE  = '6e400001-b5a3-f393-e0a9-e50e24dccca9e'.replace(/c$/, '9'); // typo guard
const NUS_TX_CHAR  = '6e400003-b5a3-f393-e0a9-e50e24dccca9e';
const NUS_RX_CHAR  = '6e400002-b5a3-f393-e0a9-e50e24dccca9e';

// 正しい UUID (NUS 標準)
const NUS = {
  SERVICE: '6e400001-b5a3-f393-e0a9-e50e24dcca9e',
  TX:      '6e400003-b5a3-f393-e0a9-e50e24dcca9e',  // FW → Web (notify)
  RX:      '6e400002-b5a3-f393-e0a9-e50e24dcca9e',  // Web → FW (write)
};

export class BleClient extends EventTarget {
  constructor() {
    super();
    this.device = null;
    this.server = null;
    this.txChar = null;
    this.rxChar = null;
    this.connected = false;
    this.buffer = '';
  }

  async connect() {
    if (!('bluetooth' in navigator)) {
      throw new Error('Web Bluetooth API is not supported. Use Chrome/Edge on Desktop or Chrome Android.');
    }
    this.device = await navigator.bluetooth.requestDevice({
      filters: [
        { services: [NUS.SERVICE] },
        { namePrefix: 'Burst' },
        { namePrefix: 'Motion' },
      ],
      optionalServices: [NUS.SERVICE],
    });

    this.device.addEventListener('gattserverdisconnected', this._onDisconnected.bind(this));

    this.server = await this.device.gatt.connect();
    const service = await this.server.getPrimaryService(NUS.SERVICE);

    this.txChar = await service.getCharacteristic(NUS.TX);
    this.rxChar = await service.getCharacteristic(NUS.RX);

    // notify enable
    await this.txChar.startNotifications();
    this.txChar.addEventListener('characteristicvaluechanged', this._onTxData.bind(this));

    this.connected = true;
    this.dispatchEvent(new CustomEvent('connected', { detail: { name: this.device.name } }));
  }

  async disconnect() {
    this.connected = false;
    try {
      if (this.txChar) {
        await this.txChar.stopNotifications();
      }
    } catch (_) {}
    try {
      if (this.server && this.server.connected) {
        this.server.disconnect();
      }
    } catch (_) {}
    this.device = null;
    this.server = null;
    this.txChar = null;
    this.rxChar = null;
    this.dispatchEvent(new CustomEvent('disconnected'));
  }

  _onDisconnected() {
    this.connected = false;
    this.dispatchEvent(new CustomEvent('disconnected'));
  }

  _onTxData(event) {
    const value = event.target.value; // DataView
    const bytes = new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
    const text = new TextDecoder().decode(bytes);
    this.buffer += text;
    let idx;
    while ((idx = this.buffer.indexOf('\n')) >= 0) {
      const line = this.buffer.substring(0, idx).replace(/\r$/, '');
      this.buffer = this.buffer.substring(idx + 1);
      if (line.length === 0) continue;
      this._handleLine(line);
    }
  }

  _handleLine(line) {
    this.dispatchEvent(new CustomEvent('raw', { detail: line }));
    let msg;
    try {
      msg = JSON.parse(line);
    } catch (err) {
      this.dispatchEvent(new CustomEvent('unparsed', { detail: line }));
      return;
    }
    this.dispatchEvent(new CustomEvent('message', { detail: msg }));
    if (msg.type) {
      this.dispatchEvent(new CustomEvent(`type:${msg.type}`, { detail: msg }));
    }
  }

  async send(obj) {
    if (!this.rxChar) throw new Error('Not connected');
    const text = JSON.stringify(obj) + '\n';
    const bytes = new TextEncoder().encode(text);
    // BLE MTU は 23 (default)、ESP32 で拡張可能。安全に 20 byte 区切り。
    const CHUNK = 20;
    for (let off = 0; off < bytes.length; off += CHUNK) {
      const slice = bytes.slice(off, off + CHUNK);
      // writeValueWithoutResponse があれば速い、なければ writeValue
      if (this.rxChar.writeValueWithoutResponse) {
        await this.rxChar.writeValueWithoutResponse(slice);
      } else {
        await this.rxChar.writeValue(slice);
      }
    }
    this.dispatchEvent(new CustomEvent('sent', { detail: obj }));
  }

  async request(cmdObj, expectedType, timeoutMs = 3000) {
    return new Promise((resolve, reject) => {
      const handler = (ev) => {
        this.removeEventListener(`type:${expectedType}`, handler);
        clearTimeout(tid);
        resolve(ev.detail);
      };
      const tid = setTimeout(() => {
        this.removeEventListener(`type:${expectedType}`, handler);
        reject(new Error(`request timeout (cmd=${cmdObj.cmd}, expected type=${expectedType})`));
      }, timeoutMs);
      this.addEventListener(`type:${expectedType}`, handler);
      this.send(cmdObj).catch((err) => {
        clearTimeout(tid);
        this.removeEventListener(`type:${expectedType}`, handler);
        reject(err);
      });
    });
  }
}
