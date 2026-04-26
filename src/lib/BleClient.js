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
    // Phase 5.31: 自動再接続 + 送信直列化
    this.autoReconnect = true;
    this._reconnecting = false;
    this._reconnectAttempt = 0;
    this._sendQueue = Promise.resolve();
  }

  async _gattConnectWithRetry(maxAttempts = 3) {
    let lastErr = null;
    for (let attempt = 1; attempt <= maxAttempts; attempt++) {
      try {
        const server = await this.device.gatt.connect();
        return server;
      } catch (err) {
        lastErr = err;
        // 指数バックオフ: 200ms, 600ms, 1400ms
        const wait = 200 * Math.pow(3, attempt - 1) - 200;
        if (attempt < maxAttempts) {
          this.dispatchEvent(new CustomEvent('reconnect_attempt', { detail: { attempt, error: err.message } }));
          await new Promise((r) => setTimeout(r, wait));
        }
      }
    }
    throw lastErr;
  }

  async _setupServices() {
    const service = await this.server.getPrimaryService(NUS.SERVICE);
    this.txChar = await service.getCharacteristic(NUS.TX);
    this.rxChar = await service.getCharacteristic(NUS.RX);
    await this.txChar.startNotifications();
    this.txChar.addEventListener('characteristicvaluechanged', this._onTxData.bind(this));
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

    this.server = await this._gattConnectWithRetry(3);
    await this._setupServices();

    this.connected = true;
    this._reconnectAttempt = 0;
    this.dispatchEvent(new CustomEvent('connected', { detail: { name: this.device.name } }));
  }

  async disconnect() {
    this.autoReconnect = false;  // 意図的な切断、自動再接続させない
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
    // Phase 5.31: 意図的な disconnect() 以外なら自動再接続を試みる
    if (this.autoReconnect && this.device && !this._reconnecting) {
      this._tryReconnect();
    }
  }

  async _tryReconnect() {
    if (this._reconnecting) return;
    this._reconnecting = true;
    try {
      this._reconnectAttempt++;
      this.dispatchEvent(new CustomEvent('reconnect_attempt', { detail: { attempt: this._reconnectAttempt } }));
      this.server = await this._gattConnectWithRetry(3);
      await this._setupServices();
      this.connected = true;
      this._reconnectAttempt = 0;
      this.dispatchEvent(new CustomEvent('reconnected', { detail: { name: this.device?.name } }));
      // 互換: 既存リスナは 'connected' を期待しているケースに備えて発火
      this.dispatchEvent(new CustomEvent('connected', { detail: { name: this.device?.name } }));
    } catch (err) {
      this.dispatchEvent(new CustomEvent('reconnect_failed', { detail: { error: err.message } }));
    } finally {
      this._reconnecting = false;
    }
  }

  _onTxData(event) {
    const value = event.target.value; // DataView
    const bytes = new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
    const text = new TextDecoder().decode(bytes);
    this.buffer += text;
    // Phase 5.31: 改行が来ない壊れた送信で無限肥大しないよう上限ガード
    const MAX_BUF = 64 * 1024;
    if (this.buffer.length > MAX_BUF) {
      this.dispatchEvent(new CustomEvent('unparsed', { detail: '[buffer overflow, dropped ' + this.buffer.length + 'B]' }));
      this.buffer = '';
      return;
    }
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
    // Phase 5.31: 送信を直列化 (並列 send で書込みが交互に挟まり破損するのを防ぐ)
    //   Promise チェーンで前の送信完了を待つ。失敗しても次の送信ができるよう catch する。
    const task = this._sendQueue.then(() => this._sendInternal(obj));
    this._sendQueue = task.catch(() => {});
    return task;
  }

  async _sendInternal(obj) {
    const text = JSON.stringify(obj) + '\n';
    const bytes = new TextEncoder().encode(text);
    // Phase 5.31: FW Phase 5.31+ で MTU 247 を要求するので、180B chunk を使用
    //   旧 FW (MTU 23) でも writeWithoutResponse の auto-fragment が働くので破綻しない
    //   安全マージンとして 180 (244 上限の 73%)
    const CHUNK = 180;
    for (let off = 0; off < bytes.length; off += CHUNK) {
      const slice = bytes.slice(off, off + CHUNK);
      try {
        if (this.rxChar.writeValueWithoutResponse) {
          await this.rxChar.writeValueWithoutResponse(slice);
        } else {
          await this.rxChar.writeValue(slice);
        }
      } catch (err) {
        // GATT 一時失敗は 1 回だけリトライ
        await new Promise((r) => setTimeout(r, 50));
        if (this.rxChar.writeValueWithoutResponse) {
          await this.rxChar.writeValueWithoutResponse(slice);
        } else {
          await this.rxChar.writeValue(slice);
        }
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
