// Burst Motion - SerialClient
// Web Serial API + JSON Lines protocol
// 自動再接続: 初回認可後、navigator.serial.getPorts() で記憶ポートを再利用

// M5C / Burst Motion Controller の USB-Serial 候補 VID/PID
// FTDI FT231 (M5StickC 一部の OEM 変換): 0403:6001, 0403:6015
// CP210x (M5StickC 標準): 10C4:EA60
// CH9102 (M5StickC 新): 1A86:55D4
// CH340: 1A86:7523
// ESP32-S3 native USB: 303A:1001 (esp32) / 303A:0002 / 303A:1006
const KNOWN_USB_FILTERS = [
  { usbVendorId: 0x0403 },   // FTDI
  { usbVendorId: 0x10C4 },   // SiLabs (CP210x)
  { usbVendorId: 0x1A86 },   // QinHeng (CH340 / CH9102)
  { usbVendorId: 0x303A },   // Espressif native
];

export class SerialClient extends EventTarget {
  constructor() {
    super();
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.readLoopPromise = null;
    this.buffer = '';
    this.connected = false;
  }

  /**
   * 既に認可されたポートを取得 (chooser を出さない)
   * 初回接続後、ブラウザがポート許可を記憶していれば次回からこれが返る。
   * @returns {Promise<SerialPort[]>}
   */
  async getAuthorizedPorts() {
    if (!('serial' in navigator)) return [];
    return await navigator.serial.getPorts();
  }

  /**
   * 自動接続を試みる。認可済みポートが 1 つでもあれば接続成功、無ければ false。
   * 自動接続後の baudRate を引数で指定。
   */
  async autoConnect(baudRate = 115200) {
    const ports = await this.getAuthorizedPorts();
    if (ports.length === 0) return false;
    // 複数ある場合は最初のもの (通常 1 つ)
    this.port = ports[0];
    await this._openPort(baudRate);
    return true;
  }

  /**
   * ユーザー操作で chooser を開いて接続。
   * @param {number} baudRate
   * @param {boolean} useFilter VID/PID フィルタで M5C 系のみ表示
   */
  async connect(baudRate = 115200, useFilter = true) {
    if (!('serial' in navigator)) {
      throw new Error('Web Serial API is not supported. Use Chrome/Edge on Desktop, or Chrome 148+ on Android.');
    }
    const opts = useFilter ? { filters: KNOWN_USB_FILTERS } : {};
    this.port = await navigator.serial.requestPort(opts);
    await this._openPort(baudRate);
  }

  async _openPort(baudRate) {
    await this.port.open({ baudRate });
    this.writer = this.port.writable.getWriter();
    this.connected = true;
    // 切断検知
    if (navigator.serial && !this._disconnectListenerAdded) {
      navigator.serial.addEventListener('disconnect', (ev) => {
        if (ev.target === this.port) {
          this.disconnect();
        }
      });
      this._disconnectListenerAdded = true;
    }
    // ポート情報をイベントで通知 (UI 表示用)
    let info = null;
    try { info = this.port.getInfo(); } catch (_) {}
    this.dispatchEvent(new CustomEvent('connected', { detail: { info } }));
    this.readLoopPromise = this._readLoop();
  }

  /**
   * ユーザー操作なしで全認可済みポートをクリア (デバッグ用)
   */
  async forgetAllPorts() {
    const ports = await this.getAuthorizedPorts();
    for (const p of ports) {
      try { await p.forget(); } catch (_) {}
    }
  }

  async disconnect() {
    this.connected = false;
    try {
      if (this.reader) {
        await this.reader.cancel();
        await this.reader.releaseLock();
      }
    } catch (_) {}
    try {
      if (this.writer) {
        await this.writer.releaseLock();
      }
    } catch (_) {}
    try {
      if (this.port) await this.port.close();
    } catch (_) {}
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.dispatchEvent(new CustomEvent('disconnected'));
  }

  async _readLoop() {
    const decoder = new TextDecoderStream();
    const readableStreamClosed = this.port.readable.pipeTo(decoder.writable);
    this.reader = decoder.readable.getReader();

    try {
      while (this.connected) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (!value) continue;
        this.buffer += value;
        let idx;
        while ((idx = this.buffer.indexOf('\n')) >= 0) {
          const line = this.buffer.substring(0, idx).replace(/\r$/, '');
          this.buffer = this.buffer.substring(idx + 1);
          if (line.length === 0) continue;
          this._handleLine(line);
        }
      }
    } catch (err) {
      console.error('[SerialClient] read loop error:', err);
      this.dispatchEvent(new CustomEvent('error', { detail: err }));
    }
    try { await readableStreamClosed; } catch (_) {}
  }

  _handleLine(line) {
    // 生ログ通知 (デバッグペイン用)
    this.dispatchEvent(new CustomEvent('raw', { detail: line }));

    // JSON 試行
    let msg;
    try {
      msg = JSON.parse(line);
    } catch (err) {
      this.dispatchEvent(new CustomEvent('unparsed', { detail: line }));
      return;
    }
    this.dispatchEvent(new CustomEvent('message', { detail: msg }));
    const type = msg.type;
    if (type) {
      this.dispatchEvent(new CustomEvent(`type:${type}`, { detail: msg }));
    }
  }

  async send(obj) {
    if (!this.writer) throw new Error('Not connected');
    const text = JSON.stringify(obj) + '\n';
    const bytes = new TextEncoder().encode(text);
    await this.writer.write(bytes);
    this.dispatchEvent(new CustomEvent('sent', { detail: obj }));
  }

  // 簡易: コマンド送信 + 応答待ち (type フィルタ)
  async request(cmdObj, expectedType, timeoutMs = 2000) {
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
