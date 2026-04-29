// pages/index/index.js
const ble = require('../../utils/ble.js');

Page({
  data: {
    activeTab: 'control', // control | params

    // ---- 蓝牙状态 ----
    bleState: 'idle',        // idle | scanning | found | connecting | connected | failed
    bleStateText: '未连接',
    devices: [],
    selectedDevice: null,
    scanProgress: 0,

    // ---- 参数 ----
    params: [
      { key: 'FOLLOW_DISTANCE_M',      label: '跟随距离 (m)',       value: 1.0,  min: 0.3, max: 5.0,  step: 0.1, index: 0 },
      { key: 'OBSTACLE_DIST_CM',       label: '超声波避障距离 (cm)', value: 30,   min: 10,  max: 100,  step: 5,   index: 1 },
      { key: 'LIDAR_OBSTACLE_DIST_MM', label: '雷达避障距离 (mm)',   value: 300,  min: 100, max: 1000, step: 50,  index: 2 },
      { key: 'MOTOR_BASE_SPEED',       label: '基准电机速度',        value: 500,  min: 0,   max: 999,  step: 10,  index: 3 },
      { key: 'MOTOR_TURN_SPEED',       label: '转弯外侧轮速度',     value: 700,  min: 0,   max: 999,  step: 10,  index: 4 },
      { key: 'MOTOR_SLOW_SPEED',       label: '转弯内侧轮速度',     value: 200,  min: 0,   max: 999,  step: 10,  index: 5 },
      { key: 'UWB_TIMEOUT_MS',         label: 'UWB数据超时 (ms)',    value: 500,  min: 100, max: 2000, step: 50,  index: 6 },
      { key: 'ULTRASONIC_POLL_MS',     label: '超声波轮询间隔 (ms)', value: 50,   min: 10,  max: 500,  step: 10,  index: 7 },
      { key: 'UWB_ANGLE_TOLERANCE_DEG', label: 'UWB角度容许值 (°)',  value: 20,   min: 0,   max: 90,   step: 1,   index: 8 },
      { key: 'EMERGENCY_STOP_DIST_CM',  label: '紧急停车距离 (cm)',   value: 15,   min: 5,   max: 80,   step: 1,   index: 9 },
      { key: 'PID_DIST_KP',             label: '距离PID Kp',         value: 300,  min: 0,   max: 1000, step: 1,   index: 10 },
      { key: 'PID_DIST_KI',             label: '距离PID Ki',         value: 5,    min: 0,   max: 100,  step: 0.1, index: 11 },
      { key: 'PID_DIST_KD',             label: '距离PID Kd',         value: 80,   min: 0,   max: 500,  step: 1,   index: 12 },
      { key: 'PID_ANGLE_KP',            label: '角度PID Kp',         value: 4,    min: 0,   max: 50,   step: 0.1, index: 13 },
      { key: 'PID_ANGLE_KI',            label: '角度PID Ki',         value: 0.3,  min: 0,   max: 10,   step: 0.1, index: 14 },
      { key: 'PID_ANGLE_KD',            label: '角度PID Kd',         value: 1.5,  min: 0,   max: 50,   step: 0.1, index: 15 }
    ],

    // ---- 控制 ----
    joyX: 0,
    joyY: 0,
    joyThumbX: 0,   // px, 居中初始化在onReady
    joyThumbY: 0
  },

  // =============== 生命周期 ===============
  onLoad() {
    // 页面加载
  },

  onReady() {
    // 获取摇杆底盘尺寸并居中手柄
    this._initJoystick();
  },

  onUnload() {
    this._closeBLE();
  },

  onSwitchTab(e) {
    const tab = e.currentTarget.dataset.tab;
    if (!tab || tab === this.data.activeTab) return;
    this.setData({ activeTab: tab });
    if (tab === 'control') {
      setTimeout(() => this._initJoystick(), 50);
    }
  },

  noop() {},

  // =============== 蓝牙流程 ===============
  onTapConnect() {
    this._startScan();
  },

  onTapReconnect() {
    this._closeBLE();
    setTimeout(() => this._startScan(), 500);
  },

  onTapStopScan() {
    this._stopScan();
    const count = this.data.devices.length;
    if (count > 0) {
      this.setData({
        bleState: 'found',
        bleStateText: `搜索已停止，共 ${count} 个设备，请选择`,
        scanProgress: 100
      });
    } else {
      this.setData({
        bleState: 'failed',
        bleStateText: '搜索已停止，未发现设备',
        scanProgress: 100
      });
    }
  },

  _startScan() {
    this.setData({ bleState: 'scanning', bleStateText: '正在初始化蓝牙...', devices: [], scanProgress: 0 });

    wx.openBluetoothAdapter({
      success: () => {
        this.setData({ bleStateText: '蓝牙已开启，正在搜索设备...' });
        this._doScan();
      },
      fail: (err) => {
        console.error('openBluetoothAdapter fail', err);
        this.setData({ bleState: 'failed', bleStateText: '蓝牙初始化失败，请检查蓝牙是否开启' });
      }
    });
  },

  _doScan() {
    // 进度模拟
    let progress = 0;
    this._scanTimer = setInterval(() => {
      progress = Math.min(progress + 5, 90);
      this.setData({ scanProgress: progress });
    }, 500);

    wx.startBluetoothDevicesDiscovery({
      allowDuplicatesKey: true,   // 允许重复上报，这样能捕获后续带名称的广播
      success: () => {
        // 监听发现设备 — 显示所有蓝牙设备，不做名称过滤
        wx.onBluetoothDeviceFound((res) => {
          const current = this.data.devices;
          let changed = false;

          res.devices.forEach(d => {
            const name = d.name || d.localName || '';
            const rssi = d.RSSI || 0;

            // 过滤信号太弱的设备 (低于 -90dBm 基本不可用)
            if (rssi < -90 && rssi !== 0) return;

            // 查找是否已在列表中
            const existIdx = current.findIndex(cd => cd.deviceId === d.deviceId);

            if (existIdx >= 0) {
              // 已存在: 更新名称(如果之前为空现在有了)和信号强度
              const existing = current[existIdx];
              const oldName = existing.name || existing.localName || '';
              if ((!oldName && name) || rssi > existing.RSSI) {
                current[existIdx] = Object.assign({}, existing, {
                  name: name || existing.name,
                  localName: d.localName || existing.localName,
                  RSSI: rssi
                });
                changed = true;
              }
            } else {
              // 新设备: 加入列表
              current.push(d);
              changed = true;
            }
          });

          if (changed) {
            // 按信号强度排序 (强的在前)，有名称的优先
            current.sort((a, b) => {
              const aName = a.name || a.localName || '';
              const bName = b.name || b.localName || '';
              // 有名称的排前面
              if (aName && !bName) return -1;
              if (!aName && bName) return 1;
              // 同类按信号强度降序
              return (b.RSSI || -100) - (a.RSSI || -100);
            });

            this.setData({
              devices: current,
              bleState: 'found',
              bleStateText: `找到 ${current.length} 个设备，请选择`
            });
          }
        });

        // 15 秒后自动停止搜索 (但不关闭列表，用户仍可选择)
        this._scanTimeout = setTimeout(() => {
          this._stopScan();
          if (this.data.devices.length === 0) {
            this.setData({ bleState: 'failed', bleStateText: '搜索超时，未发现任何蓝牙设备', scanProgress: 100 });
          } else {
            this.setData({
              scanProgress: 100,
              bleStateText: `搜索完成，共 ${this.data.devices.length} 个设备`
            });
          }
        }, 15000);
      },
      fail: (err) => {
        console.error('startDiscovery fail', err);
        clearInterval(this._scanTimer);
        this.setData({ bleState: 'failed', bleStateText: '搜索设备失败，请确认蓝牙和定位权限已开启' });
      }
    });
  },

  _stopScan() {
    clearInterval(this._scanTimer);
    clearTimeout(this._scanTimeout);
    wx.stopBluetoothDevicesDiscovery({ complete() {} });
  },

  onTapDevice(e) {
    const device = e.currentTarget.dataset.device;
    this._stopScan();
    this._connectDevice(device);
  },

  _connectDevice(device) {
    this.setData({
      bleState: 'connecting',
      bleStateText: `正在连接 ${device.name || device.localName}...`,
      selectedDevice: device
    });

    wx.createBLEConnection({
      deviceId: device.deviceId,
      timeout: 8000,
      success: () => {
        this._deviceId = device.deviceId;
        this._discoverServices();
      },
      fail: (err) => {
        console.error('createBLEConnection fail', err);
        this.setData({ bleState: 'failed', bleStateText: '连接失败，请重试' });
      }
    });
  },

  _discoverServices() {
    wx.getBLEDeviceServices({
      deviceId: this._deviceId,
      success: (res) => {
        // 查找 FFE0 服务
        const svc = res.services.find(s =>
          s.uuid.toUpperCase().indexOf('FFE0') >= 0
        );
        if (!svc) {
          this.setData({ bleState: 'failed', bleStateText: '未找到JDY-33服务' });
          return;
        }
        this._serviceId = svc.uuid;
        this._discoverCharacteristics();
      },
      fail: () => {
        this.setData({ bleState: 'failed', bleStateText: '获取服务失败' });
      }
    });
  },

  _discoverCharacteristics() {
    wx.getBLEDeviceCharacteristics({
      deviceId: this._deviceId,
      serviceId: this._serviceId,
      success: (res) => {
        const ch = res.characteristics.find(c =>
          c.uuid.toUpperCase().indexOf('FFE1') >= 0
        );
        if (!ch) {
          this.setData({ bleState: 'failed', bleStateText: '未找到通信特征值' });
          return;
        }
        this._characteristicId = ch.uuid;

        // 开启 notify 监听回传数据
        wx.notifyBLECharacteristicValueChange({
          deviceId: this._deviceId,
          serviceId: this._serviceId,
          characteristicId: this._characteristicId,
          state: true,
          success: () => {
            wx.onBLECharacteristicValueChange((r) => {
              this._onReceive(r.value);
            });
          }
        });

        // 保存到全局
        const app = getApp();
        app.globalData.connected = true;
        app.globalData.deviceId = this._deviceId;
        app.globalData.serviceId = this._serviceId;
        app.globalData.characteristicId = this._characteristicId;

        this.setData({
          bleState: 'connected',
          bleStateText: '蓝牙连接成功'
        });

        wx.showToast({ title: '连接成功', icon: 'success' });
      },
      fail: () => {
        this.setData({ bleState: 'failed', bleStateText: '获取特征值失败' });
      }
    });

    // 监听断开
    wx.onBLEConnectionStateChange((res) => {
      if (!res.connected) {
        this.setData({ bleState: 'failed', bleStateText: '蓝牙连接已断开' });
        getApp().globalData.connected = false;
      }
    });
  },

  _closeBLE() {
    if (this._deviceId) {
      wx.closeBLEConnection({ deviceId: this._deviceId, complete() {} });
    }
    wx.closeBluetoothAdapter({ complete() {} });
    this._deviceId = null;
    this._serviceId = null;
    this._characteristicId = null;
    getApp().globalData.connected = false;
  },

  // =============== 数据发送 ===============
  _send(buffer) {
    if (this.data.bleState !== 'connected') {
      wx.showToast({ title: '蓝牙未连接', icon: 'none' });
      return;
    }
    wx.writeBLECharacteristicValue({
      deviceId: this._deviceId,
      serviceId: this._serviceId,
      characteristicId: this._characteristicId,
      value: buffer,
      fail: (err) => {
        console.error('BLE write fail', err);
        wx.showToast({ title: '发送失败', icon: 'none' });
      }
    });
  },

  _onReceive(buffer) {
    const frame = ble.parseFrame(buffer);
    if (!frame) return;
    if (frame.cmd === ble.CMD.PARAM_ACK && frame.data.length >= 5) {
      const idx = frame.data[0];
      const view = new DataView(frame.data.buffer, frame.data.byteOffset + 1, 4);
      const val = view.getFloat32(0, true);
      const params = this.data.params.slice();
      const p = params.find(pp => pp.index === idx);
      if (p) {
        p.value = parseFloat(val.toFixed(2));
        this.setData({ params });
      }
    }
  },

  // =============== 参数修改 ===============
  onParamChange(e) {
    const idx = e.currentTarget.dataset.idx;
    const val = parseFloat(e.detail.value);
    const params = this.data.params.slice();
    params[idx].value = val;
    this.setData({ params });
  },

  onParamChanging(e) {
    const idx = e.currentTarget.dataset.idx;
    const val = parseFloat(e.detail.value);
    const params = this.data.params.slice();
    params[idx].value = val;
    this.setData({ params });
  },

  onParamConfirm(e) {
    const idx = parseInt(e.currentTarget.dataset.idx);
    const param = this.data.params[idx];
    const buf = ble.buildSetParamFrame(param.index, param.value);
    this._send(buf);
    wx.showToast({ title: `${param.label} 已发送`, icon: 'none', duration: 1000 });
  },

  onSendAllParams() {
    if (this.data.bleState !== 'connected') {
      wx.showToast({ title: '蓝牙未连接', icon: 'none' });
      return;
    }
    const params = this.data.params;
    let i = 0;
    const sendNext = () => {
      if (i >= params.length) {
        wx.showToast({ title: '全部参数已发送', icon: 'success' });
        return;
      }
      const buf = ble.buildSetParamFrame(params[i].index, params[i].value);
      this._send(buf);
      i++;
      setTimeout(sendNext, 100); // 间隔100ms逐个发送，避免BLE拥塞
    };
    sendNext();
  },

  // =============== 摇杆控制 ===============
  _initJoystick() {
    const query = wx.createSelectorQuery().in(this);
    query.select('#joystickBase').boundingClientRect((rect) => {
      if (!rect) return;
      this._joyBaseX = rect.left;
      this._joyBaseY = rect.top;
      this._joyRadius = rect.width / 2;
      // 手柄初始居中 (px值, 用于style绑定)
      const cx = rect.width / 2;
      const cy = rect.height / 2;
      this.setData({ joyThumbX: cx, joyThumbY: cy });
    }).exec();
  },

  onJoyStart(e) {
    // 重新获取位置(防止页面滚动后偏移)
    const query = wx.createSelectorQuery().in(this);
    query.select('#joystickBase').boundingClientRect((rect) => {
      if (!rect) return;
      this._joyBaseX = rect.left;
      this._joyBaseY = rect.top;
      this._joyRadius = rect.width / 2;
      this._joyActive = true;

      // 启动定时发送 (100ms间隔)
      this._joySendTimer = setInterval(() => {
        if (!this._joyActive) return;
        this._send(ble.buildJoystickFrame(this.data.joyX, this.data.joyY));
      }, 100);

      // 处理当前触摸点
      this._updateJoyPos(e.touches[0]);
    }).exec();
  },

  onJoyMove(e) {
    if (!this._joyActive) return;
    this._updateJoyPos(e.touches[0]);
  },

  onJoyEnd() {
    this._joyActive = false;
    clearInterval(this._joySendTimer);
    this._joySendTimer = null;

    // 回中
    const cx = this._joyRadius || 75;
    this.setData({ joyX: 0, joyY: 0, joyThumbX: cx, joyThumbY: cx });
    // 发送停止
    this._send(ble.buildJoystickFrame(0, 0));
  },

  _updateJoyPos(touch) {
    const r = this._joyRadius || 75;
    // 触摸点相对底盘中心的偏移
    let dx = touch.clientX - this._joyBaseX - r;
    let dy = touch.clientY - this._joyBaseY - r;

    // 限制在圆形范围内
    const dist = Math.sqrt(dx * dx + dy * dy);
    const maxR = r * 0.85;  // 留一点边距
    if (dist > maxR) {
      dx = dx / dist * maxR;
      dy = dy / dist * maxR;
    }

    // 转换为 -100 ~ +100
    const joyX = Math.round(dx / maxR * 100);   // 右正左负
    const joyY = Math.round(-dy / maxR * 100);   // 上正(前进)下负(后退)

    // 手柄位置 (px)
    const thumbX = r + dx;
    const thumbY = r + dy;

    this.setData({ joyX, joyY, joyThumbX: thumbX, joyThumbY: thumbY });
  }
});
