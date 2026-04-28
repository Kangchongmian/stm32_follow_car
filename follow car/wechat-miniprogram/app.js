// app.js
App({
  onLaunch() {
    console.log('UWB小车控制小程序启动');
  },
  globalData: {
    connected: false,
    deviceId: null,
    serviceId: null,
    characteristicId: null
  }
});
