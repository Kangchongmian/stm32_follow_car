/**
 * BLE 蓝牙通信工具 — 适配 JDY-33 双模蓝牙模块
 * BLE Service UUID: FFE0, Characteristic UUID: FFE1
 */

const JDY_SERVICE_UUID   = '0000FFE0-0000-1000-8000-00805F9B34FB';
const JDY_CHAR_UUID      = '0000FFE1-0000-1000-8000-00805F9B34FB';
const JDY_NAME_PREFIX    = 'JDY';

/**
 * 通信协议定义
 * 帧格式:  HEAD(0xAA) + CMD(1B) + LEN(1B) + DATA(nB) + CHECKSUM(1B)
 *
 * CMD 列表:
 *   0x01  设置参数   DATA = paramIndex(1B) + value(4B float LE)
 *   0x02  摇杆控制   DATA = x(int8) + y(int8)  范围 -100~+100
 *         x: 左负右正, y: 后负前正, (0,0)=停止
 *   0x03  读取参数   DATA = paramIndex(1B)
 *   0x04  参数应答   DATA = paramIndex(1B) + value(4B float LE)
 */

const CMD = {
  SET_PARAM:  0x01,
  JOYSTICK:   0x02,
  READ_PARAM: 0x03,
  PARAM_ACK:  0x04
};

// 参数索引（与小车端保持一致）
const PARAM_INDEX = {
  FOLLOW_DISTANCE_M:      0,
  OBSTACLE_DIST_CM:       1,
  LIDAR_OBSTACLE_DIST_MM: 2,
  MOTOR_BASE_SPEED:       3,
  MOTOR_TURN_SPEED:       4,
  MOTOR_SLOW_SPEED:       5,
  UWB_TIMEOUT_MS:         6,
  ULTRASONIC_POLL_MS:     7,
  UWB_ANGLE_TOLERANCE_DEG: 8,
  EMERGENCY_STOP_DIST_CM:  9,
  PID_DIST_KP:             10,
  PID_DIST_KI:             11,
  PID_DIST_KD:             12,
  PID_ANGLE_KP:            13,
  PID_ANGLE_KI:            14,
  PID_ANGLE_KD:            15
};

/**
 * 计算校验和 (XOR)
 */
function checksum(bytes) {
  let cs = 0;
  for (let i = 0; i < bytes.length; i++) {
    cs ^= bytes[i];
  }
  return cs & 0xFF;
}

/**
 * 构建发送帧
 */
function buildFrame(cmd, data) {
  const len = data.length;
  const frame = new Uint8Array(3 + len + 1); // HEAD + CMD + LEN + DATA + CS
  frame[0] = 0xAA;
  frame[1] = cmd;
  frame[2] = len;
  for (let i = 0; i < len; i++) {
    frame[3 + i] = data[i];
  }
  frame[3 + len] = checksum(frame.slice(0, 3 + len));
  return frame.buffer;
}

/**
 * 将 float 转为 4 字节 Little-Endian
 */
function floatToBytes(val) {
  const buf = new ArrayBuffer(4);
  new DataView(buf).setFloat32(0, val, true);
  return new Uint8Array(buf);
}

/**
 * 构建设置参数帧
 */
function buildSetParamFrame(paramIndex, value) {
  const valBytes = floatToBytes(value);
  const data = new Uint8Array(5);
  data[0] = paramIndex;
  data.set(valBytes, 1);
  return buildFrame(CMD.SET_PARAM, data);
}

/**
 * 构建摇杆控制帧
 * @param {number} x  左右 -100~+100 (左负右正)
 * @param {number} y  前后 -100~+100 (后负前正)
 */
function buildJoystickFrame(x, y) {
  // 将有符号 int8 转为无符号字节
  const bx = (x < 0) ? (256 + x) : x;
  const by = (y < 0) ? (256 + y) : y;
  return buildFrame(CMD.JOYSTICK, new Uint8Array([bx & 0xFF, by & 0xFF]));
}

/**
 * 解析接收帧
 */
function parseFrame(buffer) {
  const bytes = new Uint8Array(buffer);
  if (bytes.length < 4 || bytes[0] !== 0xAA) return null;
  const cmd = bytes[1];
  const len = bytes[2];
  if (bytes.length !== 3 + len + 1) return null;
  const cs = checksum(bytes.slice(0, 3 + len));
  if (cs !== bytes[3 + len]) return null;
  const data = bytes.slice(3, 3 + len);
  return { cmd, data };
}

module.exports = {
  JDY_SERVICE_UUID,
  JDY_CHAR_UUID,
  JDY_NAME_PREFIX,
  CMD,
  PARAM_INDEX,
  buildSetParamFrame,
  buildJoystickFrame,
  parseFrame,
  floatToBytes
};
