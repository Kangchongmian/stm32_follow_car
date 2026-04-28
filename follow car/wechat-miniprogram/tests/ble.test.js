const assert = require('assert');
const ble = require('../utils/ble.js');

function toArray(buf) {
  return Array.from(new Uint8Array(buf));
}

(function testBuildJoystickFrame() {
  const frame = toArray(ble.buildJoystickFrame(-100, 100));
  assert.strictEqual(frame[0], 0xAA);
  assert.strictEqual(frame[1], ble.CMD.JOYSTICK);
  assert.strictEqual(frame[2], 2);
  assert.strictEqual(frame[3], 156); // -100 int8
  assert.strictEqual(frame[4], 100);
})();

(function testParseFrameAcceptsValidFrame() {
  const buf = ble.buildSetParamFrame(3, 123.5);
  const parsed = ble.parseFrame(buf);
  assert(parsed);
  assert.strictEqual(parsed.cmd, ble.CMD.SET_PARAM);
  assert.strictEqual(parsed.data.length, 5);
  assert.strictEqual(parsed.data[0], 3);
})();

(function testParseFrameRejectsTrailingBytes() {
  const valid = new Uint8Array(ble.buildJoystickFrame(12, -34));
  const withTrailing = new Uint8Array(valid.length + 1);
  withTrailing.set(valid, 0);
  withTrailing[withTrailing.length - 1] = 0x00;
  const parsed = ble.parseFrame(withTrailing.buffer);
  assert.strictEqual(parsed, null);
})();

(function testParamIndexMatchesStm32Count() {
  assert.strictEqual(ble.PARAM_INDEX.FOLLOW_DISTANCE_M, 0);
  assert.strictEqual(ble.PARAM_INDEX.PID_ANGLE_KD, 15);
  assert.strictEqual(Object.keys(ble.PARAM_INDEX).length, 16);
})();

console.log('ble tests passed');
