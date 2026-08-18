/*
 * Quattro low-level actuator bridge — Teensy 4.0
 *
 * REAL CAN MODE (when transceivers are available)
 *   CAN1: TX=22, RX=23 -> transceiver #1 -> motors 0..5
 *   CAN2: TX=1,  RX=0  -> transceiver #2 -> motors 6..11
 *   bitrate: 500 kbit/s
 *
 * VIRTUAL MOTOR MODE (current default)
 *   No CAN transceiver is required.
 *   ROS JointCommand -> USB Serial -> Teensy
 *     -> GIM6010 MIT 0x008 command is packed exactly as in real mode
 *     -> the packed 8-byte frame is decoded by an internal virtual motor
 *     -> virtual q/qdot/torque are generated
 *     -> GIM6010-style MIT 0x008 feedback bytes are packed
 *     -> the SAME real feedback parser decodes them
 *     -> USB Serial -> ROS /joint_states -> RViz
 *
 * Change QUATTRO_VIRTUAL_MOTOR_MODE to 0 later to enable physical dual-CAN.
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

#define QUATTRO_VIRTUAL_MOTOR_MODE 1

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBusA;  // TX22/RX23, node 0..5
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CanBusB;  // TX1/RX0,   node 6..11

static constexpr uint8_t SOF0 = 0xAA;
static constexpr uint8_t SOF1 = 0x55;
static constexpr uint8_t TYPE_COMMAND = 0x01;
static constexpr uint8_t TYPE_FEEDBACK = 0x02;
static constexpr uint8_t MOTOR_COUNT = 12;
static constexpr uint8_t MOTORS_PER_BUS = 6;

static constexpr uint32_t SERIAL_BAUD = 921600;
static constexpr uint32_t CAN_BAUD = 500000;
static constexpr uint32_t CONTROL_PERIOD_US = 10000;       // 100 Hz
static constexpr uint32_t COMMAND_TIMEOUT_US = 300000;     // 300 ms
static constexpr uint32_t FEEDBACK_STALE_US = 100000;      // 100 ms
static constexpr float CONTROL_DT = CONTROL_PERIOD_US * 1.0e-6f;

// GIM6010 command IDs used by the legacy real-robot driver.
static constexpr uint8_t CMD_SET_AXIS_STATE      = 0x007;
static constexpr uint8_t CMD_MIT_CONTROL         = 0x008;
static constexpr uint8_t CMD_SET_CONTROLLER_MODE = 0x00B;
static constexpr uint8_t CMD_SET_LIMITS          = 0x00F;
static constexpr uint8_t CMD_CLEAR_ERRORS        = 0x018;

static constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
static constexpr uint32_t CONTROL_MODE_POSITION = 3;
static constexpr uint32_t INPUT_MODE_MIT = 9;
static constexpr float DEFAULT_CURRENT_LIMIT_A = 20.0f;
static constexpr float DEFAULT_VELOCITY_LIMIT_RAD_S = 50.0f;

static constexpr float P_MIN=-12.5f, P_MAX=12.5f;
static constexpr float V_MIN=-65.0f, V_MAX=65.0f;
static constexpr float KP_MIN=0.0f, KP_MAX=500.0f;
static constexpr float KD_MIN=0.0f, KD_MAX=5.0f;
static constexpr float T_MIN=-50.0f, T_MAX=50.0f;

// Simple internal actuator model. These are simulation parameters only.
static constexpr float VIRTUAL_INERTIA = 0.08f;
static constexpr float VIRTUAL_VISCOUS_DAMPING = 0.30f;
static constexpr float VIRTUAL_MAX_ACCEL = 120.0f;  // rad/s^2

struct JointCmd {
  float q;
  float qd;
  float kp;
  float kd;
  float tau;
};

struct JointFb {
  float q;
  float qd;
  float tau;
  uint32_t stampUs;
  bool valid;
};

struct VirtualMotor {
  float q;
  float qd;
  float tau;
  JointCmd decodedCmd;
};

JointCmd cmd[MOTOR_COUNT];
JointFb fb[MOTOR_COUNT];
VirtualMotor virtualMotor[MOTOR_COUNT];
uint32_t lastCommandUs = 0;
uint16_t feedbackSeq = 0;

// Watchdog position-hold state.
// When host commands become stale, latch the current joint positions ONCE
// and keep holding those fixed references until fresh host commands return.
float holdPosition[MOTOR_COUNT] = {0.0f};
bool holdPositionValid[MOTOR_COUNT] = {false};
bool timeoutActive = false;

uint16_t crc16ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0; i<len; i++) {
    crc ^= uint16_t(data[i]) << 8;
    for (int b=0; b<8; b++) {
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
  }
  return crc;
}

uint32_t floatToUInt(float x, float xMin, float xMax, int bits) {
  x = constrain(x, xMin, xMax);
  return uint32_t((x-xMin) * float((1UL<<bits)-1) / (xMax-xMin));
}

float uintToFloat(uint32_t x, float xMin, float xMax, int bits) {
  return float(x) * (xMax-xMin) / float((1UL<<bits)-1) + xMin;
}

void writeU32LE(uint8_t* dst, uint32_t value) {
  dst[0] = uint8_t(value);
  dst[1] = uint8_t(value >> 8);
  dst[2] = uint8_t(value >> 16);
  dst[3] = uint8_t(value >> 24);
}

void writeF32LE(uint8_t* dst, float value) {
  memcpy(dst, &value, sizeof(float)); // Teensy 4.0: little-endian IEEE-754
}

CAN_message_t makeRawFrame(uint8_t nodeId, uint8_t cmdId, const uint8_t* data=nullptr, uint8_t len=0) {
  CAN_message_t m;
  m.id = (uint32_t(nodeId) << 5) | cmdId;
  m.len = (len > 8) ? 8 : len;
  for (uint8_t i=0; i<m.len; i++) m.buf[i] = data[i];
  return m;
}

bool writeCan(uint8_t nodeId, const CAN_message_t& message) {
#if QUATTRO_VIRTUAL_MOTOR_MODE
  (void)nodeId;
  (void)message;
  return true;
#else
  if (nodeId < MOTORS_PER_BUS) return CanBusA.write(message);
  return CanBusB.write(message);
#endif
}

void sendRaw(uint8_t nodeId, uint8_t cmdId, const uint8_t* data=nullptr, uint8_t len=0) {
#if !QUATTRO_VIRTUAL_MOTOR_MODE
  CAN_message_t m = makeRawFrame(nodeId, cmdId, data, len);
  writeCan(nodeId, m);
#else
  // Initialization/configuration commands are intentionally ignored by the
  // internal emulator. MIT control frames still go through full packing below.
  (void)nodeId; (void)cmdId; (void)data; (void)len;
#endif
}

void setAxisState(uint8_t nodeId, uint32_t state) {
  uint8_t d[4];
  writeU32LE(d, state);
  sendRaw(nodeId, CMD_SET_AXIS_STATE, d, 4);
}

void clearErrors(uint8_t nodeId) {
  sendRaw(nodeId, CMD_CLEAR_ERRORS, nullptr, 0);
}

void setLimits(uint8_t nodeId, float velocityLimit, float currentLimit) {
  uint8_t d[8];
  writeF32LE(d, velocityLimit);
  writeF32LE(d + 4, currentLimit);
  sendRaw(nodeId, CMD_SET_LIMITS, d, 8);
}

void setMitMode(uint8_t nodeId) {
  uint8_t d[8];
  writeU32LE(d, CONTROL_MODE_POSITION);
  writeU32LE(d + 4, INPUT_MODE_MIT);
  sendRaw(nodeId, CMD_SET_CONTROLLER_MODE, d, 8);
}

void initializeMotor(uint8_t nodeId) {
  clearErrors(nodeId);
  delay(2);
  setLimits(nodeId, DEFAULT_VELOCITY_LIMIT_RAD_S, DEFAULT_CURRENT_LIMIT_A);
  delay(2);
  setMitMode(nodeId);
  delay(2);
  setAxisState(nodeId, AXIS_STATE_CLOSED_LOOP_CONTROL);
  delay(2);
}

CAN_message_t makeMitCommandFrame(uint8_t id, const JointCmd& c) {
  const uint32_t p = floatToUInt(c.q, P_MIN, P_MAX, 16);
  const uint32_t v = floatToUInt(c.qd, V_MIN, V_MAX, 12);
  const uint32_t kp = floatToUInt(c.kp, KP_MIN, KP_MAX, 12);
  const uint32_t kd = floatToUInt(c.kd, KD_MIN, KD_MAX, 12);
  const uint32_t t = floatToUInt(c.tau, T_MIN, T_MAX, 12);

  CAN_message_t m;
  m.id = (uint32_t(id) << 5) | CMD_MIT_CONTROL;
  m.len = 8;
  m.buf[0] = p >> 8;
  m.buf[1] = p;
  m.buf[2] = v >> 4;
  m.buf[3] = ((v & 0xF) << 4) | (kp >> 8);
  m.buf[4] = kp;
  m.buf[5] = kd >> 4;
  m.buf[6] = ((kd & 0xF) << 4) | (t >> 8);
  m.buf[7] = t;
  return m;
}

JointCmd decodeMitCommandFrame(const CAN_message_t& m) {
  JointCmd c{};
  const uint32_t p  = (uint32_t(m.buf[0]) << 8) | m.buf[1];
  const uint32_t v  = (uint32_t(m.buf[2]) << 4) | (m.buf[3] >> 4);
  const uint32_t kp = (uint32_t(m.buf[3] & 0x0F) << 8) | m.buf[4];
  const uint32_t kd = (uint32_t(m.buf[5]) << 4) | (m.buf[6] >> 4);
  const uint32_t t  = (uint32_t(m.buf[6] & 0x0F) << 8) | m.buf[7];
  c.q   = uintToFloat(p,  P_MIN,  P_MAX,  16);
  c.qd  = uintToFloat(v,  V_MIN,  V_MAX,  12);
  c.kp  = uintToFloat(kp, KP_MIN, KP_MAX, 12);
  c.kd  = uintToFloat(kd, KD_MIN, KD_MAX, 12);
  c.tau = uintToFloat(t,  T_MIN,  T_MAX,  12);
  return c;
}

CAN_message_t makeMitFeedbackFrame(uint8_t id, float q, float qd, float tau) {
  const uint32_t p = floatToUInt(q, P_MIN, P_MAX, 16);
  const uint32_t v = floatToUInt(qd, V_MIN, V_MAX, 12);
  const uint32_t t = floatToUInt(tau, T_MIN, T_MAX, 12);

  CAN_message_t m;
  m.id = (uint32_t(id) << 5) | CMD_MIT_CONTROL;
  m.len = 6;
  m.buf[0] = id;
  m.buf[1] = p >> 8;
  m.buf[2] = p;
  m.buf[3] = v >> 4;
  m.buf[4] = ((v & 0x0F) << 4) | (t >> 8);
  m.buf[5] = t;
  return m;
}

void parseCanFeedback(const CAN_message_t& m) {
  const uint8_t id = uint8_t(m.id >> 5);
  const uint8_t cmdId = uint8_t(m.id & 0x1F);
  if (id >= MOTOR_COUNT || cmdId != CMD_MIT_CONTROL || m.len < 6) return;

  const uint32_t p = (uint32_t(m.buf[1]) << 8) | m.buf[2];
  const uint32_t v = (uint32_t(m.buf[3]) << 4) | (m.buf[4] >> 4);
  const uint32_t t = (uint32_t(m.buf[4] & 0xF) << 8) | m.buf[5];

  fb[id].q = uintToFloat(p, P_MIN, P_MAX, 16);
  fb[id].qd = uintToFloat(v, V_MIN, V_MAX, 12);
  fb[id].tau = uintToFloat(t, T_MIN, T_MAX, 12);
  fb[id].stampUs = micros();
  fb[id].valid = true;
}

void virtualAcceptMitFrame(const CAN_message_t& m) {
  const uint8_t id = uint8_t(m.id >> 5);
  const uint8_t cmdId = uint8_t(m.id & 0x1F);
  if (id >= MOTOR_COUNT || cmdId != CMD_MIT_CONTROL || m.len != 8) return;
  virtualMotor[id].decodedCmd = decodeMitCommandFrame(m);
}

void virtualStepOne(uint8_t id, float dt) {
  VirtualMotor& vm = virtualMotor[id];
  const JointCmd& c = vm.decodedCmd;

  // Same MIT control meaning as the actuator:
  // tau = kp(q_des-q) + kd(qdot_des-qdot) + tau_ff
  float tau = c.kp * (c.q - vm.q) + c.kd * (c.qd - vm.qd) + c.tau;
  tau = constrain(tau, T_MIN, T_MAX);

  float accel = (tau - VIRTUAL_VISCOUS_DAMPING * vm.qd) / VIRTUAL_INERTIA;
  accel = constrain(accel, -VIRTUAL_MAX_ACCEL, VIRTUAL_MAX_ACCEL);

  vm.qd += accel * dt;
  vm.qd = constrain(vm.qd, V_MIN, V_MAX);
  vm.q += vm.qd * dt;
  vm.q = constrain(vm.q, P_MIN, P_MAX);
  vm.tau = tau;

  // Re-pack exactly as a GIM6010-style MIT feedback frame, then feed that
  // frame into the same parser used by physical CAN mode.
  const CAN_message_t feedbackFrame = makeMitFeedbackFrame(id, vm.q, vm.qd, vm.tau);
  parseCanFeedback(feedbackFrame);
}

void sendMit(uint8_t id, const JointCmd& c) {
  const CAN_message_t frame = makeMitCommandFrame(id, c);
#if QUATTRO_VIRTUAL_MOTOR_MODE
  virtualAcceptMitFrame(frame);
#else
  writeCan(id, frame);
#endif
}

void drainCan() {
#if !QUATTRO_VIRTUAL_MOTOR_MODE
  CAN_message_t m;
  while (CanBusA.read(m)) parseCanFeedback(m);
  while (CanBusB.read(m)) parseCanFeedback(m);
#endif
}

void writeFloat(uint8_t* p, float v) { memcpy(p, &v, 4); }
float readFloat(const uint8_t* p) { float v; memcpy(&v, p, 4); return v; }

void sendFeedbackSerial() {
  // Header 6 bytes + 12 * (id1 + valid1 + q4 + qd4 + tau4) + CRC2 = 176 bytes.
  uint8_t frame[176];
  size_t o = 0;
  frame[o++] = SOF0;
  frame[o++] = SOF1;
  frame[o++] = TYPE_FEEDBACK;
  frame[o++] = uint8_t(feedbackSeq);
  frame[o++] = uint8_t(feedbackSeq >> 8);
  frame[o++] = MOTOR_COUNT;

  const uint32_t now = micros();
  for (uint8_t i=0; i<MOTOR_COUNT; i++) {
    const bool fresh = fb[i].valid && uint32_t(now - fb[i].stampUs) <= FEEDBACK_STALE_US;
    frame[o++] = i;
    frame[o++] = fresh ? 1 : 0;
    writeFloat(frame + o, fb[i].q); o += 4;
    writeFloat(frame + o, fb[i].qd); o += 4;
    writeFloat(frame + o, fb[i].tau); o += 4;
  }

  const uint16_t crc = crc16ccitt(frame, o);
  frame[o++] = uint8_t(crc);
  frame[o++] = uint8_t(crc >> 8);
  Serial.write(frame, o);
  feedbackSeq++;
}

bool readCommandFrame() {
  static uint8_t buf[300];
  static size_t n = 0;

  while (Serial.available()) {
    const uint8_t b = Serial.read();
    if (n == 0 && b != SOF0) continue;
    if (n == 1 && b != SOF1) { n = 0; continue; }
    buf[n++] = b;

    if (n >= 6) {
      const uint8_t count = buf[5];
      if (count > MOTOR_COUNT) { n = 0; continue; }
      const size_t total = 6 + size_t(count) * 21 + 2;

      if (n == total) {
        const uint16_t rx = uint16_t(buf[total-2]) | (uint16_t(buf[total-1]) << 8);
        const bool ok = (buf[2] == TYPE_COMMAND && crc16ccitt(buf, total-2) == rx);
        if (ok) {
          size_t o = 6;
          for (uint8_t k=0; k<count; k++) {
            const uint8_t id = buf[o++];
            if (id >= MOTOR_COUNT) { o += 20; continue; }
            cmd[id].q = readFloat(buf + o); o += 4;
            cmd[id].qd = readFloat(buf + o); o += 4;
            cmd[id].kp = readFloat(buf + o); o += 4;
            cmd[id].kd = readFloat(buf + o); o += 4;
            cmd[id].tau = readFloat(buf + o); o += 4;
          }
          lastCommandUs = micros();
        }
        n = 0;
        return ok;
      }
      if (n > total) n = 0;
    }
  }
  return false;
}

JointCmd makeSafeCommand(uint8_t id, bool hostStale) {
  JointCmd out = cmd[id];
  if (!hostStale) return out;

  // Timeout policy:
  //  - hold the position latched ONCE when timeout began
  //  - never preserve feedforward torque
  //  - never preserve desired velocity
  //
  // Do NOT continuously copy fb[id].q into q_des here.
  // Re-latching every control tick can turn feedback quantization/noise
  // into a slowly moving position reference.
  out.q = holdPosition[id];
  out.qd = 0.0f;
  out.tau = 0.0f;

  // If no trustworthy feedback existed when timeout began, do not apply
  // position/velocity feedback gains using an invented hold reference.
  if (!holdPositionValid[id]) {
    out.kp = 0.0f;
    out.kd = 0.0f;
  }

  return out;
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  for (auto &x : cmd) x = {0.0f, 0.0f, 60.0f, 0.8f, 0.0f};
  for (auto &x : fb) x = {0.0f, 0.0f, 0.0f, 0, false};
  for (auto &x : virtualMotor) {
    x.q = 0.0f;
    x.qd = 0.0f;
    x.tau = 0.0f;
    x.decodedCmd = {0.0f, 0.0f, 60.0f, 0.8f, 0.0f};
  }

#if !QUATTRO_VIRTUAL_MOTOR_MODE
  CanBusA.begin();
  CanBusA.setBaudRate(CAN_BAUD);
  CanBusB.begin();
  CanBusB.setBaudRate(CAN_BAUD);

  delay(100);
  for (uint8_t id=0; id<MOTOR_COUNT; id++) initializeMotor(id);
#endif

  lastCommandUs = micros();
}

void loop() {
  readCommandFrame();
  drainCan();

  static elapsedMicros controlTimer = 0;
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;
    const bool hostStale = uint32_t(micros() - lastCommandUs) > COMMAND_TIMEOUT_US;

    // Detect only the transition into timeout and latch the hold reference once.
    if (hostStale && !timeoutActive) {
      const uint32_t nowUs = micros();

      for (uint8_t i=0; i<MOTOR_COUNT; i++) {
        const bool fresh = fb[i].valid &&
                           uint32_t(nowUs - fb[i].stampUs) <= FEEDBACK_STALE_US;

        if (fresh) {
          holdPosition[i] = fb[i].q;
          holdPositionValid[i] = true;
        } else {
          // No trustworthy feedback: use a neutral reference only as a
          // placeholder. makeSafeCommand() will disable kp/kd for this joint.
          holdPosition[i] = 0.0f;
          holdPositionValid[i] = false;
        }
      }

      timeoutActive = true;
    } else if (!hostStale && timeoutActive) {
      // Fresh host commands have resumed.
      timeoutActive = false;
    }

    for (uint8_t i=0; i<MOTOR_COUNT; i++) {
      sendMit(i, makeSafeCommand(i, hostStale));
    }

#if QUATTRO_VIRTUAL_MOTOR_MODE
    for (uint8_t i=0; i<MOTOR_COUNT; i++) virtualStepOne(i, CONTROL_DT);
#else
    // Read immediate MIT responses that arrived after this control tick's TX.
    drainCan();
#endif

    sendFeedbackSerial();
  }
}