/*
  ============================================================
  Multi Scissor Lift Controller
  多剪式升降台非阻塞控制器

  当前配置：
    Lift 0:
      BTS7960 + BTS7960
      actuator0 RPWM=2,  LPWM=3,  sensor A0
      actuator1 RPWM=4,  LPWM=5,  sensor A1

    Lift 1:
      BTS7960 + BTS7960
      actuator2 RPWM=6,  LPWM=7,  sensor A2
      actuator3 RPWM=8,  LPWM=9,  sensor A3

    Lift 2:
      BTS7960 + BTS7960
      actuator4 RPWM=10, LPWM=11, sensor A4
      actuator5 RPWM=12, LPWM=13, sensor A5

  串口输入：
    12.5             -> 所有升降台都去 12.5
    12.5,18.0,20.0   -> L0 去 12.5，L1 去 18.0，L2 去 20.0
    *,18.0,*         -> L0 不动，L1 去 18.0，L2 不动
    12.5,*,20.0      -> L0 去 12.5，L1 不动，L2 去 20.0

  蓝牙输入：
    1,2,3            -> 取前三位，等同于指令 1,2,3
    1,2,3,4          -> 只取前三位，等同于指令 1,2,3

  指令筛选：
    如果新目标和当前目标一样，就跳过，不重新 setTarget，避免原地抽搐

  硬等待模式：
    上升时，更高的一边停止，另一边继续上升追平
    下降时，更低的一边停止，另一边继续下降追平

  单位：inch
  适配：Arduino Mega
  ============================================================
*/

// ==============================
// Motor driver 抽象基类
// ==============================
class MotorDriver {
public:
  virtual void begin() = 0;
  virtual void setSpeed(int pwm) = 0;
  virtual void extend() = 0;
  virtual void retract() = 0;
  virtual void stop() = 0;
};

// ==============================
// L298N 驱动器
// IN1 + IN2 + PWM
// ==============================
class L298NDriver : public MotorDriver {
public:
  int pinIn1;
  int pinIn2;
  int pinPWM;
  int speedPWM;

  L298NDriver(int in1, int in2, int pwmPin)
    : pinIn1(in1), pinIn2(in2), pinPWM(pwmPin), speedPWM(255) {}

  void begin() override {
    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);
    pinMode(pinPWM, OUTPUT);
    stop();
  }

  void setSpeed(int pwm) override {
    speedPWM = constrain(pwm, 0, 255);
  }

  void extend() override {
    digitalWrite(pinIn1, HIGH);
    digitalWrite(pinIn2, LOW);
    analogWrite(pinPWM, speedPWM);
  }

  void retract() override {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, HIGH);
    analogWrite(pinPWM, speedPWM);
  }

  void stop() override {
    analogWrite(pinPWM, 0);
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
  }
};

// ==============================
// BTS7960 驱动器
// RPWM + LPWM
// R_EN / L_EN 建议直接接 5V
// ==============================
class BTS7960Driver : public MotorDriver {
public:
  int pinRPWM;
  int pinLPWM;
  int speedPWM;

  BTS7960Driver(int rPWM, int lPWM)
    : pinRPWM(rPWM), pinLPWM(lPWM), speedPWM(255) {}

  void begin() override {
    pinMode(pinRPWM, OUTPUT);
    pinMode(pinLPWM, OUTPUT);
    stop();
  }

  void setSpeed(int pwm) override {
    speedPWM = constrain(pwm, 0, 255);
  }

  void extend() override {
    analogWrite(pinRPWM, speedPWM);
    analogWrite(pinLPWM, 0);
  }

  void retract() override {
    analogWrite(pinRPWM, 0);
    analogWrite(pinLPWM, speedPWM);
  }

  void stop() override {
    analogWrite(pinRPWM, 0);
    analogWrite(pinLPWM, 0);
  }
};

// ==============================
// 单个线性执行器
// ==============================
class ActuatorUnit {
public:
  int id;
  MotorDriver* motor;
  int sensorPin;
  float slope;
  float intercept;
  float currentHeight;

  ActuatorUnit(int unitId,
               MotorDriver* driver,
               int analogPin,
               float k,
               float b)
    : id(unitId),
      motor(driver),
      sensorPin(analogPin),
      slope(k),
      intercept(b),
      currentHeight(0.0f) {}

  void begin() {
    motor->begin();
    pinMode(sensorPin, INPUT);
    updateHeight();
  }

  int readRaw() {
    const int samples = 10;
    long sum = 0;

    for (int i = 0; i < samples; i++) {
      sum += analogRead(sensorPin);
    }

    return (int)(sum / samples);
  }

  float rawToHeight(int raw) {
    return slope * (float)raw + intercept;
  }

  float updateHeight() {
    currentHeight = rawToHeight(readRaw());
    return currentHeight;
  }

  float getHeight() {
    return updateHeight();
  }

  void extend(int pwm) {
    motor->setSpeed(pwm);
    motor->extend();
  }

  void retract(int pwm) {
    motor->setSpeed(pwm);
    motor->retract();
  }

  void stop() {
    motor->stop();
  }
};

// ==============================
// 状态
// ==============================
enum LiftState {
  IDLE,
  MOVING,
  REACHED,
  ERROR_STOP
};

const char* stateName(LiftState s) {
  switch (s) {
    case IDLE: return "IDLE";
    case MOVING: return "MOVING";
    case REACHED: return "REACHED";
    case ERROR_STOP: return "ERROR_STOP";
  }
  return "UNKNOWN";
}

// ==============================
// 单台升降台控制器
// ==============================
class ScissorLiftController {
public:
  const char* name;

  ActuatorUnit* unit0;
  ActuatorUnit* unit1;

  float minH;
  float maxH;

  LiftState liftState;

  float targetHeight;
  float iTermH;

  unsigned long moveStartMs;
  unsigned long lastCtrlMs;
  unsigned long lastPrintMs;

  bool hardWaitMode;

  ScissorLiftController(const char* liftName,
                        ActuatorUnit* a,
                        ActuatorUnit* b,
                        float minHeight,
                        float maxHeight)
    : name(liftName),
      unit0(a),
      unit1(b),
      minH(minHeight),
      maxH(maxHeight),
      liftState(IDLE),
      targetHeight(0.0f),
      iTermH(0.0f),
      moveStartMs(0),
      lastCtrlMs(0),
      lastPrintMs(0),
      hardWaitMode(false) {}

  void begin() {
    unit0->begin();
    unit1->begin();
    stop();

    float h0 = getHeight0();
    float h1 = getHeight1();
    targetHeight = 0.5f * (h0 + h1);
    liftState = IDLE;
  }

  float getHeight0() {
    return unit0->getHeight();
  }

  float getHeight1() {
    return unit1->getHeight();
  }

  void stop() {
    unit0->stop();
    unit1->stop();
  }

  void setTarget(float newTarget) {
    targetHeight = constrain(newTarget, minH, maxH);
    iTermH = 0.0f;
    moveStartMs = millis();
    lastCtrlMs = 0;
    lastPrintMs = 0;
    liftState = MOVING;
    hardWaitMode = false;

    Serial.print("[");
    Serial.print(name);
    Serial.print("] New target: ");
    Serial.print(newTarget, 3);
    Serial.print(" -> constrained: ");
    Serial.println(targetHeight, 3);
  }

  void drivePair(bool goExtend, int pwm0, int pwm1) {
    if (pwm0 == 0) {
      unit0->stop();
    } else {
      if (goExtend) unit0->extend(pwm0);
      else unit0->retract(pwm0);
    }

    if (pwm1 == 0) {
      unit1->stop();
    } else {
      if (goExtend) unit1->extend(pwm1);
      else unit1->retract(pwm1);
    }
  }

  int outputToPWM(float mag, bool isExtend, bool lowZone) {
    int pwm = (int)mag;
    pwm = constrain(pwm, 0, PWM_MAX);

    if (pwm < PWM_DEADBAND) {
      return 0;
    }

    if (isExtend) {
      int minPwm = lowZone ? PWM_MIN_EXTEND_LOW_ZONE : PWM_MIN_EXTEND;
      if (pwm > 0 && pwm < minPwm) pwm = minPwm;
    } else {
      if (pwm > 0 && pwm < PWM_MIN_RETRACT) pwm = PWM_MIN_RETRACT;
    }

    return constrain(pwm, 0, PWM_MAX);
  }

  void printSimpleStatus() {
    float h0 = getHeight0();
    float h1 = getHeight1();
    float avgH = 0.5f * (h0 + h1);
    float hErr = targetHeight - avgH;
    float syncErr = h0 - h1;

    Serial.print("[");
    Serial.print(name);
    Serial.print("] state=");
    Serial.print(stateName(liftState));
    Serial.print(", target=");
    Serial.print(targetHeight, 3);
    Serial.print(", h0=");
    Serial.print(h0, 3);
    Serial.print(", h1=");
    Serial.print(h1, 3);
    Serial.print(", avg=");
    Serial.print(avgH, 3);
    Serial.print(", e=");
    Serial.print(hErr, 3);
    Serial.print(", sync=");
    Serial.println(syncErr, 3);
  }

  void printStatus(float h0, float h1, float avgH, float hErr, float syncErr,
                   float baseMag, float corrMag, float u0, float u1,
                   int pwm0, int pwm1, bool lowZone, bool hardSync) {
    Serial.print("[");
    Serial.print(name);
    Serial.print("] state=");
    Serial.print(stateName(liftState));
    Serial.print(", target=");
    Serial.print(targetHeight, 3);
    Serial.print(", h0=");
    Serial.print(h0, 3);
    Serial.print(", h1=");
    Serial.print(h1, 3);
    Serial.print(", avg=");
    Serial.print(avgH, 3);
    Serial.print(", e=");
    Serial.print(hErr, 3);
    Serial.print(", sync=");
    Serial.print(syncErr, 3);
    Serial.print(", lowZone=");
    Serial.print(lowZone ? "YES" : "NO");
    Serial.print(", hardWaitMode=");
    Serial.print(hardWaitMode ? "YES" : "NO");
    Serial.print(", hardSync=");
    Serial.print(hardSync ? "YES" : "NO");
    Serial.print(", base=");
    Serial.print(baseMag, 1);
    Serial.print(", corr=");
    Serial.print(corrMag, 1);
    Serial.print(", u0=");
    Serial.print(u0, 1);
    Serial.print(", u1=");
    Serial.print(u1, 1);
    Serial.print(", pwm0=");
    Serial.print(pwm0);
    Serial.print(", pwm1=");
    Serial.println(pwm1);
  }

  void update() {
    unsigned long now = millis();

    if (liftState != MOVING) {
      if (now - lastPrintMs >= IDLE_PRINT_DT_MS) {
        lastPrintMs = now;
        printSimpleStatus();
      }
      return;
    }

    if (lastCtrlMs != 0 && now - lastCtrlMs < CTRL_DT_MS) {
      return;
    }

    float dt;
    if (lastCtrlMs == 0) {
      dt = CTRL_DT_MS / 1000.0f;
    } else {
      dt = (now - lastCtrlMs) / 1000.0f;
    }
    lastCtrlMs = now;

    float h0 = getHeight0();
    float h1 = getHeight1();
    float avgH = 0.5f * (h0 + h1);
    float syncErr = h0 - h1;
    float hErr = targetHeight - avgH;

    float baseMag = 0.0f;
    float corrMag = 0.0f;
    float u0 = 0.0f;
    float u1 = 0.0f;
    int pwm0 = 0;
    int pwm1 = 0;

    bool lowZone = avgH <= LOW_HEIGHT_ZONE;
    bool hardSync = false;

    if (now - moveStartMs > MOVE_TIMEOUT_MS) {
      stop();
      liftState = ERROR_STOP;
      hardWaitMode = false;
      Serial.print("[");
      Serial.print(name);
      Serial.println("] Move timeout. Stopped.");
      printSimpleStatus();
      return;
    }

    if (fabs(hErr) <= HEIGHT_TOL && fabs(syncErr) <= SYNC_TOL) {
      stop();
      liftState = REACHED;
      hardWaitMode = false;
      Serial.print("[");
      Serial.print(name);
      Serial.println("] Reached target.");
      printSimpleStatus();
      return;
    }

    if (hErr < 0 && (h0 <= minH + HEIGHT_TOL || h1 <= minH + HEIGHT_TOL)) {
      stop();
      liftState = ERROR_STOP;
      hardWaitMode = false;
      Serial.print("[");
      Serial.print(name);
      Serial.println("] Near min height. Stopped.");
      printSimpleStatus();
      return;
    }

    if (hErr > 0 && (h0 >= maxH - HEIGHT_TOL || h1 >= maxH - HEIGHT_TOL)) {
      stop();
      liftState = ERROR_STOP;
      hardWaitMode = false;
      Serial.print("[");
      Serial.print(name);
      Serial.println("] Near max height. Stopped.");
      printSimpleStatus();
      return;
    }

    if (fabs(syncErr) > MAX_SYNC_ERROR) {
      stop();
      liftState = ERROR_STOP;
      hardWaitMode = false;
      Serial.print("[");
      Serial.print(name);
      Serial.print("] Sync error too large. sync=");
      Serial.println(syncErr, 3);
      printSimpleStatus();
      return;
    }

    iTermH += hErr * dt;
    iTermH = constrain(iTermH, -I_LIMIT_H, I_LIMIT_H);

    float baseU = KP_H * hErr + KI_H * iTermH;
    bool goExtend = (baseU > 0.0f);
    bool goRetract = (baseU < 0.0f);

    if (!goExtend && !goRetract) {
      stop();
      hardWaitMode = false;
      return;
    }

    baseMag = fabs(baseU);
    baseMag = constrain(baseMag, 0.0f, (float)PWM_MAX);

    // ==============================
    // 方向感知硬等待模式
    // 上升：更高的一边停止，更低的一边继续上升追平
    // 下降：更低的一边停止，更高的一边继续下降追平
    // 用进入/退出阈值做迟滞，避免刚到边界就反复抽动
    // ==============================
    if (!hardWaitMode && fabs(syncErr) > WAITMODE_ENTER_ERR) {
      hardWaitMode = true;
      Serial.print("[");
      Serial.print(name);
      Serial.println("] Enter direction-aware hard wait mode.");
    }

    if (hardWaitMode && fabs(syncErr) < WAITMODE_EXIT_ERR) {
      hardWaitMode = false;
      Serial.print("[");
      Serial.print(name);
      Serial.println("] Exit direction-aware hard wait mode.");
    }

    if (hardWaitMode) {
      hardSync = true;
      baseMag = PWM_MAX;
      corrMag = PWM_MAX;

      int chasePwm;
      if (goExtend) {
        chasePwm = lowZone ? WAITMODE_EXTEND_LOW_PWM : WAITMODE_EXTEND_PWM;
      } else {
        chasePwm = WAITMODE_RETRACT_PWM;
      }

      // syncErr = h0 - h1
      // syncErr > 0: unit0 更高，unit1 更低
      // syncErr < 0: unit1 更高，unit0 更低
      if (goExtend) {
        // 上升时，高的一边已经领先，所以高的一边停，低的一边继续上升
        if (syncErr > 0.0f) {
          pwm0 = 0;
          pwm1 = chasePwm;
        } else {
          pwm0 = chasePwm;
          pwm1 = 0;
        }
      } else {
        // 下降时，低的一边已经领先，所以低的一边停，高的一边继续下降
        if (syncErr > 0.0f) {
          pwm0 = chasePwm;
          pwm1 = 0;
        } else {
          pwm0 = 0;
          pwm1 = chasePwm;
        }
      }

      u0 = pwm0;
      u1 = pwm1;

      drivePair(goExtend, pwm0, pwm1);

      if (now - lastPrintMs >= PRINT_DT_MS) {
        lastPrintMs = now;
        printStatus(h0, h1, avgH, hErr, syncErr,
                    baseMag, corrMag, u0, u1,
                    pwm0, pwm1, lowZone, hardSync);
      }

      return;
    }

    // ==============================
    // 普通同步逻辑
    // 小误差压低快边
    // 大误差进入硬同步
    // ==============================
    u0 = baseMag;
    u1 = baseMag;

    if (fabs(syncErr) > SYNC_DEADBAND) {
      hardSync = fabs(syncErr) >= SYNC_HARD_ERR;

      if (hardSync) {
        float fastRatio = lowZone ? HARD_FAST_RATIO_LOW_ZONE : HARD_FAST_RATIO_NORMAL;
        float fastOutput = baseMag * fastRatio;

        if (goExtend) {
          if (syncErr > 0.0f) {
            u0 = fastOutput;
            u1 = baseMag;
          } else {
            u0 = baseMag;
            u1 = fastOutput;
          }
        }

        if (goRetract) {
          if (syncErr > 0.0f) {
            u0 = baseMag;
            u1 = fastOutput;
          } else {
            u0 = fastOutput;
            u1 = baseMag;
          }
        }

        corrMag = baseMag - fastOutput;
      } else {
        corrMag = KP_SYNC_PWM * fabs(syncErr);

        float maxReductionByRatio = baseMag * MAX_SYNC_REDUCTION_RATIO;
        corrMag = constrain(corrMag, 0.0f, maxReductionByRatio);

        float minActiveRatio = lowZone ? MIN_ACTIVE_RATIO_LOW_ZONE : MIN_ACTIVE_RATIO_NORMAL;
        float minActiveOutput = baseMag * minActiveRatio;

        if (goExtend) {
          if (syncErr > 0.0f) {
            u0 = baseMag - corrMag;
            u1 = baseMag;
          } else {
            u0 = baseMag;
            u1 = baseMag - corrMag;
          }
        }

        if (goRetract) {
          if (syncErr > 0.0f) {
            u0 = baseMag;
            u1 = baseMag - corrMag;
          } else {
            u0 = baseMag - corrMag;
            u1 = baseMag;
          }
        }

        if (u0 > 0.0f && u0 < minActiveOutput) u0 = minActiveOutput;
        if (u1 > 0.0f && u1 < minActiveOutput) u1 = minActiveOutput;
      }
    }

    if (u0 < 0.0f) u0 = 0.0f;
    if (u1 < 0.0f) u1 = 0.0f;

    pwm0 = outputToPWM(u0, goExtend, lowZone);
    pwm1 = outputToPWM(u1, goExtend, lowZone);

    if (goExtend && lowZone) {
      if (pwm0 > 0 && pwm0 < PWM_MIN_EXTEND_LOW_ZONE) pwm0 = PWM_MIN_EXTEND_LOW_ZONE;
      if (pwm1 > 0 && pwm1 < PWM_MIN_EXTEND_LOW_ZONE) pwm1 = PWM_MIN_EXTEND_LOW_ZONE;

      if (pwm0 == 0 && pwm1 > 0) pwm0 = PWM_MIN_EXTEND_LOW_ZONE;
      if (pwm1 == 0 && pwm0 > 0) pwm1 = PWM_MIN_EXTEND_LOW_ZONE;
    }

    drivePair(goExtend, pwm0, pwm1);

    if (now - lastPrintMs >= PRINT_DT_MS) {
      lastPrintMs = now;
      printStatus(h0, h1, avgH, hErr, syncErr,
                  baseMag, corrMag, u0, u1,
                  pwm0, pwm1, lowZone, hardSync);
    }
  }

  // ==============================
  // 控制参数
  // ==============================
  static const unsigned long CTRL_DT_MS = 30UL;
  static const unsigned long PRINT_DT_MS = 300UL;
  static const unsigned long IDLE_PRINT_DT_MS = 1500UL;
  static const unsigned long MOVE_TIMEOUT_MS = 120000UL;

  static constexpr float HEIGHT_TOL = 0.15f;
  static constexpr float SYNC_TOL = 0.15f;
  static constexpr float MAX_SYNC_ERROR = 3.60f;

  static constexpr float KP_H = 200.0f;
  static constexpr float KI_H = 0.0f;
  static constexpr float I_LIMIT_H = 20.0f;

  static constexpr float KP_SYNC_PWM = 180.0f;
  static constexpr float SYNC_DEADBAND = 0.05f;
  static constexpr float SYNC_HARD_ERR = 0.35f;
  static constexpr float MAX_SYNC_REDUCTION_RATIO = 0.45f;

  static constexpr float LOW_HEIGHT_ZONE = 10.0f;

  static const int PWM_MAX = 255;
  static const int PWM_DEADBAND = 10;
  static const int PWM_MIN_EXTEND = 150;
  static const int PWM_MIN_RETRACT = 200;
  static const int PWM_MIN_EXTEND_LOW_ZONE = 230;

  static constexpr float MIN_ACTIVE_RATIO_NORMAL = 0.55f;
  static constexpr float MIN_ACTIVE_RATIO_LOW_ZONE = 0.85f;

  static constexpr float HARD_FAST_RATIO_NORMAL = 0.35f;
  static constexpr float HARD_FAST_RATIO_LOW_ZONE = 0.75f;

  static constexpr float WAITMODE_ENTER_ERR = 0.30f;
  static constexpr float WAITMODE_EXIT_ERR = 0.10f;

  // 硬等待时，领先的一边 PWM = 0，落后的一边用下面的 PWM 追平
  static const int WAITMODE_EXTEND_PWM = 255;
  static const int WAITMODE_EXTEND_LOW_PWM = 255;
  static const int WAITMODE_RETRACT_PWM = 100;
};

// ==============================
// 硬件配置区
// ==============================

// 当前 PWM 排布：
// Arduino pins 2 到 13 分别对应三组升降台的 6 个 BTS7960 driver 输入
//
// Lift 0：第一套升降台，BTS7960 + BTS7960
// actuator0: RPWM=2,  LPWM=3,  sensor=A0
// actuator1: RPWM=4,  LPWM=5,  sensor=A1
//
// Lift 1：第二套升降台，BTS7960 + BTS7960
// actuator2: RPWM=6,  LPWM=7,  sensor=A2
// actuator3: RPWM=8,  LPWM=9,  sensor=A3
//
// Lift 2：第三套升降台，BTS7960 + BTS7960
// actuator4: RPWM=10, LPWM=11, sensor=A4
// actuator5: RPWM=12, LPWM=13, sensor=A5
//
// 如果某一根 actuator 方向反了，不要改控制逻辑，直接把那一组的 RPWM / LPWM 对调。

BTS7960Driver motor0(2, 3);
BTS7960Driver motor1(4, 5);
const int BTS_ENABLE_PIN = 22;

ActuatorUnit actuator0(
  1,
  &motor0,
  A0,
  0.0385,
  5.70);

ActuatorUnit actuator1(
  2,
  &motor1,
  A1,
  0.0385,
  5.70);

BTS7960Driver motor2(6, 7);
BTS7960Driver motor3(8, 9);

ActuatorUnit actuator2(
  3,
  &motor2,
  A2,
  0.0385,
  5.70);

ActuatorUnit actuator3(
  4,
  &motor3,
  A3,
  0.0385,
  5.70);

BTS7960Driver motor4(10, 11);
BTS7960Driver motor5(12, 13);

ActuatorUnit actuator4(
  5,
  &motor4,
  A4,
  0.0385,
  5.70);

ActuatorUnit actuator5(
  6,
  &motor5,
  A5,
  0.0385,
  5.70);

// 每台升降台自己的高度限制
const float LIFT0_MIN_H = 6.0f;  // 6.7
const float LIFT0_MAX_H = 34.0f;

const float LIFT1_MIN_H = 5.0f;
const float LIFT1_MAX_H = 45.0f;

const float LIFT2_MIN_H = 6.0f;
const float LIFT2_MAX_H = 34.0f;

ScissorLiftController lift0("L0", &actuator0, &actuator1, LIFT0_MIN_H, LIFT0_MAX_H);
ScissorLiftController lift1("L1", &actuator2, &actuator3, LIFT1_MIN_H, LIFT1_MAX_H);
ScissorLiftController lift2("L2", &actuator4, &actuator5, LIFT2_MIN_H, LIFT2_MAX_H);

const int NUM_LIFTS = 3;
ScissorLiftController* lifts[NUM_LIFTS] = { &lift0, &lift1, &lift2 };

String line = "";
String btLine = "";

const int BT_BAUD = 9600;

// 指令筛选：目标变化小于这个值，就认为是同一个目标，不重复下发
const float TARGET_UPDATE_EPS = 0.25f;

// ==============================
// 输入解析
// ==============================
bool isNumericCommand(String s) {
  s.trim();
  if (s.length() == 0) return false;

  bool hasDigit = false;
  bool hasDot = false;

  for (int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);

    if (c >= '0' && c <= '9') {
      hasDigit = true;
      continue;
    }

    if (c == '.' && !hasDot) {
      hasDot = true;
      continue;
    }

    if ((c == '-' || c == '+') && i == 0) {
      continue;
    }

    return false;
  }

  return hasDigit;
}

int splitCSV(String input, String parts[], int maxParts) {
  int count = 0;
  input.trim();

  while (input.length() > 0 && count < maxParts) {
    int commaIndex = input.indexOf(',');

    if (commaIndex < 0) {
      parts[count] = input;
      parts[count].trim();
      count++;
      break;
    }

    parts[count] = input.substring(0, commaIndex);
    parts[count].trim();
    count++;

    input = input.substring(commaIndex + 1);
    input.trim();
  }

  return count;
}


String keepFirstCSVValues(String input, int keepCount) {
  input.trim();

  String output = "";
  int count = 0;

  while (input.length() > 0 && count < keepCount) {
    int commaIndex = input.indexOf(',');
    String part;

    if (commaIndex < 0) {
      part = input;
      input = "";
    } else {
      part = input.substring(0, commaIndex);
      input = input.substring(commaIndex + 1);
    }

    part.trim();

    if (output.length() > 0) {
      output += ",";
    }

    output += part;
    count++;
  }

  return output;
}

String subtractOneFromCSV(String msg) {
  String result = "";
  int start = 0;

  while (start < msg.length()) {
    int comma = msg.indexOf(',', start);
    String part;

    if (comma == -1) {
      part = msg.substring(start);
      start = msg.length();
    } else {
      part = msg.substring(start, comma);
      start = comma + 1;
    }

    part.trim();

    float value = part.toFloat();
    value -= 1;

    if (result.length() > 0) {
      result += ",";
    }

    result += String(value, 2);
  }

  return result;
}

float constrainedTargetForLift(int liftIndex, float rawTarget) {
  return constrain(rawTarget, lifts[liftIndex]->minH, lifts[liftIndex]->maxH);
}

bool targetSameAsCurrent(int liftIndex, float rawTarget) {
  float newConstrainedTarget = constrainedTargetForLift(liftIndex, rawTarget);
  float oldTarget = lifts[liftIndex]->targetHeight;
  return fabs(newConstrainedTarget - oldTarget) <= TARGET_UPDATE_EPS;
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String parts[NUM_LIFTS];

  // 注意：这里必须是 NUM_LIFTS，不能是 NUM_LIFTS + 1
  int count = splitCSV(cmd, parts, NUM_LIFTS);

  // 单数字：广播给所有升降台
  if (count == 1 && isNumericCommand(parts[0]) && cmd.indexOf(',') < 0) {
    float target = parts[0].toFloat();
    bool anyChanged = false;

    Serial.print("Broadcast target: ");
    Serial.println(target, 3);

    for (int i = 0; i < NUM_LIFTS; i++) {
      if (targetSameAsCurrent(i, target)) {
        Serial.print("[L");
        Serial.print(i);
        Serial.print("] Same target, skipped. target=");
        Serial.println(lifts[i]->targetHeight, 3);
        continue;
      }

      lifts[i]->setTarget(target);
      anyChanged = true;
    }

    if (!anyChanged) {
      Serial.println("Command ignored: all targets already match.");
    }

    return;
  }

  // 多升降台模式，数量必须等于升降台数量
  if (count != NUM_LIFTS) {
    Serial.print("Ignored input. Expected ");
    Serial.print(NUM_LIFTS);
    Serial.println(" values, or one broadcast number.");
    Serial.println("Examples:");
    Serial.println("  12.5");
    Serial.println("  12.5,18.0,20.0");
    Serial.println("  *,18.0,*");
    Serial.println("  12.5,*,20.0");
    return;
  }

  // 先检查全部格式
  for (int i = 0; i < NUM_LIFTS; i++) {
    parts[i].trim();

    if (parts[i] == "*") {
      continue;
    }

    if (!isNumericCommand(parts[i])) {
      Serial.print("Ignored input. Bad value at index ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(parts[i]);
      return;
    }
  }

  // 格式都没问题后再筛选更新
  bool anyChanged = false;

  for (int i = 0; i < NUM_LIFTS; i++) {
    if (parts[i] == "*") {
      Serial.print("[L");
      Serial.print(i);
      Serial.println("] Target unchanged by *.");
      continue;
    }

    float target = parts[i].toFloat();

    if (targetSameAsCurrent(i, target)) {
      Serial.print("[L");
      Serial.print(i);
      Serial.print("] Same target, skipped. target=");
      Serial.println(lifts[i]->targetHeight, 3);
      continue;
    }

    lifts[i]->setTarget(target);
    anyChanged = true;
  }

  if (!anyChanged) {
    Serial.println("Command ignored: no target changed.");
  }
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      line.trim();
      handleCommand(line);
      line = "";
    } else {
      line += c;

      if (line.length() > 80) {
        line = "";
        Serial.println("Input too long, cleared.");
      }
    }
  }
}

void readBluetoothCommand() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '\r') continue;

    if (c == '\n') {
      btLine.trim();

      if (btLine.length() > 0) {
        String shortCmd = keepFirstCSVValues(btLine, NUM_LIFTS);
        shortCmd = subtractOneFromCSV(shortCmd); //确保在收起状态下驱动器可以完全收缩并且触发内置限位开关
        Serial.print("[BT raw] ");
        Serial.println(btLine);
        Serial.print("[BT cmd] ");
        Serial.println(shortCmd);

        handleCommand(shortCmd);

        // 这里只表示指令已收到并录入。
        // 如果你希望等升降台真正到位后再发 DONE，需要额外加一个运动完成检测。
        Serial1.println("OK");
      }

      btLine = "";
    } else {
      btLine += c;

      if (btLine.length() > 80) {
        btLine = "";
        Serial.println("Bluetooth input too long, cleared.");
        Serial1.println("ERR");
      }
    }
  }
}

// ==================== EDITED: UI TELEMETRY START ====================
// ==============================
// UI position telemetry
// Sends one machine-readable line over HC-05 every 200 ms:
// POS,<lift0 average>,<lift1 average>,<lift2 average>
// The browser ignores all other debug text and uses only POS lines.
// ==============================
void sendBluetoothPositions() {
  static unsigned long lastSendMs = 0;
  const unsigned long now = millis();
  if (now - lastSendMs < 200UL) return;
  lastSendMs = now;

  Serial1.print("POS,");
  for (int i = 0; i < NUM_LIFTS; i++) {
    float h0 = lifts[i]->getHeight0();
    float h1 = lifts[i]->getHeight1();
    float avg = 0.5f * (h0 + h1);
    Serial1.print(avg, 2);
    if (i < NUM_LIFTS - 1) Serial1.print(',');
  }
  Serial1.println();
}
// ===================== EDITED: UI TELEMETRY END =====================

// ==============================
// Arduino setup / loop
// ==============================
void setup() {
  Serial.begin(9600);
  Serial1.begin(BT_BAUD);
  pinMode(BTS_ENABLE_PIN, OUTPUT);
  digitalWrite(BTS_ENABLE_PIN, LOW);

  for (int i = 0; i < NUM_LIFTS; i++) {
    lifts[i]->begin();
  }

  Serial.println("Multi scissor lift controller ready. All 3 lifts use BTS7960 PWM pairs 2-13.");
  Serial.println("Bluetooth ready on Serial1. HC-05 uses Mega pins 18/19.");
  Serial.println("Input examples:");
  Serial.println("  12.5            -> all lifts go to 12.5");
  Serial.println("  12.5,18.0,20.0  -> L0 to 12.5, L1 to 18.0, L2 to 20.0");
  Serial.println("  *,18.0,*        -> L0 unchanged, L1 to 18.0, L2 unchanged");
  Serial.println("  12.5,*,20.0     -> L0 to 12.5, L1 unchanged, L2 to 20.0");
  Serial.println("Bluetooth keeps first 3 CSV values. Example: 1,2,3,4 -> 1,2,3");

  for (int i = 0; i < NUM_LIFTS; i++) {
    lifts[i]->printSimpleStatus();
  }
  digitalWrite(BTS_ENABLE_PIN, HIGH);
}

void loop() {
  readSerialCommand();
  readBluetoothCommand();

  for (int i = 0; i < NUM_LIFTS; i++) {
    lifts[i]->update();
  }

  // EDITED: Stream current lift positions to the browser UI over HC-05.
  sendBluetoothPositions();
}