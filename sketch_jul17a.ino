#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14

PS2X ps2x;

const int motorLeftForward = 10;
const int motorLeftBackward = 11;
const int motorRightForward = 12;
const int motorRightBackward = 13;

const int MOTOR_LEFT_FORWARD = 8;
const int MOTOR_LEFT_BACKWARD= 9;
const int MOTOR_RIGHT_FORWARD = 14;
const int MOTOR_RIGHT_BACKWARD = 15;

#define SERVO360_1_CHANNEL  2
#define SERVO360_2_CHANNEL  3
#define SERVO180_1_CHANNEL  4

#define BTN_LEFT     PSB_R1
#define BTN_RIGHT    PSB_R2

#define PWM_STOP 307
#define PWM_RUN_left  370
#define PWM_RUN_right 260

// -------------------- Các hàm chức năng --------------------
void stopDriveMotors() {
  pwm.setPWM(motorLeftForward, 0, 0);
  pwm.setPWM(motorLeftBackward, 0, 0);
  pwm.setPWM(motorRightForward, 0, 0);
  pwm.setPWM(motorRightBackward, 0, 0);
}

void stopLiftMotors() {
  pwm.setPWM(MOTOR_LEFT_FORWARD, 0, 0);
  pwm.setPWM(MOTOR_LEFT_BACKWARD, 0, 0);
  pwm.setPWM(MOTOR_RIGHT_FORWARD, 0, 0);
  pwm.setPWM(MOTOR_RIGHT_BACKWARD, 0, 0);
}

void stopServo() {
  pwm.setPWM(SERVO360_1_CHANNEL, 0, PWM_STOP);
  pwm.setPWM(SERVO360_2_CHANNEL, 0, PWM_STOP);
}

int constrainPWM(int value) {
  return constrain(value, 0, 4095);
}

// -------------------- Hàm khởi tạo --------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Dang ket noi PS2...");

  int error = -1;
  for (int i = 0; i < 10; i++) {
    delay(100);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
    Serial.print(".");
    if (!error) break;
  }

  if (error) {
    Serial.println("\nKhong the ket noi tay cam PS2.");
    while (true);
  }

  Serial.println("\nDa ket noi thanh cong tay cam PS2!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  stopDriveMotors();
  stopLiftMotors();
  stopServo();
}

// -------------------- Hàm liên tục  --------------------
void loop() {
  ps2x.read_gamepad();

  // --------- 1. ĐIỀU KHIỂN DI CHUYỂN ----------
  int joyLY = ps2x.Analog(PSS_LY);
  int joyRY = ps2x.Analog(PSS_RY);
  int leftY = 128 - joyLY;
  int rightY = 128 - joyRY;

  int pwmLeft = map(abs(leftY), 0, 128, 0, 1500);
  int pwmRight = map(abs(rightY), 0, 128, 0, 1500);

  if (leftY > 10) {
    pwm.setPWM(motorLeftForward, 0, pwmLeft);
    pwm.setPWM(motorLeftBackward, 0, 0);
  } else if (leftY < -10) {
    pwm.setPWM(motorLeftForward, 0, 0);
    pwm.setPWM(motorLeftBackward, 0, pwmLeft);
  } else {
    pwm.setPWM(motorLeftForward, 0, 0);
    pwm.setPWM(motorLeftBackward, 0, 0);
  }

  if (rightY > 10) {
    pwm.setPWM(motorRightForward, 0, pwmRight);
    pwm.setPWM(motorRightBackward, 0, 0);
  } else if (rightY < -10) {
    pwm.setPWM(motorRightForward, 0, 0);
    pwm.setPWM(motorRightBackward, 0, pwmRight);
  } else {
    pwm.setPWM(motorRightForward, 0, 0);
    pwm.setPWM(motorRightBackward, 0, 0);
  }

  // --------- 2. ĐIỀU KHIỂN NÂNG ----------
  if (ps2x.Button(PSB_L1)) {
    pwm.setPWM(MOTOR_LEFT_FORWARD, 0, 1750);
    pwm.setPWM(MOTOR_LEFT_BACKWARD, 0, 0);
    pwm.setPWM(MOTOR_RIGHT_FORWARD, 0, 1500);
    pwm.setPWM(MOTOR_RIGHT_BACKWARD, 0, 0);
  }
  else if (ps2x.Button(PSB_L2)) {
    pwm.setPWM(MOTOR_LEFT_FORWARD, 0, 0);
    pwm.setPWM(MOTOR_LEFT_BACKWARD, 0, 1750);
    pwm.setPWM(MOTOR_RIGHT_FORWARD, 0, 0);
    pwm.setPWM(MOTOR_RIGHT_BACKWARD, 0, 1500);
  }
  else {
    stopLiftMotors();
  }

  // --------- 3. ĐIỀU KHIỂN SERVO 360 ĐỘ ----------
  bool servo1Running = false;
  if (ps2x.Button(BTN_PAD_UP) {
    pwm.writeMicroseconds(SERVO360_1_CHANNEL, 1000);
    //servo1Running = true;
  } else {
    pwm.writeMicroseconds(SERVO360_1_CHANNEL, 1500);
  }

  
  if (ps2x.Button(BTN_PAD_DOWN)) {
    pwm.writeMicroseconds(SERVO360_1_CHANNEL, 2000);
    //servo1Running = true;
  } else {
    pwm.writeMicroseconds(SERVO360_1_CHANNEL, 1500);
  }

  bool servo2Running = false;
  if (ps2x.Button(PSB_PAD_RIGHT)) {
    pwm.writeMicroseconds(SERVO360_2_CHANNEL, 1000);
    //servo1Running = true;
  } else {
    pwm.writeMicroseconds(SERVO360_2_CHANNEL, 1500);
  }
  if (ps2x.Button(PSB_PAD_DOWN)) {
    pwm.writeMicroseconds(SERVO360_1_CHANNEL, 2000);
    //servo1Running = true;
  } else {
    pwm.writeMicroseconds(SERVO360_1_CHANNEL, 1500);
  }
 
 // if (!servo1Running && !servo2Running) {
   // stopServo();
  //}

  // --------- 4. ĐIỀU KHIỂN SERVO TREO ----------
  if (ps2x.Button(PSB_PINK)) {
    pwm.setPWM(4, 0, map(90, 0, 180, 150, 600));
  }
  if (ps2x.Button(PSB_BLUE)) {
    pwm.setPWM(4, 0, map(0, 0, 180, 150, 600));
  }
}
