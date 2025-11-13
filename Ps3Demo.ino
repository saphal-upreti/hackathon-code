#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ps3Controller.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels
#define SERVO_FL 3   // Front Left
#define SERVO_FR 7   // Front Right
#define SERVO_BL 11  // Back Left
#define SERVO_BR 15  // Back Right

// Servo PWM values
#define SERVO_STOP     364
#define SERVO_FORWARD  150
#define SERVO_BACKWARD 500

bool systemEnabled = false;

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  delay(100);
  
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(100);
  
  // Stop all servos initially
  stopAllServos();
  delay(500);
  
  Ps3.begin("a0:5a:5b:a0:07:cb");
  Serial.println("=================================");
  Serial.println("System LOCKED - Servos STOPPED");
  Serial.println("Press X button to ENABLE control");
  Serial.println("Press O button to DISABLE control");
  Serial.println("=================================");
}

void loop() {
  if (Ps3.isConnected()) {

    // Enable control
    if (Ps3.data.button.cross) {
      systemEnabled = true;
      Serial.println(">>> SYSTEM ENABLED <<<");
    }

    // Disable control
    if (Ps3.data.button.circle) {
      systemEnabled = false;
      Serial.println(">>> SYSTEM DISABLED <<<");
      stopAllServos();
    }

    if (!systemEnabled) {
      stopAllServos();
      delay(200);
      return;
    }

    // Get joystick inputs
    int leftY = Ps3.data.analog.stick.ly;  // Forward/Backward
    int rightX = Ps3.data.analog.stick.rx; // Turning

    int deadzone = 20;
    if (abs(leftY) < deadzone && abs(rightX) < deadzone) {
      stopAllServos();
      Serial.println("CENTERED - STOPPED");
      delay(50);
      return;
    }

    // Map to -100..100
    int forward = map(rightX, -128, 127, -100, 100);
    int turn = map(leftY, -128, 127, -100, 100);

    // Tank drive mixing
    int leftPower  = constrain(forward + turn, -100, 100);
    int rightPower = constrain(forward - turn, -100, 100);

    // Convert to PWM values
    int pwmLeft  = map(leftPower, -100, 100, SERVO_BACKWARD, SERVO_FORWARD);
    int pwmRight = map(rightPower, -100, 100, SERVO_BACKWARD, SERVO_FORWARD);

    // Apply direction correction (right side reversed)
    pwm.setPWM(SERVO_FL, 0, pwmLeft);
    pwm.setPWM(SERVO_BL, 0, pwmLeft);
    pwm.setPWM(SERVO_FR, 0, 766 - pwmRight); // invert
    pwm.setPWM(SERVO_BR, 0, 766 - pwmRight); // invert

    Serial.print("Forward: "); Serial.print(forward);
    Serial.print(" Turn: "); Serial.print(turn);
    Serial.print(" | LeftPWM: "); Serial.print(pwmLeft);
    Serial.print(" RightPWM: "); Serial.println(pwmRight);

    delay(50);

  } else {
    stopAllServos();
    Serial.println("Controller NOT connected - STOPPED");
    delay(300);
  }
}

void stopAllServos() {
void setAllServosStop() {
  pwm.setPWM(SERVO3, 0, 364);
  pwm.setPWM(SERVO7, 0, 364);
  pwm.setPWM(SERVO11, 0, 365);
  pwm.setPWM(SERVO15, 0, 364);
}
}
