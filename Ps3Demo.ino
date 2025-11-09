#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ps3Controller.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels
#define SERVO3 3
#define SERVO7 7
#define SERVO11 11
#define SERVO15 15

#define SERVO_STOP     383
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
  
  // FORCE STOP all servos
  pwm.setPWM(SERVO3, 0, 383);
  pwm.setPWM(SERVO7, 0, 383);
  pwm.setPWM(SERVO11, 0, 384);
  pwm.setPWM(SERVO15, 0, 383);
  delay(500);
  
  Ps3.begin("a0:5a:5b:a0:07:cb");
  Serial.println("=================================");
  Serial.println("System LOCKED - Servos STOPPED");
  Serial.println("Press X button to enable control");
  Serial.println("=================================");
}

void loop() {
  if (Ps3.isConnected()) {
    
    // Press X button to enable system
    if (Ps3.data.button.cross) {
      systemEnabled = true;
      Serial.println(">>> SYSTEM ENABLED <<<");
    }
    
    // Press Circle button to disable
    if (Ps3.data.button.circle) {
      systemEnabled = false;
      Serial.println(">>> SYSTEM DISABLED <<<");
      setAllServos(SERVO_STOP);
    }
    
    if (!systemEnabled) {
      setAllServos(SERVO_STOP);
      Serial.println("System disabled - Press X to enable");
      delay(500);
      return;
    }
    
    // Get stick values (-128 to +127 range for PS3)
    int leftY = Ps3.data.analog.stick.ly;
    int rightX = Ps3.data.analog.stick.rx;
    
    Serial.print("LY: "); Serial.print(leftY);
    Serial.print(" RX: "); Serial.print(rightX);
    
    int deadzone = 20;
    
    // Check deadzone - center is 0 for PS3 sticks!
    if (abs(leftY) < deadzone && abs(rightX) < deadzone) {
      setAllServos(SERVO_STOP);
      Serial.println(" | CENTERED - STOPPED");
      delay(50);
      return;
    }
    
    // Map stick input from -128 to +127 → -100 to +100
    int forward = map(leftY, -128, 127, -100, 100);
    int turn = map(rightX, -128, 127, -100, 100);  // Increased turn range to match forward
    
    // Tank drive mixing - CRITICAL CHANGE HERE
    int leftPower  = constrain(forward + turn, -100, 100);
    int rightPower = constrain(forward - turn, -100, 100);
    
    // Convert to PWM values
    int speedFL = map(leftPower, -100, 100, SERVO_BACKWARD, SERVO_FORWARD);
    int speedBL = speedFL;
    int speedFR = map(rightPower, -100, 100, SERVO_BACKWARD, SERVO_FORWARD);
    int speedBR = speedFR;
    
    // Set the actual speeds
    pwm.setPWM(SERVO3, 0, speedFL);
    pwm.setPWM(SERVO7, 0, speedFR);
    pwm.setPWM(SERVO11, 0, speedBL);
    pwm.setPWM(SERVO15, 0, speedBR);
    
    Serial.print(" | Forward: "); Serial.print(forward);
    Serial.print(" Turn: "); Serial.print(turn);
    Serial.print(" | LeftPwr: "); Serial.print(leftPower);
    Serial.print(" RightPwr: "); Serial.print(rightPower);
    Serial.print(" | FL: "); Serial.print(speedFL);
    Serial.print(" FR: "); Serial.print(speedFR);
    Serial.println();
    
    delay(50);
    
  } else {
    // Controller NOT connected - STOP servos
    setAllServos(SERVO_STOP);
    Serial.println("Controller NOT connected - STOPPED");
    delay(200);
  }
}

void setAllServos(int pulse) {
  pwm.setPWM(SERVO3, 0, pulse);
  pwm.setPWM(SERVO7, 0, pulse);
  pwm.setPWM(SERVO11, 0, pulse == 383 ? 384 : pulse);
  pwm.setPWM(SERVO15, 0, pulse);
}