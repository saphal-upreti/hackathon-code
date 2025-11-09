#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ps3Controller.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels
#define SERVO1 3
#define SERVO2 7
#define SERVO3 11
#define SERVO4 15

#define SERVO_STOP     384
#define SERVO_FORWARD  320
#define SERVO_BACKWARD 430

bool systemEnabled = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(100);
  
  // FORCE STOP all servos multiple times
  for(int i = 0; i < 5; i++) {
    setAllServos(SERVO_STOP);
    delay(100);
  }
  
  Ps3.begin("a0:5a:5b:a0:07:cb");
  Serial.println("=================================");
  Serial.println("System LOCKED - Servos STOPPED");
  Serial.println("Press X button to enable control");
  Serial.println("=================================");
}

void loop() {
  // ALWAYS stop servos first
  setAllServos(SERVO_STOP);
  
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
    }
    
    if (!systemEnabled) {
      Serial.println("System disabled - Press X to enable");
      delay(500);
      return;
    }
    
    // Get stick values
    int leftY = Ps3.data.analog.stick.ly;
    int rightX = Ps3.data.analog.stick.rx;
    
    Serial.print("LY: "); Serial.print(leftY);
    Serial.print(" RX: "); Serial.print(rightX);
    
    int deadzone = 30;
    
    // Check deadzone
    if (abs(leftY - 128) < deadzone && abs(rightX - 128) < deadzone) {
      Serial.println(" | CENTERED - STOP");
      delay(50);
      return;
    }
    
    // Map stick input
    int forward = map(leftY, 0, 255, 100, -100);
    int turn = map(rightX, 0, 255, -60, 60);
    
    int leftPower  = constrain(forward + turn, -100, 100);
    int rightPower = constrain(forward - turn, -100, 100);
    
    int speedFL = map(leftPower, -100, 100, SERVO_BACKWARD, SERVO_FORWARD);
    int speedBL = speedFL;
    int speedFR = map(rightPower, -100, 100, SERVO_BACKWARD, SERVO_FORWARD);
    int speedBR = speedFR;
    
    // NOW set the actual speeds
    pwm.setPWM(SERVO1, 0, speedFL);
    pwm.setPWM(SERVO2, 0, speedFR);
    pwm.setPWM(SERVO3, 0, speedBL);
    pwm.setPWM(SERVO4, 0, speedBR);
    
    Serial.print(" | FL: "); Serial.print(speedFL);
    Serial.print(" FR: "); Serial.print(speedFR);
    Serial.println(" MOVING");
    
    delay(50);
    
  } else {
    Serial.println("Controller NOT connected");
    delay(200);
  }
}

void setAllServos(int pulse) {
  pwm.setPWM(SERVO1, 0, pulse);
  pwm.setPWM(SERVO2, 0, pulse);
  pwm.setPWM(SERVO3, 0, pulse);
  pwm.setPWM(SERVO4, 0, pulse);
}