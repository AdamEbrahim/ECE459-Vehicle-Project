#include <Wire.h>

// I2C address
#define I2C_ADDR 0x48

// Motor pins
const int MOTOR_A1 = 25;  // Left motor
const int MOTOR_A2 = 26;
const int MOTOR_B1 = 32;  // Right motor
const int MOTOR_B2 = 33;
const int PWM_A = 14;    // Left motor speed
const int PWM_B = 27;    // Right motor speed

// PWM properties
const int FREQ = 5000;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_RESOLUTION = 8;

// Speed settings
const int SPEED_SLOW = 128;     // 50% duty cycle
const int SPEED_NORMAL = 192;   // 75% duty cycle
const int SPEED_MAX = 255;      // 100% duty cycle

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  
  // Configure motor pins
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  // Configure PWM
  ledcSetup(PWM_CHANNEL_A, FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_A, PWM_CHANNEL_A);
  ledcAttachPin(PWM_B, PWM_CHANNEL_B);
  
  Serial.println("Motor controller initialized");
}

void loop() {
  // Main loop is empty as we use I2C interrupts
  delay(10);
}

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    byte command = Wire.read();
    processCommand(command);
  }
}

void processCommand(byte command) {
  // Extract speed modifier (bits 7-4)
  byte speedMod = command & 0xF0;
  // Extract base command (bits 3-0)
  byte baseCmd = command & 0x0F;
  
  // Determine speed based on modifier
  int speed;
  if (speedMod == 0x10) {  // Slow
    speed = SPEED_SLOW;
  }
  else if (speedMod == 0x20) {  // Normal
    speed = SPEED_NORMAL;
  }
  else if (speedMod == 0x30) {  // Custom
    // Convert 4-bit value (0-15) to full range (0-255)
    byte speedValue = command & 0x0F;
    speed = (speedValue * SPEED_MAX) / 15;
  }
  else {
    speed = SPEED_NORMAL;  // Default to normal speed
  }
  
  // Process base command
  switch (baseCmd) {
    case 0x00:  // STOP
      stopMotors();
      break;
    case 0x01:  // FORWARD
      moveForward(speed);
      break;
    case 0x02:  // LEFT
      turnLeft(speed);
      break;
    case 0x03:  // RIGHT
      turnRight(speed);
      break;
    case 0x04:  // BACKWARD
      moveBackward(speed);
      break;
    default:
      stopMotors();
      break;
  }
}

void stopMotors() {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}

void moveForward(int speed) {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
  ledcWrite(PWM_CHANNEL_A, speed);
  ledcWrite(PWM_CHANNEL_B, speed);
}
