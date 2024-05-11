#include <Arduino.h>
#include <BluetoothSerial.h>

#define IN1_A 23
#define IN2_A 22
#define IN1_B 32
#define IN2_B 33
#define IN1_C 19
#define IN2_C 21
#define IN1_D 26
#define IN2_D 25

#define PWM_FREQUENCY 1000 // PWM frequency in Hz
#define PWM_RESOLUTION 8   // PWM resolution, can be between 1-16

char command;

BluetoothSerial SerialBT; // Bluetooth object

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT"); // Initialize Bluetooth

  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  pinMode(IN1_C, OUTPUT);
  pinMode(IN2_C, OUTPUT);
  pinMode(IN1_D, OUTPUT);
  pinMode(IN2_D, OUTPUT);
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, LOW);
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, LOW);

  // Configure PWM channels
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(4, PWM_FREQUENCY, PWM_RESOLUTION);  
  ledcSetup(5, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(6, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(7, PWM_FREQUENCY, PWM_RESOLUTION);


  ledcAttachPin(IN1_A, 0);
  ledcAttachPin(IN2_A, 1); //v1 atras
  ledcAttachPin(IN1_B, 2);
  ledcAttachPin(IN2_B, 3); //v2 atras
  ledcAttachPin(IN1_C, 4);
  ledcAttachPin(IN2_C, 5);  //v3 atras
  ledcAttachPin(IN1_D, 6);
  ledcAttachPin(IN2_D, 7); //v4 atras
}

void loop() {
  if (SerialBT.available()) { // Check if there's data available over Bluetooth
    command = SerialBT.read(); // Read the command received over Bluetooth
    executeCommand(command);   // Execute the received command
  }
  
}

void executeCommand(char command) {
  switch (command) {
    case 'F':
      moveForward();
      Serial.println("Forward");
      break;
    case 'B':
      moveBackward();
      Serial.println("Backward");
      break;
    case 'L':
      turnLeft();
      Serial.println("Left");
      break;
    case 'R':
      turnRight();
      Serial.println("Right");
      break;
    case 'S':
      stopMotion();
      Serial.println("Stop");
      break;
    case '1':
      v1();
      Serial.println("v1");
      break;
    case '2':
      v2();
      Serial.println("v2");
      break;
    case '3':
      v3();
      Serial.println("v3");
      break;
    case '4':
      v4();
      Serial.println("v4");
      break;
    default:
      break;
  }
}

void moveForward() {
  ledcWrite(0, 100);
  ledcWrite(1, 0);
  ledcWrite(2, 100);
  ledcWrite(3, 0);
  ledcWrite(4, 100);
  ledcWrite(5, 0);
  ledcWrite(6, 100);
  ledcWrite(7, 0);
}

void moveBackward() {
  ledcWrite(0, 0);
  ledcWrite(1, 100);
  ledcWrite(2, 0);
  ledcWrite(3, 100);
  ledcWrite(4, 0);
  ledcWrite(5, 100);
  ledcWrite(6, 0);
  ledcWrite(7, 100);

}

void turnLeft() {
  ledcWrite(0, 0);
  ledcWrite(1, 150);
  ledcWrite(2, 150);
  ledcWrite(3, 0);
  ledcWrite(4, 150);
  ledcWrite(5, 0);
  ledcWrite(6, 0);
  ledcWrite(7, 150);
}

void turnRight() {
  ledcWrite(0, 150);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 150);
  ledcWrite(4, 0);
  ledcWrite(5, 150);
  ledcWrite(6, 150);
  ledcWrite(7, 0);
}

void stopMotion() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  ledcWrite(4, 0);
  ledcWrite(5, 0);
  ledcWrite(6, 0);
  ledcWrite(7, 0);
}

void v1() {
  ledcWrite(0, 0);
  ledcWrite(1, 150);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  ledcWrite(4, 0);
  ledcWrite(5, 0);
  ledcWrite(6, 0);
  ledcWrite(7, 0);
}

void v2() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 150);
  ledcWrite(4, 0);
  ledcWrite(5, 0);
  ledcWrite(6, 0);
  ledcWrite(7, 0);
}

void v3() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  ledcWrite(4, 0);
  ledcWrite(5, 150);
  ledcWrite(6, 0);
  ledcWrite(7, 0);
}

void v4() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  ledcWrite(4, 0);
  ledcWrite(5, 0);
  ledcWrite(6, 0);
  ledcWrite(7, 150);
}

