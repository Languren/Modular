#include <Arduino.h>
#include <BluetoothSerial.h>

#define PWM_FREQUENCY 1000  // PWM frequency in Hz
#define PWM_RESOLUTION 10   // PWM resolution, can be between 1-16

#define IN1_A 23  //v1 delante
#define IN2_A 22  //v1 atras
#define IN1_B 32  //v2 delante
#define IN2_B 33  //v2 atras
#define IN1_D 26  //v4 delante
#define IN2_D 25  //v4 atras
#define IN1_C 19  //v3 delante
#define IN2_C 21  //v3 atras

#define FRASE1 15
#define FRASE2 13
#define FRASE3 12

BluetoothSerial SerialBT;  //inicializa objeto SerialBT

HardwareSerial Receiver(2);  // Define a Serial port instance called 'Receiver' using serial port 2

//definir pines Tx y Rx
#define Receiver_Txd_pin 17
#define Receiver_Rxd_pin 16

//definir buffer de info recibida (coords)
#define MAX_SIZE_BLOCK 16

//Buffer de lectura de coordenadas
#define SIZE_BUFFER 18
#define MAX_SIZE_BLOCK 16
byte buffer[SIZE_BUFFER] = { 0 };

//array donde llegaran las velocidades
char myData[30] = { 0 };
int v1 = 0, v2 = 0, v3 = 0, v4 = 0;

const int finalC = 27;  //final de carrera
int a = 1;              // GPIO pin to configure the interrupt

int bocina = 0;  //audios de bocina 1.-final trayectoria, 2.-inicio de trayectoria, 3.-punto medio de llegada

void setup() {
  Serial.begin(9600);
  Receiver.begin(9600, SERIAL_8N1, Receiver_Txd_pin, Receiver_Rxd_pin);  // Define and start Receiver serial port
  SerialBT.begin("ESP32_BT_2");                                          //inicializa bluetooth como ESP_BT_2

  pinMode(finalC, INPUT);  //final de carrera

  //inicializacion de pines para motores
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  pinMode(IN1_C, OUTPUT);
  pinMode(IN2_C, OUTPUT);
  pinMode(IN1_D, OUTPUT);
  pinMode(IN2_D, OUTPUT);
  pinMode(FRASE1, OUTPUT);
  pinMode(FRASE2, OUTPUT);
  pinMode(FRASE3, OUTPUT);
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, LOW);
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, LOW);
  digitalWrite(FRASE1, HIGH);
  digitalWrite(FRASE2, HIGH);
  digitalWrite(FRASE3, HIGH);

  // Configuracion canales para PWM
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(4, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(5, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(6, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(7, PWM_FREQUENCY, PWM_RESOLUTION);

  //configuracion de pines con canales
  ledcAttachPin(IN1_A, 0);
  ledcAttachPin(IN2_A, 1);
  ledcAttachPin(IN1_B, 2);
  ledcAttachPin(IN2_B, 3);
  ledcAttachPin(IN1_C, 4);
  ledcAttachPin(IN2_C, 5);
  ledcAttachPin(IN1_D, 6);
  ledcAttachPin(IN2_D, 7);
}

void loop() {
  //espera la informacion del RFID
  while (a <= 16) {
    if (Receiver.available()) {       // Wait for the Receiver to get the characters
      byte coords = Receiver.read();  // Display the Receivers characters
      Serial.write(coords);
      SerialBT.write(coords);
      delay(100);  // Display the result on the serial monitor
      a = a + 1;
    }
    //Serial.println(a);
  };
  // Espera a que haya info de bluetooth
  if (SerialBT.available()) {
    //lectura de velocidades que le llegan por BT
    byte m = SerialBT.readBytesUntil('\n', myData, 50);
    myData[m] = '\0';
    //Serial.print(myData);
    v1 = atof(strtok(myData, ","));
    v2 = atof(strtok(NULL, ","));
    v3 = atof(strtok(NULL, ","));
    v4 = atof(strtok(NULL, ","));
    bocina = atof(strtok(NULL, ","));

    //envia las velocidades a las llantas
    if (v1 > 0) {
      ledcWrite(1, 0);
      ledcWrite(0, v1);
    }
    if (v1 == 0) {
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
    if (v1 < 0) {
      ledcWrite(0, 0);
      ledcWrite(1, -v1);
    }

    if (v2 > 0) {
      ledcWrite(3, 0);
      ledcWrite(2, v2);
    }
    if (v2 == 0) {
      ledcWrite(2, 0);
      ledcWrite(3, 0);
    }
    if (v2 < 0) {
      ledcWrite(2, 0);
      ledcWrite(3, -v2);
    }


    if (v3 > 0) {
      ledcWrite(5, 0);
      ledcWrite(4, v3);
    }
    if (v3 == 0) {
      ledcWrite(4, 0);
      ledcWrite(5, 0);
    }
    if (v3 < 0) {
      ledcWrite(4, 0);
      ledcWrite(5, -v3);
    }

    if (v4 > 0) {
      ledcWrite(7, 0);
      ledcWrite(6, v4);
    }
    if (v4 == 0) {
      ledcWrite(6, 0);
      ledcWrite(7, 0);
    }
    if (v4 < 0) {
      ledcWrite(6, 0);
      ledcWrite(7, -v4);
    }

    //lee el ultimo digito para enviarlo a la bocina
    if (bocina == 1) {
      digitalWrite(FRASE1, LOW);
      a = 1;
      delay(2000);
    }
    if (bocina == 2) {
      digitalWrite(FRASE2, LOW);
      delay(2000);
    }
    if (bocina == 3) {
      digitalWrite(FRASE3, LOW);
      delay(2000);
    } else {
      digitalWrite(FRASE1, HIGH);
      digitalWrite(FRASE2, HIGH);
      digitalWrite(FRASE3, HIGH);
    }
  }
}