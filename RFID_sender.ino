//library for communicating of SPI bus
#include <SPI.h>
//library for communicating with the module RFID-RC522
#include <MFRC522.h>
#include "DFRobotDFPlayerMini.h"
#define SS_PIN 21
#define RST_PIN 15
#define SIZE_BUFFER 18
#define MAX_SIZE_BLOCK 16
// time to wait for serial input = 10 secs
#define TIME_OUT 10000L
//used in authentication
MFRC522::MIFARE_Key key;
//authentication return status
MFRC522::StatusCode status;
// Defined pins to module RC522
MFRC522 mfrc522(SS_PIN, RST_PIN);

HardwareSerial Sender(1);  // Define a Serial port instance called 'Sender' using serial port 1
HardwareSerial Bocinas(2);
DFRobotDFPlayerMini myDFPlayer;
//HardwareSerial Receiver(2); // Define a Serial port instance called 'Receiver' using serial port 2

//Definir Tx y Rx de ambas tarjetas
#define Sender_Txd_pin 17
#define Sender_Rxd_pin 16

#define Bocinas_Txd_pin 12
#define Bocinas_Rxd_pin 14

#define FRASE1 35
#define FRASE2 27
#define FRASE3 26

//GPIO de interrupciones
int a = 0;

void setup() {
  pinMode(FRASE1, INPUT);
  pinMode(FRASE2, INPUT);
  pinMode(FRASE3, INPUT);
  delay(1000);
  Serial.begin(9600);


  // Definir puertos seriales para comunicacion UART
  Sender.begin(9600, SERIAL_8N1, Sender_Txd_pin, Sender_Rxd_pin);
  Bocinas.begin(9600, SERIAL_8N1, Bocinas_Rxd_pin, Bocinas_Txd_pin);  // Define and start Sender serial port
  Serial.println("El dispositivo está listo para emparejar");
  // Init SPI bus
  SPI.begin();
  // Init MFRC522
  mfrc522.PCD_Init();

  // display info
  Serial.println("Approach your card or tag...");

  myDFPlayer.setTimeOut(500);  //Set serial communictaion time out 500ms

  if (!myDFPlayer.begin(Bocinas)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true)
      ;
  }
  //----Set volume----
  myDFPlayer.volume(30);  //Set volume value (0~30).

  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);

  //----Set device we use SD as default----
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  delay(3000);
}


void loop() {

  //waiting the card approach

  if (!mfrc522.PICC_IsNewCardPresent()) {
    if (digitalRead(FRASE1) == LOW) {
      myDFPlayer.play(1);
      delay(4000);
      Serial.print("A");
    }
    if (digitalRead(FRASE2) == LOW){
      myDFPlayer.play(2);
      delay(4000);
      Serial.print("B");
    }
    if (digitalRead(FRASE3) == LOW) {
      myDFPlayer.play(3);
      delay(4000);
      Serial.print("C");
    }
    Serial.print(digitalRead(FRASE1));
    Serial.print(digitalRead(FRASE2));
    Serial.println(digitalRead(FRASE3));
    return;
  }

  // Select a card
  if (!mfrc522.PICC_ReadCardSerial()) {
    //Serial.println("4");
    return;
  }

  readingData();  //leer datos de tarjeta NFC
  //instructs the PICC when in the ACTIVE state to go to a "STOP" state
  mfrc522.PICC_HaltA();
  // "stop" the encryption of the PCD
  // it must be called after communication with authentication
  // otherwise new communications can not be initiated
  mfrc522.PCD_StopCrypto1();
}
void readingData() {
  //prints the technical details of the card/tag
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid));
  //prepare the key - all keys are set to FFFFFFFFFFFFh
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  //buffer for read data
  byte buffer[SIZE_BUFFER] = { 0 };
  //the block to operate
  byte block = 1;
  byte size = SIZE_BUFFER;
  //authenticates the block to operate
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block,
                                    &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentication failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  } else {
    Serial.print("Authentication successful: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
  }
  //read data from block
  status = mfrc522.MIFARE_Read(block, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Reading failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  } else {
    Serial.print("Reading successful: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
  }
  Serial.print("\nData from block [");
  Serial.print(block);
  Serial.print("]: ");
  //prints read data
  Sender.write(buffer, MAX_SIZE_BLOCK);  //enviar a otra esp los datos
  Serial.write(buffer, MAX_SIZE_BLOCK);  //enviar a otra esp los datos
  myDFPlayer.play(4);
  delay(4000);
  return;
}