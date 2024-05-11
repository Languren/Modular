//library for communicating of SPI bus
#include <SPI.h>
//library for communicating with the module RFID-RC522
#include <MFRC522.h>
#define SS_PIN 5
#define RST_PIN 17
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

//LCD Screen library
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
char coords[SIZE_BUFFER];
//Keypad library and declaration
#include <Keypad.h>
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
uint8_t colPins[COLS] = { 32, 33, 25, 26 };
// pines correspondientes a las columnas
uint8_t rowPins[ROWS] = { 27, 14, 12, 13 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
//array for coords
char enteredKeys[MAX_SIZE_BLOCK];
int length = 0;

int escribir = 4;  //azul
int leer = 15;     //amarillo

void setup() {
  Serial.begin(115200);
  pinMode(leer, INPUT);
  pinMode(escribir, INPUT);

  // Init SPI bus
  SPI.begin();

  lcd.begin();
  lcd.clear();
  lcd.backlight();  // Make sure backlight is on

  // Init MFRC522
  mfrc522.PCD_Init();
  delay(2000);

  // Print a message on both lines of the LCD.
  lcd.setCursor(0, 0);  //Set cursor to character 0 on line 0
  lcd.print("Acerca tu tarjeta");
}
void loop() {
  //waiting the card approach
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Select a card
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  //call menu function and retrieve the desired option
  int op = menu();
  if (op == 0) {
    readingData();  //function for reading
  } else {
    if (op == 1) {
      writingData();  //function for writing
    } else {
      if (op < 9) {
        lcd.clear();  //clear LCD
        lcd.setCursor(0, 0);
        lcd.print("Incorrect Option!");
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Time out!");
      }
    }
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Retira tu tarjeta");
  delay(3000);
  // display info
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Acerca tu tarjeta");
  //instructs the PICC when in the ACTIVE state to go to a "STOP" state
  mfrc522.PICC_HaltA();
  // "stop" the encryption of the PCD
  // it must be called after communication with authentication
  // otherwise new communications can not be initiated
  mfrc522.PCD_StopCrypto1();
}
//reads data from card/tag
void readingData() {

  //prepare the key - all keys are set to FFFFFFFFFFFFh/ llave para lectura de tarjeta
  for (byte i = 0; i < 6; i++)
    key.keyByte[i] = 0xFF;

  //buffer for read data
  byte buffer[SIZE_BUFFER] = { 0 };
  //the block to operate
  byte block = 1;
  byte size = SIZE_BUFFER;
  //authenticates the block to operate
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block,
                                    &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Verificacion fallida");
    delay(2000);
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Verificacion exitosa");
    delay(2000);
    Serial.println(mfrc522.GetStatusCodeName(status));
  }
  //read data from block
  status = mfrc522.MIFARE_Read(block, buffer, &size);
  //status of reading
  if (status != MFRC522::STATUS_OK) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lectura fallida ");
    delay(2000);
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lectura exitosa");
    Serial.println(mfrc522.GetStatusCodeName(status));
    delay(2000);
  }
  //prints read data
  for (uint8_t i = 0; i < MAX_SIZE_BLOCK; i++) {
    // Serial.write(buffer[i]); para escribir en el serial
    coords[i] = buffer[i];
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Datos:");
  lcd.setCursor(0, 1);
  lcd.print(coords);
  delay(2000);
}

// write data to the card
void writingData() {
  //Impresion en LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ingresa la ruta con");
  lcd.setCursor(0, 1);
  lcd.print("'#' al final");
  lcd.setCursor(0, 2);
  lcd.print("max de 16");
  //prepare the key - all keys are set to FFFFFFFFFFFFh/ La llave para leer la tarjeta
  for (byte i = 0; i < 6; i++)
    key.keyByte[i] = 0xFF;
  //buffer for storing data to write
  byte buffer[MAX_SIZE_BLOCK] = "";
  //the block to operate1
  byte block = 1;
  //size of data (bytes)
  byte dataSize;
  // lectura de teclado matricial y conversion en array
  while (enteredKeys[length] != '#') {
    char keyp = keypad.getKey();
    if (keyp) {
      storeKey(keyp);
      lcd.setCursor(0, 3);
      lcd.print(enteredKeys);  //muestra el array
    }
  }
  //completa el array con espacios vacios
  for (byte i = length; i < MAX_SIZE_BLOCK; i++) {
    enteredKeys[i] = ' ';
  }

  String keysString = String(enteredKeys);  //conversion de array a string

  char keysArray[MAX_SIZE_BLOCK];
  keysString.toCharArray(keysArray, MAX_SIZE_BLOCK);  //conversion de ar ray a buffer

  //authenticates the block to operate
  //Authenticate is a command to hability a secure communication
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                    block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Validacion fallida");
    delay(2000);
    Serial.println(mfrc522.GetStatusCodeName(status));
    clearKeys();
    return;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Validacion exitosa");
    delay(2000);
    Serial.println(mfrc522.GetStatusCodeName(status));
  }

  status = mfrc522.MIFARE_Write(block, (byte *)keysArray, MAX_SIZE_BLOCK);  //Writes in the block

  if (status != MFRC522::STATUS_OK)  //valida si se leyo exitosamente
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Escritura fallida");
    delay(2000);
    Serial.println(mfrc522.GetStatusCodeName(status));
    clearKeys();
    return;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);  //Set cursor to character 2 on line 0
    lcd.print("Escritura exitosa");
    delay(2000);
    Serial.println(mfrc522.GetStatusCodeName(status));
    clearKeys();
  }
}
//menu to operation choice
int menu() {
  unsigned long timestart = millis();
  unsigned long now = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Lectura");
  lcd.setCursor(0, 2);
  lcd.print("Escritura");

  //retrieves the chosen option
  while (1) {
    if (digitalRead(leer) == LOW) {
      return 0;
    } else if (digitalRead(escribir) == HIGH) {
      return 1;
    }
  };
}

// guardar en un array
void storeKey(char key) {
  // Get the length of the array
  length = strlen(enteredKeys);
  // Add the key to the array
  enteredKeys[length] = key;
}

void clearKeys() {
  // Set all elements of the array to 0
  memset(enteredKeys, 0, sizeof(enteredKeys));
}