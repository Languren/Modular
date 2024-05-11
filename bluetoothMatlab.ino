#include <BluetoothSerial.h>

BluetoothSerial SerialBT;


HardwareSerial Sender(1);   // Define a Serial port instance called 'Sender' using serial port 1

HardwareSerial Receiver(2); // Define a Serial port instance called 'Receiver' using serial port 2

#define Sender_Txd_pin 17
#define Sender_Rxd_pin 16

#define Receiver_Txd_pin 17
#define Receiver_Rxd_pin 16

#define MAX_SIZE_BLOCK 16

 #define SIZE_BUFFER 18
#define MAX_SIZE_BLOCK 16
byte buffer[SIZE_BUFFER] = {0};



void setup() {
  Serial.begin(115200);
  Sender.begin(115200, SERIAL_8N1, Sender_Txd_pin, Sender_Rxd_pin);       // Define and start Sender serial port
  Receiver.begin(115200, SERIAL_8N1, Receiver_Txd_pin, Receiver_Rxd_pin); // Define and start Receiver serial port

  SerialBT.begin("ESP32_BT_2"); //Nombre del dispositivo Bluetooth
  //Serial.println("El dispositivo está listo para emparejar");
}
/*
void loop() {
  // Para enviar la cadena de números "12345"
  SerialBT.println("1,2,3,4,5,A,B,C,");

  delay(20);
}
*/
void loop() {
  //float sensor_temperature = 22.141;                    // Set an example value
  //Sender.print(sensor_temperature);                     // Send it to Sender serial port
  while (Receiver.available()) {                        // Wait for the Receiver to get the characters
    byte received_temperature = Receiver.read(); // Display the Receivers characters
    Serial.write(received_temperature);  
    SerialBT.write(received_temperature);   
    delay(1000);          // Display the result on the serial monitor
  };
//  delay(2000);
}