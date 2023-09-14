#include "BluetoothSerial.h"
#include <QTRSensors.h>

#define sensor1 = 13
#define sensor2 = 27
#define sensor3 = 26
#define sensor4 = 25
#define sensor5 = 33
#define sensor6 = 32
#define sensor7 = 35
#define sensor8 = 34

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

int Led = 5;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test");
  pinMode(Led, OUTPUT);
}

void loop() {
  if (SerialBT.available()) {
    char Mensaje = SerialBT.read();
    if (Mensaje == 'A') {
      digitalWrite(Led, HIGH);
      Serial.println("Encender Led");
    }
    else if (Mensaje == 'B') {
      digitalWrite(Led, LOW);
      Serial.println("Apagar Led");
    }
  }


}