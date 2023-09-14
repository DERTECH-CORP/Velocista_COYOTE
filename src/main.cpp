#include "BluetoothSerial.h"
#include <QTRSensors.h>

// #define sensor1 = 13
// #define sensor2 = 27
// #define sensor3 = 26
// #define sensor4 = 25
// #define sensor5 = 33
// #define sensor6 = 32
// #define sensor7 = 35
// #define sensor8 = 34


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int Led = 5;

void calibration()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){
      13, 27, 25, 26, 33, 32, 35, 34
      }, SensorCount);

    delay(500);
    pinMode(Led, OUTPUT);
    digitalWrite(Led, HIGH);
    for (uint16_t i = 0; i < 1000; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(Led, LOW); //

    // print the calibration minimum values measured when emitters were on
}

void getPosition(){
  int position = qtr.readLineWhite(sensorValues);
      // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    
    for (uint8_t i = 0; i < SensorCount; i++)
      {
        SerialBT.print(sensorValues[i]);
        SerialBT.print(" || ");
      }
      SerialBT.println(position);
      SerialBT.print('\n');

    
     delay(250);
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test");
  pinMode(Led, OUTPUT);
  calibration();
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

  getPosition();


}