#include <Arduino.h>

#define LED_1 4
#define LED_2 5
#define LED_3 2
#define BUZZER 18

#define M1A 19
#define M1B 21
#define M2A 23
#define M2B 22
#define DER_VELOCIDAD 12
#define IZQ_VELOCIDAD 14
#define DER_DIRECCION 13
#define IZQ_DIRECCION 15
#define left 1
#define right 0
#define backward 0
#define Forward 255
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;
void setUpPinModes()
{
  ledcSetup(IZQ_DIRECCION, 1000, 8);
  ledcSetup(IZQ_VELOCIDAD, 1000, 8);
  ledcSetup(DER_DIRECCION, 1000, 8);
  ledcSetup(DER_VELOCIDAD, 1000, 8);

  ledcAttachPin(M1A, IZQ_DIRECCION);
  ledcAttachPin(M1B, IZQ_VELOCIDAD);
  ledcAttachPin(M2A, DER_DIRECCION);
  ledcAttachPin(M2B, DER_VELOCIDAD);
}

void motor(int motor, int direccion, int velocidad) { 
  if (motor == left) {
    if (direccion == 0) {
      ledcWrite(IZQ_DIRECCION, velocidad);
      ledcWrite(IZQ_VELOCIDAD, 0);
    }
    else {
      ledcWrite(IZQ_DIRECCION, 0);
      ledcWrite(IZQ_VELOCIDAD, velocidad);
    }
    // return ;
  }
  else if (motor == right) {
    if (direccion == 0) {
      ledcWrite(DER_DIRECCION, 0);
      ledcWrite(DER_VELOCIDAD, velocidad);

    }
    else {
      ledcWrite(DER_DIRECCION, velocidad);
      ledcWrite(DER_VELOCIDAD, 0);
    }
  }
}

void setup() {
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  setUpPinModes();
  motor(left, Forward, 0);
  motor(right, Forward, 0);
}



void loop() {
  digitalWrite(LED_1, HIGH);
  delay(1000);
  digitalWrite(LED_1, LOW);
  delay(1000);
  digitalWrite(LED_2, HIGH);
  delay(1000);
  digitalWrite(LED_2, LOW);
  delay(1000);
  digitalWrite(LED_3, HIGH);
  delay(1000);
  digitalWrite(LED_3, LOW);
  delay(1000);
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  delay(1000);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  delay(1000);
  digitalWrite(BUZZER, HIGH);
  delay(1000);
  digitalWrite(BUZZER, LOW);
  delay(1000);
  motor(left, Forward, 50);
  motor(right, Forward, 50);
  delay(3000);
  motor(right, Forward, 0);
  motor(left, backward, 0);
  delay(3000);
  motor(right, backward, 40);
  motor(left, backward, 40);
  delay(3000);
  motor(right, Forward, 0);
  motor(left, backward, 0);
}