#include "BluetoothSerial.h"
#include <QTRSensors.h>
#include <Motor.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// boleano para que se ejecute el pid
bool run = false;

#define M1A 19
#define M1B 21
#define M2A 23
#define M2B 22
#define M1A_CHANEL 12
#define M1B_CHANEL 14
#define M2A_CHANEL 13
#define M2B_CHANEL 15

#define BUTTON 16

// valores pre definidos

// para sumar o restar velocidades
#define VALUE_5 5

// para sumar o restar el gate
#define VALUE_500 500

// para sumar o restas a
//  kp, ki, kd
#define VALUE_0_1 0.1
#define VALUE_0_01 0.01

#define COMPENSATION_PWM 5

int proportional = 0;
int derivative = 0;
int integral = 0;
int lastErr = 0;

// variables configurables para el pid
float kp = 0.007;
float ki = 0;
float kd = 0;

float speed = 0;

// velocidad crucero es la velocidad que se
// utiliza para el pid y es el pico en rectas
int velocity = 90;

int velocityTurn = 110;

// floats para almacenar los valores de los pid
// de cada motor
float pidLeft = 0;
float pidRight = 0;

// estos son los valores maximos y minimos
// de los motores cuando se le aplica el pid
int maxSpeed = 150;
int minSpeed = 40;

// gete es la barrera que sirve como flag
// para las condicionales de giro
int gateLeft = 5000;
int gateRight = 6000;

// lock : es el pwm  que va a
// usar el motor para frenar
int lockLeft = 75;
int lockRight = 80;

BluetoothSerial SerialBT;

QTRSensors qtr;

MOTOR *motorLeft = new MOTOR(M1B, M1A, M1B_CHANEL, M1A_CHANEL);

MOTOR *motorRight = new MOTOR(M2A, M2B, M2B_CHANEL, M2A_CHANEL);

// setpoint : es el punto en el que se desea
int setpoint = 5500;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int Led = 5;
int LedB = 2;

void calibration()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){
                        13, 27, 25, 26, 33, 32, 35, 34},
                    SensorCount);

  delay(500);
  pinMode(Led, OUTPUT);
  digitalWrite(Led, HIGH);
  for (uint16_t i = 0; i < 500; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(Led, LOW);
}

// funcion que nos va a retornar la posicion
//  de la linea
int getPosition()
{
  int position = qtr.readLineBlack(sensorValues);
  return position;
}

void PID()
{
  int position = getPosition();

  proportional = position - setpoint;

  derivative = proportional - lastErr;

  integral += proportional;

  speed = (proportional * kp) + (derivative * kd) + (integral * ki);

  lastErr = proportional;

  pidLeft = (velocity + speed + COMPENSATION_PWM);
  pidRight = (velocity - speed);

  if (pidLeft > maxSpeed + COMPENSATION_PWM)
    pidLeft = maxSpeed + COMPENSATION_PWM;
  else if (pidLeft < minSpeed)
    pidLeft = minSpeed;

  if (pidRight < minSpeed)
    pidRight = minSpeed;
  else if (pidRight > maxSpeed)
    pidRight = maxSpeed;

  if (pidLeft <= minSpeed)
  {
    SerialBT.println("LEFT atras");
    SerialBT.println(pidLeft + velocity);

    SerialBT.println("RIGHT");
    SerialBT.println(pidRight);

    motorLeft->GoBack(pidLeft + velocity);
    motorRight->GoAvance(pidRight);
  }
  else if (pidRight <= minSpeed)
  {

    SerialBT.println("LEFT");
    SerialBT.println(pidLeft);

    SerialBT.println("RIGHT atras");
    SerialBT.println(pidRight + velocity);

    motorLeft->GoAvance(pidLeft);
    motorRight->GoBack(pidRight + velocity);
  }
  else
  {
    
    SerialBT.println("LEFT");
    SerialBT.println(pidLeft);

    SerialBT.println("RIGHT");
    SerialBT.println(pidRight);

    motorLeft->GoAvance(pidLeft);
    motorRight->GoAvance(pidRight);
  }
}

// funcion que imprime un menu por bluetooth
void printOptions()
{
  // clean the serial
  for (int i = 0; i < 10; i++)
  {
    SerialBT.println("");
  }

  SerialBT.println("Configuracion Actual:");

  SerialBT.print("- KP = ");
  SerialBT.println(kp,5);

  SerialBT.print("- KI = ");
  SerialBT.println(ki,5);

  SerialBT.print("- KD = ");
  SerialBT.println(kd,5);

  SerialBT.print("- maxSpeed = ");
  SerialBT.println(maxSpeed);

  SerialBT.print("- lockLeft = ");
  SerialBT.println(lockLeft);

  SerialBT.print("- lockRight = ");
  SerialBT.println(lockRight);

  SerialBT.print("- gateLeft = ");
  SerialBT.println(gateLeft);

  SerialBT.print("- gateRight = ");
  SerialBT.println(gateRight);

  SerialBT.print("- velocityTurn = ");
  SerialBT.println(velocityTurn);

  SerialBT.println(" (K) KP + 0.1 / (L) KP - 0.1");
  SerialBT.println(" (Q) KP + 0.01 / (W) KP - 0.01");

  SerialBT.println(" (R) KI + 0.1 / (T) KI - 0.1");
  SerialBT.println(" (U) KI + 0.01 / (I) KI - 00.1");

  SerialBT.println(" (Z) KD + 0.1 / (X) KD - 0.1");
  SerialBT.println(" (G) KD + 0.01 / (H) KD - 00.1");

  SerialBT.println(" (1) lockLeft + 5 / (2) lockLeft - 5");
  SerialBT.println(" (3) lockRight + 5 / (4) lockRight - 5");

  SerialBT.println(" (5) maxSpeed + 5 / (6) maxSpeed - 5");

  SerialBT.println(" (7) gateLeft + 500 / (8) gateLeft - 500");
  SerialBT.println(" (9) gateRight + 500 / (0) gateRight - 500");

  SerialBT.println(" (+) velocityTurn + 1 / (-) velocityTurn - 1");

  SerialBT.println(" (F) para ver la posicion");
}

void menuBT()
{
  if (SerialBT.available())
  {
    char Menssage = SerialBT.read();
    switch (Menssage)
    {
    case 'M':
    {
      printOptions();
      break;
    }
    case 'A':
    {
      run = true;
      break;
    }
    case 'B':
    {
      run = false;
      break;
    }
    case 'K':
    {
      kp += VALUE_0_1;
      printOptions();
      break;
    }
    case 'L':
    {
      kp -= VALUE_0_1;
      printOptions();
      break;
    }
    case 'Q':
    {
      kp += VALUE_0_01;
      printOptions();
      break;
    }
    case 'W':
    {
      kp -= VALUE_0_01;
      printOptions();
      break;
    }

    case 'R':
    {
      ki += VALUE_0_1;
      printOptions();
      break;
    }
    case 'T':
    {
      ki -= VALUE_0_1;
      printOptions();
      break;
    }
    case 'U':
    {
      ki += VALUE_0_01;
      printOptions();
      break;
    }
    case 'I':
    {
      ki -= VALUE_0_01;
      printOptions();
      break;
    }

    case 'Z':
    {
      kd += VALUE_0_1;
      printOptions();
      break;
    }
    case 'X':
    {
      kd -= VALUE_0_1;
      printOptions();
      break;
    }
    case 'G':
    {
      kd += VALUE_0_01;
      printOptions();
      break;
    }
    case 'H':
    {
      kd -= VALUE_0_01;
      printOptions();
      break;
    }
    case 'F':
    {
      SerialBT.println(getPosition());
      break;
    }
    case '1':
    {
      lockLeft += VALUE_5;
      printOptions();
      break;
    }
    case '2':
    {
      lockLeft -= VALUE_5;
      printOptions();
      break;
    }
    case '3':
    {
      lockRight += VALUE_5;
      printOptions();
      break;
    }
    case '4':
    {
      lockRight -= VALUE_5;
      printOptions();
      break;
    }
    case '5':
    {
      maxSpeed += VALUE_5;
      printOptions();
      break;
    }
    case '6':
    {
      maxSpeed -= VALUE_5;
      printOptions();
      break;
    }
    case '7':
    {
      gateLeft += VALUE_500;
      printOptions();
      break;
    }
    case '8':
    {
      gateLeft -= VALUE_500;
      printOptions();
      break;
    }
    case '9':
    {
      gateRight += VALUE_500;
      printOptions();
      break;
    }
    case '0':
    {
      gateRight -= VALUE_500;
      printOptions();
      break;
    }
    case '+':
    {
      velocityTurn += 1;
      printOptions();
      break;
    }
    case '-':
    {
      velocityTurn -= 1;
      printOptions();
      break;
    }

    default:
      break;
    }
  }
}

void setup()
{
  SerialBT.begin("coyote");
  pinMode(Led, OUTPUT);
  calibration();
  pinMode(LedB, OUTPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(LedB, HIGH);
}

bool isPress = false;

void loop()
{
  if (digitalRead(BUTTON) == LOW)
    isPress = true;

  menuBT();

  // SerialBT.println(getPosition());

  if (run)
  {
    PID();
  }
}