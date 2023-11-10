#include "BluetoothSerial.h"
#include <QTRSensors.h>
#include <Motor.h>

#define LOG_BT 1
#define LOG_SERIAL 2
#define LOG LOG_BT
class Ilogs
{
public:
  virtual int read() = 0;
  virtual int available() = 0;
  virtual void println(const char *) = 0;
  virtual void println(const int) = 0;
  virtual void println(const String) = 0;
  virtual void println(const float) = 0;

  virtual void print(const char *) = 0;
  virtual void print(const int) = 0;
  virtual void print(const String) = 0;
  virtual void print(const float) = 0;
};

class LogsSerial : public Ilogs
{

public:
  LogsSerial(int baudios)
  {
    Serial.begin(baudios);
  }
  int available()
  {
    return Serial.available();
  }

  int read()
  {
    return Serial.read();
  }

  void println(const char *message)
  {
    Serial.println(message);
  }
  void println(const int message)
  {
    Serial.println(message);
  }
  void println(const String message)
  {
    Serial.println(message);
  }
  void println(const float message)
  {
    Serial.println(message);
  }

  void print(const char *message)
  {
    Serial.print(message);
  }
  void print(const int message)
  {
    Serial.print(message);
  }
  void print(const String message)
  {
    Serial.print(message);
  }
  void print(const float message)
  {
    Serial.print(message);
  }
};

class LogSerialBt : public Ilogs
{
private:
  BluetoothSerial SerialBT;

public:
  LogSerialBt(const String btName)
  {
    this->SerialBT.begin(btName);
  }

  int available()
  {
    return this->SerialBT.available();
  }

  int read()
  {
    return this->SerialBT.read();
  }
  void println(const char *message)
  {
    this->SerialBT.println(message);
  }
  void println(const int message)
  {
    this->SerialBT.println(message);
  }
  void println(const String message)
  {
    this->SerialBT.println(message);
  }
  void println(const float message)
  {
    this->SerialBT.println(message);
  }

  void print(const char *message)
  {
    this->SerialBT.print(message);
  }
  void print(const int message)
  {
    this->SerialBT.print(message);
  }
  void print(const String message)
  {
    this->SerialBT.print(message);
  }
  void print(const float message)
  {
    this->SerialBT.print(message);
  }
};
class Logger
{
private:
  Ilogs *logs;
  int enable = true;

public:
  Logger(Ilogs *logsImpl)
  {
    logs = logsImpl;
  }

  void disable()
  {
    enable = false;
  }

  int available()
  {
    return logs->available();
  }

  int read()
  {
    return logs->read();
  }
  void println(const char *message)
  {
    if (enable)
      logs->println(message);
  }
  void println(const int message)
  {
    if (enable)
      logs->println(message);
  }
  void println(const String message)
  {
    if (enable)
      logs->println(message);
  }
  void println(const float message)
  {
    if (enable)
      logs->println(message);
  }

  void print(const char *message)
  {
    if (enable)
      logs->print(message);
  }
  void print(const int message)
  {
    if (enable)
      logs->print(message);
  }
  void print(const String message)
  {
    if (enable)
      logs->print(message);
  }
  void print(const float message)
  {
    if (enable)
      logs->print(message);
  }
};

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

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

#define BZ 18

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

#define COMPENSATION_PWM 0

int proportional = 0;
int derivative = 0;
int integral = 0;
int lastErr = 0;

// variables configurables para el pid
float kp = 0.08700;
float ki = 0;
float kd = 0.4000;

float speed = 0;

// velocidad crucero es la velocidad que se
// utiliza para el pid y es el pico en rectas
int velocity = 180;

int velocityTurn = 110;

// floats para almacenar los valores de los pid
// de cada motor
float pidLeft = 0;
float pidRight = 0;

float pidMax = 0;

// estos son los valores maximos y minimos
// de los motores cuando se le aplica el pid
int maxSpeed = 180;
int minSpeed = 130;

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

Logger *logger = LOG == LOG_BT ? new Logger(new LogSerialBt("TheBlackBust")) : new Logger(new LogsSerial(115200));

// setpoint : es el punto en el que se desea
int setpoint = 3500;

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
  for (uint16_t i = 0; i < 300; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(Led, LOW);
}

// funcion que nos va a retornar la posicion
//  de la linea
int getPosition()
{
  int position = qtr.readLineWhite(sensorValues);
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

  pidLeft = (velocity + speed);
  pidRight = (velocity - speed);

  if (pidLeft > maxSpeed)
    pidLeft = maxSpeed;
  else if (pidLeft < minSpeed)
    pidLeft = minSpeed;

  if (pidRight < minSpeed + COMPENSATION_PWM)
    pidRight = minSpeed + COMPENSATION_PWM;
  else if (pidRight > maxSpeed)
    pidRight = maxSpeed;

  if (pidLeft <= minSpeed)
  {
    motorLeft->GoBack(pidLeft + 50);
    motorRight->GoAvance(pidRight);
  }
  else if (pidRight <= minSpeed)
  {
    motorLeft->GoAvance(pidLeft);
    motorRight->GoBack(pidRight + 40);
  }
  else
  {
    motorLeft->GoAvance(pidLeft);
    motorRight->GoAvance(pidRight);
    // if(speed > pidMax){
    //     pidMax = speed;
    // }
  }
}

// funcion que imprime un menu por bluetooth
void printOptions()
{
  // clean the serial
  for (int i = 0; i < 10; i++)
  {
    logger->println("");
  }

  logger->println("Configuracion Actual:");

  logger->print("- KP = ");
  logger->println(kp);

  logger->print("- KI = ");
  logger->println(ki);

  logger->print("- KD = ");
  logger->println(kd);

  logger->print("- maxSpeed = ");
  logger->println(maxSpeed);

  logger->print("- lockLeft = ");
  logger->println(lockLeft);

  logger->print("- lockRight = ");
  logger->println(lockRight);

  logger->print("- gateLeft = ");
  logger->println(gateLeft);

  logger->print("- gateRight = ");
  logger->println(gateRight);

  logger->print("- velocityTurn = ");
  logger->println(velocityTurn);

  logger->println(" (K) KP + 0.1 / (L) KP - 0.1");
  logger->println(" (Q) KP + 0.01 / (W) KP - 0.01");

  logger->println(" (R) KI + 0.1 / (T) KI - 0.1");
  logger->println(" (U) KI + 0.01 / (I) KI - 00.1");

  logger->println(" (Z) KD + 0.1 / (X) KD - 0.1");
  logger->println(" (G) KD + 0.01 / (H) KD - 00.1");

  logger->println(" (1) lockLeft + 5 / (2) lockLeft - 5");
  logger->println(" (3) lockRight + 5 / (4) lockRight - 5");

  logger->println(" (5) maxSpeed + 5 / (6) maxSpeed - 5");

  logger->println(" (7) gateLeft + 500 / (8) gateLeft - 500");
  logger->println(" (9) gateRight + 500 / (0) gateRight - 500");

  logger->println(" (+) velocityTurn + 1 / (-) velocityTurn - 1");

  logger->println(" (F) para ver la posicion");
  logger->println(" (E) para ver la el pid maximo");
}

void menuBT()
{
  if (logger->available())
  {
    char Menssage = logger->read();
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
      logger->println(getPosition());
      break;
    }
    case 'E':
    {
      logger->print("Pid maximo: ");
      logger->println(pidMax);
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
  // SerialBT.begin("coyote");
  Serial.begin(115200);
  pinMode(Led, OUTPUT);
  calibration();
  pinMode(LedB, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(BZ, OUTPUT);
  digitalWrite(LedB, HIGH);
}

bool isPress = false;

void loop()
{
  if (digitalRead(BUTTON) == LOW)
    isPress = true;

  menuBT();

  if (isPress)
  {
    PID();
  }
  else
  {
    motorLeft->Still();
    motorRight->Still();
  }
}