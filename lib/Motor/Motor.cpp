#include <Motor.h>
#include <QTRSensors.h>
#include <Arduino.h>

MOTOR::MOTOR(int dir, int pwm, int chanel){
    this->dir = dir;
    this->pwm = pwm;
    this->chanel = chanel;

    this->frequency = 1000;
    this->resolution = 8;

    ledcSetup(this->chanel, this->frequency, this->resolution);

    pinMode(this->dir, OUTPUT); 
    
    // pinMode(this->pwm,OUTPUT);

    ledcAttachPin(this->pwm, this->chanel);
}

void MOTOR::GoAvance(int speed){

    digitalWrite(dir, LOW);
    ledcWrite(this->pwm, speed);
}

void MOTOR::GoBack(int speed){

    digitalWrite(dir, speed);
    ledcWrite(this->pwm, LOW);

}

void MOTOR::Still(){
    ledcWrite(this->pwm, 0);
}