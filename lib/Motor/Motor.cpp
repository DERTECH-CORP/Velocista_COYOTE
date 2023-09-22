#include <Motor.h>
#include <QTRSensors.h>
#include <Arduino.h>

MOTOR::MOTOR(int dirRight, int dirLeft, int chanelRight, int chanelLeft){
    this->dirRight = dirRight;
    this->dirLeft = dirLeft;
    this->chanelRight = chanelRight;
    this->chanelLeft = chanelLeft;


    //frecuencia recomendada 1k = 1kz 
    this->frequency = 1000;
    this->resolution = 8;

    ledcSetup(this->chanelRight, this->frequency, this->resolution);
    ledcSetup(this->chanelLeft, this->frequency, this->resolution);


    ledcAttachPin(this->dirRight, this->chanelRight); 
    ledcAttachPin(this->dirLeft, this->chanelLeft);
}

void MOTOR::GoAvance(int speed){

    ledcWrite(this->chanelLeft, 0);
    ledcWrite(this->chanelRight, speed);
}

void MOTOR::GoBack(int speed){

    ledcWrite(this->chanelLeft, speed);
    ledcWrite(this->chanelRight, LOW);

}

void MOTOR::Still(){
    ledcWrite(this->chanelLeft, LOW);
    ledcWrite(this->chanelRight, LOW);
}