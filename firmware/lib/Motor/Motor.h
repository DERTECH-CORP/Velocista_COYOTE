#ifndef Motor_h
#include <Arduino.h>

class MOTOR
{
private:
    int dirRight, dirLeft, resolution, frequency, chanelRight, chanelLeft;

public:
    MOTOR(int dirRight, int dirLeft, int chanelRight, int chanelLeft);

    void GoAvance(int speed);

    void GoBack(int speed);

    void Still();
};

#endif