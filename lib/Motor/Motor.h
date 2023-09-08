#ifndef Motor_h
#include <Arduino.h>

class MOTOR
{
private:
    int dir, pwm, resolution, frequency, chanel;

public:
    MOTOR(int dir, int pwm, int chanel);

    void GoAvance(int speed);

    void GoBack(int speed);

    void Still();
};

#endif