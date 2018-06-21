#ifndef ShadowPuppet_h
#define ShadowPuppet_h
#include "Arduino.h"

class Puppet
{
  public:
    Puppet(int pin, float servoMidAngle)
    ~Puppet(); 
       
    //Member functions for IR sensors in 6 axi
    void setUpIR(int pinNum);
    void setDownIR(int pinNum);
    void setLeftIR(int pinNum);
    void setRightIR(int pinNum);
    void setFrontIR(int pinNum);
    void setBackIR(int pinNum);

    //Directions puppet will move
    void forward();
    void backward();
    void left();
    void right();
    void up();
    void down();

    //Special Moves Puppet will take
    void flipUpFor(float dur);
    void flipUp();
    void flipDownFor(float dur);
    void flipDown();
    void objledBlink(float interval);
    void objledBlink(float interval,byte repetitions);

  private:
   int _pin;
   float _servoMidAngle;
   int _IRpinNum[6];
   
};

class LightSource
{
  public:
    LightSource(int pin);
    void LightOn();
    void LightBlink(float interval, byte repetitions);
  private:
    int _pin
}




#endif 
