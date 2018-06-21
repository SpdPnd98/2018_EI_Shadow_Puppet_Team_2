#ifndef ShadowPuppet_h
#define ShadowPuppet_h
#include "Arduino.h"
#include <Servo.h>

class Puppet
{
  public:
    Puppet();
    Puppet(int pin, float servoMidAngle);
    Puppet(int pin, float servoMidAngle, int rotatePin);
    ~Puppet(); 
       
    //Member functions for IR sensors in 6 axi
    void setUpIR(int pinNum);
    void setDownIR(int pinNum);
    void setLeftIR(int pinNum);
    void setRightIR(int pinNum);
    void setFrontIR(int pinNum);
    void setBackIR(int pinNum);

    //Reading Values from IR pins
    int upIRVal();
    int downIRVal();
    int leftIRVal();
    int rightIRVal();
    int frontIRVal();
    int backIRVal();

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
    void setAngle(int degree);
    void rotateLeft();
    void rotateRight();
    
  private:
   int _pin;
   int _rotatePin;
   float _servoMidAngle;
   int _IRpinNum[6];
   int _flipPin;
   Servo _shdwpup;
   Servo _rotateservo;
};

class LightSource
{
  public:
    LightSource(int pin);
    void LightOn();
    void LightBlink(float interval, byte repetitions);
  private:
    int _pin;
};




#endif 
