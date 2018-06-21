#include "Shadow_Puppet.h"

// Object Instance
// By default servo should be moving the whole puppet
// 
Puppet::Puppet(int pin, float servoMidAngle)
{
  Serial.println("2D puppet initiated!\nPuppet can move in 6 axis.(See documentation for calling functions)");
  _pin = pin; //take on values specified in parameters 
  _servoMidAngle = servoMidAngle;
  _shdwpup.attach(_pin); 
}

Puppet::Puppet(int pin, float servoMidAngle, int rotatePin)
{
  Serial.println("3D puppet initiated!\nPuppet can move in 6 axis and rotate in 6 axis.(See documentation for calling functions)");
  _pin = pin; //take on values specified in parameters 
  _rotatePin = rotatePin;
  _servoMidAngle = servoMidAngle;
  _shdwpup.attach(_pin); 
  _rotateservo.attach(_rotatePin);    
}

Puppet::Puppet()
{
  Serial.println("Warning! Empty Puppet object created! Puppet cannot move in 6 axis nor rotate in place!\nExpected puppet is stationary in the xy plane!\nThis puppet should only do miscellaneous actions!");
}


// Possible to condense to one template function
// Initializing IR Pins
void Puppet::setUpIR(int pinNum)
{_IRpinNum[0] = pinNum;}

void Puppet::setDownIR(int pinNum)
{_IRpinNum[1] = pinNum;}

void Puppet::setLeftIR(int pinNum)
{_IRpinNum[2] = pinNum;}

void Puppet::setRightIR(int pinNum)
{_IRpinNum[3] = pinNum;}

void Puppet::setFrontIR(int pinNum)
{_IRpinNum[4] = pinNum;}

void Puppet::setBackIR(int pinNum)
{_IRpinNum[5] = pinNum;}


// Possible to condense to one template function
// Reading IR Values
int Puppet::upIRVal()
{return digitalRead(_IRpinNum[0]);}

int Puppet::downIRVal()
{return digitalRead(_IRpinNum[1]);}

int Puppet::leftIRVal()
{return digitalRead(_IRpinNum[2]);}

int Puppet::rightIRVal()
{return digitalRead(_IRpinNum[3]);}

int Puppet::frontIRVal()
{return digitalRead(_IRpinNum[4]);}

int Puppet::backIRVal()
{return digitalRead(_IRpinNum[5]);}


//Possible to Condense to one template function
// moves objects in a certain direction
void Puppet::left()
{}
void Puppet::right()
{}
void Puppet::up()
{}
void Puppet::down()
{}
void Puppet::forward()
{}
void Puppet::backward()
{}


// Misc actions for a puppet
void Puppet::flipUpFor(float dur)
{}
void Puppet::flipUp()
{}
void Puppet::flipDownFor(float dur)
{}
void Puppet::flipDown()
{}
void Puppet::objledBlink(float interval)
{}
void Puppet::objledBlink(float interval,byte repetitions)
{}
void Puppet::setAngle(int degree)
{}
void Puppet::rotateLeft()
{}
void Puppet::rotateRight()
{}

// LightSource Obj Instance
LightSource::LightSource(int pin)
{}
void LightSource::LightOn()
{}
void LightSource::LightBlink(float interval, byte repetitions)
{}

