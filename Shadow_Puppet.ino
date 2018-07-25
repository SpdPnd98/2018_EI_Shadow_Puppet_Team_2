#include <Adafruit_PWMServoDriver.h>
//The pwm driver has index of 0-15. choosing which goes where is dependant on the .ino file

Adafruit_PWMServoDriver Puppets = Adafruit_PWMServoDriver();

// Copied code for PWM driver
#define FREQUENCY 50
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

// direction definition for PWM
#define UP_PWM 180
#define DOWN_PWM 0

//definition for midpoint of all puppet objs
#define MID 90

// Define puppet indices
#define ROCKET_1 0

int pulseWidth(int angle);

class Puppet
{
  public:
    Puppet(int index );
    void up(int pwm);
    void down (int pwm);
    void stop_puppet ();
  private:
    int _index;
};

Puppet::Puppet(int index)
{
  _index = index;
}

void Puppet::up (int pwm)
{
  Puppets.setPWM(ROCKET_1, 0, pulseWidth(pwm));
}

void Puppet::down (int pwm)
{
  Puppets.setPWM(ROCKET_1, 0, pulseWidth(pwm));
}

void Puppet::stop_puppet ()
{
  Puppets.setPWM(ROCKET_1, 0, pulseWidth(MID));
}


void setup()
{
  Serial.begin(9600);
  Puppets.begin();
  Puppets.setPWMFreq(FREQUENCY);
  Puppet rocket1 (ROCKET_1);
}

void loop()
{
  /*Puppets.setPWM(ROCKET_1, 0, pulseWidth(75));
    delay(5000);
    Puppets.setPWM(ROCKET_1, 0,pulseWidth(90));
    delay(1000);
    Puppets.setPWM(ROCKET_1, 0, pulseWidth(180));
    delay(5000);
    Puppets.setPWM(ROCKET_1, 0,pulseWidth(90));
    delay(1000);*/
  Puppet rocket1(ROCKET_1);
  
  //rocket1.up(DOWN_PWM);
  delay(3200-445);
  Serial.println("UP");
  rocket1.stop_puppet();
  delay(1000);
  rocket1.down(UP_PWM);
  delay(3150-475);
  Serial.println("DOWN");
  rocket1.stop_puppet();
  delay(1000);
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}



