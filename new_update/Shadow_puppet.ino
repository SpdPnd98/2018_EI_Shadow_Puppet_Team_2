#include <Adafruit_PWMServoDriver.h>
//#include <TimeLib.h>

//The pwm driver has index of 0-15. choosing which goes where is dependant on the .ino file
Adafruit_PWMServoDriver Puppets = Adafruit_PWMServoDriver();

// Change to 0 only when ready to deploy!!!
bool prototypeState = 1;

#define INIT_IR 10

// Copied code for PWM driver
#define FREQUENCY 50
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

// direction definition for PWM driver
#define UP_PWM 180
#define DOWN_PWM 0
#define SERVO_INIT 0
#define SERVO_FINAL 180

// definition of IR sensors
#define ROCKET1_IR 8

//definition for midpoint of all puppet objs
#define MID 90

// Define puppet indices
#define ROCKET_1 4
#define ROCKET_2 1
#define ROVER_ARM 2

// Define Light control pin (relay, Noe=rmally Open)
#define RELAY_PIN 12

// Define Filter index
#define FILTER 0
#define FILTER_IR 9

//Difine DC motor IR pins
#define DCIR 6

// Specific definitions for each puppets
#define ROCKET_MOV_DUR 2755 // Through trial and error, this seems to be the right value /*Archived*/

// Global variables
int sec = 0;
int firstRocketTime = 0;
byte DCMotorPinA = 2; // DC motor Pin A
byte DCMotorPinB = 4; // DC motor Pin B
byte DCMotorPinPWM = 3; //DC motor PWM pin

// Prototype Function:
int pulseWidth(int angle);
void motormove(bool, bool, float);
void first_Segment(void);
void second_Segment(void);
void third_Segment(void);

// Puppet class for each puppet
class Puppet
{
  public:
    Puppet(int index );
    ~Puppet();
    void up(int pwm);
    void down (int pwm);
    void stop_puppet ();
    void pos_servo(int pos);
  private:
    int _index;
};

Puppet::Puppet(int index)
{
  _index = index;
}

Puppet::~Puppet()
{
  ;
}

void Puppet::up (int pwm)
{
  Puppets.setPWM(_index, 0, pulseWidth(pwm));
}

void Puppet::down (int pwm)
{
  Puppets.setPWM(_index, 0, pulseWidth(pwm));
}

void Puppet::stop_puppet ()
{
  Puppets.setPWM(_index, 0, pulseWidth(MID));
}
void Puppet::pos_servo (int pos)
{
  Puppets.setPWM(_index, 0, pulseWidth(pos));
}

// Light Prototype functions: No classes required since it is simple
void lightON(void);
void lightOFF(void);

// Puppets declaration
Puppet rocket1(ROCKET_1);
Puppet filter(FILTER);

void setup()
{
  Serial.begin(9600);

  // Initialize PWM driver
  Puppets.begin();
  Puppets.setPWMFreq(FREQUENCY);
  lightOFF();

  // define sensors and pins directly connected to the arduino:
  pinMode(ROCKET1_IR, INPUT);
  pinMode(INIT_IR, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FILTER_IR, INPUT);
  pinMode(DCMotorPinA, OUTPUT);
  pinMode(DCMotorPinB, OUTPUT);
  pinMode(DCMotorPinPWM, OUTPUT);
  pinMode(DCIR, INPUT);
  
  
  // Initialize all puppets to stop position

  // Rocket 1:
  rocket1.stop_puppet();

  // Rocket 2:
  /*Puppet rocket2(ROCKET_2);
    rocket2.stop_puppet();*/

  // Rover hand and head:
  /*Puppet roverarm(ROVER_ARM);
    roverarm.pos_servo(SERVO_INIT);*/

  // Rover tyres
  DCmotorstop();

  //Rocks:

  // Solo Knuckles:

  // Group Knuckles:

  // Light:

  // Filter:
  filter.stop_puppet();

  //while(digitalRead(ROCKET1_IR));
  // initialize time of program start
  Serial.println("waiting........");
  delay(2000);
  sec = millis();
}

void loop()
{
  //first_Segment();
  rocket1.up(DOWN_PWM);
  //second_Segment();
  third_Segment();
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  //Serial.println(analog_value);
  return analog_value;
}

void DCmotormove(bool A, bool B, float spd) {

  digitalWrite(DCMotorPinA, A);
  digitalWrite(DCMotorPinB, B);
  analogWrite(DCMotorPinPWM, spd);
}

void DCmotorstop ()
{
  digitalWrite(DCMotorPinA, LOW);
  digitalWrite(DCMotorPinB, LOW);
}

void lightON()
{
  digitalWrite(RELAY_PIN, HIGH);
}

void lightOFF()
{
  digitalWrite(RELAY_PIN, LOW);
}

void first_Segment()
{

  //Lights On, then

  // Lights On
  lightON();

  // MP3 track



  // "Rocket lift all the way up"
  Puppet rocket1(ROCKET_1);
  Puppet filter(FILTER);

  if (prototypeState) while (digitalRead(INIT_IR)) {
      Serial.println("WAIT!");
      Serial.print("ROCKET IR value: ");
      Serial.print(digitalRead(FILTER_IR));
    }
  sec = millis();
  Serial.print("Time has value of: ");
  Serial.println(sec);
  rocket1.up(DOWN_PWM);
  filter.pos_servo(100);

  while (digitalRead(ROCKET1_IR))
  { //rocket go up for 1 second and filter starts to move up for 1 sec
    Serial.println(digitalRead(FILTER_IR) == 0);
    if (digitalRead(FILTER_IR) == 0)
    {
      filter.stop_puppet();
    }
  }
  filter.stop_puppet();
  delay(500);
  firstRocketTime = millis() - sec;
  rocket1.stop_puppet();


  /*if (prototypeState)
    {
    Serial.print("The firstrockettime is: ");
    Serial.println(digitalRead(firstRocketTime));
    delay(2000);
    Serial.print("Duration is: ");
    Serial.println(firstRocketTime);
    rocket1.down(UP_PWM);
    delay(firstRocketTime - 60);
    //while (!digitalRead(ROCKET1_IR));
    rocket1.stop_puppet();
    delay(firstRocketTime);
      rocket1.stop_puppet();
    }
  else*/ if (!prototypeState) {
    ;
  }
  while (digitalRead(INIT_IR))Serial.println("1st Segment is completed, flash INIT_IR to continue");
}

void second_Segment()
{
  //Second rocket moves to the right, rover talks about jokes
  ;

}

void third_Segment()
{
  // rocket comes down Rover comes out

  // MP3 play

  // Rocket comes down
  /*Serial.print("The firstrockettime is: ");
  Serial.println(firstRocketTime);
  delay(2000);
  rocket1.down(UP_PWM);
  delay(firstRocketTime - 60);
  //while (!digitalRead(ROCKET1_IR));
  rocket1.stop_puppet();*/

  // MP3 sound for rover entrance


  // Rover moves to the screen
  DCmotormove(1,0,255);
  Serial.println(digitalRead(DCIR));
  while(digitalRead(DCIR) != 0){
    ;
  }
  DCmotorstop();
}
