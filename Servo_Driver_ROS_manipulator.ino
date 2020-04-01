/* 
 * ros simple manipulator control using joint states
 * https://www.amazon.com/diymore-Aluminium-Mechanical-Robotic-Arduino/dp/B07MDFGHMC/ref=sr_1_15?dchild=1&keywords=robot%2Bmanipulator&qid=1585716671&sr=8-15&th=1
 * https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * Ernesto Hernandez-Hinojosa
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH 750
#define MAX_PULSE_WIDTH 2250
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 60

uint8_t servonum = 6;

ros::NodeHandle  nh;


void servo1Cb( const sensor_msgs::JointState& msg){


//double a1 = map(msg.position[0]*100,-314, 314, 0, 120);
//double a2 = map(msg.position[1]*100,-314, 314, 80, 180);
//double a3 = map(msg.position[2]*100, 0, 314, 180, 0);
//double a4 = map(msg.position[3]*100, 0, 314, 180, 0);
//double a5 = map(msg.position[4]*100, 0, 314, 180, 0);
//double a6 = map(msg.position[5]*100, 0, 314, 180, 0);

//Value between 0-180
double a1 = msg.position[0];
double a2 = msg.position[1];
double a3 = msg.position[2];
double a4 = msg.position[3];
double a5 = msg.position[4];
double a6 = msg.position[5];

pwm.setPWM(0, 0, pulseWidth(a1));
pwm.setPWM(1, 0, pulseWidth(a2));
pwm.setPWM(2, 0, pulseWidth(a3));
pwm.setPWM(3, 0, pulseWidth(a4));
pwm.setPWM(4, 0, pulseWidth(a5));
pwm.setPWM(5, 0, pulseWidth(a6));
  
}


int pulseWidth(int angle)
{
int pulse_wide, analog_value;
pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
//Serial.println(analog_value);
return analog_value;
}

ros::Subscriber<sensor_msgs::JointState> sub1("joint_states", &servo1Cb );


void setup()
{ 
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub1);

}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
