/*
 * Author: Kaan Ural
 * Email: kaan.urall@outlook.com
 * 
 * Two wheel differential drive robot driver using rosserial 
 * Driver takes data from ROS and sends it to BLDC hub motor drivers via PWM.
 * This sketch demonstrates the control 
 * using ROS and the ESP32
 * 
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle  nh;   //ROS Node handler object

// Pins and Variables ****************************************************

// Pin Assingments
const int  Rdir   = 15;   // Right motor direction pin
const int  Ldir   =  4;   // Left  motor direction pin
const int  Rtick  = 16;   // Right motor encoder input
const int  Ltick  = 17;   // Left  motor encoder input
const int  Rmotor = 21;   // Right motor signal output
const int  Lmotor = 18;   // Left  motor signal output
const int  Rbrake = 19;   // Right motor brake signal
const int  Lbrake =  5;   // Right motor brake signal
const int  brPin  = 22;   // Brake button
const int  ledpin = 23;   // LED indicator output

const int maxspeedmultiplier = 1; // default: 1

const int freq         = 5000;    // PWM frequency   
const int ledChannel   = 0;       // PWM channel
const int rightChannel = 1;       // PWM channel
const int leftChannel  = 6;       // PWM channel
const int resolution   = 8;       // PWM res.(8-bit = 0-255)


float x                = 0;       //Linear speed command
float z                = 0;       //Angular speed command
float wR               = 0;       //Right wheel actual angular speed
float wL               = 0;       //Left  wheel actual angular speed

//int lasttimer1         = 0;       //Timer to check if ROS is connected
//int lasttimer          = 0;       //Last 

double Rlast1          = 1;       //Reading one before the last
double Rlast           = 2;       //Last reading
double Rinterval       = 1;       //Time between last two readings
float  Rrpm            = 1;       //RPM
float  Rkmh            = 1;       //KMH
volatile int Rcount    = 0;       //Total steps
  
double Llast1          = 1;       //Reading one before the last
double Llast           = 2;       //Last reading
double Linterval       = 1;       //Time between last two readings
float Lrpm             = 1;       //RPM
float  Lkmh            = 1;       //KMH
volatile int Lcount    = 0;       //Total steps

// ***********************************************************************

// Callback Functions ****************************************************

//void timer_cb( const std_msgs::Time& timeIn){
//  lasttimer  = timeIn;
//  lasttimer1 = lasttimer;
//  digitalWrite(2, HIGH-digitalRead(2));  //toggle led  
//}

// Receive LED PWM
void led_cb( const std_msgs::UInt16& cmd_msg){
  int pwm = cmd_msg.data;
  ledcWrite(ledChannel, pwm);
  //digitalWrite(2, HIGH-digitalRead(2));  //toggle led  
}

// Receive velocity commands
void velCallback(  const geometry_msgs::Twist& vel)
  {
  x = vel.linear.x;  
  z = vel.angular.z; 
  }
  
//  **********************************************************************

// Read encoder signals  *************************************************

void Rread()
{
  Rlast1 = Rlast;
  Rlast = millis();
  
  if  (digitalRead(Rdir))  {  
    Rcount++; 
  }else {  
    Rcount--; }
}

void Lread()
{
  Llast1 = Llast;
  Llast = millis();

  if  (!digitalRead(Ldir)) {  
    Lcount++;
  }else  {  
    Lcount--; }
  }
//  **********************************************************************

// ROS Objects //  *******************************************************

std_msgs::Float64MultiArray info;
std_msgs::Time timer;
ros::Subscriber<std_msgs::UInt16> led_sub("LED", led_cb);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel" , velCallback);
//ros::Subscriber<std_msgs::Time> time_sub("timer", timer_cb);
ros::Publisher info_pub("odomInfo", &info);

//  **********************************************************************


void setup(){
  //nh.getHardware()->setBaud(115200); 

  //Set pin modes and PWM channels.
  pinMode(Rdir,   OUTPUT);
  pinMode(Ldir,   OUTPUT);
  pinMode(Rtick,   INPUT);
  pinMode(Ltick,   INPUT);
  pinMode(Rmotor, OUTPUT);
  pinMode(Lmotor, OUTPUT);
  pinMode(Rbrake, OUTPUT);
  pinMode(Lbrake, OUTPUT);
  pinMode(ledpin, OUTPUT);
  pinMode(brPin,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Rtick), Rread, CHANGE); // Right Motor Encoder Input
  attachInterrupt(digitalPinToInterrupt(Ltick), Lread, CHANGE); // Left Motor Encoder Input 
  ledcSetup(ledChannel,   freq, resolution);
  ledcSetup(rightChannel, freq, resolution);
  ledcSetup(leftChannel,  freq, resolution);
  ledcAttachPin(ledpin, ledChannel);
  ledcAttachPin(Rmotor, rightChannel);
  ledcAttachPin(Lmotor, leftChannel);

  //Initialize ROS nodes
  nh.initNode();
//nh.subscribe(time_sub);
  nh.subscribe(led_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(info_pub);

  Rcount = 0;
  Lcount = 0;
}

void loop(){
  
// double lasttimer = nh.now().toSec(); 
// if(lasttimer - lasttimer1 == 1) 
//  {

  if(!digitalRead(brPin)) // Read brake pin state
    {
    digitalWrite(Rbrake, HIGH);
    digitalWrite(Lbrake, HIGH);
    }
  else                    // Release brake if not pressed
    {
    digitalWrite(Rbrake, LOW);
    digitalWrite(Lbrake, LOW);
    }

  // Angular speeds calculations for wheels
  wR = ((x - (z * 0.225)) / 0.0825) * maxspeedmultiplier;  
  wL = ((x + (z * 0.225)) / 0.0825) * maxspeedmultiplier; 

  // Set R direction
  if(wR>=0) {  digitalWrite(Rdir, HIGH);  } 
  else      {  digitalWrite(Rdir, LOW );  } 

  // Set L direction
  if(wL>=0) {  digitalWrite(Ldir, LOW) ;  } 
  else      {  digitalWrite(Ldir, HIGH ); } 

  //Send speed values to pins
  ledcWrite(rightChannel, abs(wR));
  ledcWrite(leftChannel,  abs(wL));

  //Evaluate encoder values and calculate real speed
  Rinterval = (Rlast - Rlast1); //seconds  
  Linterval = (Llast - Llast1); //seconds   
  
  Rrpm = (1 / ((Rinterval/1000) * 90)) * 60;
  Lrpm = (1 / ((Linterval/1000) * 90)) * 60;

  if(millis() > Rlast + 500) { Rrpm = 0; }
  if(millis() > Llast + 500) { Lrpm = 0; }

  Rkmh = (0.165 * 3.14159 * Rrpm * 60) / 1000;
  Lkmh = (0.165 * 3.14159 * Lrpm * 60) / 1000;

//  }//timer if
//
//  If ROS is not connected, send stop signal to motors.
//  else {
//    ledcWrite(rightChannel, 0);
//    ledcWrite(leftChannel,  0);
//  }

  //Publish sent data to the network
  info.data_length = 6;
  //info.layout.dim_length = 0;
  float value[6] = { Rcount, Rrpm, Rkmh, 
                     Lcount, Lrpm, Lkmh};
  info.data = value; 
  
  info_pub.publish( &info );


  //lasttimer1 = lasttimer;
  nh.spinOnce();
  delay(1);
}
