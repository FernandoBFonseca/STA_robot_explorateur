#include <Arduino.h>
#include <stdio.h>
#include <PID.hpp>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <speedEstimator.hpp>

void ISR_VA_LINEAR();
void ISR_VA_STEERING();
void turn(int motor, double *voltage );
double getPos(int motor);
double getSpeed(int motor);
void publishSpeed(double time);

#define SGN(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
#define ABS(x) ((x) < 0 ? (-x) : (x))

#define MAX_VOLTAGE 12.0
#define SAT_VOLTAGE 9.0
#define SAMPLE_TIME 10

#define LINEAR 1
#define STEERING 2

#define encoderALinearPIN 19
#define encoderBLinearPIN 38
#define IN1_LINEAR 37
#define IN2_LINEAR 36
#define EN_LINEAR 8

#define encoderASteeringPIN 18
#define encoderBSteeringPIN 31
#define IN1_STEERING 34
#define IN2_STEERING 35
#define EN_STEERING 12

volatile long encoderLinearPos = 0;
volatile long encoderSteeringPos = 0;
volatile long t_ISR_linear = 0, t_ISR_last_linear = 0;
volatile long t_ISR_steering = 0, t_ISR_last_steering = 0;


#define KP_LINEAR 26.11 //SpeedControl (ref = [m/s])
#define KI_LINEAR 2184.2232
//#define KP_LINEAR 0.038753 //Position Control (ref = [encoder])
//#define KI_LINEAR 0.074189
#define KD_LINEAR 0.0
#define KC_LINEAR 100.0
#define ENCODER_2_M 3.82e-05
#define MAX_SPEED_LINEAR 0.0882

#define KP_STEERING 2.521 //Speed Control (ref = [rad/s])
#define KI_STEERING 203.1
//#define KP_STEERING 0.01311 //Position Control (ref = [encoder])
//#define KI_STEERING 0.00556
#define KD_STEERING 0.0
#define KC_STEERING 100.0
#define ENCODER_2_RAD 5.2e-4
#define MAX_SPEED_STEERING 0.9486
#define MAX_STEERING_ANGLE_RAD 0.5235988 //30°


//********** --- CONTROL --- **********/

double PositionLinear, VelocityEstLinear, OutputLinear, SetpointLinear;
double const sys_Linear[4] = {0.6036,0.003876,0.007883,2.109e-5};
//double const L_Linear[3] = {180.028407512287,2.59708116023640,25533.5643547191};
double const L_Linear[3] = {45.72,1.59,0};

speedEstimator speedLinear(&PositionLinear, &OutputLinear, &VelocityEstLinear, 
              sys_Linear,L_Linear, SAMPLE_TIME);
PID pidLinear(&VelocityEstLinear, &OutputLinear, &SetpointLinear,
    KP_LINEAR, KI_LINEAR, KD_LINEAR, KC_LINEAR, P_ON_E, DIRECT);


double PositionSteering, VelocityEstSteering, OutputSteering, SetpointSteering;
double const sys_Steering[4] = {0.6117,0.04093,0.007931,0.0002222};
//double const L_Steering[3] = {181.207509792776,2.60430970272214,2415.44175916169};
double const L_Steering[3] = {46.6087,1.6043,0};

speedEstimator speedSteering(&PositionSteering, &OutputSteering, &VelocityEstSteering, 
              sys_Steering,L_Steering, SAMPLE_TIME);
PID pidSteering(&VelocityEstSteering, &OutputSteering, &SetpointSteering,
    KP_STEERING, KI_STEERING, KD_STEERING, KC_STEERING, P_ON_E, DIRECT);


//************ --- ROS --- ********//
ros::NodeHandle nh;

String buf;

void messageCb( const geometry_msgs::Twist& speed_msg) {
  
  SetpointLinear = speed_msg.linear.x;
  SetpointSteering = speed_msg.angular.z;

  SetpointLinear = constrain(SetpointLinear, -MAX_SPEED_LINEAR, MAX_SPEED_LINEAR);
  SetpointSteering = constrain(SetpointSteering, -MAX_SPEED_STEERING, MAX_SPEED_STEERING);

  nh.loginfo("Receiving Message:");
  buf = String(String("linear: ") + String(SetpointLinear) + "m/s, angular: " + String(SetpointSteering) +  "rad/s");
  nh.loginfo(buf.c_str());
  //buf = String(String(PositionLinear) + "  " + String(PositionSteering));
  //nh.loginfo(buf.c_str());
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", messageCb );

geometry_msgs::Vector3Stamped speed_msg;  //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg); 



unsigned long lastTime;



void setup() {
  
  pinMode(encoderALinearPIN, INPUT_PULLUP);
  pinMode(encoderBLinearPIN, INPUT_PULLUP);
  pinMode(IN1_LINEAR, OUTPUT);
  pinMode(IN2_LINEAR, OUTPUT);
  pinMode(EN_LINEAR, OUTPUT);

  pinMode(encoderASteeringPIN, INPUT_PULLUP);
  pinMode(encoderBSteeringPIN, INPUT_PULLUP);
  pinMode(IN1_STEERING, OUTPUT);
  pinMode(IN2_STEERING, OUTPUT);
  pinMode(EN_STEERING, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderALinearPIN), 
    ISR_VA_LINEAR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderASteeringPIN), 
    ISR_VA_STEERING, CHANGE);

  
  pidLinear.SetMode(AUTOMATIC);
  pidSteering.SetMode(AUTOMATIC);

  pidLinear.SetOutputLimits(-SAT_VOLTAGE,SAT_VOLTAGE);
  pidSteering.SetOutputLimits(-SAT_VOLTAGE,SAT_VOLTAGE);

  pidLinear.SetSampleTime(SAMPLE_TIME);
  pidSteering.SetSampleTime(SAMPLE_TIME);

  nh.getHardware()->setBaud(57600);
  nh.getHardware()->setPort(&Serial);

  nh.initNode();

  nh.subscribe(cmd_vel);
  nh.advertise(speed_pub);

  lastTime = millis();

}

double oldPositionSteering, oldPositionLinear, freq = 1000.0/SAMPLE_TIME;

void loop() {

  unsigned long now = millis();
  if(now-lastTime>=SAMPLE_TIME){



    PositionSteering = getPos(STEERING);
    PositionLinear = getPos(LINEAR);

    //This is a fix while the speed estimation lib does not work
    VelocityEstLinear = getSpeed(LINEAR);
    VelocityEstSteering = getSpeed(STEERING);


    //speedLinear.Compute();
    pidLinear.Compute();
    turn(LINEAR,&OutputLinear);

    //speedSteering.Compute();
    pidSteering.Compute();
    turn(STEERING,&OutputSteering);
    
    publishSpeed(SAMPLE_TIME);   //Publish odometry on ROS topic

    lastTime = now;

  }

  
  nh.spinOnce();

}

void turn(int motor, double *voltage ){

  if(abs(*voltage) <= 1.6) *voltage = 0;

  int IN1,IN2,EN;
  float pos;

  if(motor == LINEAR){
    IN1 = IN1_LINEAR; IN2 = IN2_LINEAR; EN = EN_LINEAR; 
  }
  else if(motor == STEERING){
    pos = getPos(STEERING);
    
    if(ABS(pos) >= MAX_STEERING_ANGLE_RAD && SGN(pos) ==  SGN(*voltage)) *voltage = 0; //Steering Angle Saturation: 
    //If steering angle is greater than 30° and the controler is trying to increment it,
    //we send 0V to the motor to avoid to pass the limit

    IN1 = IN1_STEERING; IN2 = IN2_STEERING; EN = EN_STEERING;
  }

  if(*voltage>0){
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
    }
    else{
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
    }
    *voltage = constrain(*voltage, -SAT_VOLTAGE, SAT_VOLTAGE);

    analogWrite(EN, abs(*voltage)*255.0/MAX_VOLTAGE);
}

double getPos(int motor){

  double pos_copy = 0;
  noInterrupts();  
  if(motor == LINEAR){
    pos_copy = ENCODER_2_M*encoderLinearPos;    
    //return encoderLinearPos;
  }
  else if(motor == STEERING){
    pos_copy = ENCODER_2_M*encoderLinearPos;  
    //return encoderSteeringPos;

  }
  interrupts();
  return pos_copy;


}

//This is a fix while the speed estimation lib does not work :(
double getSpeed(int motor){
    double  t_ISR_last_copy;
    double  t_ISR_copy;
    double  coef;
    noInterrupts();
    if(motor == LINEAR){
      t_ISR_last_copy =t_ISR_last_linear;
      t_ISR_copy = t_ISR_linear;
      coef = ENCODER_2_M;   
      //return encoderLinearPos;
    }
    else if(motor == STEERING){
      t_ISR_last_copy =t_ISR_last_steering;
      t_ISR_copy = t_ISR_steering;
      coef = ENCODER_2_RAD;
      //return encoderSteeringPos;
    }
    interrupts();
    if( t_ISR_copy==t_ISR_last_copy)
        return 0;
    else
        return (coef/(t_ISR_copy-t_ISR_last_copy) )*1e6;

}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = VelocityEstLinear;    //rear wheel speed (in m/s)
  speed_msg.vector.y = VelocityEstSteering;   //steering axes speed (in rad/s)
  //speed_msg.vector.x = OutputLinear;    //rear wheel speed (in m/s)
  //speed_msg.vector.y = OutputSteering;   //steering axes speed (in rad/s)
  speed_msg.vector.z = time * 1e-3;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  //nh.loginfo("Publishing odometry");
}

void ISR_VA_LINEAR(){

  if (digitalRead(encoderALinearPIN) == HIGH){

    if (digitalRead(encoderBLinearPIN) == HIGH)encoderLinearPos += 1;
    else encoderLinearPos -= 1;

  }
  else{

    if (digitalRead(encoderBLinearPIN) == LOW)encoderLinearPos += 1;
    else encoderLinearPos -= 1;

  }

  t_ISR_last_linear = t_ISR_linear;
  t_ISR_linear = micros();

}

void ISR_VA_STEERING(){

  if (digitalRead(encoderASteeringPIN) == HIGH){

    if (digitalRead(encoderBSteeringPIN) == HIGH)encoderSteeringPos += 1;
    else encoderSteeringPos -= 1;

  }
  else{

    if (digitalRead(encoderBSteeringPIN) == LOW)encoderSteeringPos += 1;
    else encoderSteeringPos -= 1;

  }

  t_ISR_last_steering = t_ISR_steering;
  t_ISR_steering = micros();


}