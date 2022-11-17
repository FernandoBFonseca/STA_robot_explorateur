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
volatile int sense_linear = 1, sense_steering = 1;
volatile long t_ISR_linear = 0, t_ISR_last_linear = 0;
volatile long t_ISR_steering = 0, t_ISR_last_steering = 0;


//#define KP_LINEAR 26.11 //SpeedControl (ref = [m/s])
//#define KI_LINEAR 2184.2232
#define KP_LINEAR 12.41 //SpeedControl (ref = [m/s])
#define KI_LINEAR 408.4
//#define KP_LINEAR 0.038753 //Position Control (ref = [encoder])
//#define KI_LINEAR 0.074189
#define KD_LINEAR 0.0
#define KC_LINEAR 1.0
#define ENCODER_2_M 3.82e-05
#define MAX_SPEED_LINEAR 3*0.0882
#define TF 100//Low pass filter response time (ms)

//#define KP_STEERING 0.01311 //Position Control (ref = [encoder])
//#define KI_STEERING 0.00556
#define KP_STEERING 25.2115 //Position Control (ref = [rad])
#define KI_STEERING 10.6923
//#define KP_STEERING 205.6 //Position Control (ref = [rad])
//#define KI_STEERING 72.09
#define KD_STEERING 0.0
#define KC_STEERING 5.0
#define ENCODER_2_RAD 5.2e-4
#define MAX_SPEED_STEERING 0.9486
#define WHEELBASE_SIZE 0.192
#define WHEELRADIUS 0.03
//#define MAX_STEERING_ANGLE_RAD 0.5235988 //30°
#define MAX_STEERING_ANGLE_RAD 0.4363323 //25°
//#define MAX_STEERING_ANGLE_RAD 0.2618 //15°



//********** --- CONTROL --- **********/

double PositionLinear, VelocityEstLinear, OutputLinear, SetpointLinear;
//double const sys_Linear[4] = {0.6036,0.003876,0.007883,2.109e-5};
//double const L_Linear[3] = {180.028407512287,2.59708116023640,25533.5643547191};
//double const L_Linear[3] = {45.72,1.59,0};

//speedEstimator speedLinear(&PositionLinear, &OutputLinear, &VelocityEstLinear, 
//              sys_Linear,L_Linear, SAMPLE_TIME);
PID pidLinear(&VelocityEstLinear, &OutputLinear, &SetpointLinear,
    KP_LINEAR, KI_LINEAR, KD_LINEAR, KC_LINEAR, P_ON_E, DIRECT);


double PositionSteering, VelocityEstSteering, OutputSteering, SetpointSteering;
//double const sys_Steering[4] = {0.6117,0.04093,0.007931,0.0002222};
//double const L_Steering[3] = {181.207509792776,2.60430970272214,2415.44175916169};
//double const L_Steering[3] = {46.6087,1.6043,0};

//speedEstimator speedSteering(&PositionSteering, &OutputSteering, &VelocityEstSteering, 
//              sys_Steering,L_Steering, SAMPLE_TIME);
PID pidSteering(&PositionSteering, &OutputSteering, &SetpointSteering,
    KP_STEERING, KI_STEERING, KD_STEERING, KC_STEERING, P_ON_E, DIRECT);


//************ --- ROS --- ********//
ros::NodeHandle nh;

String buf;

//Message reader Callback function for speed and position serpoint for the linear and steering motors
void messageCb( const geometry_msgs::Twist& speed_msg) {
  
  SetpointLinear = speed_msg.linear.x;
  SetpointLinear = constrain(SetpointLinear, -MAX_SPEED_LINEAR, MAX_SPEED_LINEAR);

  SetpointSteering = speed_msg.angular.z;
  SetpointSteering = constrain(SetpointSteering, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD);

  nh.loginfo("Receiving Message:");
  buf = String(String("linear: ") + String(SetpointLinear) + "m/s, angular: " + String(SetpointSteering) +  "rad");
  nh.loginfo(buf.c_str());
  buf = String(String(OutputLinear) + " - " + String(VelocityEstLinear) + ", " + String(OutputSteering) + " - " + String(PositionSteering));
  nh.loginfo(buf.c_str());
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", messageCb ); //create a "cmd_vel" ROS subscriber

geometry_msgs::Vector3Stamped speed_msg;  //create a "speed_msg" ROS message
ros::Publisher speed_pub("est_vel", &speed_msg); 


unsigned long lastTime;



void setup() {
  
  //Encoders setup and motor setup
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

  //Control setup
  pidLinear.SetMode(AUTOMATIC);
  pidSteering.SetMode(AUTOMATIC);

  pidLinear.SetOutputLimits(-SAT_VOLTAGE,SAT_VOLTAGE);
  pidSteering.SetOutputLimits(-SAT_VOLTAGE,SAT_VOLTAGE);

  pidLinear.SetSampleTime(SAMPLE_TIME);
  pidSteering.SetSampleTime(SAMPLE_TIME);

  //ROS setup
  nh.getHardware()->setBaud(57600);
  nh.getHardware()->setPort(&Serial);

  nh.initNode();

  nh.subscribe(cmd_vel);
  nh.advertise(speed_pub);

  lastTime = millis();

}

//This is a fix
const double alpha = (double) TF/(TF+SAMPLE_TIME);//Settting a low pass filter for the velocity estimation
unsigned int counter = 0;
void loop() {

  unsigned long now = millis();
  if(now-lastTime>=SAMPLE_TIME){

    //Getting sensor data
    PositionSteering = getPos(STEERING);
    //This is a fix while the speed estimation lib does not work
    VelocityEstLinear = alpha*VelocityEstLinear + (1-alpha)*getSpeed(LINEAR);//Low pass filter

    
    //Computing PID and Sending voltage to the motors
    //speedLinear.Compute();
    pidLinear.Compute();
    turn(LINEAR,&OutputLinear);

    //speedSteering.Compute();
    pidSteering.Compute();
    turn(STEERING,&OutputSteering);
    
    publishSpeed(SAMPLE_TIME); //Publish odometry on ROS topic

    lastTime = now;

  }

  
  nh.spinOnce();

}

//Function to send voltage to the motors
void turn(int motor, double *voltage ){


  int IN1,IN2,EN;
  float pos;

  //Getting ports 
  if(motor == LINEAR){
    IN1 = IN1_LINEAR; IN2 = IN2_LINEAR; EN = EN_LINEAR; 
  }
  else if(motor == STEERING){
    //Small tests for limit the voltage we send the motor
    if(abs(*voltage) <= 1.6) *voltage = 0;//If voltage is to low, don't send nothing
    pos = getPos(STEERING);
    
    if(ABS(pos) >= MAX_STEERING_ANGLE_RAD && SGN(pos) ==  SGN(*voltage)) *voltage = 0; //Steering Angle Saturation: 
    //If steering angle is greater than 30° and the controler is trying to increment it,
    //we send 0V to the motor to avoid to pass the limit

    IN1 = IN1_STEERING; IN2 = IN2_STEERING; EN = EN_STEERING;
  }

  //Setting sense (clockwise or anti-clockwise)
  if(*voltage>0){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }
  else{
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }

    //Saturation limiting
    *voltage = constrain(*voltage, -SAT_VOLTAGE, SAT_VOLTAGE);

    //Senting voltage to the motors
    analogWrite(EN, abs(*voltage)*255.0/MAX_VOLTAGE);
}

//Calculation of Enconder absolute position in S.I units
double getPos(int motor){

  double pos_copy = 0;
  noInterrupts();  
  if(motor == LINEAR){
    pos_copy = ENCODER_2_M*encoderLinearPos;    
    //return encoderLinearPos;
  }
  else if(motor == STEERING){
    pos_copy = ENCODER_2_RAD*encoderSteeringPos;  
    //return encoderSteeringPos;

  }
  interrupts();
  return pos_copy;


}

//This is a fix while the speed estimation lib does not work :(
double getSpeed(int motor){
    double  t_ISR_last_copy;
    double  t_ISR_copy;
    int sense_copy;
    double  coef;

    noInterrupts();
    //Copying enconder data
    if(motor == LINEAR){
      t_ISR_last_copy =t_ISR_last_linear;
      t_ISR_copy = t_ISR_linear;
      sense_copy = sense_linear;
      coef = ENCODER_2_M;   

    }
    else if(motor == STEERING){
      t_ISR_last_copy =t_ISR_last_steering;
      t_ISR_copy = t_ISR_steering;
      sense_copy = sense_steering;
      coef = ENCODER_2_RAD;
    }
    interrupts();
    //Calcutation of velocity using v_est = ΔS/ΔT
    //Where ΔS is always the distance between to enconder steps
    //And ΔT is the time between to enconder readings
    if( t_ISR_copy==t_ISR_last_copy)
        return 0;
    else
        return sense_copy*(coef/(t_ISR_copy-t_ISR_last_copy) )*1e6;

}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = VelocityEstLinear;    //rear wheel speed (in m/s)
  speed_msg.vector.y = PositionSteering;   //steering axe position (in rad)
  speed_msg.vector.z = time * 1e-3;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  //nh.loginfo("Publishing odometry");
}

void ISR_VA_LINEAR(){//ISR to read linear motor enconder

  if (digitalRead(encoderALinearPIN) == HIGH){

    if (digitalRead(encoderBLinearPIN) == HIGH)sense_linear =1;
    else sense_linear =-1;

  }
  else{

    if (digitalRead(encoderBLinearPIN) == LOW)sense_linear = 1;
    else sense_linear =-1;

  }

  encoderLinearPos += sense_linear;

  t_ISR_last_linear = t_ISR_linear;
  t_ISR_linear = micros();

}

void ISR_VA_STEERING(){//IST to read steering motor encoder

  if (digitalRead(encoderASteeringPIN) == HIGH){

    if (digitalRead(encoderBSteeringPIN) == HIGH)sense_steering = 1;
    else sense_steering =- 1;

  }
  else{

    if (digitalRead(encoderBSteeringPIN) == LOW)sense_steering = 1;
    else sense_steering =- 1;

  }

  encoderSteeringPos += sense_steering;

  t_ISR_last_steering = t_ISR_steering;
  t_ISR_steering = micros();
}