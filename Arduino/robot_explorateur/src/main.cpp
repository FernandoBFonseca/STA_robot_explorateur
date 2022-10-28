#include <Arduino.h>
#include <PID.hpp>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <speedEstimator.hpp>

void ISR_VA_LINEAR();
void ISR_VA_STEERING();
void turn(int motor, float voltage );
double getPos(int motor);

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

#define KP_LINEAR 0.001311
#define KI_LINEAR 0.1056
#define KD_LINEAR 0.0
#define KC_LINEAR 1000.0
#define ENCODER_2_M 5.2e-4

#define KP_STEERING 0.01311
#define KI_STEERING 0.00556
#define KD_STEERING 0.0
#define KC_STEERING 1000.0
#define ENCODER_2_RAD 5.2e-4
#define K_STEERING 0.1054
#define TAU_STEERING 0.0204


double InputLinear, OutputLinear, SetpointLinear;
PID pidLinear(&InputLinear, &OutputLinear, &SetpointLinear,
    KP_LINEAR, KI_LINEAR, KD_LINEAR, KC_LINEAR, P_ON_E, DIRECT);

double PositionSteering, OutputSteering, SetpointSteering, VelocityEstSteering;
double const L_Steering[3] = {4.315e4,3.9e2,5.4573e5};
speedEstimator speedSteering(&PositionSteering, &OutputSteering, &VelocityEstSteering, 
              K_STEERING, TAU_STEERING,L_Steering, SAMPLE_TIME);
PID pidSteering(&PositionSteering, &OutputSteering, &SetpointSteering,
    KP_STEERING, KI_STEERING, KD_STEERING, KC_STEERING, P_ON_E, DIRECT);

ros::NodeHandle nh;
void messageCb( const std_msgs::Float64& speed_msg) {
  
  SetpointSteering = speed_msg.data;
}

ros::Subscriber<std_msgs::Float64> sub("ref_speed", messageCb );

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

  //pidSteering.SetControllerDirection(REVERSE);

  pidLinear.SetSampleTime(SAMPLE_TIME);
  pidSteering.SetSampleTime(SAMPLE_TIME);

  //Serial.begin(115200);

  nh.initNode();

  nh.subscribe(sub);

  //Serial.println("Go boy");

  
  lastTime = millis();

}

void loop() {
  //float voltage;
  unsigned long now = millis();
  if(now-lastTime>=SAMPLE_TIME){

    PositionSteering = getPos(STEERING);

    //if(2500<now && now<7500) OutputSteering = (PI/12)/ENCODER_2_RAD;
    //else OutputSteering = 0;

    speedSteering.Compute();
    pidSteering.Compute();
    turn(STEERING,OutputSteering);

    /*if(now <= 30000){
      Serial.print(now);
      Serial.print(", ");
      Serial.print(OutputSteering);
      Serial.print(", ");
      Serial.print(getPos(STEERING));
      Serial.print(", ");
      Serial.println(SetpointSteering);
    }*/
    lastTime = now;

    
  }

  nh.spinOnce();

}

void turn(int motor, float voltage ){

  if(abs(voltage) <= 1.6) voltage = 0;

  int IN1,IN2,EN;
  if(motor == LINEAR){
    IN1 = IN1_LINEAR; IN2 = IN2_LINEAR; EN = EN_LINEAR; 
  }
  else if(motor == STEERING){
    IN1 = IN1_STEERING; IN2 = IN2_STEERING; EN = EN_STEERING;
  }

  if(voltage>0){
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
    }
    else{
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
    }
    voltage = constrain(voltage, -SAT_VOLTAGE, SAT_VOLTAGE);

    analogWrite(EN, abs(voltage)*255.0/MAX_VOLTAGE);
}

double getPos(int motor){
  if(motor == LINEAR){
    return ENCODER_2_M*encoderLinearPos;
  }
  else if(motor == STEERING){
    //return ENCODER_2_RAD*encoderSteeringPos;
    return encoderSteeringPos;
  }
  return 0.0;


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


}