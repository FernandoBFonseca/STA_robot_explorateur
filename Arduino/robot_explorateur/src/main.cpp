#include <Arduino.h>
#include <PID.hpp>

void ISR_VA_LINEAR();
void ISR_VA_STEERING();
void turn(int motor, float voltage );

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

#define KP 0.9824
#define KI 75.1801
#define KD 0.1
#define KC 1000.0


double InputLinear, OutputLinear, SetpointLinear;
PID pidLinear(&InputLinear, &OutputLinear, &SetpointLinear,
    KP, KI, KD, KC, P_ON_E, DIRECT);

double InputSteering, OutputSteering, SetpointSteering;
PID pidSteering(&InputSteering, &OutputSteering, &SetpointSteering,
    KP, KI, KD, KC, P_ON_E, DIRECT);

unsigned long lastTime = 0;

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

  Serial2.begin(115200);

  Serial2.println("Go boy");

}

void loop() {
  float voltage;
  unsigned long now = millis();
  if(now-lastTime>=SAMPLE_TIME){

    if(5000 <=now && now <= 10000) voltage = 5.0;
    else voltage = 0.0;
    turn(LINEAR,voltage);

    if(now <= 10000){
      Serial2.print(now);
      Serial2.print(", ");
      Serial2.print(voltage);
      Serial2.print(", ");
      Serial2.println(encoderLinearPos);
    }

    lastTime = now;
  }

  

}

void turn(int motor, float voltage ){
  int IN1,IN2,EN;
  if(motor == LINEAR){
    IN1 = IN2_LINEAR; IN2 = IN1_LINEAR; EN = EN_LINEAR; 
  }
  else if(motor == STEERING){
    IN1 = IN1_STEERING; IN2 = IN2_STEERING; EN = EN_STEERING;
  }

  if(voltage>0){
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
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
    return encoderLinearPos;
  }
  else if(motor == STEERING){
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