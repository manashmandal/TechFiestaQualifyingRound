#ifndef LFR_H_
#define LFR_H_

#include <Motor.h>
#define QTR_THRESHOLD 600
#define SETPOINT 4500
#define REDUCE 70
#define led 13

int iMotorSpeed = 110;

float kp = .5;
float kd = .667;
float ki = 0;



int reduced_speed = iMotorSpeed - REDUCE;


Motor *motor;

motorPins pins = {2, 3, 5, 4};

int sensor[] = {54, 55, 56, 57, 58, 59, 60, 61};
int reading[8] = {0, 0, 0, 0, 0, 0, 0, 0};

long total_weight = 0;
int total_active_sensor = 0;


byte leftIr = 63;
byte rightIr = 62;


const int tcrt_threshold = 700;
bool tcrt_inverse_logic = false;

int left_reading = 0;
int right_reading = 0;

int left_sensor(void){
  if (analogRead(leftIr) > tcrt_threshold){
    left_reading = 1;
    return 1;
  } else {
    left_reading = 0;
    return 0;
  }
}

int right_sensor(void){
  if (analogRead(rightIr) > tcrt_threshold){
    right_reading = 1;
    return 1;
  } else {
    right_reading = 0;
    return 0;
  }
}



//Updates current reading of irs
void updateIr(void){
  left_reading = left_sensor();
  right_reading = right_sensor();
}

//Debug left and right sensor
void debugIr(void){
  updateIr();
  Serial.println("====== BEGIN =======");
  Serial.print("LEFT : ");
  Serial.println(analogRead(leftIr));
  Serial.print(" ");
  Serial.println(left_reading);
  Serial.print("RIGHT : ");
  Serial.println(analogRead(rightIr));
  Serial.print(" ");
  Serial.println(right_reading);
  Serial.println("====== END =======");
}

//Setting up irs
void setupIr(void){
  pinMode(leftIr, INPUT);
  pinMode(rightIr, INPUT);
}


//Init sensor
void initSensor(void){
  for (int i = 0; i < 8; i++){
    pinMode(sensor[i], INPUT);
  }
}


void blink(int times){
  for (int i = 0; i < times; i++){
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }
}


//error
float error = 0;
float previousError = 0;
float totalError = 0;
float power = 0;

int PWM_Left = 0;
int PWM_Right = 0;

int iLastRead;

int iReadArray(void){
  int iRead = 0;
  int iActive = 0;
  for (int i = 0; i < 8; i++){
    if (analogRead(sensor[i]) > QTR_THRESHOLD){
      reading[i] = 1;
      iRead += (i+1) * 1000;
      iActive++;
    } else {
      reading[i] = 0;
    }
  }
  total_weight = iRead;
  total_active_sensor = iActive;
  iRead = map(iRead/iActive, 0, 8000, 0, 1023);
  if (!iRead) return iLastRead;
  else {
    iLastRead = iRead;
    return iRead;
  }
}


void PID(void){
  int avgSensor = iReadArray();
  previousError = error;
  error = avgSensor - map(SETPOINT, 0, 8000, 0, 1023);
  totalError += error;
  power = (kp * error) + (kd*(error - previousError)) + (ki*totalError);
  if (power > iMotorSpeed) { power = iMotorSpeed; }
  if (power < -1 * iMotorSpeed) { power = -1 * iMotorSpeed; }
  if (power < 0){
    PWM_Right = iMotorSpeed;
    PWM_Left = iMotorSpeed - abs(int(power));
  } else {
    PWM_Right = iMotorSpeed - int(power);
    PWM_Left = iMotorSpeed;
  }
  motor->go(PWM_Left, PWM_Right, FORWARD);
}

void stop(void){
  motor->go(0, 0, NOWHERE);
}


void setup(){
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  
  initSensor();
  motor = new Motor(pins);
  setupIr();  
}


#endif
