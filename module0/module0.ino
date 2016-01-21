/*
 * 
 * Check leftmost and rightmost sensor
 */

#include "LFR.h"
#include "DEBUG.h"

void left_and_right(void){
  updateIr();
  if (left_reading == 1){
    stop();
    blink(3);
  } 
}

void _stop(){
  iReadArray();
  if (reading[0] == 1 || reading[7] == 1){
    stop();
    blink(10);
  }
}

void turn_left(){
  while(numberof_active_sensors() != 0){
    motor->go(80, 80, FORWARD);
  }
  while(true) stop();
}

void turn_right(){
  while(numberof_active_sensors() != 0){
    motor->go(80, 80, FORWARD);
  }
  while(true)  stop();
}

void loop()
{
  if (check_turn() == TURN_LEFT) { Serial.println("TURN LEFT"); turn_left(); }
  else if (check_turn() == TURN_RIGHT) { Serial.println("TURN RIGHT"); turn_right(); }
  else if (check_turn() == NO_TURN) {  Serial.println("PID"); PID(); }
}

