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

void loop()
{
//  iReadArray();
//  updateIr();
//  PID();
//  left_and_right();
//  _stop();
//  left_and_right();
  debugIr();
  debug_get_turn_weight();
//debugIr();
}

