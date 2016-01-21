#ifndef DEBUG_H_
#define DEBUG_H_

void check_left_right(void){
  Serial.print("Left: ");
  Serial.println(analogRead(leftIr));
  Serial.print("Right: ");
  Serial.println(analogRead(rightIr));
  delay(500);
  debugIr();
  delay(500);
}

#endif
