//////////////////For sonar/////////////

int req_dis=12;
int s_pGain=100;
int s_iGain=0;
int s_dGain=0;
int s_iInteg=0;
int s_eInteg=0;
int s_ePrev=0;
///////////////For IR array///////////////       
int pGain=100;
int iGain=0;
int dGain=50;
int iInteg=0;
int eInteg=0;
int ePrev=0;

int echoPort =30;
int triggerPort =31;


int INA=22;  
int INB=23;
int INC=24;
int IND=25;

int EN1=2;
int EN2=3;

int speedA;
int speedB;
int avgSpeed=110;
int s_avg=60;

float pid=0;
float error=0;

void setup() {
 
pinMode( INA, OUTPUT );
pinMode( INB, OUTPUT );
pinMode( INC, OUTPUT );
pinMode( IND, OUTPUT );

pinMode( triggerPort, OUTPUT );
pinMode( echoPort, INPUT );
Serial.begin( 9600 );
}
int flag=0;
int flag2=0;
void loop() {


//float sensor_value= read_sensor();
//float turn=PID(sensor_value,3);
//delay(1000);
//read_sensor();
float wall= distance();
if(wall<15)
{
  flag=1;
  flag2++;
  
  
}
if(flag2==1)
{
  a_for();
  b_for();
  analogWrite(EN1,50);
  analogWrite(EN2,50);
  delay(1000);
  flag2++;
}

if(wall>5 && wall<20)
{
   test();
}
else
{
  motor();
}

}


//float i =front_sensor();
//Serial.println(i);
//a_for();
//analogWrite(EN1,50);
//analogWrite(EN2,0);

//////////////*******************//////////////////////////
float turn;

void motor()
{
float x;
float sensor_value= read_sensor();

if(sensor_value=='l')  //For hard turn lift
{
 x=front_sensor();
 //Serial.print("front_sensor");
 //Serial.println(x);
  
if(x==0)
{
  a_bac();
  b_for();
  analogWrite(EN1,65);
  analogWrite(EN2,65);
  while(analogRead(0)>347);
}
sensor_value= front_sensor();
}
//
if(sensor_value=='r')      //For hard turn right
{
  float x=front_sensor();
  if(x==0)
  {
  b_bac();
  a_for();
  analogWrite(EN1,65);
  analogWrite(EN2,65);
  while(analogRead(4)>428);
  }
  sensor_value= front_sensor();
}

////////

////////

if(sensor_value=='s')
{
   analogWrite(EN1,0);
  analogWrite(EN2,0);
  delay(7000);
}
////////////////////////*********************************///////////////////////////
//turn=PID(sensor_value,3);
if(sensor_value!=0)
{
  turn=PID(sensor_value,3.0);
}

//Serial.println(turn);
 if(turn>=500)          //Limit the pid value
   turn=500;
 if(turn<=-500)
   turn=-500;

 if(turn>0)            // when left arrayes touch line
 {
   if(turn>250)        //Vary hard turn
   {
     a_bac();
     speedA=(turn-250)/5;
   }
   else                //Normar turn
   {
     a_for();
     speedA=(250-turn)/5;
   }
   speedB=avgSpeed;
 }
 
 
 if(turn==0)           //When On the line
 {
   a_for();
   b_for();
   speedA=avgSpeed;
   speedB=avgSpeed;
 }


 if(turn<0)            //When Right sensor touchs line
 {
   if(turn<-250)        //Vary hard turn
   {
     b_bac();
     speedB=(-turn-250)/5;
   }
   else                //Normar turn
   {
     b_for();
     speedB=(250+turn)/5;
   }
   speedA=avgSpeed;
 }
 analogWrite(EN1,speedA);
 analogWrite(EN2,speedB);
 int wall= distance();
  if (wall<20)
  {
    flag=1;
    loop;
  }
}

 /*void test()
{
  float value=distance();
  if(value<15)
  {
  analogWrite(EN1,speedA);
  analogWrite(EN2,speedB);
  }
  else 
  {
    analogWrite(EN1,0);
    analogWrite(EN2,0);
  }
}*/
void test()
{ 
 
// analogWrite(EN1,0);
// analogWrite(EN2,0);
//  delay(5000);
  a_for();
  b_for();
  float cur_distance=distance();
  float turn = s_PID(cur_distance,req_dis);
  
Serial.println(turn);  
  if(turn>=255)
   turn=255;
 if(turn<=-255)
   turn=-255;
 
 if(turn>=0){
 speedA=s_avg;
 speedB=s_avg-turn/20;
 }
 else{
 speedA=s_avg+turn/20;
 speedB=s_avg;
 }
 
 analogWrite(EN1,speedA);
 analogWrite(EN2,speedB);
 if(front_sensor !=0)
 {
   flag=0;
 }
 
}



/////////////////For IR///////////////////////
float PID(float cur_value,float req_value)
{
	
        
	error = req_value - cur_value;
	pid = (pGain * error) + (dGain * (error - ePrev));

	eInteg += error;                  // integral is simply a summation over time
	ePrev = error;                    // save previous for derivative

	return pid;
}

//////////////////For Sonar /////////////////////////////////////////
float s_PID(float cur_value,float req_value)
{
	

	error = req_value - cur_value;
	pid = (s_pGain * error) + (s_dGain * (error - s_ePrev));

	s_eInteg += error;                  // integral is simply a summation over time
	s_ePrev = error;                    // save previous for derivative

	return pid;
}

//////////////*******************//////////////////////////
float distance()
{
digitalWrite(triggerPort, LOW);			// set to LOW trigger's output
digitalWrite(triggerPort, HIGH);		// send a 10us pulse to the trigger
delayMicroseconds( 10 );
digitalWrite(triggerPort, LOW);
 
long duration = pulseIn(echoPort, HIGH);
 
long r = 3.4 * duration / 2;			// here we calculate the distance using duration

float distance = r / 100.00;

 
//Serial.print("duration: ");
//Serial.println(duration);
//Serial.print(" , ");
//Serial.print("distance: ");
//// 
//
//if( duration > 38000 ) Serial.println("out of reach");		// if duration in greather than 38ms, the obstacle is out of reach
//else { Serial.print(distance); Serial.println("cm");}
// 
//
//delay( 500);
Serial.println(distance);
							// wait for 1s
return distance;
}
void a_for()        ///Motor A forward direction
{
digitalWrite(INA,1);
digitalWrite(INB,0);
}

void a_bac()        ///Motor A backward direction
{
digitalWrite(INA,0);
digitalWrite(INB,1);
}

void b_for()        ///Motor B forward direction
{
digitalWrite(INC,1);
digitalWrite(IND,0);

}

void b_bac()        ///Motor B backward direction
{
digitalWrite(INC,0);
digitalWrite(IND,1);

}

void a_stop()
{
  digitalWrite(INA,0);
  digitalWrite(INB,0);
}
void b_stop()
{
  digitalWrite(INC,0);
  digitalWrite(IND,0);
}
float read_sensor()
{
  int s[5];
  int value[7]={347,405,457,537,428,90,120};
  float avgSensor;
  for(int i=0;i<5;i++)
  {
    if(analogRead(i)<value[i])
    {
       s[i]=1;
    }
    else
    {
      s[i]=0;
    }
  }
  if(analogRead(5)<value[5])
   {
     return 'l';              //when left sensor touches
   }
   if(analogRead(6)<value[6])
   {
     return 'r';              //when right sensor touches
   }
   //347,405,457,537,428
   if(analogRead(0)<347)
   {
     if(analogRead(4)<428)
     {
       if(analogRead(2)>457)
       {
         return 's';
       }
     }
     //return 's';
     
   }
   //325,355,440,550,425,137,150
   /*if(analogRead(0) > 325 && analogRead(3) < 550 && analogRead(4) < 425 )
   return 'd';
   
   if(analogRead(4) > 425 && analogRead(0) < 325 && analogRead(0) < 355 )
   return 'b';*/
//  for(int i=0;i<7;i++)    //For test
//  {
//     Serial.print(value[i]);  //
//     Serial.print('\t'); //     
//  }
//  Serial.println();      //
  
  avgSensor = s[0]*1 + s[1]*2 + s[2]*3 + s[3]*4 + s[4]*5 ;
  if(avgSensor==0)
  {
    return 0;
  }
  avgSensor = (float) avgSensor / (s[0] + s[1] + s[2] + s[3] + s[4]);
  return avgSensor;

}
  
  float front_sensor()
{
  int s[5];
  int value[5]={347,405,457,537,428};
  float avgSensor;
  for(int i=0;i<5;i++)
  {
    if(analogRead(i)<value[i])
    {
       s[i]=1;
    }
    else
    {
      s[i]=0;
    }
  }
  
   
  avgSensor = s[0]*1 + s[1]*2 + s[2]*3 + s[3]*4 + s[4]*5 ;
  if(avgSensor==0)
  {
    return 0;
  }
  avgSensor = (float) avgSensor / (s[0] + s[1] + s[2] + s[3] + s[4]);
  return avgSensor;

}

