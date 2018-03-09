#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

byte flagTimeout; 
int i;
uint16_t ch2_temp[10];
#define RC_CH1_INPUT  10
#define RC_CH2_INPUT  11
#define RC_CH3_INPUT  12
#define RC_CH4_INPUT  13

int average = 0;
#define MOTOR_1_PWM 2
#define m1a    24
#define m1b    26

#define MOTOR_2_PWM 3
#define m2a    28
#define m2b    30

int sensorPin = A0;

#define RELAY_DELAY 200
#define MOS_DELAY 10

#define PUL 4 //define Pulse pin
#define DIR 5  //define Direction pin
#define ENA 6  //define Enable Pin


uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];


int straight_speed;
int turn_speed;
int left_speed;
int right_speed;


void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }


//void resetPins(void){
//  digitalWrite(MOTOR_1_PWM, LOW);
//  digitalWrite(MOTOR_2_PWM, LOW);
//  digitalWrite(m1a, HIGH);
//  digitalWrite(m1b, HIGH);
//  digitalWrite(m2a, HIGH);
//  digitalWrite(m2b, HIGH);
//}

void leftmotor(int left_speed)
{
  int pwmValue = abs(left_speed) /2 ;
  if (pwmValue> 255)
  {pwmValue=255;}

if(left_speed<-100)
{ 
    analogWrite(MOTOR_1_PWM,pwmValue);
    analogWrite(MOTOR_2_PWM,pwmValue);
    delay(MOS_DELAY);
    digitalWrite(m1b,HIGH);
    delay(RELAY_DELAY);
    digitalWrite(m1a, LOW);
}
else if(left_speed>100)
{ 
    analogWrite(MOTOR_1_PWM,pwmValue);
    analogWrite(MOTOR_2_PWM,pwmValue);
    delay(MOS_DELAY); 
    digitalWrite(m1a, HIGH); 
    delay(RELAY_DELAY);  
    digitalWrite(m1b, LOW);
}
else
  { 
    digitalWrite(m1a, HIGH);
    digitalWrite(m1b, HIGH);
    analogWrite(MOTOR_1_PWM,0);
    analogWrite(MOTOR_2_PWM,0);
  }

}



void rightmotor(int right_speed)
{
  int pwmValue = abs(right_speed) / 2;
  if (pwmValue> 255)
  {pwmValue=255;}

if(right_speed<-100)
{ analogWrite(MOTOR_1_PWM,pwmValue);
  analogWrite(MOTOR_2_PWM,pwmValue);
  delay(MOS_DELAY);
  digitalWrite(m2b, HIGH);
  delay(RELAY_DELAY);
  digitalWrite(m2a, LOW);

}
else if(right_speed>100)
{ analogWrite(MOTOR_1_PWM,pwmValue);
  analogWrite(MOTOR_2_PWM,pwmValue);
  delay(MOS_DELAY);
  digitalWrite(m2a, HIGH);
  delay(RELAY_DELAY);
  digitalWrite(m2b, LOW);
}
else
{ 
  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, HIGH);
  analogWrite(MOTOR_1_PWM,0);
  analogWrite(MOTOR_2_PWM,0);
}

}



void stepper_reset(void)
{
  digitalWrite(PUL,LOW);
}

void fwd_step(void)
{  
 // for (int i=0; i<2000; i++)    //Forward 5000 steps
 // {
    digitalWrite(DIR,HIGH);
    digitalWrite(ENA,LOW);
    digitalWrite(PUL,LOW);
    delayMicroseconds(50);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(50);
  //}
  
}

void bck_step(void)
{
 // for (int i=0; i<2000; i++)    //Forward 5000 steps
 // {
    digitalWrite(DIR,LOW);
    digitalWrite(ENA,LOW);
    digitalWrite(PUL,LOW);
    delayMicroseconds(50);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(50);
 // }
  
 } 

void stop_Motor(void)
{
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, HIGH);
  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, HIGH);
    
  analogWrite(MOTOR_1_PWM, 0);
  analogWrite(MOTOR_2_PWM, 0);
  
  delay(RELAY_DELAY);
  delay(MOS_DELAY);
  
}


void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  
  pinMode( sensorPin, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);
  stop_Motor();
  
  i=0;

}

void loop() {
  flagTimeout = 0;   
  rc_read_values();


//Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print('\t');
//Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print('\t');
//Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print('\t');
//Serial.print("CH4:"); Serial.print(rc_values[RC_CH4]); Serial.print('\n');

uint16_t ch1=rc_values[RC_CH1];
uint16_t ch2=rc_values[RC_CH2];
uint16_t ch3=rc_values[RC_CH3];
uint16_t ch4=rc_values[RC_CH4];
//Serial.println(ch2);



//int sum = 0;


unsigned long time = millis();
int sensorValue = digitalRead(sensorPin);

Serial.print(sensorValue);

if ((time%200) < 100)
{ 
//  Serial.print("Time: ");
//  Serial.println(time);
//i++;
//if(i>9){i = 0;}
//  ch2_temp[i]=ch2;

i++;
if(i>5){i = 0;}

  ch2_temp[i]= sensorValue;
  
  
}
//if(average == ch2)
//  if (ch2_temp[0] == ch2_temp[1] && ch2_temp[1] == ch2_temp[2] && ch2_temp[2] == ch2_temp[3]&& ch2_temp[3] == ch2_temp[4]&& ch2_temp[4] == ch2_temp[5]&& ch2_temp[5] == ch2_temp[6]&& ch2_temp[6] == ch2_temp[7]&& ch2_temp[7] == ch2_temp[8]&& ch2_temp[8] == ch2_temp[9] )
if (ch2_temp[0] == ch2_temp[1] && ch2_temp[1] == ch2_temp[2] && ch2_temp[2] == ch2_temp[3]&& ch2_temp[3] == ch2_temp[4] )
{
//  Serial.println("Motor Stopper");
  flagTimeout = 0;
}
else
{
  flagTimeout = 1;
}


if (ch1<=10 || ch2<=10 || flagTimeout == 1)
{
 stop_Motor();
}
else 
{    
    flagTimeout = 0; 
    straight_speed = ch2 -1450;
    turn_speed = ch1 - 1450;
    left_speed = straight_speed + turn_speed;
    right_speed = straight_speed - turn_speed;

    leftmotor(left_speed);
    rightmotor(right_speed);
  
}


if ((ch4 <=10) || (ch4>=1300 && ch4<=1500))
{
 stepper_reset();
}
while (ch4>1500 && ch4 <=1950)
{
  fwd_step() ;
  rc_read_values();
  ch4=rc_values[RC_CH4]; 
}

 while (ch4>900 && ch4<1300)
{
  bck_step();  
  rc_read_values();
  ch4=rc_values[RC_CH4];
}
  delay(8);
  
  
}
