#define encoderRevCounts 1670   //ticks per revolution of wheel belt
#define circumference 217   //mm //on the bassis a belt
#define diameter_zumo 69  //mm  //taken out from circumference
#define length_zumo 85   //mm   // distance between wheels
#include <Wire.h>
#include <Zumo32U4.h>

#define mySerial Serial1   //Can Select Serial Port from here

Zumo32U4Motors motors;      //motors initialize
Zumo32U4Encoders encoders;  //encoder initialize

long encCurrL, encCurrR;    //encoder left and right

unsigned long previousMillis = 0;
int interval = 10; // in ms         // PID sample interval
unsigned long currentMillis = 0;

unsigned long timeOutPreviousMillis = 0;  //Velocity Time Out.
int velocityTimeOut = 5000; //ms  // changing this will change the velocity time out        

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

struct pidParam {   //structure for PID parameters
  double kp;
  double ki;
  double kd;
  };
  
pidParam motorL = {1,0,0}; //giving initial values of PID for motor Left
pidParam motorR = {1,0,0}; //giving initial values of PID for motor Right

#include <PID_v1.h>
double measuredVelL = 0, measuredVelR = 0;  //Our required
double pwmL = 0, pwmR = 0;                  //Output PWM For zumo
double velL = 0, velR = 0;                  //Our set point for PID
// PID (&input, &output, &setpoint, kp, ki, kd, DIRECT/REVERSE)
PID pidL(&measuredVelL, &pwmL, &velL, motorL.kp, motorL.ki, motorL.kd, DIRECT);
PID pidR(&measuredVelR, &pwmR, &velR, motorL.kp, motorR.ki, motorR.kd, DIRECT);

uint16_t batteryLevel = 0;
uint16_t battery_low = 7150; //7.15mv
//      PID////
boolean pidActive = false;              // Code itself deal with this parameter

Zumo32U4Buzzer buzzer;

const char fugue[] PROGMEM =
  "! O5 L16 agafaea dac+adaea fa<aa<bac#a dac#adaea f";
  
  
void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600);                   //initialize mySerial Baudrate
  Serial.begin(9600);
  initPID();                            //initializing and defining PID parameters
  velL = 00; // "m/s" 
  velR = 00; 
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*Serial.print(measuredVelL);
  Serial.print(", ");
  Serial.println(measuredVelR);*/
  currentMillis = millis();
  mySerialRead();                               //Reading the mySerial Command
  if(stringComplete){
    Serial.print(currentMillis);
    Serial.print(" RECEIVED DATA: ");
    Serial.print(inputString);
    interpretmySerialData();
    stringComplete = false;
    inputString="";
//    Serial.println(readBatteryMillivolts());
  }
batteryLevel = readBatteryMillivolts();
  if(batteryLevel<=(battery_low-100) && !usbPowerPresent())
  { 
    if(!buzzer.isPlaying())
    buzzer.playFromProgramSpace(fugue);
    }
  else if(batteryLevel>=(battery_low+100) || usbPowerPresent())
  {  buzzer.stopPlaying();}
  
  if (int(currentMillis - previousMillis) >= interval) { //timed  loop for measuring Velocities
    previousMillis = currentMillis;
    encCurrL = encoders.getCountsAndResetLeft();  //encoderLeft
    encCurrR = encoders.getCountsAndResetRight(); //
    float distance1 = (encCurrL/(float)encoderRevCounts)*1000;
    measuredVelL = float(distance1/interval)*circumference;
    float distance2 = (encCurrR/(float)encoderRevCounts)*1000;
    measuredVelR = float(distance2/interval)*circumference;
  /*  Serial.print(measuredVelL);
    Serial.print(" , ");
    Serial.println(measuredVelR);    */
    }

  if(pidActive){
      pidL.Compute();
      pidR.Compute();
      motors.setLeftSpeed(pwmL);
      motors.setRightSpeed(pwmR);
  }

  if (int(currentMillis - timeOutPreviousMillis) >= velocityTimeOut ) {  // Break Velocity after Certain Time.
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      //Serial.println("Hello");
      pidActive = false;
  }
}

void interpretmySerialData(void) {
  int c1 = 1, c2 = 1;
  float val1 = 0, val2 = 0;
  float a = 0, b = 0;
  switch (inputString[0]) {
    case 'D':
      // COMMAND:  D,speed_motor_left,speed_motor_right\n
      c1 = inputString.indexOf(',') + 1;
      c2 = inputString.indexOf(',', c1);
      val1 = inputString.substring(c1, c2).toFloat();
      val1 = constrain( val1, -300 , 300 );
      c1 = c2 + 1;
      val2 = inputString.substring(c1).toFloat();
      val2 = constrain( val2, -300 , 300 );
      velL=val1;
      velR=val2;
      pidActive = true;
      timeOutPreviousMillis = millis();
      Serial.println(val1);
      Serial.println('d');
      break;
    case 'L':
      // COMMAND:  L,speed_motor_left,speed_motor_right\n
      c1 = inputString.indexOf(',') + 1;
      c2 = inputString.indexOf(',', c1);
      val1 = inputString.substring(c1, c2).toInt();
      val1 = constrain( val1, -200 , +200 );
      c1 = inputString.indexOf(',', c2) + 1;
      val2 = inputString.substring(c1).toInt();
      val2 = constrain( val2, -200 , 200 );
      motors.setLeftSpeed(val1);
      motors.setRightSpeed(val2);
      Serial.println('l');
      timeOutPreviousMillis = millis();
      pidActive = false;
      break;
    case 'W':
      // COMMAND:  W,linear_speed,angular_speed\n
      c1 = inputString.indexOf(',') + 1;
      c2 = inputString.indexOf(',', c1);
      val1 = inputString.substring(c1, c2).toFloat();
      c1 = c2 + 1;
      val2 = inputString.substring(c1).toFloat();
      val1 = constrain( val1, -2 , 2 );
      val2 = constrain( val2, -1 , 1 );
      
      a=2*val1;
      b=val2*length_zumo;
      velL=(a-b)/diameter_zumo;
      if(abs(velL)<0.05)
      velL = 0;
      velR=(a+b)/diameter_zumo;
      if(abs(velR)<0.05)
      velR = 0;
      Serial.print(velL);
      Serial.print(" ");
      Serial.println(velR);
      pidActive = true;
      timeOutPreviousMillis = millis();
      Serial.println('w');
      break;
      case 'H':
      // COMMAND:  H,P,I,D,1/2\n
      float p, i, d;
      c1 = inputString.indexOf(',') + 1;
      c2 = inputString.indexOf(',', c1);
      p = inputString.substring(c1, c2).toFloat();
      c1 = c2 + 1;
      c2 = inputString.indexOf(',', c1);
      i = inputString.substring(c1, c2).toFloat();
      c1 = c2 + 1;
      c2 = inputString.indexOf(',', c1);
      d = inputString.substring(c1, c2).toFloat();
      c1 = c2 + 1;
      val1 = inputString.substring(c1).toInt();
      if (val1 == 1) {
        motorL.kp = p;
        motorL.ki = i;
        motorL.kd = d;
        pidL.SetTunings(motorL.kp, motorL.ki, motorL.kd);
        Serial.println("motorPIDL ");
      }
      else if (val1 == 2) {
        motorR.kp = p;
        motorR.ki = i;
        motorR.kd = d;
        pidR.SetTunings(motorR.kp, motorR.ki, motorR.kd);
        Serial.println("motorPIDR ");
      }
      Serial.print("h,");
      Serial.print(val1);
      Serial.print(',');
      Serial.print(p);
      Serial.print(',');
      Serial.print(i);
      Serial.print(',');
      Serial.println(d);
      //delay(1000);
      break;
    default:
      Serial.print("UNKNOWN COMMAND: ");
      Serial.println(inputString);
      break;
  }
}

void mySerialRead(void)
{
  if(mySerial.available()){
    while (mySerial.available()) {
      char inChar = (char)mySerial.read();
      inputString += inChar;
      if (inChar == '\n') {
        stringComplete = true;
      }
    }
  }
}

void initPID(void) {
  pidL.SetMode(MANUAL); // PID CONTROL OFF
  pidR.SetMode(MANUAL);
  pidL.SetSampleTime(interval); // sample time for PID
  pidR.SetSampleTime(interval);
  pidL.SetOutputLimits(-250, 250); // min/max PWM
  pidR.SetOutputLimits(-250, 250);
}
