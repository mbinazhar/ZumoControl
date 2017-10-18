#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
const long interval = 1000;
unsigned long previousMillis = 0;

void setup() {
  // put your setup code here, to run once:
    inputString.reserve(200);
    Serial1.begin(9600);  
    Serial.begin(9600);  
//  buttonA.waitForButton();
    delay(1000);
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
    
  
}

void loop() {
  byte speeds[2];
  int left,right;
  byte received,counter;
  int temp;
  unsigned long currentMillis = millis();
  

if (stringComplete) {
  stringComplete = false;
  ledYellow(1);
  left = (byte)inputString[0];
  right = (byte)inputString[1];
  inputString = "";
  previousMillis = currentMillis;  
  
motors.setLeftSpeed( (left-128)*3 );
motors.setRightSpeed(( right-128)*3 );
ledGreen(1);ledRed(0);


Serial.print(left);
Serial.print(' ');
Serial.print(right);
Serial.print('\n');

  }

if(currentMillis - previousMillis >= interval) {
  
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  ledRed(1);ledGreen(0);
}



//delay(100);
ledYellow(0);

}

void serialEvent1() {
  //received = Serial1.available(); 
  
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    
    if (inChar ==  0)
    stringComplete = true;
    else  // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a NULL
    
    
  }
}

