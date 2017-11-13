#include <Wire.h>
#include <Zumo32U4.h>
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ProximitySensors proxSensors;
String inputString = "";
// a string to hold incoming data
boolean stringComplete = false;
// whether the string is complete
const long interval = 1000;
unsigned long previousMillis = 0;
int n=0;
void setup() {
  // put your setup code here, to run once:
  inputString.reserve(200);
  Serial1.begin(9600);
  Serial.begin(9600);
  //  buttonA.waitForButton();
  delay(1000);
  proxSensors.initThreeSensors();
}
void loop() {
  byte speeds[2];
  proxSensors.read();
  // NEED TO SET JUMPERS TO WORK PROPERLY
  byte pF = proxSensors.readBasicFront();
  byte pL = proxSensors.readBasicLeft();
  byte pR = proxSensors.readBasicRight();
  while (Serial1.available()) {
    Serial.print(Serial1.read());
  }
  delay(200);
  Serial1.print("S,");
  Serial1.print(pL);
  Serial1.print(pF);
  Serial1.print(pR);
  Serial1.print('\n');
  // n=n+1;
  // while (Serial.available()) {
  //    char temp = (char)Serial.read();
  //    Serial.println(temp);
  //    Serial1.println(temp);
  //}
}
