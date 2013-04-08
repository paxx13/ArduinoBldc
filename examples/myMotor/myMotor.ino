#include <BLDC_Control.h>


BldcControl myMotor;

void setup() {
  /* setup motor */
  myMotor.Config();  
  
  /* setup serial communication */
  Serial.begin(9600);
}

void loop() {
  /* print duty cycle speed */
  Serial.print("\nRequested DC:");
  Serial.println(myMotor.speedRequest,4);  
  Serial.print("Actual DC:");
  Serial.println(myMotor.getActualSpeed(),4);
  
  /* read new duty clcle request */
  if (Serial.available())
  {
    myMotor.speedRequest = Serial.parseFloat();
  }
  // wait a second so as not to send massive amounts of data
  delay(1000);
}

