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
  Serial.print("\nRequested Current in Duty Cycle ticks:");
  Serial.println(myMotor.currentRef,4);  
  Serial.print("Actual Speed:");
  Serial.println(myMotor.getActualSpeed(),4);
  
  /* read new duty clcle request */
  if (Serial.available())
  {
    myMotor.setCurrentRef(Serial.parseFloat());
  }
  // wait a second so as not to send massive amounts of data
  delay(1000);
}

