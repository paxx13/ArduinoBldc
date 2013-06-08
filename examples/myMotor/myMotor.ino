#include <BLDC_Control.h>


BldcControl myMotor;             // motor instance
String inputString = "";         // a string to hold incoming data

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
    Serial.println(myMotor.debug);//myMotor.getActualSpeed(),4);
  
    /* read commands from serial port */
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        // add it to the inputString:
        inputString += inChar;
        
        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it:
        if (inChar == '\n') {      
            if (inputString.startsWith(String("Iref="))){
                inputString = inputString.substring(5);
                char floatVar[5];
                inputString.toCharArray(floatVar,5);
                myMotor.setCurrentRef(atof(floatVar));     
            }
            if (inputString.startsWith(String("start"))){
                myMotor.start();             
            }
            if (inputString.startsWith(String("stop"))){
                myMotor.stop();             
            }
            inputString = ""; 
        }
    }
  
    // wait a second so as not to send massive amounts of data
    delay(1000);
}


