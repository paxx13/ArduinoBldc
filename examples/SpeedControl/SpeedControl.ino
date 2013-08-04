#include <BLDC_Control.h>

#define SPEED_CTRL_FRQ 500 /* Speed control frequency in Hz */

BldcControl myMotor;
String inputString = "";
//boolean stringComplete = false;

float speedReference; /* reference for speed control */
float speedMax = 9000; /* maximum Speed in rpm */
float SpdCtrlKint = 0.0005; /* integral gain for speed control */
float SpdCtrlKprp = 0.0005; /* proportinal gain for speed control */
float iInt; /* integrator memory */
float vErr;
uint16_t controlCnt;
float speedFilt;
float ActualSpeed;


/* interrupt routine for speed control */
void TC6_Handler(){

  float vOut;
  float vPrp;

  /* clear interrupt flag */
  TC_GetStatus(TC2, 0);

  controlCnt++;
  /* filter commutation period */
  speedFilt += myMotor.getActualSpeed();
  if(controlCnt>=10)
  {
	  ActualSpeed = speedFilt / controlCnt;
	  speedFilt = 0;
	  controlCnt = 0;
  }

  /* calculate error */
  vErr = speedReference - ActualSpeed;

  /* proportional path */
  vPrp = vErr * SpdCtrlKprp;

  /* integral path */
  if (vErr > 0)
    {
        iInt = iInt + SpdCtrlKint;
    }
    else if (vErr < 0)
    {
        iInt = iInt - SpdCtrlKint;
    }
    /* integral value limiter - anti wind up */
    if (iInt > 500)
    {
        iInt = 500;
    }
    else if (iInt < - 500)
    {
        iInt = -(500);
    }

  /* calculate current setpoint and send to motor instance */
  vOut = vPrp + iInt;
  myMotor.setCurrentRef(vOut);
}

/* timer setup */
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

void setup() {
  /* setup motor */
  myMotor.Config();

  /* setup interrupt for speed control */
  startTimer(TC2, 0, TC6_IRQn, SPEED_CTRL_FRQ);
  /* setup serial communication */
  Serial.begin(9600);
}

void loop() {
  /* print duty cycle speed */
  Serial.print("\nRequested Speed in rpm:");
  Serial.println(speedReference,4);
  Serial.print("Actual Speed in rpm:");
  Serial.println(myMotor.getActualSpeed(),4);

    /* read commands from serial port */
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        // add it to the inputString:
        inputString += inChar;

        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it:
        if (inChar == '\n') {
            if (inputString.startsWith(String("SetV="))){
                inputString = inputString.substring(5);
                char floatVar[7];
                inputString.toCharArray(floatVar,7);
                speedReference=atof(floatVar);
            }
            if (inputString.startsWith(String("Vprp="))){
                inputString = inputString.substring(5);
                char floatVar[7];
                inputString.toCharArray(floatVar,7);
                SpdCtrlKprp=atof(floatVar);
            }
            if (inputString.startsWith(String("Vint="))){
                inputString = inputString.substring(5);
                char floatVar[7];

                inputString.toCharArray(floatVar,7);
                SpdCtrlKint=atof(floatVar);
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
