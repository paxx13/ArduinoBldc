#include <BLDC_Control.h>

#define SPEED_CTRL_FRQ  1000 /* Speed control frequency in Hz */

BldcControl myMotor;
float       speedReference;       /* reference for speed control */
float       speedMax    = 9000;   /* maximum Speed in rpm */
float       SpdCtrlKint = 0.0001; /* integral gain for speed control */
float       SpdCtrlKprp = 0.0005; /* proportinal gain for speed control */
float       iInt;                 /* integrator memory */

/* interrupt routine for speed control */ 
void TC6_Handler(){
  float iErr;
  float iOut;
  float iPrp;
  
  /* clear interrupt flag */
  TC_GetStatus(TC2, 0);

  /* calculate error */
  iErr = speedReference - myMotor.getActualSpeed();
    
  /* proportional path */
  iPrp = iErr * SpdCtrlKprp;
    
  /* integral path */
  iInt += iErr * SpdCtrlKint;

  /* calculate current setpoint and send to motor instance */
  iOut = iPrp + iInt;
  myMotor.setCurrentRef(iOut);
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
  Serial.println(speedReference);  
  Serial.print("Actual Speed in rpm:");
  Serial.println(myMotor.getActualSpeed(),4);
  
  /* read new speed request */
  if (Serial.available())
  {
    speedReference = Serial.parseFloat();
  }
  // wait a second so as not to send massive amounts of data
  delay(1000);
}
