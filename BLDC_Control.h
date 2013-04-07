/*******************************************************************************
  BLDC_Control.h - Library for brushless DC Motor Control.
  Author:   Swen Wahl
  Date:     07.03.2013
  License:  Released into the public domain
*******************************************************************************/
#ifndef BLDC_CONTROL_H
#define BLDC_CONTROL_H

#include "Arduino.h"

struct mach#define _useTimer1

/*
  TC0, chan 0 => TC0_Handler
  TC0, chan 1 => TC1_Handler
  TC0, chan 2 => TC2_Handler
  TC1, chan 0 => TC3_Handler
  TC1, chan 1 => TC4_Handler
  TC1, chan 2 => TC5_Handler
  TC2, chan 0 => TC6_Handler
  TC2, chan 1 => TC7_Handler
  TC2, chan 2 => TC8_Handler
 */

#if defined (_useTimer1)
#define TC_FOR_TIMER1       TC1
#define CHANNEL_FOR_TIMER1  0
#define ID_TC_FOR_TIMER1    ID_TC3
#define IRQn_FOR_TIMER1     TC3_IRQn
#define HANDLER_FOR_TIMER1  TC3_Handler
#endif    
#if defined (_useTimer2)
#define TC_FOR_TIMER2       TC1
#define CHANNEL_FOR_TIMER2  1
#define ID_TC_FOR_TIMER2    ID_TC4
#define IRQn_FOR_TIMER2     TC4_IRQn
#define HANDLER_FOR_TIMER2  TC4_Handler
#endif

typedef enum { _timer1, _timer2, _Nbr_16timers } timer16_Sequence_t ;

#define MAX_MOTORS  1machineProperties
{
    uint8_t   polePairs;
    uint16_t  ratedSpeed;
};


class BldcControl
{
    public:
        /* member methods */
        BldcControl(void);
        void     Config(Tc         *tc, 
      voidtationControl(void);
        float    getActualSpeed(void);

        /* member variables */
        uint32_t interruptCounter;
        float    speedRequest;
        int32_t            debug;
    /*----------------------------------------------------------*/    
    private:  
        /* member methods */    
        void      configureTimerInterrupt(Tc         *tc, 
                                          uint32_t   channel, 
                                          IRQn_Type  irq, 
                                          uint32_t   frequency);
        void      configurePIOC(void);
        void      configureADC(void);
        void      configurePWMC(void);
        void      configurePMC(void);
        int16_t  CurrentControl(int16_t iFbk);
        uint8__t   pwmSwitchingCU(uint8_t hallState);
        uint8_t   pwmSwitchingIU(uint8_t hallState);
        int16_t     firU(int16_t NewSample);
        in/* member variables */
        uint8_t             motorIndex; int16_t             rotorPosition;
        int16_t             phiElec;
        int16_t             phiElecOld;
        uint16_t            pwmPeriod; 
        int8_t              commutationTable[8][3];

        machineProperties   motorProperties;
        uint8_t             previousHallStat             currentRef;
        int16_t             Kprp;
        uint16_t            Kint;
        int16_t             iInt;
        int16_t             test;
        int32_t             IfbkFilt;
        int16_t             Iu_old;
        int16_t             Iv};

#endif /* BLDC_CONTROL_H */