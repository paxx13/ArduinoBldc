/*******************************************************************************
  BLDC_Control.h - Library for brushless DC Motor Control.
  Author:   Swen Wahl
  Date:     07.03.2013
  License:  Released into the public domain
*******************************************************************************/
#ifndef BLDC_CONTROL_H
#define BLDC_CONTROL_H

#include "Arduino.h"

struct machineProperties
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
                        uint32_t   channel,
                        IRQn_Type  irq);
        void     CommutationControl(void);
        float    getActualSpeed(void);

        /* member variables */
        uint32_t interruptCounter;
        float    speedRequest;
        int32_t            debug;
    /*--------------------------------------------------------------------*/    
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
        uint8_t   pwmSwitchingCU(uint8_t hallState);
        uint8_t   pwmSwitchingIU(uint8_t hallState);
        int16_t     firU(int16_t NewSample);
        int16_t     firV(int16_t NewSample);
        /* member variables */
        float               actualSpeed;
        int16_t             rotorPosition;
        int16_t             phiElec;
        int16_t             phiElecOld;
        uint16_t            pwmPeriod; 
        int8_t              commutationTable[8][3];
        machineProperties   motorProperties;
        uint8_t             previousHallState;
        uint16_t            deadTime;
        int16_t             currentFbk;
        int16_t             currentRef;
        int16_t             Kprp;
        uint16_t            Kint;
        int16_t             iInt;
        int16_t             test;
        int32_t             IfbkFilt;
        int16_t             Iu_old;
        int16_t             Iv_old;
        int16_t             Iw_old;
};

#endif /* BLDC_CONTROL_H */