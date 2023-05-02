#include <math.h>
#include "F28x_Project.h"
#include "control/include/initial_cpu.h"
#include "control/include/main_control.h"
#include "control/include/transforms.h"
#include "SFO_V8.h"

float m;       //Modulation ratio
float theta;
#define PIE 3.1415926535897932384626433832795
#define M 0.83 //Modulation ratio
const float PIEx2 = 6.283185307179586476925286766559;
const float PIEx100 = 314.15926535897932384626433832795;


void mainDataInit(void)
{
    theta = 0;

}

__interrupt void MainControlISR(void)
{
    float volume;
    Uint32 temp,temp1;
    float adcresult;
    /*
     * theta=wt , w=2pie f
     */
    theta = theta + 0.00005*PIEx100; //0.00005:interrupt time 20K
    theta = theta > PIEx2 ? (theta - PIEx2) : theta;
    theta = theta < 0 ? (theta + PIEx2) : theta;

    //temp = m * sin(theta);
    volume = ((1.0 +sin(theta))*0.5);

    /*
     * test HRPWM proportion
     */
//    volume = (1.0-0.473);
//    UpdateFine = 0;       //UpdateFine=1:HRPWM   ;   UpdateFine=0:PWM

    /*
     * protect duty for 5%-95%
     */
    volume = volume>0.03 ? volume : 0.03;
    volume = volume<0.97 ? volume : 0.97;
    DutyFine = volume*32768.0;


    if(UpdateFine==1&&volume>=0.015)    //HRPWM cannot be used for the first 3 cycles   3/200
                                       // 3/EPwm1Regs.TBPRD
    {
        CMPA_reg_val = ((long)DutyFine * (EPwm1Regs.TBPRD)) >> 15;
        temp = ((long)DutyFine * (EPwm1Regs.TBPRD )) ;
        temp = temp - ((long)CMPA_reg_val << 15);

        CMPB_reg_val = ((long)DutyFine * (EPwm1Regs.TBPRD)) >> 15;
        temp1 = ((long)DutyFine * (EPwm1Regs.TBPRD )) ;
        temp1 = temp1 - ((long)CMPB_reg_val << 15);

       #if(AUTOCONVERT)
        CMPAHR_reg_val = temp << 1; // convert to Q16
        CMPAHR_reg_val = CMPAHR_reg_val*2;

        CMPBHR_reg_val = temp1 << 1; // convert to Q16
        CMPBHR_reg_val = CMPBHR_reg_val*2;
       #else
        CMPAHR_reg_val = ((temp * MEP_ScaleFactor) + (0x0080 << 7)) >> 15;
        CMPAHR_reg_val = CMPAHR_reg_val << 8;
        CMPAHR_reg_val = CMPAHR_reg_val*2;

        CMPBHR_reg_val = ((temp1 * MEP_ScaleFactor) + (0x0080 << 7)) >> 15;
        CMPBHR_reg_val = CMPBHR_reg_val << 8;
        CMPBHR_reg_val = CMPBHR_reg_val*2;
       #endif

        /*
         * protect for duty >
         *
         */
       //
       // Example for a 32 bit write to CMPA:CMPAHR
       //
        EPwm1Regs.CMPA.all = ((long)CMPA_reg_val) << 16 |
                              CMPAHR_reg_val; // loses lower 8-bits
        EPwm2Regs.CMPA.all = ((long)CMPA_reg_val) << 16 |
                              CMPAHR_reg_val; // loses lower 8-bits

        EPwm1Regs.CMPB.all = ((long)CMPB_reg_val) << 16 |
                              CMPBHR_reg_val; // loses lower 8-bits
        EPwm2Regs.CMPB.all = ((long)CMPB_reg_val) << 16 |
                              CMPBHR_reg_val; // loses lower 8-bits

    }
    else
    {
        //
        // CMPA_reg_val is calculated as a Q0.
        // Since DutyFine is a Q15 number, and the period is Q0
        // the product is Q15. So to store as a Q0, we shift right
        // 15 bits.
        //
            EPwm1Regs.CMPA.bit.CMPA = (((long)DutyFine *
                                        (EPwm1Regs.TBPRD)) >> 15);
            EPwm2Regs.CMPA.bit.CMPA = (((long)DutyFine *
                                        (EPwm2Regs.TBPRD)) >> 15);

            EPwm1Regs.CMPB.bit.CMPB = (((long)DutyFine *
                                        (EPwm1Regs.TBPRD)) >> 15);
            EPwm2Regs.CMPB.bit.CMPB = (((long)DutyFine *
                                        (EPwm2Regs.TBPRD)) >> 15);

    }
    //
    // Call the scale factor optimizer lib function SFO()
    // periodically to track for any change due to temp/voltage.
    // This function generates MEP_ScaleFactor by running the
    // MEP calibration module in the HRPWM logic. This scale
    // factor can be used for all HRPWM channels. The SFO()
    // function also updates the HRMSTEP register with the
    // scale factor value.
    //
   status = SFO(); // in background, MEP calibration module
                    // continuously updates MEP_ScaleFactor

    if(status == SFO_ERROR)
    {
        error();   // SFO function returns 2 if an error occurs & #
                   // of MEP steps/coarse step
    }

//    EPwm1Regs.CMPA.bit.CMPA = 0.5 * EPWM_TBPRD_100K_100M* (1.0 +sin(theta));//Ts = 100K
//    EPwm2Regs.CMPA.bit.CMPA = 0.5 * EPWM_TBPRD_100K_100M* (1.0 +sin(theta));//

    //
    // Clear INT flag for this timer
    //
    EPwm12Regs.ETCLR.bit.INT = 1;
    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}








