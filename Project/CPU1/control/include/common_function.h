#ifndef CONTROL_INCLUDE_COMMON_FUNCTION_H_
#define CONTROL_INCLUDE_COMMON_FUNCTION_H_

#include "F2837xD_device.h"

#ifdef __cplusplus
extern "C" {
#endif


//-------------------------------------------------//
// 3. PWM control funtions                         //
//-------------------------------------------------//
//pwm enable and disable functions
inline void PWM_Disable(void)
{
     EALLOW;
     EPwm1Regs.TZFRC.bit.OST=1;
     EPwm2Regs.TZFRC.bit.OST=1;
//     EPwm3Regs.TZFRC.bit.OST=1;
     EDIS;
 }
inline void PWM_Enable(void)
{
     EALLOW;
     EPwm1Regs.TZCLR.bit.OST=1;
     EPwm2Regs.TZCLR.bit.OST=1;
//     EPwm3Regs.TZCLR.bit.OST=1;
     EDIS;
}


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
