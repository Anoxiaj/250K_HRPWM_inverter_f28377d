//
// Included Files
//
#include "F28x_Project.h"
#include "control/include/initial_cpu.h"
#include "control/include/main_control.h"
#include "control/include/common_function.h"


//
// Main
//
void main(void)
{
    mainDataInit();
    PWM_Disable();

    InitCPU();

    for(;;)
          {
//        GpioDataRegs.GPDDAT.bit.GPIO111 = 1;
              asm ("    NOP");
          }

}

//
// End of file
//
