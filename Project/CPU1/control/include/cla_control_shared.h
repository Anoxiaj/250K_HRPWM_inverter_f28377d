#ifndef CLA_CONTROL_SHARED_H
#define CLA_CONTROL_SHARED_H
//*****************************************************************************
// includes
//*****************************************************************************
//#include "F2837xD_Cla_defines.h"
//#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// defines
//*****************************************************************************

//*****************************************************************************
// typedefs
//*****************************************************************************


//*****************************************************************************
// globals
//*****************************************************************************
//Task 1 (C) Variables

//extern volatile float pll_theta;




//Task 8 (C) Variables

//Common (C) Variables
//extern float CLAasinTable[]; //The arcsine lookup table

//*****************************************************************************
// function prototypes
//*****************************************************************************
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.

//CLA C Tasks
__interrupt void Cla1Task1();
__interrupt void Cla1Task8();


#ifdef __cplusplus
}
#endif // extern "C"

#endif //end of CLA_INV_CURRENT_CONTROL_SHARED_H definition
