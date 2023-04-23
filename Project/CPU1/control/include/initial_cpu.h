#ifndef CONTROL_INCLUDE_INITIAL_CPU_H_
#define CONTROL_INCLUDE_INITIAL_CPU_H_



#ifdef __cplusplus
extern "C" {
#endif

//
// Globals Variables
//

//EPWM Macro Definition(f28377d: max epwm frequency:100M
//                               min-max HRpwm :60-100M )

#define EPWM_TBPRD_1M_100M      50    //for switching frequency 1MHz
#define EPWM_TBPRD_250K_100M    200   //for switching frequency 250KHz
#define EPWM_TBPRD_200K_100M    250   //for switching frequency 200kHz  100M/(250¼ÆÊý*2)= switching frequency
#define EPWM_TBPRD_100K_100M    500   //for switching frequency 100kHz
#define EPWM_TBPRD_50K_100M     1000   //for switching frequency 50kHz
#define EPWM_TBPRD_20K_100M     2500   //for switching frequency 50kHz

#define EPWM_MIN_CMPA       1
#define EPWM_MAX_CMPA       49
#define EPWM_DB_RED         8    //TBCLK=EPWMCLK=100M :RED=1-->10ns
#define EPWM_DB_FED         8

//
// Globals
//
extern Uint16 UpdateFine;
extern Uint16 DutyFine;
extern Uint16 status;
extern Uint16 CMPA_reg_val;
extern Uint16 CMPAHR_reg_val;
extern Uint16 CMPB_reg_val;
extern Uint16 CMPBHR_reg_val;

#define PWM_CH            9        // # of PWM channels

extern int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO() function.


void InitCPU(void);
void InitGPIO(void);
void InitADC(void);
void InitEPWM(void);
void error(void);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
