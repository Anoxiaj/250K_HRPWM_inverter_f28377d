


#include "F28x_Project.h"
#include "control/include/initial_cpu.h"
#include "control/include/main_control.h"
#include "control/include/cla_control_shared.h"
#include "SFO_V8.h"

#define STATUS_SUCCESS    1
#define STATUS_FAIL       0

#define AUTOCONVERT       0       // 1 = Turn auto-conversion ON
                                  // 0 = Turn auto-conversion OFF


//
// Globals
//
Uint16 UpdateFine;
Uint16 DutyFine;
Uint16 status;
Uint16 CMPA_reg_val;
Uint16 CMPAHR_reg_val;
Uint16 CMPB_reg_val;
Uint16 CMPBHR_reg_val;

volatile struct EPWM_REGS *ePWM[PWM_CH] =
{  &EPwm1Regs, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs,
   &EPwm6Regs, &EPwm7Regs, &EPwm8Regs};

int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO() function.

void InitCPU(void)
{
    //
    // Step 1. Initialize System Control:
    //
    InitSysCtrl();
    //
    // Step 2. Initialize GPIO:
    //
    InitGpio();
    //
    // enable PWM1, PWM2;
    //
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM12=1;
    //
    // For this case just init GPIO pins for ePWM1, ePWM2
    // These functions are in the f2838x_epwm.c file
    //
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm12Gpio();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;
    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the f2838x_piectrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in f2838x_defaultisr.c.
    // This function is found in f2838x_pievect.c.
    //
    InitPieVectTable();

    UpdateFine = 1;
    DutyFine = 0;
    status = SFO_INCOMPLETE;

//       //
    // Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
    // HRMSTEP must be populated with a scale factor value prior to enabling
    // high resolution period control.
    //
      while(status == SFO_INCOMPLETE)
       {
            status = SFO();
            if(status == SFO_ERROR)
            {
                error();   // SFO function returns 2 if an error occurs & # of MEP
            }              // steps/coarse step exceeds maximum of 255.
       }

//    // Step 5. User specific code, enable interrupts:
    //

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW; // This is needed to write to EALLOW protected registers
    //PieVectTable.EPWM1_INT = &main_control_isr;
    PieVectTable.EPWM12_INT = &MainControlISR; //function for ADCA interrupt 1
    EDIS;   // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Configure the CLA memory spaces first followed by
    // the CLA task vectors
    //
InitCLA();

    //
    // Step 5. Initialize the Device Peripheral.
    //

    // Step 5.1 Initialize GPIO
    InitGPIO();


    // Step 5.2 Initialize the ADC Device Peripheral.
    InitADC();

    // Step 5.5 Initialize the EPWM Device Peripheral.
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPWM();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //
    // Step 6. User specific code, enable interrupts:
    //

    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    //
    IER |= M_INT3;

    //
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1
    //
    PieCtrlRegs.PIEIER3.bit.INTx12 = 1;


    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


}

void InitGPIO(void)
{
    EALLOW;
    GpioCtrlRegs.GPDPUD.bit.GPIO111 = 0;   // Enable pullup on GPIO111
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;   // Load output latch
    GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;  // GPIO111 = GPIO
    GpioCtrlRegs.GPDDIR.bit.GPIO111 = 1;   // GPIO111 = output
    GpioCtrlRegs.GPDCSEL2.bit.GPIO111 = 1;  //select CPU1.CLA to control GPIO
    EDIS;

}

void InitCLA(void)
{
    EALLOW;                     // Enable EALLOW protected register access

    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    #ifdef _FLASH
    //Copy over code from FLASH to RAM
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
            (uint32_t)&Cla1funcsLoadSize);
    #endif //_FLASH
    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    //--- Memory Configuration - Master CPU and CLA Select
    //configure LS0RAM and LS1RAM as data spaces for the CLA
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;
    //Select LS4RAM and LS5RAM to be the programming space for the CLA
    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;

    //--- Initialize CLA task interrupt vectors
    //    On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    //    opposed to offsets used on older Type-0 CLAs.
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);
    //--- Select Task interrupt source
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = CLA_TRIG_ADCAINT1;

    //--- CLA1TASKSRCSELx register lock control
    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL1 = 6;     // Write a 1 to lock (cannot be cleared once set)
    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL2 = 0;     // Write a 1 to lock (cannot be cleared once set)

    //--- Enable use software to start a task (IACK)
    Cla1Regs.MCTL.bit.IACKE = 1;        // Enable IACKE to start task using software
   //--- Force one-time initialization Task 8
    Cla1Regs.MIER.all = 0x0080;            // Enable CLA interrupt 8
    asm("  IACK  #0x0080");                // IACK - CLA task force instruction
    asm("  RPT #3 || NOP");                // Wait at least 4 cycles
    while(Cla1Regs.MIRUN.bit.INT8 == 1);   // Loop until task completes
    //--- Enable CLA task interrupts
    Cla1Regs.MIER.all = 0x0001;        // Enable CLA interrupt 1

    EDIS;                      // Disable EALLOW protected register access
}

void InitADC(void)
{
    EALLOW;
    //allocate ADC A-D to CPU1:ADC Result connect with CPU1
    DevCfgRegs.CPUSEL11.bit.ADC_A = 0;  //0:CPU1  ;  1:CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_B = 0;
    DevCfgRegs.CPUSEL11.bit.ADC_C = 0;
    DevCfgRegs.CPUSEL11.bit.ADC_D = 0;

    //Clear All ADC data and must be manually cleared after being set.
    DevCfgRegs.SOFTPRES13.bit.ADC_A = 1;    // ADCA is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_A = 0;    // ADCA is released from reset
    DevCfgRegs.SOFTPRES13.bit.ADC_B = 1;    // ADCB is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_B = 0;    // ADCB is released from reset
    DevCfgRegs.SOFTPRES13.bit.ADC_C = 1;    // ADCC is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_C = 0;    // ADCC is released from reset
    DevCfgRegs.SOFTPRES13.bit.ADC_D = 1;    // ADCD is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_D = 0;    // ADCD is released from reset
    EDIS;

    /*
     ************ADC configuration************
     */
    EALLOW;
    /**********Main ADC Configuration**********/

    AdcaRegs.ADCCTL1.all = 0x0004;    //3-0:0100
    AdcbRegs.ADCCTL1.all = 0x0004;    //INTPULSEPOS, ADC Interrupt Pulse Position.
    AdccRegs.ADCCTL1.all = 0x0004;    // 0=start of conversion
    AdcdRegs.ADCCTL1.all = 0x0004;    // 1=end of conversion
    // bit 15-14     00:     reserved
    // bit 13        0:      ADCBSY, ADC busy, read-only
    // bit 12        0:      reserved
    // bit 11-8      0's:    ADCBSYCHN, ADC busy channel, read-only
    // bit 7         0:      ADCPWDNZ, ADC power down, 0=powered down, 1=powered up
    // bit 6-3       0000:   reserved
    // bit 2         1:      INTPULSEPOS, INT pulse generation, 0=start of conversion, 1=end of conversion
    // bit 1-0       00:     reserved

    /**********ADC Clock Configuration**********/
    AdcaRegs.ADCCTL2.all = 0x0006;      //3-0：0110
    AdcbRegs.ADCCTL2.all = 0x0006;      //0110=CPUCLK/4
    AdccRegs.ADCCTL2.all = 0x0006;      //ADCCLK : 5M--50M
    AdcdRegs.ADCCTL2.all = 0x0006;
    // bit 15-8      0's:    reserved
    // bit 7         0:      SIGNALMODE, configured by AdcSetMode() below to get calibration correct
    // bit 6         0:      RESOLUTION, configured by AdcSetMode() below to get calibration correct
    // bit 5-4       00:     reserved
    // bit 3-0       0110:   PRESCALE, ADC clock prescaler.  0110=CPUCLK/4


    /**********ADC Burst(突发) Control Register**********/
//    AdcaRegs.ADCBURSTCTL.all = 0x0000;  //0=burst mode disabled
//    AdcbRegs.ADCBURSTCTL.all = 0x0000;
//    AdccRegs.ADCBURSTCTL.all = 0x0000;
//    AdcdRegs.ADCBURSTCTL.all = 0x0000;
    // bit 15        0:      BURSTEN, 0=burst mode disabled, 1=burst mode enabled
    // bit 14-12     000:    reserved
    // bit 11-8      0000:   BURSTSIZE, 0=1 SOC converted (don't care)
    // bit 7-6       00:     reserved
    // bit 5-0       000000: BURSTTRIGSEL, 00=software only (don't care)


    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //epwm1 trigger
    //ADCa Current sampling
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;      //SOC0 will convert pin ADCA0, ADCA0->PIN1->Vdc
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0;      //SOC1 will convert pin ADCA0
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 1;      //SOC2 will convert pin ADCA1, ADCA1->PIN3->Vdc
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 1;      //SOC3 will convert pin ADCA1
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 2;      //SOC4 will convert pin ADCA2, ADCA2->PIN5->IL
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 2;      //SOC5 will convert pin ADCA2
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC6CTL.bit.CHSEL = 3;      //SOC6 will convert pin ADCA3, ADCA3->PIN7->IL
    AdcaRegs.ADCSOC6CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOC7CTL.bit.TRIGSEL =  5;  // Trigger using ePWM1-ADCSOCA
    AdcaRegs.ADCSOC7CTL.bit.CHSEL = 3;      //SOC7 will convert pin ADCA3
    AdcaRegs.ADCSOC7CTL.bit.ACQPS = 14;  //sample window is (acqps+1)*SYSCLK cycles 75ns    SYSCLK=5ns

    AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 0;  // All SOCs handled in round-robin mode

    //--- ADCA's EOC3 will interrupt ADCINTA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;      // EOC3 triggers the interrupt, depends on the number of ADCSOC
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable the interrupt in the ADC
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;     // Interrupt pulses regardless of flag state
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared

    //
    // Power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    // Delay for 10ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;


}
void InitEPWM(void)
{

//    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; //SYS CLOCK 0:200M 1:100M

    //
    // Setup TBCLK
    //
    //
    EPwm1Regs.TBPRD = EPWM_TBPRD_250K_100M;       // Set timer period 801 TBCLKs
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter

    EPwm2Regs.TBPRD = EPWM_TBPRD_250K_100M;       // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter

    EPwm12Regs.TBPRD =  EPWM_TBPRD_20K_100M;//40000     // Set timer period 1000 TBCLKs = 100K
    EPwm12Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm12Regs.TBCTR = 0x0000;
    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.bit.CMPA = EPWM_TBPRD_250K_100M/2;    // Set compare A value
    EPwm1Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm1Regs.CMPB.bit.CMPB = EPWM_TBPRD_250K_100M/2;    // Set compare A value
    EPwm1Regs.CMPB.bit.CMPBHR = (1 << 8);   // initialize HRPWM extension

    EPwm2Regs.CMPA.bit.CMPA = EPWM_TBPRD_250K_100M/2;    // Set compare A value
    EPwm2Regs.CMPA.bit.CMPAHR = (1 << 8);   // initialize HRPWM extension
    EPwm2Regs.CMPB.bit.CMPB = EPWM_TBPRD_250K_100M/2;    // Set compare A value
    EPwm2Regs.CMPB.bit.CMPBHR = (1 << 8);   // initialize HRPWM extension

    EPwm12Regs.CMPA.bit.CMPA = EPWM_TBPRD_20K_100M/2;    // Set compare A value
    //
    // Setup counter mode
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm12Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm12Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm12Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm12Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 11;
    //
    // Setup shadowing
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm12Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm12Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm12Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; // Load on Zero
    EPwm12Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1A on event A, down count

    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A, down count
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1A on event A, up count
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1A on event A, down count

    //
    // Setup HRPWM (HRPWM Configuration Register)
    //
    EALLOW;
    (*ePWM[1]).HRCNFG.all = 0x0;
    (*ePWM[2]).HRCNFG.all = 0x0;

    (*ePWM[1]).HRCNFG.bit.EDGMODE = HR_REP;  // MEP control on both edge
    (*ePWM[1]).HRCNFG.bit.EDGMODEB = HR_FEP;  // MEP control on both edge
    (*ePWM[2]).HRCNFG.bit.EDGMODE = HR_REP;  // MEP control on both edge
    (*ePWM[2]).HRCNFG.bit.EDGMODEB = HR_FEP;  // MEP control on both edge

    (*ePWM[1]).HRCNFG.bit.CTLMODE = HR_CMP;
    (*ePWM[1]).HRCNFG.bit.CTLMODEB = HR_CMP;
    (*ePWM[2]).HRCNFG.bit.CTLMODE = HR_CMP;  //Selects the register (CMP/TBPRD) that controls the MEP
    (*ePWM[2]).HRCNFG.bit.CTLMODEB = HR_CMP;

    (*ePWM[1]).HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD;
    (*ePWM[1]).HRCNFG.bit.HRLOADB = HR_CTR_ZERO_PRD;
    (*ePWM[2]).HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD;
    (*ePWM[2]).HRCNFG.bit.HRLOADB = HR_CTR_ZERO_PRD;


    #if(AUTOCONVERT)
    (*ePWM[1]).HRCNFG.bit.AUTOCONV = 1;      // Enable auto-conversion logic
    (*ePWM[2]).HRCNFG.bit.AUTOCONV = 1;      // Enable auto-conversion logic

    #endif

    (*ePWM[1]).HRPCTL.bit.HRPE = 0; // Turn off high-resolution period control.
    (*ePWM[2]).HRPCTL.bit.HRPE = 0; // Turn off high-resolution period control.



    EDIS;

    //
    // Setup Deadband - Active high complementary PWMs
    //
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBRED.bit.DBRED = EPWM_DB_RED;
    EPwm1Regs.DBFED.bit.DBFED = EPWM_DB_FED;
//
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBRED.bit.DBRED = EPWM_DB_RED;
    EPwm2Regs.DBFED.bit.DBFED = EPWM_DB_FED;
//    EPwm2Regs.DBCTL.bit.OUTSWAP = 3;     //why this need to flip???   for delay?
    /***************************************************************************/
    //
    // Setup PWM chopper
    //
    EPwm1Regs.PCCTL.bit.CHPEN = CHP_DISABLE;      // PWM chopper unit disabled
    EPwm2Regs.PCCTL.bit.CHPEN = CHP_DISABLE;
    //
    // Setup PWM Trip Zone（故障捕获子模块）
    //
    EALLOW;

/*        select trip zone event!!!!
 * EPwm1Regs.TZSEL.bit.OSHT1 =  *or*  EPwm1Regs.TZSEL.bit.CBC1 =
 * EPwm2Regs.TZSEL.bit.OSHT1 =  *or*  EPwm2Regs.TZSEL.bit.CBC1 =
 */

    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;            // Forced Hi (EPWM1A = High state)
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;            // Forced Hi (EPWM1B = High state)
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;            // Forced Hi (EPWM1A = High state)
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;            // Forced Hi (EPWM1B = High state)
    EDIS;
    //
    //EPWM12 trigger INTERRUPT
    //
    EPwm12Regs.ETSEL.bit.INTEN = 1;
    EPwm12Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;   // Select TBCTR = 0x00
    EPwm12Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate pulse on 1st event
    //
    //EPWM12 trigger SOCA
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;   // Select SOC on Zero
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;       // Generate pulse on 1st event
    EPwm1Regs.ETCLR.bit.INT = 1;

}

//
// error - Halt debugger when called
//
void error (void)
{
    ESTOP0;         // Stop here and handle error
}

//
// End of file
//

