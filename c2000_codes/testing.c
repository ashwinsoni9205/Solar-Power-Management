#include "F28x_Project.h"                               // Contains basic device definitions
#include "F2837xD_Examples.h"                           // Changed from F2837xS_Examples.h to F2837xD_Examples.h
#include <stdint.h>
#include <math.h>
#include "Functions_and_Variables.h"

float voltage = 0.0f;

__interrupt void adca2_isr(void)                        // main calculations is done in this ISR;
{
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              // Clear ADCINT1 flag for adc A;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear ADCINT2 flag for ADCA (EOC1)

    while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0);         // wait till ADC_B_INT1 interrupt is not triggered, this is triggered
                                                        // by our EOC0 event, i.e. when adc B has done its conversion;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              // Clear ADCINT1 flag for adc B;

    while(AdcbRegs.ADCINTFLG.bit.ADCINT2 == 0);         // Wait until_bi ADCB EOC1 is complete

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear ADCINT2 flag for ADCB (EOC1)


    while(AdccRegs.ADCINTFLG.bit.ADCINT1 == 0);         // wait till ADC_C_INT1 interrupt is not triggered, this is triggered
                                                        // by our EOC0 event, i.e. when adc C has done its conversion;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              // Clear ADCINT1 flag for adc C;


    voltage = adc_val_Vpv(AdccResultRegs.ADCRESULT0);

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;             // Acknowledge PIE group 10;

    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;               //Clearing test gpio pin after eoc;
}


void main()
{

    InitSysCtrl();                                      // Must be called to initialize PLL & Timers, this sets 200MHz freq.
    InitPieCtrl();
    InitPieVectTable();

    adcA_init();
    adcB_init();
    adcC_init();
    volatile uint32_t i;                                // Simple delay for proper turning on of adcs;
    for(i = 0; i < 20000; i++) { } // need to increase this delay for proper adc working, so setting it to about 1ms;

    epwm2_init();

    setup_interrupts();

    while(1)
    {

    }
}
