#include "F28x_Project.h"                               // Contains basic device definitions
#include "F2837xD_Examples.h"                           // Changed from F2837xS_Examples.h to F2837xD_Examples.h
#include <stdint.h>
#include <math.h>
#include <Functions_and_Variables.h>
//#define LED_GPIO 4                                    // LED on GPIO4


//-------- Defining static variables --------//
//uint32_t SCALINGFACTOR = 100000;                        // For preventing floating numbers, scaling all the values for calculation.

static float error_v_int_bi;
static float error_i_int_bi;
static float I_ref_old_bi;
static float windup_bi;
static int   count_bi;
static int   first_time_entry_bi;


//-----    mppt function    -----//

void mppt(void)
{
    if(enable_mppt == 1)
    {

    uint16_t adc_Vpv = AdccResultRegs.ADCRESULT0;
    uint16_t adc_Ipv = AdcaResultRegs.ADCRESULT0;
    uint16_t adc_IL  = AdcbResultRegs.ADCRESULT0;

    // Vpvsum += (((float)adc_Vpv) / 4095.0f)*300.0f;  // For RTDS;
    Vpvsum += adc_val_Vpv(adc_Vpv);

    Ipvsum += (((float)adc_Ipv - 2047.0f) / 4095.0f)*60.0f;

    IL_nmin2 = IL_nmin1;
    IL_nmin1 = IL_continuous;

    IL_continuous = (((float)adc_IL  - 2047.0f)/ 4095.0f)*60.0f;

    ILsum +=  IL_continuous;

    if(count >= count_max-1){ // 200 vlaues : 0 to 199;
        count = 0;
    Vpv = Vpvsum/(float)count_max;
    Ipv = Ipvsum/(float)count_max;
    IL  = ILsum/(float)count_max;

    Vpvsum = 0;
    Ipvsum = 0;
    ILsum = 0;

    }

    const float Vrefmin = 0.5f;
    const float dVref =0.1f;


    if(first_time_entry == 0)
    {
        first_time_entry = 1;

        // Vpv = (((float)adc_Vpv) / 4095.0f)*300.0f;       // For RTDS;
        Vpv = adc_val_Vpv(adc_Vpv);                         // For Hardware;

        Ipv = (((float)adc_Ipv - 2047.0f) / 4095.0f)*60.0f;

        IL = (((float)adc_IL  - 2047.0f)/ 4095.0f)*60.0f;

    Pold = Vpv*Ipv;
    Vold = Vpv;
    Vrefold = 69.0f;
    }

    if(count == 0){ // running main mppt code with averaged values once in 50 cycles;
                    // Now changes in Vref will be made once in 50 cycles;
    dV=(Vpv-Vold);
    P= Vpv * Ipv;

    dP= (P-Pold);

    if(fabsf(dP) > P_deadband){ // since dP is float so its rare for it to become exactly 0, so used a deadband for comparison;
        if(dP>0){
            if(dV>0){
                Vref=Vrefold + dVref;
            }
            else{
                Vref=Vrefold - dVref;
            }
        }
        else{
            if(dV>0){
                Vref=Vrefold - dVref;
            }
            else{
                Vref=Vrefold + dVref;
            }
        }
    }
    else{
        Vref = Vrefold;
    }

    if(Vref>= Vrefmax){
        Vref= Vrefmax;
    }
    else if(Vref <= Vrefmin)
    {
        Vref = Vrefmin;
    }

    Vrefold=Vref;
    Vold=Vpv;
    Pold=P;
    }
    }
}

//-----    pi control function    -----//

void pi_control(void) // running every pwm cycle;
{
    // All values recieved are scaled by SCALINGFACTOR

    if(enable_mppt == 1)
    {
            error_iL = ((Vpv*Ipv)/Vref) + windup - ((IL_continuous*0.8f)+(IL_nmin1*0.15f)+(IL_nmin2*0.05f));       // error_i = ILref - IL_continuous_weighted_avg;
            error_iL_int += (error_iL/10000.0f);                   // error_iL_int += (error_iL*Ts); Ts = 1e-4;
            if (error_iL_int > I_INT_MAX){ // set the max to 0.7 and min to -0.7 in global variables;
            error_iL_int = I_INT_MAX;}
            else if (error_iL_int < I_INT_MIN){
            error_iL_int = I_INT_MIN;}

            temp = (kp * error_iL) + (ki * error_iL_int); // For safety if temp becomes larger than 32 bit anyhow;

            if(temp > 0.95f)
            {
                duty = 0.95f;
            }
            else if(temp < 0.1){
            duty = 0.1f ;
            }
            else{
                duty = temp;
            }
    // final_duty = duty * 9999 / 100000;

    if(fabs(P) <= 6.0f)
    {
        duty = 0.5;
    }

    windup = duty - temp;

    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)(duty*9999);                     // Updating value in CMPA shadow register, this value will be used in next counting cycle;
    }
    }

//----- bi-directional function -----//

void bidirectional(void)
{
    float error_v, error_i;
    float I_ref;

    // Step 1: Read ADC values
    uint16_t adc_Vo= AdcaResultRegs.ADCRESULT1;
    uint16_t adc_IL= AdcbResultRegs.ADCRESULT1;

    // Vo = (((float)adc_Vo)/4095.0f)*300.0f;           // For RTDS;
    Vo = adc_val_Vo(adc_Vo);                            // For Hardware;

    IL_bi = (((float)adc_IL-2047.0f)/4095.0f)*60.0f;


    // Step 2: First time initialization
    if(first_time_entry_bi == 0)
    {
        first_time_entry_bi = 1;
        error_v = V_ref - Vo;
        error_v_int_bi += error_v * TS;

        if(error_v_int_bi >= 3.0f)
        {
            error_v_int_bi = 3.0f;
        }
        else if(error_v_int_bi <= -3.0f)
        {
            error_v_int_bi = -3.0f;
        }

        I_ref = (KP_V * error_v) + (KI_V * error_v_int_bi);

        // limiting Iref for the first time to avoid current surges.
   //        if(I_ref > 20.0f)  I_ref = 20.0f;
          //        if(I_ref < -20.0f) I_ref = -20.0f;

        I_ref_old_bi = I_ref;
    }

    // Step 3: Outer Voltage Control Loop (slower)
    if(count_bi >= 50)   // Run every 50 cycles (~200 Hz if Ts=100us)
    {
        error_v = V_ref - Vo;
        error_v_int_bi += error_v * (TS * count_bi);
        if(error_v_int_bi >= 3.0f)
                {
                    error_v_int_bi = 3.0f;
                }
                else if(error_v_int_bi <= -3.0f)
                {
                    error_v_int_bi = -3.0f;
                }
        I_ref = (KP_V * error_v) + (KI_V * error_v_int_bi);

//        if(I_ref > 20.0f)  I_ref = 20.0f;
//        if(I_ref < -20.0f) I_ref = -20.0f;

        I_ref_old_bi = I_ref;
        count_bi = 0;
    }
    else
    {
        I_ref = I_ref_old_bi;
    }

    // Step 4: Inner Current Control Loop
    error_i = I_ref - IL_bi + windup_bi;
    error_i_int_bi += error_i * TS;

    if (error_i_int_bi > I_INT_MAX){ // set the max to 0.7 and min to -0.7 in global variables;
               error_i_int_bi = I_INT_MAX;}
               else if (error_i_int_bi < I_INT_MIN){
               error_i_int_bi = I_INT_MIN;}

    duty_cycle_bi = (KP_I * error_i) + (KI_I * error_i_int_bi);

    // Step 5: Clamp duty_bi cycle
    duty_bi = duty_cycle_bi;
    if(duty_bi < 0.05f) duty_bi = 0.05f;
    if(duty_bi > 0.95f) duty_bi = 0.95f;

    // Anti-windup_bi correction
    windup_bi = duty_bi - duty_cycle_bi;


    EPwm3Regs.CMPA.bit.CMPA = (uint16_t)(duty_bi * 9999);   // Update duty_bi for EPWM3A

    // Step 7: Increment count_bier
    count_bi++;
    mppt_count++;
    if(mppt_count == 30000 && enable_mppt == 0){
        enable_mppt = 1;        // will enable mppt after 3 seconds;
    }

}

//-----    ADCA EOC0 ISR function    -----//

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

    mppt();                                             // Calling mppt function to give Vref; Also calculating values of Vpv,Ipv,IL in this only;

    pi_control();                                       // Calling pi_control function to set req. Duty for pwm pulse;

    bidirectional();

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;             // Acknowledge PIE group 10;

    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;               //Clearing test gpio pin after eoc;
    count++;
}

//-----    main function    -----//

void main(void)
{
    InitSysCtrl();                                      // Must be called to initialize PLL & Timers, this sets 200MHz freq.
    InitPieCtrl();
    InitPieVectTable();

    // SEQUENCE OF CALLING THESE FUNCTIONS MATTERS A LOT, start adc first then call epwminit, otherwise epwm will run firstly, trigger soc
    // But as adcs are not started yet, the code will get stuck in the while loop, as adc b is not started and we entered the while loop.
    // We will be stucked there only as cpu will not be able to move further in adcinit() function after delay as it got stucked in while loop.
    test_pin_setup();
    adcA_init();
    adcB_init();
    adcC_init();
    volatile uint32_t i;                                // Simple delay for proper turning on of adcs;
    for(i = 0; i < 20000; i++) { } // need to increase this delay for proper adc working, so setting it to about 1ms;

    epwm2_init();
    epwm3_init();     // Replaces epwm4 + epwm5

    setup_interrupts();

    while(1)
    {

        }
}
