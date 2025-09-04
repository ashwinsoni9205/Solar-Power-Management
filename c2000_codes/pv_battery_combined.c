#include "F28x_Project.h"                               // Contains basic device definitions
#include "F2837xD_Examples.h"                           // Changed from F2837xS_Examples.h to F2837xD_Examples.h
#include <stdint.h>
#include <math.h>
//#define LED_GPIO 4                                    // LED on GPIO4


//-------- Defining global variables --------//
//uint32_t SCALINGFACTOR = 100000;                        // For preventing floating numbers, scaling all the values for calculation.

uint16_t enable_mppt = 0;
uint16_t mppt_count = 0;

volatile float duty = 0.5f;
volatile float Ipv = 0.0f;
volatile float Vpv = 0.0f;                       // Vpv will be in 0.01V, i.e. 1 count = 0.01V, so 100 counts = 1V;
volatile float Vref = 69.0f;                       // initially Vref will be Vpv at mppt, i.e. 69;
volatile float IL = 0.0f;
float IL_continuous = 0.0f;
float IL_nmin1 = 0.0f; // IL[n-1];
float IL_nmin2 = 0.0f; // IL[n-2];
volatile float error_iL_int = 0.0f;                      // intergral current error, need to be preserved for accumulation overtime;
float error_iL=0.0f;
float Pold = 0.0f;
float Vold = 0.0f;
float Vrefold = 0.0f;
volatile uint16_t first_time_entry = 0;
float P=0.0f;
float dV=0.0f;
float dP=0.0f;
float temp=0.0f;
float Vrefmax = 200.0f;
float kp = 0.2f;
float ki = 20.0f;
float windup = 0.0f;
uint16_t count = 0;
uint16_t count_max = 200;
float Vpvsum = 0.0f;
float Ipvsum = 0.0f;
float ILsum = 0.0f;
float P_deadband = 6.0f;
float I_INT_MAX = 0.7f;
float I_INT_MIN = -0.7f;




volatile float Vo = 0.0f;
volatile float IL_bi = 0.0f;
volatile float duty_cycle_bi = 0.0f;
float duty_bi = 0.5f;

static float error_v_int_bi = 0.0f;
static float error_i_int_bi = 0.0f;
static float I_ref_old_bi   = 0.0f;
static float windup_bi      = 0.0f;
static int   count_bi       = 0;
static int   first_time_entry_bi = 0;

//============================
// Controller Constants
//============================
#define V_ref       200.0f      // Reference Voltage
#define KP_V        0.1f        // Voltage loop proportional gain
#define KI_V        8.0f        // Voltage loop integral gain
#define KP_I        0.02f       // Current loop proportional gain
#define KI_I        5.0f        // Current loop integral gain
#define TS          1e-4f       // Sampling time (10kHz)


//--------       Test pin set        --------//

__interrupt void epwm2_isr(void)                        // generating interrupt at start to set the test pin high for timing measurements;
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;                 // Set GPIO61 HIGH at TBCTR=0 (SOC event)
    EPwm2Regs.ETCLR.bit.INT = 1;                        // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;             // Acknowledge PIE group 3
}
void test_pin_setup()                                   // Setting a gpio pin for testing purposes to see time taken by certain function or block
{
    // using gpio 61 (J2 19);
    EALLOW;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;                // Set as GPIO function (not peripheral)
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;                 // Set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;               // Start LOW
    EPwm2Regs.ETSEL.bit.INTEN = 1;                      // EPWM2 INT enable
    EPwm2Regs.ETSEL.bit.INTSEL = 1;                     // INT on counter=zero
    EPwm2Regs.ETPS.bit.INTPRD = 1;                      // INT on first event
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;                  // Enable PIE group 3, INT2 (EPWM2)
    IER |= M_INT3;                                      // Enable group 3 interrupts
    // EINT;                                            // Global interrupt enable already done in interrupt setup function;
    EDIS;
}

//-----   Function to initialize epwm   -----//

void epwm2_init()
{
    // using EPWM2A for boost converter operation;
    EALLOW;

    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;                  // configuring gpio2(J4 38) as output;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;                 // selecting EPWM2A instead of GPIO2 on J4 38 pin;
    //Setting Epwm2 clk = 100MHz, so count_bi till 10,000(0 to 9999) for 10kHz clock
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;                    // upcount_bi mode on epwm2;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;                     // For clkdiv value of 1;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;                  // For hspclkdiv value of 1; epwm clock is now set to 100MHz with these two lines.
    EPwm2Regs.TBPRD = 9999;                             //Count_bier will count_bi till 9999;
    EPwm2Regs.CMPA.bit.CMPA = 4999;                     // Trial Compare value of 4999 for 50% duty_bi
    EPwm2Regs.AQCTLA.bit.ZRO = 2;                       //Set EPWM2A on reaching the 0 value;
    EPwm2Regs.AQCTLA.bit.CAU = 1;                       //Will force EPWM2A to clear to 0 on CMPA match;

    // Using shadow mode and then Transferring value from shadow reg to active CMPA reg when TBCTR = 0
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;                 // To enable shadow mode for CMPA.
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;                 // To Load in active reg (CMPA) when TBCTR becomes 0;
    // Now when we will write in CMPA bit of CMPA reg then value will go in shadow reg and go in actual reg when TBCTR = 0

    // generating epwm event for adc start of conversion
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;                     // Enable ADCSOCA pulse;
    EPwm2Regs.ETSEL.bit.SOCASEL = 1;                    // To trigger soc when time base count_bier TBCTR = 0; It will be better to do soc a bit away from 0
                                                        // to avoid effect of transients. WILL BE DOING IN FUTURE IF 0 ONE DONT WORK FINE.

    EPwm2Regs.ETPS.bit.SOCAPRD = 1;                     // Generate SOC pulse on 1st event;

    EPwm2Regs.TBCTL.bit.PHSEN = 0;              // Master, phase disabled
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 1; // Generate sync-out at CTR=0

    EDIS;
}

void epwm3_init()
{
    EALLOW;

    //
    // --- EPWM3A on GPIO4 ---
    //
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;     // GPIO4  EPWM3A

    //
    // --- EPWM3B on GPIO5 ---
    //
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;     // GPIO5  EPWM3B

    //
    // Time base setup
    //
    EPwm3Regs.TBCTL.bit.CTRMODE = 0;        // Up-count_bi
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm3Regs.TBPRD = 9999;                 // 10 kHz
    EPwm3Regs.CMPA.bit.CMPA = 4999;         // 50% duty_bi for A

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;                 // To enable shadow mode for CMPA.
    EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;                 // To Load in active reg (CMPA) when TBCTR becomes 0;

    //Epwm3B

    EPwm3Regs.AQCTLB.all = 0;          // B comes only from DB


    //Enabling these three with epwm2 only
    //EPwm3Regs.ETSEL.bit.SOCAEN = 1;                     // Enable ADCSOCA pulse for epwm3;
    //EPwm3Regs.ETSEL.bit.SOCASEL = 1;                    // pulse will be generated at ctr = 0;
    //EPwm3Regs.ETPS.bit.SOCAPRD  = 1;                    // SOC on 1st event

    // Dead-band: complementary outputs
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;  // Enable both RED and FED
    EPwm3Regs.DBCTL.bit.POLSEL   = 2;  // Active high complementary (B = ~A)
    EPwm3Regs.DBCTL.bit.IN_MODE  = 0;  // Use A as source

    // Set deadtime count_bis
    EPwm3Regs.DBRED.bit.DBRED = 50;    // e.g. 500 ns at 100 MHz
    EPwm3Regs.DBFED.bit.DBFED = 50;

    //
    // Action Qualifier
    //
    EPwm3Regs.AQCTLA.bit.ZRO = 2;           // Set A on Zero
    EPwm3Regs.AQCTLA.bit.CAU = 1;           // Clear A on CMPA


    // For synchronization between pwms
    EPwm3Regs.TBCTL.bit.PHSEN = 1;              // Enable phase loading
    EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;           // Sync from ePWM2
                                                // epwms in c2000 are chained together in increasing order so epwm2 -> epwm3
                                                // so epwm 3 will be synced as per the sync signal of epwm2;
    EPwm3Regs.TBPHS.bit.TBPHS = 0;              // No phase shift (aligned); 0 value will be loaded in CMP of TBCTR when sync signal
                                                // is recieved from epwm2;

    EDIS;
}


//-----    ADC initialization functions    -----//

void adcA_init() // function to initialize adcA ports;
{
    // ADCA for ipv measurement
    // Using SOC0 event of ADC A for ipv measurement

    EALLOW;

    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;                // for single-ended adc mode;
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;                // selecting 12-bit resolution for adc;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 14;                 // Adc_clk = SYSCLK/8 = 200MHz/8 = 25MHz;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;                  //Powering up the adc;

    // We have multiple SOCs like SOC0,SOC1,... can use any of them with epwms; Using SOC0 for now
    // Will be triggering all adc soc events by epwm2's ADCSOCA pulse
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7;                // Selecting the triggering source for SOC0 of ADC A, which is epwm2,ADCSOCA as set up above;

    //Selecting channel for each ADC
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;                  // Selecting ADCIN2 in our case will be ADCINA2(J3 29) for adc A;

    //Setting Acquisition prescalar for each
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99;                 // Aquisition Prescale of 99, so sample will be hold for (99+1 = 99) system clock cycles
                                                        // i.e. 500ns for proper input;

    // Now Enabling ADC interrupt 1 (ADCINT1) after conversion of SOC0, to do calculation with the converted value
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;              // Event EOC0 will trigger ADCINT1 interrupt;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;                // To enable Enable ADCINT1; This is req. to enable setting of interrupt flag;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              // Clear interrupt flag

    // Using SOC1 now
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 7;                // EPWM2 SOCA is the triggering source for soc1 also;
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;                  // ADCINA4 pin
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99;                 // Sample window

    // Enable interrupt after SOC1 conversion
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 1;              // EOC1 triggers ADCINT2
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;                // Enable ADCINT2
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear interrupt flag

    EDIS;
}

void adcB_init()
{
    // ADC B SOC0 for iL measurement
    // Using SOC0 event of ADC B for iL measurement

    EALLOW;

    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;                // for single-ended adc mode;
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;                // selecting 12-bit resolution for adc;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 14;                 // Adc_clk = SYSCLK/8 = 200MHz/8 = 25MHz;

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;                  //Powering up the adc;

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 7;                // Selecting the triggering source for SOC0 of ADC B,
                                                        // which is epwm2,ADCSOCA as set in epwm2_init() function;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;                  // Selecting ADCIN2 in our case will be ADCINB2(J3 28) for adc B;

    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99;                 // Aquisition Prescale of 99, so sample will be hold for (99+1 = 99) system clock cycles
                                                        // i.e. 500ns for proper input;

    // Now Enabling ADC interrupt 1 (ADCINT1) after conversion of SOC0,i.e. at EOC0 event, to confirm the completion of conversion
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;              // Event EOC0 will trigger ADCINT1 interrupt;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;                // To enable ADCINT1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              // Clear interrupt flag;

    // Using SOC1 now
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 7;                // EPWM2 SOCA
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 4;                  // ADCINB4 pin
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99;                 // Sample window

    // Enable interrupt after SOC1 conversion
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1;              // EOC1 triggers ADCINT2
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;                // Enable ADCINT2
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear interrupt flag

    EDIS;
}

void adcC_init() // Function to initialize ADCC for voltage sensor input (ADCINC2)
{
    EALLOW;

    // ADCC for voltage sensor measurement;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;                // Single-ended mode
    AdccRegs.ADCCTL2.bit.RESOLUTION = 0;                // 12-bit resolution
    AdccRegs.ADCCTL2.bit.PRESCALE = 14;                 // ADC clock = SYSCLK / 8 = 25MHz

    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;                  // Powering up ADCC

    // Configure SOC0 of ADCC to be triggered by EPWM2 SOCA event
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 7;                // Trigger source: EPWM2 SOCA
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;                  // Channel: ADCINC2 (J3 27);
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99;                 // Acquisition window: 100 SYSCLK cycles (500ns)

    // Configure ADCINT1 for ADCC
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;              // Interrupt on EOC0 (End of SOC0)
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;                // Enable ADCINT1
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              // Clear ADCINT1 flag

    EDIS;
}

//-----    mppt function    -----//

void mppt(void)
{
    if(enable_mppt == 1)
    {

    uint16_t adc_Vpv = AdccResultRegs.ADCRESULT0;
    uint16_t adc_Ipv = AdcaResultRegs.ADCRESULT0;
    uint16_t adc_IL  = AdcbResultRegs.ADCRESULT0;

    Vpvsum += (((float)adc_Vpv) / 4095.0f)*300.0f;
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
        Vpv = (((float)adc_Vpv) / 4095.0f)*300.0f;
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

    Vo = (((float)adc_Vo)/4095.0f)*300.0f;
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


//-----    Interrupt setup function    -----//

void setup_interrupts()
{
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;                  // Enable PIE block
    PieVectTable.ADCA2_INT = &adca2_isr;                // Map out own custom written ISR to the ADCA2_INT interrupt in pievect table;
    PieCtrlRegs.PIEIER10.bit.INTx2 = 1;                  // Enable ADCA2 in PIE group 10
                                                        // See PIE channel Mapping table in TRM, page 92;
    IER |= M_INT10;                                           // Enable group 10 interrupts
    EINT;                                               // Global interrupt enable
    EDIS;
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
