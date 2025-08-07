#include "F28x_Project.h"                               // Contains basic device definitions
#include "F2837xD_Examples.h"                           // Changed from F2837xS_Examples.h to F2837xD_Examples.h
#include <stdint.h>
//#define LED_GPIO 4                                    // LED on GPIO4


//-------- Defining global variables --------//

uint32_t SCALINGFACTOR = 100000;                        // For preventing floating numbers, scaling all the values for calculation.
volatile int64_t Ipv = 0;
volatile int64_t Vpv = 0;
volatile uint32_t Vref = 6900000;                            // initially Vref will be Vpv at mppt, i.e. 69;
volatile int64_t IL = 0;
volatile int64_t error_iL_int = 0;                     // intergral current error, need to be preserved for accumulation overtime;

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
    //Setting Epwm2 clk = 100MHz, so count till 10,000(0 to 9999) for 10kHz clock
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;                    // upcount mode on epwm2;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;                     // For clkdiv value of 1;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;                  // For hspclkdiv value of 1; epwm clock is now set to 100MHz with these two lines.
    EPwm2Regs.TBPRD = 9999;                             //Counter will count till 9999;
    EPwm2Regs.CMPA.bit.CMPA = 4999;                     // Trial Compare value of 4999 for 50% duty
    EPwm2Regs.AQCTLA.bit.ZRO = 2;                       //Set EPWM2A on reaching the 0 value;
    EPwm2Regs.AQCTLA.bit.CAU = 1;                       //Will force EPWM2A to clear to 0 on CMPA match;

    // Using shadow mode and then Transferring value from shadow reg to active CMPA reg when TBCTR = 0
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;                 // To enable shadow mode for CMPA.
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;                 // To Load in active reg (CMPA) when TBCTR becomes 0;
    // Now when we will write in CMPA bit of CMPA reg then value will go in shadow reg and go in actual reg when TBCTR = 0

    // generating epwm event for adc start of conversion
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;                     // Enable ADCSOCA pulse;
    EPwm2Regs.ETSEL.bit.SOCASEL = 1;                    // To trigger soc when time base counter TBCTR = 0;

    EPwm2Regs.ETPS.bit.SOCAPRD = 1;                     // Generate SOC pulse on 1st event;

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

    EDIS;
}

void adcB_init()
{
    // ADC B for iL measurement
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

    EDIS;
}

void adcC_init() // Function to initialize ADCC for voltage sensor input (ADCINC2)
{
    EALLOW;

    // ADCC for voltage sensor measurement;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;  // Single-ended mode
    AdccRegs.ADCCTL2.bit.RESOLUTION = 0;  // 12-bit resolution
    AdccRegs.ADCCTL2.bit.PRESCALE = 14;   // ADC clock = SYSCLK / 8 = 25MHz

    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;    // Powering up ADCC

    // Configure SOC0 of ADCC to be triggered by EPWM2 SOCA event
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 7;  // Trigger source: EPWM2 SOCA
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;    // Channel: ADCINC2 (J3 27);
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99;   // Acquisition window: 100 SYSCLK cycles (500ns)

    // Configure ADCINT1 for ADCC
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // Interrupt on EOC0 (End of SOC0)
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable ADCINT1
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear ADCINT1 flag

    EDIS;
}

//-----    mppt function    -----//

void mppt(void)
{   // Get raw ADC values
    uint16_t adc_Vpv = AdccResultRegs.ADCRESULT0;
    uint16_t adc_Ipv = AdcaResultRegs.ADCRESULT0;
    uint16_t adc_IL  = AdcbResultRegs.ADCRESULT0;

    // Scale ADC readings
    // 3V → 4095 counts
    // Vpv: 3V = 500V → Scale = 50000000 (0.01V * 1e5)
    // Ipv/IL: 3V = 30A → Scale = 3000000 (0.01A * 1e5)

    Vpv = ((int64_t)adc_Vpv * 50000000LL) / 4095;  // Vpv in 0.01V (per 100k)
    Ipv = (((int64_t)adc_Ipv - 2047) * 6000000LL) / 4095;
    IL  = (((int64_t)adc_IL  - 2047) * 6000000LL) / 4095;
    
    // Constants
    const uint64_t Vrefmax = 20000000;
    const uint64_t Vrefmin = 0;
    const int deltaVref = 10000;   // Equivalent to 0.1 steps

    // Persistent/static state
    static uint64_t Vold = 0;
    static uint64_t Pold = 0;
    static uint64_t Vrefold =6900000 ;

    // Compute instantaneous power and deltas
    int64_t P = Vpv * Ipv;
    int64_t dV = (int64_t)Vpv - (int64_t)Vold;
    int64_t dP = (int64_t)P - (int64_t)Pold;

    Vref = Vrefold;

    // Perturb and Observe logic
    if (dP != 0)
    {
        if (dP < 0)
        {
            if (dV < 0)
                Vref = Vrefold + deltaVref;
            else
                Vref = Vrefold - deltaVref;
        }
        else
        {
            if (dV < 0)
                Vref = Vrefold - deltaVref;
            else
                Vref = Vrefold + deltaVref;
        }
    }
    else
    {
        Vref = Vrefold;
    }

    // Bound Vref within min and max
    if (Vref >= Vrefmax || Vref <= Vrefmin)
    {
        Vref = Vrefold;
    }

    // Update static values for next iteration
    Vrefold = Vref;
    Vold = Vpv;
    Pold = P;

    // Send Vref to PI controller (to control duty cycle)
    // Example:
    // pi_controller(Vref, Vpvbit);

    // Optional: use ILbit for current monitoring, safety, or logging
}



//-----    pi control function    -----//

void pi_control(void)
{
    // All values recieved are scaled by SCALINGFACTOR
    uint32_t kp = 50000;                        // 0.5*100000;
    uint32_t ki = 200000;                       // 2*100000;
    int32_t error_iL = ((Vpv*Ipv)/Vref) - IL;  // error_i = ILref - IL;
    error_iL_int += (error_iL/10000);           // error_iL_int += (error_iL*Ts); Ts = 1e-4;

    int64_t temp = ((int64_t)kp * error_iL) + ((int64_t)ki * error_iL_int); // For safety if temp becomes larger than 32 bit anyhow;
    int32_t duty = ((int64_t)temp * 9999) / SCALINGFACTOR;
    // final_duty = duty * 9999 / 100000;

    if(duty > 9500) // applying saturation on duty
    {
        duty = 9500;
    }
    if(duty < 0) // if used unsigned value for duty and if duty become -ve then, the -ve value will be a large unsigned integer
                 // then, duty would saturate to 0.95 instead of 0 which would be dangerous, so we make duty signed interger.
    {
        duty = 0;
    }
    EPwm2Regs.CMPA.bit.CMPA = duty; // Updating value in CMPA shadow register, this value will be used in next counting cycle;
}

//-----    ADCA EOC0 ISR function    -----//

__interrupt void adca1_isr(void) // main calculations is done in this ISR;
{
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear ADCINT1 flag for adc A;

    while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0); // wait till ADC_B_INT1 interrupt is not triggered, this is triggered
                                                // by our EOC0 event, i.e. when adc B has done its conversion;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear ADCINT1 flag for adc B;

    while(AdccRegs.ADCINTFLG.bit.ADCINT1 == 0); // wait till ADC_C_INT1 interrupt is not triggered, this is triggered
                                                // by our EOC0 event, i.e. when adc C has done its conversion;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear ADCINT1 flag for adc C;

    // mppt();                                     // Calling mppt function to give Vref; Also calculating values of Vpv,Ipv,IL in this only;

    pi_control();                                // Calling pi_control function to set req. Duty for pwm pulse;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     // Acknowledge PIE group 1;

    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;      //Clearing test gpio pin after eoc;
}


//-----    Interrupt setup function    -----//

void setup_interrupts()
{
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;                  // Enable PIE block
    PieVectTable.ADCA1_INT = &adca1_isr;                // Map out own custom written ISR to the ADCA1_INT interrupt in pievect table;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;                  // Enable ADCA1 in PIE group 1
                                                        // See PIE channel Mapping table in TRM, page 92;
    IER |= 1;                                           // Enable group 1 interrupts
    EINT;                                               // Global interrupt enable
    EDIS;
}


void main(void)
{
    InitSysCtrl();  // Must be called to initialize PLL & Timers, this sets 200MHz freq.
    InitPieCtrl();
    InitPieVectTable();

    // SEQUENCE OF CALLING THESE FUNCTIONS MATTERS A LOT, start adc first then call epwminit, otherwise epwm will run firstly, trigger soc
    // But as adcs are not started yet, the code will get stuck in the while loop, as adc b is not started and we entered the while loop.
    // We will be stucked there only as cpu will not be able to move further in adcinit() function after delay as it got stucked in while loop.
    test_pin_setup();
    adcA_init();
    adcB_init();
    adcC_init();
    volatile uint32_t i; // Simple delay for proper turning on of adcs;
    for(i = 0; i < 5000; i++) { }

    epwm2_init();
    setup_interrupts();

    // Enable writing to protected registers
    EALLOW;

    // Configure GPIO4/GPIO14,etc for general purpose I/O:
    // 1. Set the multiplexer bits to 0 to select GPIO function.
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;

    // 2. Set the direction for GPIO4 to output (1 means output).
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;

    // Optionally, clear the output initially
    GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;

    // Disable writing to protected registers
    EDIS;

    while(1)
    {
//        // Toggle the state of GPIO4
//        GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1;
//        // Simple delay loop (~500ms delay; adjust count as needed)
//        volatile uint32_t j;
//        for(j = 0; j < 500000; j++) { }
    }
}
