#include "F28x_Project.h"                    // Contains basic device definitions
#include "F2837xD_Examples.h"                // Changed from F2837xS_Examples.h to F2837xD_Examples.h
//#define LED_GPIO 4                         // LED on GPIO4


//-------- Defining global variables --------//

uint32_t SCALINGFACTOR = 100000;             // For preventing floating numbers, scaling all the values for calculation.
volatile uint32_t Ipv = 0;
volatile uint32_t Vpv = 0;
volatile uint32_t Vref = 69;                 // initially Vref will be Vpv at mppt, i.e. 69;
volatile uint32_t IL = 0;
volatile uint32_t ILref = 0;

//--------       Test pin set        --------//

__interrupt void epwm2_isr(void)             // generating interrupt at start to set the test pin high for timing measurements;
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;      // Set GPIO61 HIGH at TBCTR=0 (SOC event)
    EPwm2Regs.ETCLR.bit.INT = 1;             // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  // Acknowledge PIE group 3
}
void test_pin_setup() // Setting a gpio pin for testing purposes to see time taken by certain function or block
{
    // using gpio 61 (J2 19);
    EALLOW;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;     // Set as GPIO function (not peripheral)
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;      // Set as output
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;    // Start LOW
    EPwm2Regs.ETSEL.bit.INTEN = 1;           // EPWM2 INT enable
    EPwm2Regs.ETSEL.bit.INTSEL = 1;          // INT on counter=zero
    EPwm2Regs.ETPS.bit.INTPRD = 1;           // INT on first event
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;       // Enable PIE group 3, INT2 (EPWM2)
    IER |= M_INT3;                           // Enable group 3 interrupts
    // EINT;                                 // Global interrupt enable already done in interrupt setup function;
    EDIS;
}

//---- Function to initialize epwm port; ----//

void epwm2_init()
{
    // using EPWM2A for boost converter operation;
    EALLOW;

    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1; // configuring gpio2(J4 38) as output;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // selecting EPWM2A instead of GPIO2 on J4 38 pin;
    //Setting Epwm2 clk = 100MHz, so count till 10,000(0 to 9999) for 10kHz clock;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; //upcount mode on epwm2;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // For clkdiv value of 1;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;// For hspclkdiv value of 1; epwm clock is now set to 100MHz with these two lines.
    EPwm2Regs.TBPRD = 9999; //Counter will count till 9999;
    EPwm2Regs.CMPA.bit.CMPA = 4999; // Trial Compare value of 4999 for 50% duty
    EPwm2Regs.AQCTLA.bit.ZRO = 2; //Set EPWM2A on reaching the 0 value;
    EPwm2Regs.AQCTLA.bit.CAU = 1; //Will force EPWM2A to clear to 0 on CMPA match;

    // Using shadow mode and then Transferring value from shadow reg to active CMPA reg when TBCTR = 0;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0; // To enable shadow mode for CMPA.
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0; // To Load in active reg (CMPA) when TBCTR becomes 0;
    // Now when we will write in CMPA bit of CMPA reg then value will go in shadow reg and go in actual reg when TBCTR = 0;

    // generating epwm event for adc start of conversion;
    EPwm2Regs.ETSEL.bit.SOCAEN = 1; // Enable ADCSOCA pulse;
    EPwm2Regs.ETSEL.bit.SOCASEL = 1;// To trigger soc when time base counter TBCTR = 0;

    EPwm2Regs.ETPS.bit.SOCAPRD = 1;  // Generate SOC pulse on 1st event;

    EDIS;
}
void adcA_init() // function to initialize adc ports;
{
    EALLOW;

    // ADCA for ipv measurement;
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0; // for single-ended adc mode;
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0; // selecting 12-bit resolution for adc;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 14; // Adc_clk = SYSCLK/8 = 200MHz/8 = 25MHz;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;  //Powering up the adc;
    volatile uint32_t i;
    for(i = 0; i < 5000; i++) { } // simple delay loop for getting adc turned on properly;

    // We have multiple SOCs like SOC0,SOC1,... can use any of them with epwms; Using SOC0 for now;
    // Will be triggering all adc soc events by epwm2's ADCSOCA pulse;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7; // Selecting the triggering source for SOC0 of ADC A, which is epwm2,ADCSOCA as set up above;

    //Selecting channel for each ADC:
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // Selecting ADCIN2 in our case will be ADCINA2(J3 29) for adc A;

    //Setting Acquisition prescalar for each:
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; // Aquisition Prescale of 99, so sample will be hold for (99+1 = 99) system clock cycles
    // i.e. 500ns for proper input;

    // Now Enabling ADC interrupt 1 (ADCINT1) after conversion of SOC0, to do calculation with the converted value;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // Event EOC0 will trigger ADCINT1 interrupt;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // To enable Enable ADCINT1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear interrupt flag

    EDIS;
}
void adcB_init()
{
    // ADC B for iL measurement;
    // Using SOC0 event of ADC B for iL measurement;

    EALLOW;

    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0; // for single-ended adc mode;
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0; // selecting 12-bit resolution for adc;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 14;  // Adc_clk = SYSCLK/8 = 200MHz/8 = 25MHz;

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;   //Powering up the adc;
    volatile uint32_t i;
    for(i = 0; i < 5000; i++) { }        // simple delay loop for getting adc turned on properly;

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 7; // Selecting the triggering source for SOC0 of ADC B,
                                         // which is epwm2,ADCSOCA as set in epwm2_init() function;

    //Selecting channel for each ADC:
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2; // Selecting ADCIN2 in our case will be ADCINB2(J3 28) for adc B;

    //Setting Acquisition prescalar for each:
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; // Aquisition Prescale of 99, so sample will be hold for (99+1 = 99) system clock cycles
    // i.e. 500ns for proper input;

    // Now Enabling ADC interrupt 1 (ADCINT1) after conversion of SOC0, to do calculation with the converted value;
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // Event EOC0 will trigger ADCINT1 interrupt;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;    // To enable Enable ADCINT1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear interrupt flag

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
    volatile uint32_t i;
    for(i = 0; i < 5000; i++) { }         // Stabilization delay

    // Configure SOC0 of ADCC to be triggered by EPWM2 SOCA event
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 7;  // Trigger source: EPWM2 SOCA
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;    // Channel: ADCINC2 (C2)
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99;   // Acquisition window: 100 SYSCLK cycles (500ns)

    // Configure ADCINT1 for ADCC
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // Interrupt on EOC0 (End of SOC0)
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable ADCINT1
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear ADCINT1 flag

    EDIS;
}

void mppt()
{
    // Fetch ADC results from SOC0 of ADC A, B, and C using 32-bit variables
    Uint32 Ipvbit = (Uint32)AdcaResultRegs.ADCRESULT0;  // Ipv from ADCINA2 (J3 29)
    Uint32 ILbit  = (Uint32)AdcbResultRegs.ADCRESULT0;  // IL from ADCINB2 (J3 28)
    Uint32 Vpvbit = (Uint32)AdccResultRegs.ADCRESULT0;  // Vpv from ADCINC2 (J3 27)

    // Placeholder for MPPT logic
    
}

__interrupt void adca1_isr(void)
{
    while(AdcbRegs)
}

void picontrol(void)
{
    



}

void setup_interrupts()
{
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;         // Enable PIE block
    PieVectTable.ADCA1_INT = &adca1_isr;       // Map out own custom written ISR to the ADCA1_INT interrupt in pievect table;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;         // Enable ADCINT1 in PIE group 1
    IER |= 1;                                  // Enable group 1 interrupts
    EINT;                                      // Global interrupt enable
    EDIS;
}

void main(void)
{
    InitSysCtrl();  // Must be called to initialize PLL & Timers, this sets 200MHz freq.
    InitPieCtrl();
    InitPieVectTable();

    epwm2_init();
    adcA_init();
    setup_interrupts();
    test_pin_setup();

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
        // Toggle the state of GPIO4
        GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1;
        // Simple delay loop (~500ms delay; adjust count as needed)
        volatile uint32_t j;
        for(j = 0; j < 500000; j++) { }
    }
}
