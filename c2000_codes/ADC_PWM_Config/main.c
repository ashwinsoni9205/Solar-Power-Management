#include "F28x_Project.h"      // Contains basic device definitions
#include "F2837xD_Examples.h"  // Changed from F2837xS_Examples.h to F2837xD_Examples.h

//#define LED_GPIO 4             // LED on GPIO4

void epwm_init() // function to initialize epwm port;
{
    // using EPWM2A for boost converter operation;
    // EPWM2A : J8 80 pin on the launchpad;
    EALLOW;

    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1; // configuring gpio2(J8 80) as output;
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
void adc_init() // function to initialize adc ports;
{
    //Using ADCINA2 pin(J7 pin 64) for adc input;
    EALLOW;

    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0; // for single-ended adc mode;
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0; // selecting 12-bit resolution for adc;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // Adc_clk = SYSCLK/4 = 50MHz/4 = 12.5MHz recommended (was 12 for F28377S)

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;  //Powering up the adc;
    volatile uint32_t i;
    for(i = 0; i < 5000; i++) { } // simple delay loop for getting adc turned on properly;

    //We have multiple SOCs like SOC0,SOC1,... can use any of them with epwms; Using SOC0 for now;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7; //Selecting the triggering source for SOC0, which is epwm2,ADCSOCA as set up above;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //Selecting ADCIN3 channel for SOC0; //ADCINA2 taking 3.3V ref but ADCINA3 is taking 3V as ref.
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 20; // Aquisition Prescale of 20, so sample will be hold for 20 adc cycles for proper input;

    // Now Enabling ADC interrupt 1 (ADCINT1) after conversion of SOC0, to do calculation with the converted value;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // Event EOC0 will trigger ADCINT1 interrupt;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // To enable Enable ADCINT1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear interrupt flag

    EDIS;

}

__interrupt void adca1_isr(void)
{
    Uint16 result = AdcaResultRegs.ADCRESULT0;  // Reading the result from adc0 result register;
    Uint16 new_cmpa = (Uint32)result * 9999 / 4095;
    // Limiting new_cmpa within TBPRD range to avoid PWM errors
    if(new_cmpa > 9999) new_cmpa = 9999;
    EPwm2Regs.CMPA.bit.CMPA = new_cmpa;   // Updating the duty cycle, this will go in the shodow reg firstly;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // To Clear ADC interrupt flag;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     // Acknowledge PIE group 1;
}

void setup_interrupts()
{
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;         // Enable PIE block
    PieVectTable.ADCA1_INT = &adca1_isr;       // Map out own custom written ISR to the ADCA1_INT interrupt in pievect table;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;         // Enable ADCINT1 in PIE group 1
    IER |= 1;                             // Enable group 1 interrupts
    EINT;                                      // Global interrupt enable
    EDIS;
}

void main(void)
{
    InitSysCtrl();  // Must be called to initialize PLL & Timers, this sets 200MHz freq.
    InitPieCtrl();
    InitPieVectTable();

    epwm_init();
    adc_init();
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
        // Toggle the state of GPIO4
        GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1;
        // Simple delay loop (~500ms delay; adjust count as needed)
        volatile uint32_t j;
        for(j = 0; j < 500000; j++) { }
    }
}
