#include "F28x_Project.h"                               // Contains basic device definitions
#include "F2837xD_Examples.h"                           // Changed from F2837xS_Examples.h to F2837xD_Examples.h
#include <stdint.h>
#include <math.h>
//#define LED_GPIO 4                                    // LED on GPIO4


volatile float Vo = 0.0f;
volatile float IL = 0.0f;
volatile float duty_cycle = 0.0f;
float duty = 0.0f;

//============================
// Persistent Controller States
//============================
static float error_v_int = 0.0f;
static float error_i_int = 0.0f;
static float I_ref_old   = 0.0f;
static float windup      = 0.0f;
static int   count       = 0;
static int   first_time_entry = 0;

//============================
// Controller Constants
//============================
#define V_ref       200.0f      // Reference Voltage
#define KP_V        0.1f        // Voltage loop proportional gain
#define KI_V        8.0f        // Voltage loop integral gain
#define KP_I        0.02f       // Current loop proportional gain
#define KI_I        5.0f        // Current loop integral gain
#define TS          1e-4f       // Sampling time (10kHz)


__interrupt void epwm3_isr(void)
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;      // Set GPIO61 HIGH at TBCTR=0
    EPwm3Regs.ETCLR.bit.INT = 1;             // Clear EPWM3 interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  // Acknowledge PIE group 3
}


void test_pin_setup()
{
    EALLOW;

    // --- GPIO61 as test pin ---
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // --- EPWM3 interrupt setup ---
    EPwm3Regs.ETSEL.bit.INTEN = 1;     // Enable EPWM3 interrupt
    EPwm3Regs.ETSEL.bit.INTSEL = 1;    // Interrupt on counter = zero
    EPwm3Regs.ETPS.bit.INTPRD = 1;     // Generate INT on every event

    // --- Map ISR to PIE vector ---
    PieVectTable.EPWM3_INT = &epwm3_isr; // Link to ISR
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;   // Group 3, int 3 = EPWM3

    IER |= M_INT3;

    EDIS;
}

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
    EPwm2Regs.ETSEL.bit.SOCASEL = 1;                    // To trigger soc when time base counter TBCTR = 0; It will be better to do soc a bit away from 0
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
    EPwm3Regs.TBCTL.bit.CTRMODE = 0;        // Up-count
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm3Regs.TBPRD = 9999;                 // 10 kHz
    EPwm3Regs.CMPA.bit.CMPA = 4999;         // 50% duty for A

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

    // Set deadtime counts
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


void bidirectional(void)
{
    float error_v, error_i;
    float I_ref;

    // Step 1: Read ADC values
    uint16_t adc_Vo= AdcaResultRegs.ADCRESULT1;
    uint16_t adc_IL= AdcbResultRegs.ADCRESULT1;

    Vo = (((float)adc_Vo)/4095.0f)*300.0f;
    IL = (((float)adc_IL-2047.0f)/4095.0f)*60.0f;

    // Step 2: First time initialization
    if(first_time_entry == 0)
    {
        first_time_entry = 1;
        error_v = V_ref - Vo;
        error_v_int += error_v * TS;
        I_ref = (KP_V * error_v) + (KI_V * error_v_int);
        I_ref_old = I_ref;
    }

    // Step 3: Outer Voltage Control Loop (slower)
    if(count >= 50)   // Run every 50 cycles (~200 Hz if Ts=100us)
    {
        error_v = V_ref - Vo;
        error_v_int += error_v * (TS * count);
        I_ref = (KP_V * error_v) + (KI_V * error_v_int);
        I_ref_old = I_ref;
        count = 0;
    }
    else
    {
        I_ref = I_ref_old;
    }

    // Step 4: Inner Current Control Loop
    error_i = I_ref - IL + windup;
    error_i_int += error_i * TS;
    duty_cycle = (KP_I * error_i) + (KI_I * error_i_int);

    // Step 5: Clamp duty cycle
    duty = duty_cycle;
    if(duty < 0.05f) duty = 0.05f;
    if(duty > 0.95f) duty = 0.95f;

    // Anti-windup correction
    windup = duty - duty_cycle;


    //EPwm3Regs.CMPA.bit.CMPA = (Uint16)(duty * 9999);   // Update duty for EPWM3A

    // Step 7: Increment counter
    count++;

}



void adcA_init() // function to initialize adcA ports;
{
    // ADCA for iL measurement using SOC1 triggered by EPWM3A

    EALLOW;

    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;                // Single-ended mode
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;                // 12-bit resolution
    AdcaRegs.ADCCTL2.bit.PRESCALE = 14;                 // SYSCLK/8 = 25MHz

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;                  // Power up ADC

    // Using SOC1 now (instead of SOC0)
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 7;                // EPWM2 SOCA is the triggering source for soc1 also;
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;                  // ADCINA4 pin
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99;                 // Sample window

    // Enable interrupt after SOC1 conversion
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 1;              // EOC1 triggers ADCINT2
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;                // Enable ADCINT2
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear interrupt flag




    // ADCA for ipv measurement
    // Using SOC0 event of ADC A for ipv measurement


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
    // ADCB for iL measurement using SOC1 triggered by EPWM4

    EALLOW;

    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;                // Single-ended mode
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;                // 12-bit resolution
    AdcbRegs.ADCCTL2.bit.PRESCALE = 14;                 // SYSCLK/8 = 25MHz

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;                  // Power up ADC

    // Using SOC1 now (instead of SOC0)
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 7;                // EPWM2 SOCA
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 4;                  // ADCINB4 pin
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99;                 // Sample window

    // Enable interrupt after SOC1 conversion
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1;              // EOC1 triggers ADCINT2
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;                // Enable ADCINT2
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear interrupt flag

    EDIS;
}


__interrupt void adca2_isr(void)                        // ISR triggered by ADCA EOC1 (ADCINT2)
{

    // Idealy, we should read adc values before clearing the adcint flags;


    // Clear ADCINT1 flag for ADCA (EOC0)
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // we got ipv with eoc0;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear ADCINT2 flag for ADCA (EOC1)

    // Wait until ADCB EOC1 is complete
    while(AdcbRegs.ADCINTFLG.bit.ADCINT2 == 0);
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;              // Clear ADCINT2 flag for ADCB (EOC1)

    // Main calculations
    bidirectional();

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;             // Acknowledge PIE group 10

    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;               // Clear test GPIO
}

void setup_interrupts()
{
    EALLOW;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;                   // Enable PIE block

    // Map ADCA INT1 interrupt vector to our ISR
    PieVectTable.ADCA2_INT = &adca2_isr;

    // Enable ADCA2 interrupt in PIE group 10
    PieCtrlRegs.PIEIER10.bit.INTx2 = 1;

    IER |= M_INT10;  // Ox0200;                                          // Enable group 10 interrupts
    EINT;                                                // Global interrupt enable
    EDIS;
}


void main(void)
{
    InitSysCtrl();
    InitPieCtrl();
    InitPieVectTable();

    test_pin_setup();
    adcA_init();
    adcB_init();

    volatile uint32_t i;
    for(i = 0; i < 5000; i++) { }   // Allow ADCs to stabilize

    epwm2_init();
    epwm3_init();     // Replaces epwm4 + epwm5

    setup_interrupts();

    while(1)
    {

            }
}
