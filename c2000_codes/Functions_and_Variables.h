#ifndef FUNCTIONS_AND_VARIABLES_H  // To prevent multiple dupications in future inclusions in multiple files
#define FUNCTIONS_AND_VARIABLES_H

#include "F28x_Project.h"

//============================
// Controller Constants
//============================

#define V_ref       200.0f      // Reference Voltage
#define KP_V        0.1f        // Voltage loop proportional gain
#define KI_V        8.0f        // Voltage loop integral gain
#define KP_I        0.02f       // Current loop proportional gain
#define KI_I        5.0f        // Current loop integral gain
#define TS          1e-4f       // Sampling time (10kHz)




//==============================
//          Variables
//==============================

extern uint16_t enable_mppt;
extern uint16_t mppt_count;

extern volatile float duty;
extern volatile float Ipv;
extern volatile float Vpv;                       // Vpv will be in 0.01V, i.e. 1 count = 0.01V, so 100 counts = 1V;
extern volatile float Vref ;                       // initially Vref will be Vpv at mppt, i.e. 69;
extern volatile float IL;
extern float IL_continuous;
extern float IL_nmin1; // IL[n-1];
extern float IL_nmin2; // IL[n-2];
extern volatile float error_iL_int;                      // intergral current error, need to be preserved for accumulation overtime;
extern float error_iL;
extern float Pold;
extern float Vold;
extern float Vrefold;
extern volatile uint16_t first_time_entry;
extern float P;
extern float dV;
extern float dP;
extern float temp;
extern float Vrefmax;
extern float kp;
extern float ki;
extern float windup;
extern uint16_t count;
extern uint16_t count_max;
extern float Vpvsum;
extern float Ipvsum;
extern float ILsum;
extern float P_deadband;
extern float I_INT_MAX;
extern float I_INT_MIN;

extern volatile float Vo;
extern volatile float IL_bi;
extern volatile float duty_cycle_bi;
extern float duty_bi;




//==============================
//          Functions
//==============================

void setup_interrupts(void);

__interrupt void adca2_isr(void);

__interrupt void epwm2_isr(void);

void test_pin_setup(void);

void epwm2_init(void);

void epwm3_init(void);

void adcA_init(void);

void adcB_init(void);

void adcC_init(void);

float adc_val_Vpv(uint16_t adc_result);

float adc_val_Vo(uint16_t adc_result);


#endif
