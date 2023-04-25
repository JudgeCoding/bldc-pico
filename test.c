/** @file bldc.c
 *  @brief BLDC motor controller based on Zilog code
 *
 *  This is currently a single file program
 *
 *  3 phase sensorless BLDC motor controller using Raspberry Pi Pico ported from Zilog AN0226-SC01
 *  This code operates the basic PLL with compensation
 *  Can run in speed control or torque control mode by setting LOOP_TYPE
 *  - 0 for torque (current) loop, 1 for speed loop
 *  The Potentiometer sets current or speed command depending on the mode
 *  Application Board Motor (45ZWN24-30) 24V, 2A, 3 phase, N= 4 poles, 3200rpm
 *  Back EMF test: Vpeak = Back EMF peak voltage line-to-line = 4.44V
 *  Period = Back EMF period = 0.041 sec
 *  Ke = Vpeak/SQRT(2) * Period/(2*PI()) * N/2 = 0.0410 V/rad/sec (per phase peak value)
 *  Operating off of 24V supply
 *  Back EMF and Vbus Divider 150K and 10K with 0.01uF filter capacitor
 *  ADC Reference 2V
 *  Req = 10k||150K = 9.375K, C = 0.01uF, f= 1/(2*3.1414*Req*C) = 1.7kHz
 *  t=Req * C = 93.8us
 *  For fosc = 20MHz
 *
 *  The steps in production were:
 *  1. Use blink pico code example with external led
 *
 *  @author JudgeDev
 *  @bug No known bugs
 */

/*
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
*/

/** -- System Parameters -- */
#define UART            1               // system has a terminal

#define PWM_PERIOD      50              // 50uS is 20kHz
#define PWM_COUNT_MAX   12              // pwm frequency divider for function loop normally 50

#define BLINK_MAX       800             // function loop divisor
#define LOOP_TYPE       1			          // 0 = torque control, 1 = speed control

/** @brief  Pico registers
 *
 *  Use macro REG to define registers based on address in datsheet
 *  Base values appear to be defined
 *  Register names may clash with other constants, e.g.:
 *  GPIO_OUT is used for IO direction in SDK functions
 */



#define REG(address) (*(volatile uint32_t*)(address))

#define GPIO_OUT_XOR  REG(SIO_BASE+0x01c)

#define TIMELR      REG(TIMER_BASE+0x0c) // timer low bits
#define ALARM1      REG(TIMER_BASE+0x14) // time to fire
#define ARMED       REG(TIMER_BASE+0x20) // armed status of each alarm
#define INTR        REG(TIMER_BASE+0x34) // raw interrupts (write to clear)
#define INTE        REG(TIMER_BASE+0x38) // timer interrupt enable

#define NVIC_ISER   REG(PPB_BASE+0xe100) // interrupt set enable bits
#define VTOR        REG(PPB_BASE+0xed08)

/** -- GPIO pin definitions -- */
#define LED_YEL     2
#define LED_GRN     3
#define LED_RED     4
#define SW_DIR      5
#define PWM_AH      14                  // pwm phase A, High
#define POT_SPEED   26
#define ADC_SPEED   0                   // adc0 is gpio26

/** -- Global constants -- */
#define ALARM_INT_NUM   1               // alarm interrupt number
#define UART            1               // system has a terminal
#define FWD             0
#define REV             1
// PWM and Duty Cycle control
//#define Dead_band     0u					    // 1 count = 1 Oscillator Clock Cycle
#define COM_MAG_MIN     125             // maximum pwm modulation index
#define COM_MAG_MAX     250             // maximum pwm modulation Index
#define PWM_PRESCALER   25              // pwm timer prescaler

// Speed Loop Compensation
#define SPEED_CMD_MIN   50u              // min set speed value
#define S_LOOP_COUNT_MAX 10             // speed loop update rate


/** -- Global variables -- */
// UI variables
#if UART
unsigned char ui_control = 0;
unsigned char ui_speed = 0;
unsigned char ui_direction = FWD;
#endif
// LED variables
unsigned int  blink = 0;				        // LED blink
unsigned char blink_count = 0;			    // prevents double blinking
// Direction Switch
unsigned char direction_sw = 0;		      // direction switch value
unsigned char direction_count = 128;	  // Counter for Direction Switch Filter
unsigned char direction = 0;			      // desired direction

// PWM and Duty Cycle control
unsigned char com_mag = COM_MAG_MIN;	  // pwm modulation Index

// Back EMF Sensing
unsigned char adc_vbus = 0;				      // bus voltage measurement (Neutral = 1/2*Vbus)
unsigned char adc_vdc = 0;				      // bus voltage
unsigned char adc_vbemf = 0;			      // back EMF voltage measurement
// Speed Sensing and Command
unsigned char speed_cmd = SPEED_CMD_MIN;     // set speed
unsigned char speed = 0;                // speed

// Speed Loop Compensation
unsigned int  s_loop_count = 0;			    // Counter for speed loop functions update rate

// Function Loop Updates
unsigned char pwm_count = 0;				    // count pwm interrupts per loop
unsigned char pwm_step = 0; 		        // loop sequence step

main() {
    static int x = 0;
    printf("what the fuck");
    printf("%d", x=0);
    ;
}
