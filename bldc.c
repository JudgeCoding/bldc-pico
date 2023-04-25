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
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

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
#define PWM_1H      10                  // pwm phase 1, High
#define PWM_1L      11                  // pwm phase 1, Low
#define PWM_2H      12                  // pwm phase 2, High
#define PWM_2L      13                  // pwm phase 2, Low
#define PWM_3H      14                  // pwm phase 3, High
#define PWM_3L      15                  // pwm phase 3, Low
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


/** @brief  init_in - Switch setup
 *
 *  SW_DIR is used for the direction switch input
 */
void init_in(void) {
  gpio_init(SW_DIR);
  gpio_set_dir(SW_DIR, GPIO_IN);        // set dir switch to input
  gpio_set_pulls(SW_DIR, 1, 0);         // set pullup
}

/** @brief init_led - LED setup
 *
 *  Use pico sdk function to initialise pins
 *  Delete led on/off functions - use gpio_put(led, state)
 */
void init_led(void) {                   // need to set up pins before using them
  gpio_init(LED_YEL);
  gpio_set_dir(LED_YEL, GPIO_OUT);
  gpio_init(LED_GRN);
  gpio_set_dir(LED_GRN, GPIO_OUT);
  gpio_init(LED_RED);
  gpio_set_dir(LED_RED, GPIO_OUT);
}

/**
 *  @brief  init_analog - Enable analog channels and set up ADC
 *
 *  Analog Channels are
 *  Back EMF Signals VA(PB0/ANA0), VB(PB1/ANA1), and VC(PB2/ANA2)
 *  Current signal (PB3/ANA3)
 *  DC bus voltage V+ (PB4/ANA4)
 *  Speed or Torque Command Potentiometer (PB6/ANA6)
 */
void init_analog(void) {
  /*
	PBAF0 |= 0x5F;			// 0101 1111 Alternate Function PB0, PB1, PB2, PB3, PB4, PB6
	PBAF1 &= 0xA0;			// 1010 0000 
	ADCCTL0 = 0x30;			// 0011 0000 Enable Reference and ADC
	ADCSST = 10;			// ADC Channels select Mux Settling Time 0.5us min, number of clock cycles
	ADCST = 20;				// ADC Sample and Hold Settling Time 1us min, number of clock cycles
	ADCCP = 0;				// ADC clock Prescaler
  */
  adc_init();                           // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(POT_SPEED);             // Select ADC input 0 (POT_SPEED)
}

/**
 *  @brief  alarm_isr - Alarm interrupt handler
 *
 *  Implement delay using alarm interrupt
 *  Every interrupt toggles led
 */
void alarm_isr(){
  ALARM1 = TIMELR + PWM_PERIOD;         // arm timer
  INTR = 1 << ALARM_INT_NUM;            // clear interrupt
}


/** @brief  init_commute - Commutator setup
 *
 *  Timer and alarm 1 used for commuatiation
 *  Generates the clock for the Back EMF sensing PLL
 *  When the timer reaches it's end count an interrupt is generated
 */
void init_commute(void) {
  /*
  // The timer Prescaler value is should be set by the maximum commutation frequency
  // of the target motor. 
	T0CTL1 = (0x01 | (T0_prescaler << 3));	// Disable Timer and Set Prescaler
	T0H = 0x00;				// Set Initial Counter Value
	T0L = 0x01;				//
	T0CTL0 = 0x20;			// Continuous Mode, Interrupt
	T0RH = Com_period_High;	// Set Maximum Count
	T0RL = Com_period_Low;	//
  */
  printf("%s: %x\n", "Commutation setup", 1);
}

/**
 *  @brief speed_cmd - Get speed command from speed potentiomenter
 *
 *  Connected to POT_SPEED
 *  This loop runs at 1/10th the rate of the current loop so S_loop_count
 *  Counts to 10 before updating speed loop
 *  ADC is 12-bit - shift value right by 4 bits to get speed command in range 0-255
 */
void get_speed_cmd(void) {
	s_loop_count++;
	if (s_loop_count > S_LOOP_COUNT_MAX) {
    s_loop_count = 0;                   // NEED TO MOVE THIS TO LAST SPEED FUNCTION
    adc_select_input(ADC_SPEED);        // select adc 0
    speed_cmd = adc_read() >> 4;        // scale adc 0-255
  		if (speed_cmd < SPEED_CMD_MIN) {
			speed_cmd = SPEED_CMD_MIN;
		}
	}
  #if UART
  // If UI has control overide the speed with UI speed setting not speed pot setting
  if (ui_control) {
    speed_cmd = ui_speed;
  }
  #endif
}

/**
 *  @brief  led_blink - Control blinking of status leds
 *
 *  Yel LED blinking: motor is stopped (UART is in Control) but the MCU has power 
 *  Grn LED blinking: motor is running
 *  Red LED blinking: fault occured and motor stopped - cleared by
 *  toggling the "RUN STOP/RESET" switch
 *  blink freq = 1E6 / PWM_PERIOD / PWM_COUNT_MAX / BLINK_MAX
 */
void led_blink(void) {
	if ((++blink == 1) & (blink_count == 0)) {  // indicate a FAULT with red led
		//if ((PWMFSTAT & 0x02) != 0) {
    //  gpio_put(LED_RED, 1);
		//}
		//if ((PWMFSTAT & 0x01) != 0) {     // indicate the motor is in STOP/RESET with yellow led
    //  gpio_put(LED_YEL, 1);
		//	PWMFSTAT |= 0x07;
		//} else {
    //  gpio_put(LED_GRN, 1); 				  // indicate the motor is in START with green led
		//}
    #if UART
    if (ui_control) {                   // indicate the motor is in UI, thus hardware control is disabled
      gpio_put(LED_YEL, 1);
      gpio_put(LED_GRN, 0);
    } else {
      gpio_put(LED_YEL, 0);
      gpio_put(LED_GRN, 1);             // TEMP flash green in hardware mode
    }
		#endif
		blink_count = 1;
	}
	if ((blink == BLINK_MAX/2) & (blink_count == 1)) {
    gpio_put(LED_YEL, 0);
    gpio_put(LED_RED, 0);
    gpio_put(LED_GRN, 0);
		blink_count = 2;
	}
	if (blink > BLINK_MAX) {
		blink = 0;
		blink_count = 0;
	}
}

/**
 *  @brief direction_update
 *
 *  Reads direction switch on SW_DIR
 *  Switch has to be in same position for 15 readings of the direction switch
 *  = 500us x 15 = 7.5mS
 */
void direction_update(void) {
	// Direction Switch update
  direction_sw = gpio_get(SW_DIR);    // get dir switch value
	if (direction_count < 5) {
		direction = FWD;
		direction_count = 5;
	}
	if (direction_count > 20) {
		direction = REV;
		direction_count = 20;
	}
	if (direction_sw == 0) {
		direction_count--;
	} else {
		direction_count++;
	}
#if UART
if (ui_control)
  direction = ui_direction;
#endif
}

/**
 *  @brief  pwm_isr - PWM interrupt handler
 *
 *  This interrupt serves as a function loop called every PWM cycle.
 *  An initial check is made to make sure that this ISR does not conflict with
 *  the Back EMF sensing ISR which should always have priority
 *  pwm_count is a software counter that allows multiple loops to be serviced
 *  at a freq = 1E6 / PWM_PERIOD / PWM_COUNT_MAX
 *  The different service loops are:
 *  The Watch Dog Timer is Reset
 *  The update of the motor direction
 *  The blinking of the LEDs
 *  The Torque or Speed Loop
 *  More user functions can be added to this loop as required
 */
void pwm_isr() {
  pwm_clear_irq(pwm_gpio_to_slice_num(PWM_1H)); // clear the interrupt flag that brought us here

	pwm_count++;
	if (pwm_count >= PWM_COUNT_MAX) {	    // max number of steps
		pwm_count = 0;
		pwm_step = 0;
	}
  // Check for Back EMF Sensing collision if timer within T0_interrupt_delay of Com_period
	//T0_high = T0H;
	//T0_count = T0L + (T0_high << 8) + T0_interrupt_delay;
	//if (T0_count < Com_period)
	{
		if (pwm_step == 5) {
			if (LOOP_TYPE) {
				//speed_reg();
			} else {
				//current_reg();
			}
			pwm_step++;
		}
		if (pwm_step == 4) {
			if (LOOP_TYPE) {
				//speed_sample();
			} else {
				//current_sample();
			}
			pwm_step++;
		}
		if (pwm_step == 3) {
			if (LOOP_TYPE) {
				get_speed_cmd();
			} else {
				//current_cmd();
			}
			pwm_step++;
		}
		if (pwm_step == 2) {
			led_blink();
			pwm_step++;
		}
		if (pwm_step == 1) {
			direction_update();
			pwm_step++;
		}
		if (pwm_step == 0) {
			//asm("WDT");				// Refresh Watch Dog Timer
			pwm_step++;
		}
	}
}

/**
 *  @brief  init_alarm() - Setup alarm interrupt
 *
 *  To setup an alarm interrupt:
 *  - Define the exception/interrupt handler
 *  - Enable the interrupt at the timer (INTE)
 *  - Enable the appropriate timer interrupt (NVIC_ISER)
 *  - Set the time to fire (ALARM1)
 *  - Clear the interrupt after firng (INTR)
 */
void init_alarm() {
  REG(VTOR + (16 + TIMER_IRQ_1) * 4) = (uint32_t)pwm_isr;
  INTE |= 1 << ALARM_INT_NUM;
  NVIC_ISER = 1 << TIMER_IRQ_1;
  ALARM1 = TIMELR + PWM_PERIOD;         // set time to fire
}

/** @brief  init_pwm - PWM setup
 *
 *  Pico clock freq:  125MHz
 *  Prescaler:        25
 *  Wrap value:       250
 *  pwm freq:         125MHz / 25 / 255 = 20kHz
 */
void init_pwm(void) {
  printf("%s: %x\n", "PWM setup", 1);
  gpio_set_function(PWM_1H, GPIO_FUNC_PWM); // set pin for pwm
  uint slice_num = pwm_gpio_to_slice_num(PWM_1H); // get slice of pwm pin


  
  pwm_config config = pwm_get_default_config(); // get defaults for the slice configuration
  pwm_config_set_clkdiv(&config, 25.f);   // set pwm prescaler
  pwm_config_set_wrap (&config, COM_MAG_MAX);  // set pwm reload value 
  pwm_init(slice_num, &config, true);     // load configuration into pwm slice, and set it running
  pwm_set_gpio_level(PWM_1H, com_mag);    // set duty cycle
}

/**
 *  @brief  init_pwmint - PWM Interrupt Enable
 *
 *  Enable PWM Interrupt for Loop servicing
 *  Mask PWM_1's slice's IRQ output into the PWM block's single interrupt line,
 *  and register interrupt handler
 */
void init_pwmint(void)	// 
{
  uint slice_num = pwm_gpio_to_slice_num(PWM_1H); // get slice of pwm pin
  pwm_clear_irq(slice_num);
  pwm_set_irq_enabled(slice_num, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_isr);
  irq_set_enabled(PWM_IRQ_WRAP, true);
}


/** @brief  display_status - Display system status
 *
 */
void display_status() {
  printf("\nSYSTEM STATUS:\n");
  //printf("%-16s: %x\n", "PPB_BASE", PPB_BASE);
  //printf("%-16s: %x\n", "NVIC_ISER", NVIC_ISER);
  printf("%-16s: %x\n", "Direction", direction);
  printf("%-16s: %d\n", "Set Speed", speed_cmd);
}

void test_pwm_leds() {
  uint8_t led;
  for (led = PWM_1H; led < PWM_3L + 1; ++led) {
  gpio_init(led);
  gpio_set_dir(led, GPIO_OUT);
  }
  for (led = PWM_1H; led < PWM_3L + 1; ++led) {
    gpio_put(led, 1);
    sleep_ms(250);
    gpio_put(led, 0);
    sleep_ms(250);
  }
}

//////////////////////////////////////////////////////////////////////////////
// Main program brings here 
//////////////////////////////////////////////////////////////////////////////
/** @brief  main - Main program */
int main() {
  stdio_init_all();
  printf("Welcome to PicoBLDC\n");
  init_in();                            // initialize switch inputs
  init_led();                           // initialise leds
	//init_amp();		                      // enable op amp for current amplification
	//init_comp();	                      // enable comparator for pulse-by-pulse current limiting
	init_analog();	                      // set up Analog Channels for Back EMF and Bus Voltage Sensing
	init_commute();	                      // set up timer for commutation period
	init_pwm();                           // set up 20kHz PWM and BLDC commutation
	//init_commute_int();	                // enable commutation interrupt
	init_pwmint();	                      // enable PWM interrupt for loop servicing
  //display_status();
  //test_pwm_leds();

  while (true) {
    sleep_ms(250);

		#if UART
		printf("\n\nPress O for options:");
    char ch = toupper(getchar());
    getchar();                           // To consume newline
    switch (ch) {
      case 'O':
        printf("\nD: Display status");
        printf("\nU: User Interface");
        printf("\nH: Give back to hardware");
        printf("\nS: Start motor");
        printf("\nE: Stop motor");
        printf("\nF: Forward direction");
        printf("\nR: Reverse direction");
        printf("\nV: DC Voltage reading");
        printf("\nC: Current speed reading");
        printf("\nM: Set motor speed");
        break;
      case 'D':
        display_status();
        break;
      case 'U':
        ui_control = 1;
        ui_direction = direction;
        ui_speed = speed;
        printf("\nUI Enabled, Hardware Control disabled");
        break;
      case 'H':
        ui_control = 0;
        //PWMCTL0 |= 0x80;
        printf("\nHardware Control enabled, UI Disabled");
        break;
      /*??
        else if ((PWMFSTAT & 0x01) != 0) // indicate the motor is in STOP/RESET with yellow led
      {
          yel_on();
          //PWMFSTAT |= 0x07;
      }
      */
      case 'S':
        //PWMCTL0 |= 0x80;
        printf("\nMotor Start");
        break;
      case'E':
        //PWMCTL0 &= 0x7F;
        printf("\nMotor Stop");
        break;
      case 'F':
      ui_direction = FWD;
        printf("\nForward Direction");
        break;
      case 'R':
        ui_direction = REV;
        printf("\nReverse Direction");
        break;
      case 'V':
        adc_vdc = (adc_vbus / 4);
        printf("\nDC Voltage:%4d Volts", adc_vdc);
        break;
      case 'C':
        printf("\nCurrent Speed:%4d", speed); // Speed Shown in Decimal although inputting Speed is in HEX
        break;
      case 'M':                   // input speed in HEX from 32 - 9B (50~150 in decimal)
        printf("\r\nEnter Speed 32-9B (HEX):  ");
        //ui_speed= ScanHex(2);
        break;
      default:
        printf("\nCommand not recognised");
    }
    #endif
  }
}