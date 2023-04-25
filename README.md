# bldc-pico
Port of BLDC code to Raspberry Pi Pico


BLDC Motor Controller Form Simulator

**Zilog application**

Open [Zilog code](https://www.zilog.com/index.php?option=com_doc&Itemid=99) in some editor:

\> Flash MC > Z8FMC16100 MCU > AN0226-SC01

Application note AN0226 explains how the code works in general terms.

**Raspberry Pi Pico**

The steps follow the Pico Deep Dive classes.

The video refers to the [work document](https://docs.google.com/document/d/1x__3WVVQx2JcAXIoAZsPCeZu68XgWKRKsDpYr5_GFRc/edit#heading=h.54gel0soeue1). 

[Class 1](https://www.youtube.com/watch?v=Duel_Oaases&list=PL_tws4AXg7auiZHZsL-qfrXoMiUONBB0U&index=2) - starts with the Blink on Pi Pico example. Open the example. Sign up and save as bldc.

Add header file to bldc code, e.g. from [here](https://www.cs.cmu.edu/~410/doc/doxygen.html). 

Using the /\*\* … \*/ format will automatically make each part of the code collapsible, which will be useful when the code gets longer… Add headers to setup and loop, which appear to be an Aduino feature – probably will need to revert to standard main() to compile with Pico SDK.

Generally, I will take code from the Zilolg application function by function and add it to this file and test it.

Since the Pico example code already blinks an LED, I started by implementing the status LEDs of the Zilog application. 

**LEDs**

Add section for GPIO pin definitions and define LED\_YEL/GRN/RED as 2, 3 and 4 (0 and 1 are for the serial i/o, on the hardware Pico and, apparently on the simulation – although I could not get the input working – see below).

Add corresponding LEDs to the drawing observing the correct polarity.

Generally, there are three ways of accessing peripherals: Arduino functions, low-level register operations and Pico SDK LED functions. For the code to run on the hardware Pico, I avoid the Arduino functions. To simplify the code and reduce bugs I use the SDK functions were possible. However, it seems that not all of them have been implemented on the simulator. For these, I use the low-level register operations.

Add an init\_led() function, based on the Zilog version, that initialises the GPIO pins for use as outputs:

void init\_led(void) {                  		// need to set up pins before using them

`  `\_gpio\_init(LED\_YEL);             		// seem to need the “\_”for the simulator

`  `gpio\_set\_dir(LED\_YEL, GPIO\_OUT);

`  `…

}

Call the function from setup(). Replace the LED code in the main loop() and toggle the LEDs in some way for a test using, e.g.:

`  `gpio\_put(LED\_YEL, 1);			// yellow on

`  `gpio\_put(LED\_RED, 0);			// red off

`  `…

**Serial IO**

Set up serial communication and add a test message in setup():

`  `Serial1.begin(9600);               		// use uart

`  `printf("Welcome to PicoBLDC\n");

I used the Arduino Serial1 command, but it looks like the hardware Pico will not need this. I also started using C print commands and not the Arduino ones for future compatibility. Note that the “\n” at the end of each line appears to be necessary.

**Switches**

The Zilog application has a direction switch connected to a GPIO pin and a reset switch connected to the RESET/FAULT pin. I initially added a single switch to GPIO pin 5 in the GPIO definitions (SW\_DIR) and on the diagram. Again I ported the Ziliog init\_in() function and used the Pico SDK functions:

void init\_in(void) {

`  `\_gpio\_init(SW\_DIR);

`  `gpio\_set\_dir(SW\_DIR, GPIO\_IN);	// set dir switch to input

`  `gpio\_set\_pulls(SW\_DIR, 1, 0);		// set pullup

}

Call the function from setup(). Test the switch in some way in loop() using, e.g.:

`  `direction\_sw = gpio\_get(SW\_DIR);     // get dir switch value

`  `gpio\_put(LED\_GRN, direction\_sw);

This required adding the first of many global variables to a section after the GPIO definitions:

/\*\* -- Global variables -- \*/

// Direction Switch

unsigned char direction\_sw = 0;		// direction switch value

**A Timer**

[Class 2](https://www.youtube.com/watch?v=jbcqWcE3AAo&list=PL_tws4AXg7auiZHZsL-qfrXoMiUONBB0U&index=3) deals with timers and interrupts. Here I found that the Pico SDK functions were not supported by the simulator and so I looked at the low-level implementation.

The video starts by implementing a delay with a low-level checking of the Timer and then progresses to a setting and checking Alarm1. I started with this and added a function:

/\* -- Helper Functions -- \*/

void timer\_delay(uint32\_t us\_time) {

`  `ALARM1 = TIMELR + us\_time;        	// arm timer

`  `while (bitRead(ARMED, 1)) {

`    `// wait until alarm is disarmed

`  `}

}

and replaced the calls to the Arduino delay() by call to the new function (parameter now in micro Seconds).

This required adding the first register definition to a section after the GPIO definitions:

#define REG(address) (\*(volatile uint32\_t\*)(address))

#define TIMELR    REG(TIMER\_BASE+0x0c)

#define ALARM1    REG(TIMER\_BASE+0x14)

The REG macro is a standard way of accessing registers by name without having to use pointers. It uses the volatile keyword so that the compiler doesn’t optimise out seemingly redundant code that just writes to a register without reading it back. The names of the base values of the various groups of registers used in the Pico datasheet, i.e. TIMER\_BASE for the timer, appear to be already defined.


**Interrupts**

To enable an interrupt for an alarm we need the following steps (see also Section 4.6.3 of the datasheet):

- Define the exception/interrupt handler
- Enable the interrupt at the timer (INTE)
- Enable the appropriate timer interrupt (NVIC\_ISER)
- Set the target time to fire (ALARM1)
- Clear the interrupt after firing (INTR)

The first step replaces the timer\_delay() by an interrupt handler which also includes the last step:

void alarm\_isr(){

`  `ALARM1 = TIMELR + 250000;            	// arm timer

`  `GPIO\_OUT\_XOR = 1 << LED\_RED;  	// toggle red led for test

`  `INTR = 1 << ALARM\_INT\_NUM;        	// clear interrupt

}

The remaining steps are implemented in a init function that is called after the other init functions:

void init\_alarm() {

`  `REG(VTOR + (16 + TIMER\_IRQ\_1) \* 4) = (uint32\_t);  // interrupt handler

`  `INTE |= 1 << ALARM\_INT\_NUM;	// enable the interrupt at the timer

`  `NVIC\_ISER = 1 << TIMER\_IRQ\_1;	// enable the timer interrupt

`  `ALARM1 = TIMELR + 250000;            	// set time to fire

}

The flashing red led indicated a correct functioning of the interrupt.


**Pico SDK**

There is also a “Pico SDK” project option in the simulator. This solves the i/o reading problem. To convert to an SDK project, start a new project in the wokwi main page.

Cut and paste the existing code after the two includes in the project template. Move the includes down below the file header.

The required changes follow from the compiler errors:

- Remove the old timer\_delay function as it uses an Arduino function call
- Delete the underscores from the gpio\_init() calls
- Move the alarm\_isr() definition above init\_alarm() function
- Remove the call to Serial1 in the original setup() code

Now the code compiles. To make it work:

- Add the three LED and the switch definitions to the “parts” section of the diagram.json file. Also add the led and sw definitions to the “connections” section after the GP0 and GP1 definitions. Remember JSON is fussy with the commas – it they are wrong the diagram will go black.
- Move the code from the setup() function to the main() function after the stdio\_init\_all(); line. The interrupt should work and flash whatever test you set up.
- Move the code from the loop() function somewhere inside the while() loop in main(). Season to taste.


**Serial Monitor** 

With an SDK project, the c read function works with the simulator. This means we can add the UI code to control and debug the controller. The Zilog UI code is quite long but I copied everything inside the while() loop to the while() in the simulator code.

This actually compiled, because the UART constant is not defined so the UI code does not run. Define UART as 1 in the global constants. Now, the required changes follow from the compiler errors:

- Replace the first line with char ch = getchar();
- Added undefined ui\_ variables and speed variable to global variables and FWD/REF to constants
- Commented out lines using PWMxxx registers
- Added adc\_ variables to globals
- Removed spurious backslash and commented out call to ScanHex()

This compiled and ran, but with slightly strange logic. At this point I tidied up the code a bit and replaced all the else if statements with a switch() statement.

At this point, I added a display status function, called after the init functions, to be able to show the status of certain parameters and variables:

void display\_setup() {

`  `printf("SETUP PARAMETERS:\n");

`  `printf("%-16s: %x\n", "PPB\_BASE", PPB\_BASE);

`  `printf("%-16s: %x\n", "NVIC\_ISER", NVIC\_ISER);

}


**Motor Control Interrupts**

The Zilog application uses two interrupts:

- A constant 20kHz interrupt, pwm\_timer\_isr, triggered by the completion of a PWM cycle. This is used to call periodic controller and system functions, such as reading the switches, blinking the LEDs, reading the commanded and actual speed and controlling the pwm duty cycle depending on the speed error.
- A variable timer interrupt (back EMF Sensing service routine), t0\_intrp. This alternately commutes the motor and sets the new timer period depending on the speed in one period, and updates the speed according to the sensed back emf in the other period.

Each of these interrupts has an init function which I copied and commented out all the code. I renamed them init\_pwm() and init\_commute() to better reflect their function.

Since the PWM timer has a constant period, we can test it by modifying the timer/alarm code that we have already got working. So call init\_alarm() from init\_pwm() and change the interrupt handler to pwm\_isr. Add Zilog pwm\_timer\_isr() code and rename it as pwm\_isr. Call alarm\_isr() as the first statement. Define a system parameters section at the top of the file after the includes and move the definition of UART into it and a new constant PWM\_PERIOD as 50 to set the PWM frequency at 20kHz. Change the fixed timer values in init\_alarm() and alarm\_isr() to use PWM\_PERIOD instead of 250000.

Further changes follow from the compiler errors:

- Add count and step to the global variables as pwm\_count and pwm\_step
- Add count\_max as PWM\_COUNT\_MAX to the system parameters
- Add loop\_select to system parameters as LOOP\_TYPE
- Comment out all calls to service functions and the watchdog
- Delete code involving diagnostic Port\_B


**Blink Revisited**

One of the service functions called by pwm\_isr() is led\_blink(). This controls blinking the leds depending on the status of the system. Add the Zilog led\_blink() function above the pwm\_isr() to test the service loop.

Again proceeding by error messages:

- Add blink and blink\_count to global variables
- Comment out code involving the register PWMFSTAT
- Replace led function calls with calls to gpio\_put()

Now the yellow led should blink when the UI is enabled (with command “I”). To better remember the commands, rename this command to “U” for UI and the original “U” command to “H” for hardware control.

The blinking frequency should be: 1E6 / PWM\_PERIOD / PWM\_COUNT\_MAX / 800 = 1000000 / 50 / 50 / 800 = 0.5


With the PWM\_PERIOD at 50 the blinking seems to be 1/4 speed. To get the function calls in pwm\_isr at the right frequency, reduce PWM\_COUNT\_MAX to 12. In led\_blink() replace the 800 with the global constant BLINK\_MAX and the 400 with BLINK/MAX/2. Remove the red led toggle form alarm\_isr().


**Direction Switch**

Add code for direction\_update(). Replace switch read function with direction\_sw = gpio\_get(SW\_DIR); line from main(). Add direction\_count and direction to globals.


**ADC**

The application has a potentiometer that controls the speed of the motor. This is connected to an ADC and read in speed\_cmd(), called from the function loop.

To test the ADC copy the sdk hello\_adc.c code to main(). Connect a potentiometer to GPIO26 – only the signal line actually needs to be connected. Define the pin number 26 with a global constant POT\_SPEED and the ADC0 value with ADC\_SPEED (these should somehow be associated together later).

When this works copy in and call the init\_analog() code and move the adc\_init() and adc\_gpio\_init() commands into it. Check is still works.

Copy in the speed\_cmd() as get\_speed\_cmd() and enable the call from the function loop:

- Add the s\_loop\_count and speed\_cmd variables
- Add the S\_LOOP\_COUNT\_MAX and SPEED\_CMD\_MIN constants

Add a new UI option “D” to display the status of the system and print out the direction and the speed\_cmd values.

Move the adc\_select\_input() and adc\_read() calls into get\_speed\_cmd(). The value of speed\_cmd shown in the UI is always 50, i.e. SPEED\_CMD\_MIN. I commented out the test of the minimum value. The adc value was still wrong. I wasted some time and finally realised that it had something to do with the type definition of speed\_cmd, which was unsigned char (plus I was still redefining it again later as uint16\_t at the adc\_read()).

This raised the general question of data types on the RPC2040.





**Variables**

/[Programming in C/C++](https://raspberry-projects.com/pi/category/programming-in-c) / [Memory](https://raspberry-projects.com/pi/category/programming-in-c/memory) / Variables

I found this [table](https://raspberry-projects.com/pi/programming-in-c/memory/variables)

**For Raspberry Pi with a 32-bit processor**


|**Name**|**C++**|**Bytes**|**Range**|
| :- | :- | :- | :- |
|bool||1|true or false|
|signed char|int8\_t|1|-128 to 127|
|unsigned char|uint8\_t|1|0 to 255|
|short int|int16\_t|2|-32768 to 32767|
|unsigned short int|uint16\_t|2|0 to 65535|
|int|int32\_t|4|-2147483648 to 2147483647|
|unsigned int|uint32\_t|4|0 to 4294967295|
|long int|int32\_t|4|-2147483648 to 2147483647|
|unsigned long int|uint32\_t|4|0 to 4294967295|
|long long|int64\_t|8|−9,223,372,036,854,775,808 to 9,223,372,036,854,775,807|
|unsigned long long|uint64\_t|8|0 to 18,446,744,073,709,551,615|
|float||4|+/- 3.4e +/- 38 (~7 digits)|
|double||8|+/- 1.7e +/- 308 (~15 digits)|
|long double||8|+/- 1.7e +/- 308 (~15 digits)|
|wchar\_t||2 or 4|1 wide character|

So, unsigned char is 1 byte which is obviously not big enough for the 12-bit adc output. The Zilog code set the speed value to the ADC Data High Byte Register ADCD\_H which the Ziliog datasheet says is the upper eight bits of the 10-bit ADC output, i.e. 0-255. To get the same scale on the Pico the 12-bit value must be shifted right by 4 bits.

**PWM**

Now it’s time to bite the bullet and try to get the pulse width modulation working. I started from the sdk pwm\_fade.c example. I copied the pwm setup code into main() at the end of the init calls and the interrupt handler on\_pwm\_wrap() above main(). I added a red led to the diagram on gpio26 and changed the PICO\_DEFAULT\_LED\_PIN constant to PWM\_AL (phase A, High). Amazingly, this worked immediately.

In the application, it is the pwm interrupt that controls the function loop, so we need to replace the alarm interrupt with a pwm interrupt. I moved the pwm intitalisation code from main() to the end of init\_pwm and deleted the call to init\_alarm().

In a pwm, the interrupt frequency is equal to the clock frequency / the maximum counter value (wrap value). The duty cycle is equal to the level / wrap value. In the application, the interrupt frequency should be 20kHz. The duty cycle variable com\_mag is an 8-bit value and its maximum value, i.e. the wrap value, is COM\_MAG\_MAX = 250 (could be up to 255). So 20kHz = clock frequency / 255, which means that the clock frequency needs to be 5.1Mz. On the Pico it is 125Mz, so we need prescaler of 125/5.1 = 24.5, say 25.

Define COM\_MAG\_MIN, COM\_MAG\_MAX and PWM\_PRESCALER as constants. Declare the duty com\_mag in the globals.

Change the parameter in pwm\_config\_set\_clkdiv() to PWM\_PRESCALER. Set the wrap value with pwm\_config\_set\_wrap (&config, COM\_MAG\_MAX). Change the interrupt handler call to pwm\_isr.

In pwm\_isr(), replace the alarm\_isr() call by the pwm\_clr\_irq() from main(). Delete the remaining test code. The function loop should now run on the pwm interrupts with a status led blink frequency of about 1Hz.

This worked, but I noticed that, unfortunately, now the UI was outputting gibberish. After testing on a minimal project with just the sdk test pwm led flashing code and some printf commands, I found that the problem occurred as soon as the gpio pin was defined as 16, which has a slice and channel number of 0. This is the same as the pwm on gpio0, so maybe this was interfering with the uart on pin 0 which handles the terminal? I temporarily changed the led to gpio14, which has slice 7 and channel 0, and everything worked as expected.

To drive a bldc motor, we need three phases each with high and low signals. These need to be driven, commutated in a particular order. This needed to be tested carefully before connecting up to any hardware. The first thing is to add and test the two other phases and the three complementary signals. I thought that red, green and blue sounded good for the three phases and light and dark for the high and low sides. So I choose red for 1H, purple for 1L, dark green for 2H, light green for 2L, dark blue for 3H and light blue for 3L.





