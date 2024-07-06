/**
 * File:  Pro_Micro_periph_lib.h
 * `````
 * Peripheral function library for projects using the Sparkfun "Pro Micro" module,
 * based on Atmel ATmega32U4 MCU.  Also works for the Arduino 'Leonardo' dev board.
 *
 * Originated: April 2024  M.J.Bauer  [www.mjbauer.biz]
 * ```````````
 * This function library supports the following ATmega32U4 on-chip peripherals:
 * ````````````````````````````````````````````````````````````````````````````
 *    Timer/Counter_0 -- System timer functions, e.g. milliseconds();
 *    Timer/Counter_1 -- PWM outputs (OC1A, OC1B), 10-bit duty resolution @ 16kHz
 *    Timer/Counter_3 -- Variable-freq. PWM (OC3A), 16-bit freq. resolution
 *    Timer/Counter_4 -- PWM output (OC4D), e.g. 8-bit duty resolution @ 32kHz
 *    ADC -- Convert and read result on specified analog input
 *    EEPROM -- Read and write byte or array of bytes
 *    Flash Memory -- Access constant data stored in Program Memory space
 *    USART1 -- Send byte, check RX data available, receive byte, send text string
 */
#ifndef  PRO_MICRO_PERIPH_H     // Avoid multiple inclusion of this file
#define  PRO_MICRO_PERIPH_H

#include <avr/io.h>
#include <avr/interrupt.h>      // required for interrupt handlers
#include <avr/pgmspace.h>       // required for PGM_ReadData() function

// The application program may define symbol F_CPU = CPU clock frequency (Hz);
// otherwise the following default value (16 MHz) will be used...
#ifndef F_CPU
#define F_CPU  16000000UL
#endif

#include <util/delay.h>   // F_CPU must be defined for this module

#ifndef uint8
typedef signed char         int8;
typedef unsigned char       uint8;
typedef signed short        int16;
typedef unsigned short      uint16;
typedef signed long         int32;
typedef unsigned long       uint32;
#endif // uint8

#ifndef bool
typedef unsigned char       bool;
#endif

// Define Boolean values (if not already defined elsewhere)
#ifndef FALSE
#define FALSE   0
#define TRUE   (!FALSE)
#endif

#ifndef NULL
#define NULL ((void *) 0)  // NULL is a pointer to nowhere!
#endif

#ifndef PRIVATE
#define PRIVATE  static    // for module private functions
#endif

#ifndef HI_BYTE
#define HI_BYTE(w)  (((w) >> 8) & 0xFF)   // Extract high-order byte from unsigned word
#define LO_BYTE(w)  ((w) & 0xFF)          // Extract low-order byte from unsigned word
#endif

// Macros for byte/word/register (variable) bit manipulation...
// Examples: 1.  SET_BIT(PORTB, 5);           // Set PORTB register bit5 to 1
//           2.  if (TEST_BIT(PINC, 4)) ...   // TRUE if PINC bit4 is 1
// NB: TEST_BIT(x,n) evaluates to *any* non-zero value to indicate TRUE.
//
#ifndef CLEAR_BIT
#define TEST_BIT(var, bit)   ((var) & (1<<bit))
#define SET_BIT(var, bit)    ((var) |= (1<<bit))
#define CLEAR_BIT(var, bit)  ((var) &= ~(1<<bit))
#endif

// Swap bytes in a 16-bit word...
#define SWAP(w)    ((((w) & 0xFF) << 8) | (((w) >> 8) & 0xFF))

// Essential keyword that K & R omitted:
#define until(expr)  while(!(expr))   // Usage:  do { ... } until (expr);

// Macros to enable & disable global interrupts (all interrupt sources)
#ifndef GLOBAL_INT_ENABLE
#define GLOBAL_INT_ENABLE()    sei()
#define GLOBAL_INT_DISABLE()   cli()
#endif

// Macros to control on-board TX and RX LEDs
#define TX_LED_CONFIGURE()  SET_BIT(DDRD, 5)
#define TX_LED_TURN_OFF()   SET_BIT(PORTD, 5)
#define TX_LED_TURN_ON()    CLEAR_BIT(PORTD, 5)
#define RX_LED_CONFIGURE()  SET_BIT(DDRB, 0)  
#define RX_LED_TURN_OFF()   SET_BIT(PORTB, 0)
#define RX_LED_TURN_ON()    CLEAR_BIT(PORTB, 0)

// Macros to control Timer-Counter TC0 output compare interrupt
#define TC0_OCA_IRQ_ENABLE()    (TIMSK0 |= (1<<OCIE0A))
#define TC0_OCA_IRQ_DISABLE()   (TIMSK0 &= ~(1<<OCIE0A))

// Macros to control Timer-Counter TC1 output compare interrupt
#define TC1_OCA_IRQ_ENABLE()   (TIMSK1 |= (1<<OCIE1A))
#define TC1_OCA_IRQ_DISABLE()  (TIMSK1 &= ~(1<<OCIE1A))
#define TC1_OCB_IRQ_ENABLE()   (TIMSK1 |= (1<<OCIE1B))
#define TC1_OCB_IRQ_DISABLE()  (TIMSK1 &= ~(1<<OCIE1B))

// Macros to control Timer-Counter TC3 output compare interrupt
#define TC3_OCA_IRQ_ENABLE()   (TIMSK3 |= (1<<OCIE3A))
#define TC3_OCA_IRQ_DISABLE()  (TIMSK3 &= ~(1<<OCIE3A))

// Macros to control Timer-Counter TC3 overflow interrupt
#define TC3_OVF_IRQ_ENABLE()   (TIMSK3 |= (1<<TOIE3))
#define TC3_OVF_IRQ_DISABLE()  (TIMSK3 &= ~(1<<TOIE3))

// Macros to control Timer-Counter TC4 output compare interrupt
#define TC4_OCD_IRQ_ENABLE()   (TIMSK4 |= (1<<OCIE4D))
#define TC4_OCD_IRQ_DISABLE()  (TIMSK4 &= ~(1<<OCIE4D))

#define SREG  _SFR_IO8(0x3F)   // AVR CPU status register

/**
* Macro to delay a specified number of CPU clock cycles (nc) using Timer/Counter_4
* (assuming TC4 is not being used for any other purpose!)
* Delay time = nc/16 us (*approx*)  Maximum delay is 256 clocks => 16 microseconds.
* Minimum delay is 1 clock => 1 us (due to overhead in while-loop exit test, etc).
* Precise argument values:  nc = 12 for 2us, nc = 62 for 5us, nc = 142 for 10us.
* For time-critical delays, global interrupts must be disabled during execution.
*/
#define Delay_CPU_cycles(nc) { \
    TCNT4 = 0; \
    TCCR4B = 0x01; \
    while (TCNT4 < nc) {;} \
    TCCR4B = 0x00;  }     


// ==================  S Y S T E M   T I M E R   F U N C T I O N S  =====================
/* 
 * Function:  TC0_Setup_SystemTimer()
 *
 * This function initializes Timer-Counter TC0 to generate a periodic interrupt
 * request (IRQ) once every millisecond, precisely.  The TC0 ISR supports timer
 * function milliseconds().  It also supports a primitive task scheduling scheme for
 * tasks (application functions) which need to be executed periodically at various 
 * intervals, i.e. 5, 50, 200 and 500 milliseconds. [ See isTaskPending_*ms() ]
 */
void  TC0_Setup_SystemTimer();

/*
 * Function:  milliseconds()
 *
 * This function returns the value of a free-running 32-bit counter variable,
 * incremented once every millisecond by Timer TC0 interrupt handler (ISR).
 * It's purpose is to implement "non-blocking" time delays and event timers.
 */
unsigned long milliseconds();

/*
 * Functions:  isTaskPending_??ms()
 *
 * The application may call one or more of these functions to check if a corresponding
 * periodic task is due to be executed.
 *
 * These functions return TRUE if their respective periodic Task Flag is raised;
 * otherwise they return FALSE.  The Task Flag is cleared before the function exits,
 * so that on subsequent calls it will return FALSE, until the task period expires.
 */
bool  isTaskPending_5ms();
bool  isTaskPending_50ms();
bool  isTaskPending_200ms();
bool  isTaskPending_500ms();


// ====================  U S E R   T I M E R   F U N C T I O N S  =======================
/*
 * Function:  TC1_Setup_PWM()
 *
 * This function initializes Timer-Counter TC1 in PWM mode to generate two independent 
 * variable-duty pulse waveforms on pins OC1A and OC1B.  The PWM "carrier" frequency and
 * duty resolution are the same on each of the 2 'output compare' channels (A and B).
 *
 * Entry arg:  option -- number specifying timer configuration, as follows:
 *
 *   Option 0:   9-bit duty resolution, 32kHz carrier freq, Tclk = 0.0625 usec
 *   Option 1:  10-bit duty resolution, 16kHz carrier freq, Tclk = 0.0625 usec
 *   Option 2:  12-bit duty resolution, 4kHz  carrier freq, Tclk = 0.0625 usec
 *   Option 3:  14-bit duty resolution, 1kHz  carrier freq, Tclk = 0.0625 usec
 */
void  TC1_Setup_PWM(uint8 config_option);

/*
 * Function:  TC1_OC1A_Update_Duty() 
 *
 * Set PWM duty on pin OC1A.
 *
 * Entry arg:  duty_clks = duty value, unit = TC1 clock
 *
 * Example:  If the duty resolution is 10 bits, the range of duty_clks is 0..998.
 *           At 50% duty, duty_clks = 500;  at 99.9% duty, duty_clks = 998.
 *           <!> Do not set duty_clks = TOP count, i.e. 999 in this example.
 */
void  TC1_OC1A_UpdateDuty(unsigned duty_clks);

/*
 * Function:  TC1_OC1B_Update_Duty()
 *
 * Set PWM duty on pin OC1B.
 *
 * Entry arg:  duty_clks = duty value, unit = TC1 clock.  (See example above)
 */
void  TC1_OC1B_UpdateDuty(unsigned duty_clks);

/*
 * Function:  TC3_Setup_PWM()
 *
 * This function initializes Timer-Counter TC3 in PWM mode to generate a variable-duty
 * pulse waveform on pin OC3A.  
 *
 * Entry arg:  option -- number specifying timer configuration, as follows:
 *
 *   Option 0:  10-bit duty resolution, 16kHz carrier freq, Tclk = 0.0625 usec
 *   Option 1:  12-bit duty resolution, 4kHz  carrier freq, Tclk = 0.0625 usec
 */
void  TC3_Setup_PWM(uint8 option);

/*
 * Function:  TC3_OC3A_Update_Duty()
 *
 * Set PWM duty on pin OC3A. 
 *
 * Entry arg:  duty_clks = duty value, unit = TC3 clock period
 *
 * Example:  If the duty resolution is 12 bits, the range of duty_clks is 0..3998.
 *           At 50% duty, duty_clks = 2000;  at 99.9% duty, duty_clks = 3998.
 *           <!> Do not set duty_clks = TOP count, i.e. 3999 in this example.
 */
void  TC3_OC3A_UpdateDuty(unsigned duty_clks);

/*
 * Function:  TC3_Setup_PulseGen()
 *
 * This function initializes Timer-Counter TC3 in pulse generator mode with variable
 * frequency and fixed pulse-width output on pin OC3A. The pulse width (duty) may be
 * changed anytime by calling function TC3_OC3A_UpdateDuty().
 *
 * Entry args:  1. polarity  -- To set output polarity (0: High-going, 1: Low-going pulse)
 *              2. prescaler -- Prescaler value, 3 bit value written to TCCR3B bits[2:0]
 *                 Timer Fclk = CPU Fosc / N;  Timer Tclk = CPU Tosc x N
 *                 (prescaler = 1: N = 1;  2: N = 8;  3: N = 64;  4: N = 256;  5: N = 1024)
 *              3. duty_clks -- Initial pulse width (duty), unit = Timer clock period.
 */
void  TC3_Setup_PulseGen(uint8 polarity, uint8 prescaler, unsigned duty_clks);

/*
 * Function:  TC3_OC3A_UpdatePeriod()
 *
 * Set output pulse period on pin OC3A.
 *
 * Entry arg:  period_clks = output pulse period, unit = TC3 clock.
 *
 * Note:  Arg1 (period_clks) must be greater than the preset duty value (duty_clks).
 */
void  TC3_OC3A_UpdatePeriod(unsigned period_clks);


/*
 * Function:  TC4_Setup_PWM()
 *
 * This function initializes Timer-Counter TC4 in fast PWM mode to generate a variable-duty
 * pulse waveform on pin PD7/OC4D.
 *
 * Entry arg:  option -- number specifying timer configuration, as follows:
 *
 *   Option 0:   8-bit duty resolution, 32kHz carrier freq, Tclk = 0.125 usec
 *   Option 1:  10-bit duty resolution, 16kHz carrier freq, Tclk = 0.0625 usec
 */
void  TC4_Setup_PWM(uint8 option);

/*
 * Function:  TC4_OC4D_Update_Duty()
 *
 * Set PWM duty on pin OC4D. 
 *
 * Entry arg:  duty_clks = duty value, unit = TC4 clock source
 *
 * Example:  If the duty resolution is 10 bits, the range of duty_clks is 0..999.
 *           At 50% duty, duty_clks = 500;  at 100% duty, duty_clks = 999.
 */
void  TC4_OC4D_UpdateDuty(unsigned duty_clks);


//==========================   A D C   F U N C T I O N   ================================
/*
 * Function ADC_ReadInput() starts a one-off conversion on the given input, waits for
 * the conversion cycle to complete, then returns the 10-bit result.
 *
 * Entry arg:  muxsel = ADC MUX input select:  0 = ADC0, 1 = ADC1,... 8 = ADC8, etc.
 *                      (ADC14 = 1.1V internal ref, ADC15 = GND/0V)   [see Note 2]
 *
 * Note 1:  The function assumes the selected ADC port pin is already configured as an
 *          input and that its internal pull-up resistor is disabled.
 *
 * Note 2:  The 'Pro Micro' module analog input pin labels are misleading...
 *          'A0' is AT_32U4 pin PF7/ADC7, 'A1' is pin PF6/ADC6, 'A2' is pin PF5/ADC5,
 *          'A3' is AT_32U4 pin PF4/ADC4, 'A6' is pin PD4/ADC8, 'A8' is pin PB4/ADC11.
 */
unsigned  ADC_ReadInput(uint8 muxsel);


//========================   E E P R O M    F U N C T I O N S  ==========================
/*
 * Function:  EEPROM_ReadByte()
 *
 * Returns a single byte read from EEPROM at the specified address (arg1).
 */
uint8  EEPROM_ReadByte(unsigned address);

/*
 * Function:  EEPROM_WriteByte()
 *
 * Writes a single byte (arg2) into EEPROM at the specified address (arg1).
 */
void  EEPROM_WriteByte(unsigned address, uint8 bDat);

/*
 * Function:  EEPROM_ReadArray()
 *
 * Copies a number of consecutive bytes (nbytes) from EEPROM, beginning at addr = 0,
 * into a data memory (SRAM) array (pdat). The byte count (nbytes) must not exceed 1024.
 */
void  EEPROM_ReadArray(uint8 *pdat, int nbytes);

/*
 * Function:  EEPROM_WriteArray
 *
 * Copies a number of consecutive bytes (nbytes) into EEPROM, beginning at addr = 0,
 * from a data memory (SRAM) array (pdat). The byte count (nbytes) must not exceed 1024.
 */    
void  EEPROM_WriteArray(uint8 *pdat, int nbytes);


//==========   F L A S H   M E M O R Y   D A T A   R E A D   F U N C T I O N   ==========
/* 
 * Function:  PGM_ReadData()
 *
 * Copy data from flash program memory to an array in data memory (SRAM).
 *
 * Entry args:  src    =  address of source data in program memory
 *              dest   =  address of destination array in data memory
 *              nbytes =  number of bytes to be copied
 * 
 * The macro pgm_read_byte() is defined in header file avr/pgmspace.h
 * which must be #included in source files which call this function.
 *
 * Flash memory data is declared and initialized as in this example:
 * const char array_name[] PROGMEM = { k0, k1, k2, ... kn };
 */
void  PGM_ReadData(const uint8 *src, uint8 *dest, unsigned nbytes);


//===========================   U A R T    F U N C T I O N S  ===========================
//
// Comment out next line if UART RX is NOT interrupt-driven, i.e. RX status is polled:
// #define UART_RX_INTERRUPT_DRIVEN  
//
// If user application requires UART data reception to be interrupt-driven, the program
// can define a "call-back" function named UART_RX_IRQ_Handler() to read received data
// bytes from the UART and store the RX data in a (circular) FIFO buffer in MCU SRAM.
// Note: The library code must be re-compiled (rebuilt) to enable this function.
//
// Macros to enable and disable UART receiver interrupt requests:
#define UART_IRQ_ENABLE()    SET_BIT(UCSR1B, RXCIE1)
#define UART_IRQ_DISABLE()   CLEAR_BIT(UCSR1B, RXCIE1)
//
// Macros to check TX and RX status:
#define UART_RxDataAvail()   (TEST_BIT(UCSR1A, RXC1) != 0)
#define UART_TX_Ready()      (TEST_BIT(UCSR1A, UDRE1) != 0)
//
#ifdef UART_RX_INTERRUPT_DRIVEN
extern void  UART_RX_IRQ_Handler( void );
#endif

/*
 * Function:  UART_init()
 *
 * Initialise USART1 for asynchronous RX and TX with the specified baudrate.
 * Must be called by the application program before using USART1.
 *
 * Entry arg (baudrate) is bits per second (e.g. 300, 1200, 9600, 19200, 38400, 57600).
 * Baudrates higher than 38400 are not recommended if CPU clock is internal RC osc.
 */
void  UART_init(unsigned baudrate);

/*
 * Function:  UART_GetByte()
 *
 * Fetch next available byte from USART0 RX buffer.
 *
 * The function DOES NOT WAIT for data available in the input buffer;
 * the caller must first check using the macro UART_RxDataAvail().
 * If there is no data available, the function returns NUL (0).
 * The input char is NOT echoed back to the UART output stream.
 *
 * Returns:  Byte from USART0 RX buffer (or 0, if buffer is empty).
 */
uint8  UART_GetByte(void);    

/*
 * Function:  UART_PutByte()
 *
 * Transmit a single byte.
 * Writes a data byte into the UART TX Data register to be transmitted.
 * The function waits for the TX register to be cleared first.
 */
void  UART_PutByte(uint8 b);

/*
*   Function:  UART_PutString()
*
*   Transmit a NUL-terminated string.
*   Newline (ASCII 0x0A) is expanded to CR + LF (0x0D + 0x0A).
*
*   Note: This function delays the calling task by whatever time it takes to
*   transmit the string (= strlen(s) x 10000 / baudrate;  msec. approx.)
*/
void  UART_PutString(char *str);


#endif  //  PRO_MICRO_PERIPH_H
