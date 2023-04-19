/**
 * File:  lib_avrXmini.h
 *
 * Definitions and declarations for the AVR "X-mini" (ATmega328P) Function Library.
 * #include this file in every C source file in the project.
 *
 * v1.0  2020-03-28  M.J.Bauer  [www.mjbauer.biz]
 * _________________________________________________________________________________
 * The AVR-Pad X-mini library supports the following peripherals:
 * `````````````````````````````````````````````````````````````````````````````````
 *   - Timer-Counter TC1  |  milliseconds timer, real-time clock, task scheduler
 *   - LCD Module 1602A   |  init, write command, write character, print string
 *   - UART0              |  init, check RX data, get byte, send byte, send string
 *   - EEPROM             |  read byte, write byte, read array, write array
 *   - ADC                |  one-off conversion, 10-bit result (ADC1..ADC5)
 *   - Push-buttons       |  button scan routine, detect hits, get button status
 *   - Keypad 4x4 matrix  |  kaypad scan routine, detect hits, get key-code of hit
 * _________________________________________________________________________________
 *
 * Peripheral connections to suit library functions:
 * `````````````````````````````````````````````````
 *     Button [A] ---> PD2     PC2 ---> RS   |
 *     Button [B] ---> PD3     PC1 ---> R/W  |
 *     Button [C] ---> PD4     PC0 ---> E    |
 *     Button [D] ---> PD5                   | LCD Module
 *                             PB3 <--> DB7  |  (1602A)
 *                             PB2 <--> DB6  |
 *     USB-CDC RX <--- PD1     PB1 <--> DB5  |
 *     USB-CDC TX ---> PD0     PB0 <--> DB4  |
 *
 * Keypad wiring:  Refer to diagram: "AVR-X-mini-Peripheral-Wiring.pdf"
 * ````````````````````````````````````````````````````````````````````
 * The library allows peripherals which share common I/O pins to operate "virtually
 * concurrently". This is achieved by "time sharing" of MCU resources.
 *
 * Library functions must not be called from an interrupt service routine (ISR).
 * Otherwise, contention of resources can occur.
 *
 * Up to 6 LEDs connected to Port B (PB0..PB5) may be driven by writing directly to
 * the PORTB register. Other peripherals which share Port B (e.g. LCD, keypad) will
 * not disrupt the LED display, provided that the other functions are not called
 * excessively frequently. (See notes elsewhere on LCD and keypad operation.)
 *
 * Pins PC1, PC2 & PC3 may be used for analogue (ADC) input. PC4 and PC5 may also be
 * used as ADC inputs (or GPIO), provided the IIC peripheral bus is not needed.
 */

#ifndef  _AVRBED_LIB_H    // Avoid multiple inclusion of this file
#define  _AVRBED_LIB_H

#include <avr/io.h>
#include <avr/interrupt.h>      // required for interrupt handlers

// The application program should define symbol F_CPU = CPU clock frequency (Hz);
// otherwise the following default value (16 MHz) will be used...
#ifndef F_CPU
#define F_CPU  16000000UL
#endif

#include <util/delay.h>   // F_CPU must be defined for this module

// Define Boolean values (if not already defined elsewhere)
#ifndef FALSE
#define FALSE   0
#define TRUE   (!0)   // TRUE is *any* non-zero value! (Let compiler decide.)
#endif

#ifndef BOOL
#define BOOL  unsigned char
#endif

#ifndef BYTE
#define BYTE  unsigned char
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

// Macros to enable & disable Timer-Counter TC1 Output Compare interrupt
#define TC1_OCA_IRQ_ENABLE()   (TIMSK1 |= (1<<OCIE1A))
#define TC1_OCA_IRQ_DISABLE()  (TIMSK1 &= ~(1<<OCIE1A))

#define SREG  _SFR_IO8(0x3F)   // AVR CPU status register


// Real-Time Clock (RTC) global variables, updated automatically by Timer_1 ISR
//
extern char g_rtcHour, g_rtcMinutes, g_rtcSeconds;


/* ======================  T I M E R   F U N C T I O N S  ===============================
 *
 * The timer library uses a real-time interrupt generated by Timer TC1 to provide
 * a time-base for a real-time clock and a general-purpose millisecond timer.
 *
 * The ISR also incorporates a primitive task scheduler which sets 'Task Flags' at
 * periodic intervals of 5ms, 50ms and 500ms.  These flags may be accessed by the
 * application program to invoke periodic tasks.  See function: isTaskPending_5ms().
 * --------------------------------------------------------------------------------------
 */
// Alias for TC1_initialize() in case you prefer the correct spelling...
#define TC1_initialise()  TC1_initialize()

/*
 * Function:  TC1_initialize()
 *
 * This function initializes Timer-Counter TC1 to generate a periodic interrupt
 * request (IRQ) every millisecond precisely.
 *
 * The timer "Output Compare" feature is used to make the timing automatic.
 * When the counter register (TMR1) reaches the "TOP count" (OCR1A value), the
 * count register is automatically reset (zeroed) and an IRQ flag is raised.
 *
 * The timer clock pre-scaler is set to divide the CPU clock frequency by 8.
 * Assuming the CPU clock freq. is 8 MHz, the counter clock will be 1 MHz.
 *
 * TC1_initialize() must be called from main() before enabling global interrupts.
 * Global interrupts must be enabled for the timer functions to work.
 */
void  TC1_initialize();

/*
 * Function:  milliseconds()
 *
 * This function returns the value of a free-running 32-bit counter variable,
 * incremented every millisecond by Timer/Counter TC1 interrupt handler (ISR).
 * It's purpose is to implement "non-blocking" time delays and event timers.
 * Typical usage:
 *
 *    static unsigned long eventStartTime;
 *                  :
 *    eventStartTime = milliseconds();  // capture the starting time
 *                  :
 *    if (milliseconds() >= (eventStartTime + EVENT_DURATION))  // time's up!
 *    {
 *        // Do what needs to be done TIME_DURATION ms after eventStartTime
 *    }
 *
 * A program can implement many independent event timers, simply by declaring
 * a unique eventStartTime (variable) and a unique EVENT_DURATION (constant)
 * for each independent "event" or delay to be timed.
 *
 * Be sure to declare each eventStartTime as 'static' (permanent) so that it
 * will be kept between multiple calls to the function in which it is defined.
 */
unsigned long  milliseconds();

/*
 * Function:  isTaskPending_??ms()
 *
 * These three functions return TRUE if their respective Task Flag is raised;
 * otherwise they return FALSE.  The Task Flag is cleared before the function exits,
 * so that on subsequent calls it will return FALSE, until the next task period ends.
 */
BOOL  isTaskPending_5ms();

BOOL  isTaskPending_50ms();

BOOL  isTaskPending_500ms();


//==========================   L C D   F U N C T I O N S  ===============================
//
// LCD Controller Command bytes
//
#define LCD_OFF             0b00001000  // D=0 (off), C=0 (cursor off), B=0 (no blink)
#define LCD_CLR             0b00000001  // Clears entire display
#define LCD_HOME            0b00000010  // Return cursor to home posn, DDRAM addr = 0
#define LCD_EM_INC          0b00000110  // Increment cursor position, no display shift
#define LCD_CURSOR_OFF      0b00001100  // Display ON, cursor OFF (hidden)
#define LCD_ON              0b00001110  // Display ON, cursor ON, B=0 (no blink)
#define LCD_CGRAM_ADDR      0b01000000  // Set CGRAM address

// Macro to set cursor position to a given row and column,
// where row is 0 (top line) or 1 (bottom line),  col is 0..15
#define lcd_cursor_posn(row, col)  lcd_command(0x80 + (row * 0x40) + col)

// Alias for lcd_initialise(), for legacy AVR-Pad library compatibility
#define initialise_LCD()  lcd_initialise()

/*
 * Initialise the LCD controller (HD44780)...
 * LCD mode is set to: 2 lines, char 5x8 dots, cursor on
 */
void  lcd_initialise(void);

/*
 * Output a command byte to the LCD controller.
 * Refer to command definitions (macros) above.
 *
 * Entry arg:  cmd = LCD command code (byte)
 */
void  lcd_command( BYTE cmd );

/*
 * Display a single ASCII character.
 * Before calling this function, set cursor position using macro:
 *    lcd_cursor_posn(row, col);
 * where row is 0 (top line) or 1 (bottom line),  col is 0..15
 * (except if the cursor is already in the required position).
 *
 * The cursor position will be advanced one place to the right on exit.
 *
 * Entry arg:  c = ASCII character code (printable)
 */
void  lcd_write_char( char c );

/*
 * Function to display a NUL-terminated string.
 * Before calling this function, set cursor position using
 *    lcd_cursor_posn(row, col)
 * where row is 0 (top line) or 1 (bottom line),  col is 0..15,
 * (except if the cursor is already in the required position).
 * The cursor position will be advanced N places to the right on exit,
 * where N = number of characters in the input string (not incl. NUL terminator).
 *
 * Entry arg:  str = address of string (constant or variable)
 *
 * Usage examples:  lcd_print_string("Hello, world.");  // string constant
 *                  lcd_print_string(buff);  // where buff is an array of chars
 */
void  lcd_print_string( char *str );


//===================   P U S H - B U T T O N   F U N C T I O N S  ======================
/*
 * Function ButtonScan() must be called periodically from the application program
 * (main loop) at intervals of about 50ms for reliable "de-bounce" operation.
 *
 * It's main purpose is to detect "button hit" events, i.e. transition from "no button
 * pressed" to "button pressed" and to raise a status flag to signal the event.
 *
 * The entry argument (nButts) specifies the number of buttons (1..4) to be serviced.
 * For example, if nButts is 1, only Button_A is serviced;  if nButts is 3, then 3
 * buttons (Button_A, Button_B and Button_C) will be serviced by the scan routine.
 *
 * Note: The function (re-)configures the required port D pins (PD2..PD5) as inputs;
 *       and enables pull-ups on respective pins;  all other I/O pins are unaffected.
 */
void  ButtonScan(unsigned char nButts);

/*
 * Function button_hit() returns the Boolean value (TRUE or FALSE) of a flag indicating
 * whether or not a "button hit" event occurred since the previous call to the function.
 *
 * Entry argument 'button_ID' is an ASCII code identifying one of 4 buttons to check,
 * which must be one of: 'A', 'B', 'C' or 'D', otherwise the function will return FALSE.
 * If the given button is not serviced by ButtonScan(), button_hit() will return FALSE.
 *
 * The flag (internal static variable) is cleared "automatically" by the function so that
 * on subsequent calls the function will return FALSE (until the next button hit occurs).
 */
BOOL  button_hit(char  button_ID);

/*
 * Function button_pressed() returns the Boolean value (TRUE or FALSE) of a flag telling
 * whether or not a given button is currently pressed, i.e. held down.
 *
 * Entry argument 'button_ID' is an ASCII code identifying one of 4 buttons to check,
 * which must be one of: 'A', 'B', 'C' or 'D', otherwise the function will return FALSE.
 * If the given button is not serviced, button_pressed() will return FALSE.
 */
BOOL  button_pressed(char  button_ID);


//==========================   A D C   F U N C T I O N   ================================
/*
 * Function ADC_ReadInput() starts a one-off conversion on the given input, waits for
 * the conversion cycle to complete, then returns the 10-bit result.
 *
 * Entry arg:  muxsel = ADC MUX input select:  1 = ADC1, 2 = ADC2, ... 5 = ADC5
 *             (ADC0, ADC6..13 N/A, 14 = 1.1V internal ref, 15 = GND/0V)
 *
 * Note:  The function assumes the selected ADC port pin is already configured as an
 *        input and that its internal pull-up resistor is disabled.
 */
unsigned  ADC_ReadInput( BYTE muxsel );


//========================   E E P R O M    F U N C T I O N S  ==========================
/*
* Function to read a single byte from EEPROM at the given address
* (arg1), range:  0 <= addr <= 1023  (ATmega328P)
*/
BYTE  EEPROM_ReadByte(unsigned addr);

/*
* Function to write a single byte to EEPROM at the given address
* (arg1), range:  0 <= addr <= 1023  (ATmega328P)
*/
void  EEPROM_WriteByte(unsigned addr, BYTE bDat);

/*
* Function to read a specified number of bytes (nbytes) from EEPROM
* and to copy them to an array in data memory (pdata).
* Example of usage:
* `````````````````
*     unsigned char  dest_array[64];   // ensure size >= nbytes
*         :
*         :
*     EEPROM_ReadArray(dest_array, 40);
*/
void  EEPROM_ReadArray(BYTE *pdata, int nbytes);

/*
* Function to write a specified number of bytes (nbytes) into EEPROM,
* copied from an array in data memory (pdata).
* Example of usage:
* `````````````````
*     unsigned char  source_array[64];   // ensure size >= nbytes
*         :
*         :
*     EEPROM_WriteArray(source_array, 40);
*/
void  EEPROM_WriteArray(BYTE *pdata, int nbytes);


//==========================   U A R T   F U N C T I O N S  =============================
//
// Macros to enable and disable UART receiver interrupt requests:
#define UART_IRQ_ENABLE()    SET_BIT(UCSR0B, RXCIE0)
#define UART_IRQ_DISABLE()   CLEAR_BIT(UCSR0B, RXCIE0)

// Macros to check TX and RX status:
#define UART_RxDataAvail()   (TEST_BIT(UCSR0A, RXC0) != 0)
#define UART_TX_Ready()      (TEST_BIT(UCSR0A, UDRE0) != 0)

/*
*   UART_init() ...
*   Initialise USART0 for RX and TX with the specified baudrate.
*   Must be called by the application program before using USART0.
*
*   Entry arg (baudrate) is bits per second (e.g. 300, 1200, 9600, 38400, 57600).
*   Note:  F_CPU must be defined correctly to obtain the specified baud rate.
*/
void  UART_init( unsigned baudrate );

/*
*   UART_GetByte() ...
*   Fetch next byte from USART0 RX buffer.
*   The function DOES NOT WAIT for data available in the input buffer;
*   the caller must first check using the macro UART_RxDataAvail().
*   If there is no data available, the function returns NUL (0).
*   The input char is NOT echoed back to the UART output stream.
*
*   Returns:  Byte from USART0 RX buffer (or 0, if buffer is empty).
*/
BYTE  UART_GetByte( void );

/*
*   UART_PutByte() ... UART Transmit Byte.
*   Writes a data byte into the UART TX Data register to be transmitted.
*   The function waits for the TX register to be cleared first.
*/
void  UART_PutByte( BYTE b );

/*
*   UART_PutString() ...  Transmit a NUL-terminated string.
*   Newline (ASCII 0x0A) is expanded to CR + LF (0x0D + 0x0A).
*   Note: This function delays the calling task by whatever time it takes to
*   transmit the string (= strlen(s) x 10000 / baudrate;  msec. approx.)
*/
void  UART_PutString( char *str );

/*
*   If the UART receiver is interrupt-driven, the application program must
*   provide an interrupt "call-back" function named: UART_RX_IRQ_Handler().
*
*   The IRQ signals that a byte has been received by the UART;
*   the byte must be read out of the UART RX data register) and stored in a
*   buffer in RAM (circular FIFO) by this function.
*/
#ifdef UART_RX_INTERRUPT_DRIVEN
extern void  UART_RX_IRQ_Handler( void );
#endif


//========================   K E Y P A D    F U N C T I O N S  ==========================
/*
 * Function KeypadScan() must be called periodically from the application program
 * (main loop) at intervals of about 50ms for reliable de-bounce operation.
 * The function assumes only one key at a time (or none) is pressed.
 * Its purpose is to detect a "key hit" event, i.e. a transition from "no key pressed"
 * to "any key pressed" and to record the key code of the last pressed key.
 *
 * Note: The function exits with all port B pins set as outputs,
 *       port D pinsPD2..PD5 set as inputs,  all other I/O pins unaffected.
 */
void  KeypadScan(void);

/*
 * Function key_hit() returns the Boolean value (TRUE or FALSE) of a flag indicating
 * whether or not a "key hit" event has occurred since the previous call to the function.
 * The flag (internal static variable) is cleared "automatically" by the function so that
 * on subsequent calls the function will return FALSE (until the next key hit).
 */
BOOL  key_hit(void);

/*
 * Function key_code() returns the ASCII character code corresponding to the key last
 * pressed, i.e. when the last "key hit" event was detected.
 * Note: The ASCII codes for digits '0' to '9' are 0x30 to 0x39 (resp).
 */
char  key_code(void);


#endif  //  _AVRBED_LIB_H

