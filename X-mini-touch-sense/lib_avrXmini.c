/**
 * File:  lib_avrXmini.c  |  AVR-Pad X-mini (ATmega328P) Function Library
 *
 * Revision History:
 * v1.0  2020-03-27  M.J.Bauer  [www.mjbauer.biz]
 *
 * For full descriptions of functions, see header file: lib_avrXmini.h.
 */
#include <avr/io.h>
#include "lib_avrXmini.h"


// ------------  Global variables  --------------------------------------------
//
char  g_rtcHour, g_rtcMinutes, g_rtcSeconds;  // Real-Time Clock "registers"


// ------------  Private static variables  ------------------------------------
static unsigned long  count_millisecs;  // for function milliseconds()
static volatile char  TaskFlag_5ms;     // Flag raised every 5ms
static volatile char  TaskFlag_50ms;    // Flag raised every 50ms
static volatile char  TaskFlag_500ms;   // Flag raised every 500ms


/* ======================  T I M E R   F U N C T I O N S  =========================
 *
 * Function:  TC1_initialize()
 *
 * This function initializes Timer-Counter TC1 to generate a periodic interrupt
 * request (IRQ) every millisecond precisely.
 *
 * TC1_initialize() must be called from main() before enabling global interrupts.
 * Global interrupts must be enabled for the timer functions to work.
 */
void  TC1_initialize()
{
    unsigned  top_count = (unsigned long) F_CPU / 8000;

    TCCR1A = 0x00;
    TCCR1B = 0b00001010;            // CTC mode;  Prescaler = F_CPU / 8

    OCR1AH = HI_BYTE(top_count);    // Load OCR1A register for 1ms Top count
    OCR1AL = LO_BYTE(top_count);

    TC1_OCA_IRQ_ENABLE();           // Enable Output Compare (A) interrupts
}


/*
 * Function:  milliseconds()
 *
 * This function returns the value of a free-running 32-bit count variable,
 * incremented every millisecond by Timer TC1 interrupt handler (ISR, below).
 * It's purpose is to implement "non-blocking" time delays and event timers.
 */
unsigned long milliseconds()
{
    unsigned long temp32bits;

    // Disable TC1 interrupt to prevent corruption of count_millisecs in case
    // interrupted here in the middle of copying the 4 bytes (32 bits)...
    TC1_OCA_IRQ_DISABLE();

    temp32bits = count_millisecs;  // capture the count value (4 bytes)

    // Re-enable TC1 interrupt
    TC1_OCA_IRQ_ENABLE();

    return  temp32bits;
}


/*
 * Timer-Counter TC1 Interrupt Service Routine
 *
 * The timer will generate an IRQ, hence this routine will be executed, when the
 * timer value (TMR1) reaches the 'Top Count" (= OCR1A register value).
 * The timer will be reset (TMR1 = 0) automatically when this occurs.
 */
ISR( TIMER1_COMPA_vect )
{
    static short  counter_ms;   // count milliseconds (local)

    count_millisecs++;   // counter used for function milliseconds()

    // Periodic task scheduler --
    //
    if (counter_ms % 5 == 0)  TaskFlag_5ms = 1;
    if (counter_ms % 50 == 0)  TaskFlag_50ms = 1;
    if (counter_ms % 500 == 0)  TaskFlag_500ms = 1;

    // Real-time clock "registers" update --
    //
    counter_ms++;
    if (counter_ms == 1000)  // one second elapsed
    {
        g_rtcSeconds++;
        counter_ms = 0;
    }
    if (g_rtcSeconds >= 60)  // 60 seconds elapsed
    {
        g_rtcSeconds = 0;
        g_rtcMinutes++;
    }
    if (g_rtcMinutes >= 60)  // 60 minutes elapsed
    {
        g_rtcMinutes = 0;
        g_rtcHour++;
    }
    if (g_rtcHour >= 24) g_rtcHour = 0;  // 24 hours elapsed
}


/*
 * Functions:  isTaskPending_??ms()
 *
 * These functions return TRUE if their respective Task Flag is raised;
 * otherwise they return FALSE.  The Task Flag is cleared before the function exits,
 * so that on subsequent calls it will return FALSE, until the next task period ends.
 */
BOOL  isTaskPending_5ms()
{
    BOOL  result = TaskFlag_5ms;

    if (result) TaskFlag_5ms = 0;
    return  result;
}

BOOL  isTaskPending_50ms()
{
    BOOL  result = TaskFlag_50ms;

    if (result) TaskFlag_50ms = 0;
    return  result;
}

BOOL  isTaskPending_500ms()
{
    BOOL  result = TaskFlag_500ms;

    if (result) TaskFlag_500ms = 0;
    return  result;
}


//==========================   L C D   F U N C T I O N S  =============================//
//
#define LCD_E     0   // bit places for LCD control signals: E, R/W and RS
#define LCD_R_W   1
#define LCD_RS    2

/*
 * Write 4-bit nybble to LCD controller.
 * Entry arg's:  rs   = register select (0: commnad, 1: data)
 *               nybl = 4-bit value to write (b0..b3)
 */
PRIVATE  void  lcd_write_nybble(char rs, char nybl)
{
    CLEAR_BIT(PORTC, LCD_R_W);        // R/W = Low (to write)
    CLEAR_BIT(PORTC, LCD_RS);         // RS = Low (to select cmd reg.)
    if (rs) SET_BIT(PORTC, LCD_RS);   // RS = High (to select data reg.)

    PORTB &= 0xF0;               // set up the data
    PORTB |= nybl;
    SET_BIT(PORTC, LCD_E);       // pulse E high for 1us
    _delay_us(1);
    CLEAR_BIT(PORTC, LCD_E);     // reset E low for 1us
    _delay_us(1);
}

/*
 * Initialize the LCD controller (HD44780) for 4-bit interface operation.
 * LCD mode is set to: 2 lines, char font 5x8 dots, cursor on
 */
void  lcd_initialise(void)
{
    DDRB = DDRB | 0b00001111;   // PB0..PB3 are outputs
    DDRC = DDRC | 0b00000111;   // PC0..PC2 are ctrl outputs
    _delay_us(2);
    CLEAR_BIT(PORTC, LCD_E);

    _delay_ms(15);   // Delay for power-on/reset

    // Perform software reset on LCD controller -- see HD44780 datasheet
    lcd_write_nybble(0, 0b0011);
    _delay_ms(5);
    lcd_write_nybble(0, 0b0011);
    _delay_us(200);
    lcd_write_nybble(0, 0b0011);
    _delay_us(200);
    lcd_write_nybble(0, 0b0010);   // Select 4-bit interface mode now
    _delay_us(200);

    // Set display mode:  DL=0 (4-bit bus), N=1 (2 lines), F=0 (5x8 dots CG)
    lcd_command(0b00101000);

    // Turn OFF the display (command byte = 0x08)
    lcd_command(LCD_OFF);

    // Clear display and reset the DDRAM address
    lcd_command(LCD_CLR);

    // Set Entry Mode (increment DDRAM address after writing char)
    lcd_command(LCD_EM_INC);

    // Turn ON the display & show cursor (command byte = 0x0E)
    lcd_command(LCD_ON);

    CLEAR_BIT(DDRC, 1);   // PC1 & PC2 are inputs (High-Z)
    CLEAR_BIT(DDRC, 2);
}


/*
 * Output a command byte to the LCD controller.
 * Refer to command definitions (macros) in library header file.
 *
 * Entry arg:  cmd = LCD command code (byte)
 */
void  lcd_command( BYTE cmd )
{
    static BYTE  PortB_reg_saved;

    PortB_reg_saved = PORTB;
    DDRB = DDRB | 0b00001111;   // PB0..PB3 are outputs
    DDRC = DDRC | 0b00000111;   // PC0..PC2 are ctrl outputs
    _delay_us(1);

    lcd_write_nybble(0, cmd >> 4);    // write MS nybble
    lcd_write_nybble(0, cmd & 0x0F);  // write LS nybble
    _delay_us(50);                    // allow cmd write process time

    if (cmd == LCD_CLR || cmd == LCD_HOME)  _delay_ms(2);

    PORTB = PortB_reg_saved;     // restore caller's PORTB bits
    CLEAR_BIT(DDRC, 1);          // PC1 & PC2 are inputs (High-Z)
    CLEAR_BIT(DDRC, 2);
}


/*
 * Display a single ASCII character.
 * The cursor position will be advanced one place to the right on exit.
 *
 * Entry arg:  c = ASCII character code (printable)
 */
void  lcd_write_char( char dat )
{
    static BYTE  PortB_reg_saved;

    PortB_reg_saved = PORTB;
    DDRB = DDRB | 0b00001111;   // PB0..PB3 are outputs
    DDRC = DDRC | 0b00000111;   // PC0..PC2 are ctrl outputs
    _delay_us(1);

    lcd_write_nybble(1, dat >> 4);    // write MS nybble
    lcd_write_nybble(1, dat & 0x0F);  // write LS nybble
    _delay_us(50);                    // allow data write process time

    PORTB = PortB_reg_saved;     // restore caller's PORTB bits
    CLEAR_BIT(DDRC, 1);          // PC1 & PC2 are inputs (High-Z)
    CLEAR_BIT(DDRC, 2);
}


/*
 * Function to display a NUL-terminated string at the current cursor position.
 *
 * Entry arg:  str = address of string (constant or variable)
 */
void  lcd_print_string( char *str )
{
    BYTE  count = 40;  // limit number of chars to write into LCD memory
    char  c;

    while (count != 0)
    {
        c = *str++;
        if (c == 0) break;  // NUL terminator found
        lcd_write_char(c);
        count--;
    }
}


//===================   P U S H - B U T T O N   F U N C T I O N S  ======================
//
const char button_code[] = { 'A', 'B', 'C', 'D' };  // ASCII code table

// static (private) variables and arrays
static char max_buttons;                // maximum number of buttons serviced (0..4)
static BOOL button_down_last_scan[4];   // TRUE if button[x] found pressed on LAST scan
static BOOL is_button_down[4];          // TRUE if button[x] is pressed on THIS scan
static BOOL is_button_hit[4];           // TRUE if button[x] hit event detected

/*
 * Function ButtonScan() must be called periodically from the application program
 * (main loop) at intervals of about 50ms for reliable "de-bounce" operation.
 *
 * Entry argument 'nButts' specifies the number of buttons (1..4) to be serviced.
 * For example, if nButts is 1, only Button_A is serviced;  if nButts is 3, then 3
 * buttons (Button_A, Button_B and Button_C) will be serviced by the scan routine.
 */
void  ButtonScan(unsigned char nButts)
{
    unsigned char x;    // button array index (0..3)

    if (nButts > 4)  nButts = 4;
    max_buttons = nButts;

    for (x = 0 ; x < nButts ; x++)    // Scan buttons up to nButts
    {
        CLEAR_BIT(DDRD, (x + 2));     // Set pin as input. (Button_A is PD2, etc)
        SET_BIT(PORTD, (x + 2));      // Enable internal pull-up resistor
        _delay_us(2);                 // Very short delay (~2us) for dust settling

        is_button_down[x] = !TEST_BIT(PIND, (x + 2));  // Read button input state

        if (is_button_down[x] && !button_down_last_scan[x])   // hit detected
        {
            is_button_hit[x] = TRUE;
        }

        button_down_last_scan[x] = is_button_down[x];  // update status
    }
}


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
BOOL  button_hit(char  button_ID)
{
    BOOL  result = FALSE;
    BYTE  x;

    for (x = 0 ; x < max_buttons ; x++)    // Check buttons up to max_buttons
    {
        if (button_ID == button_code[x])   // Found button_ID
        {
            result = is_button_hit[x];
            is_button_hit[x] = FALSE;
            break;
        }
    }

    return result;
}


/*
 * Function button_pressed() returns the Boolean value (TRUE or FALSE) of a flag indicating
 * whether or not a given button is currently pressed, i.e. held down.
 *
 * Entry argument 'button_ID' is an ASCII code identifying one of 4 buttons to check,
 * which must be one of: 'A', 'B', 'C' or 'D', otherwise the function will return FALSE.
 * If the given button is not serviced by ButtonScan(), button_pressed() will return FALSE.
 */
BOOL  button_pressed(char  button_ID)
{
    BOOL  result = FALSE;
    BYTE  x;

    for (x = 0 ; x < max_buttons ; x++)    // Check buttons up to max_buttons
    {
        if (button_ID == button_code[x])   // Found button_ID
        {
            result = is_button_down[x];
            break;
        }
    }

    return result;
}


//==========================   A D C   F U N C T I O N S  ===============================
/*
 * Function ADC_ReadInput() starts a one-off conversion on the given input, waits for
 * the conversion cycle to complete, then returns the 10-bit result.
 *
 * Entry arg:  muxsel = ADC MUX input select:  1 = ADC1, 2 = ADC2, ... 7 = ADC7
 *             (ADC0, ADC8..13 N/A, 14 = 1.1V internal ref, 15 = GND/0V)
 *
 * Note:  The function assumes the selected ADC port pin is already configured as an
 *        input and that its internal pull-up resistor is disabled.
 */
unsigned  ADC_ReadInput( BYTE muxsel )
{
    BYTE  low_byte;
    unsigned  result = 0;

    if (muxsel == 0 || muxsel > 15)  return 0;  // PC0/ADC0 is N/A (= LCD_E)

    ADMUX = 0x40 + muxsel;  // Select Vref = AVCC (+5V);  select MUX channel

    ADCSRA = 0x06;            // Set prescaler to F_SYS/64 --> ADC clock = 125 kHz
    SET_BIT(ADCSRA, ADEN);    // Enable ADC
    SET_BIT(ADCSRA, ADSC);    // Start conversion

    while (TEST_BIT(ADCSRA, ADSC) != 0)
    {
        ;  // wait till conversion done (ADSC == 0)
    }

    low_byte = ADCL;
    result = ((unsigned) ADCH) << 8;  // High-order byte (2 LS bits)
    result += low_byte;  // Add low-order 8 bits

    return  result;
}


//========================   E E P R O M    F U N C T I O N S  ==========================
//
BYTE  EEPROM_ReadByte(unsigned address)
{
    BYTE  bDat;  // returned data
    BYTE  int_status = SREG & 0x80;   // save I flag from SREG

    CLEAR_BIT(SREG, 7);       // global interrupt disable

    while (TEST_BIT(EECR, EEPE) != 0)
    {
        ;  // Wait for completion of previous write
    }

    EEAR = address;           // Set up address register
    SET_BIT(EECR, EERE);      // Start eeprom read
    bDat = EEDR;              // read out the data reg.

    SREG |= int_status;       // restore caller's interrupt status
    return  bDat;
}


void  EEPROM_WriteByte(unsigned address, BYTE bDat)
{
    BYTE  int_status = SREG & 0x80;   // save I flag from SREG

    CLEAR_BIT(SREG, 7);       // global interrupt disable

    while (TEST_BIT(EECR, EEPE) != 0)
    {
        ;  // Wait for completion of previous write
    }

    EEAR = address;           // Set up address register
    EEDR = bDat;              // Set up the data reg.
    SET_BIT(EECR, EEMPE);     // Master write enable
    SET_BIT(EECR, EEPE);      // Start eeprom write

    SREG |= int_status;       // restore caller's interrupt status
}


void  EEPROM_ReadArray(BYTE *pdat, int nbytes)
{
    unsigned address = 0;

    while (nbytes != 0)
    {

        *pdat++ = EEPROM_ReadByte(address);
        address++;
        nbytes--;
    }
}


void  EEPROM_WriteArray(BYTE *pdat, int nbytes)
{
    unsigned address = 0;

    while (nbytes != 0)
    {

        EEPROM_WriteByte(address, *pdat);
        address++;
        pdat++;
        nbytes--;
    }
}


//=========================   U A R T 0    F U N C T I O N S  ===========================
/*
*   Initialise USART0 for RX and TX with the specified baudrate.
*   Must be called by the application program before using USART0.
*   Entry arg (baudrate) is bits per second (e.g. 300, 1200, 9600, 19200, 38400, 57600).
*   Baudrates higher than 38400 are not recommended if CPU clock is internal RC osc.
*/
void  UART_init(unsigned baudrate)
{
    unsigned brr = F_CPU / ((long)baudrate * 16) - 1;

    UBRR0H = HI_BYTE(brr);      // set the Buad rate
    UBRR0L = LO_BYTE(brr);

    SET_BIT(UCSR0C, UCSZ00);    // Set frame format: 8,N,1 (data,parity,stop)
    SET_BIT(UCSR0B, RXEN0);     // Enable Receiver
    SET_BIT(UCSR0B, TXEN0);     // Enable Transmitter
}

/*
*   Fetch next byte from USART0 RX buffer.
*
*   The function DOES NOT WAIT for data available in the input buffer;
*   the caller must first check using the macro UART_RxDataAvail().
*   If there is no data available, the function returns NUL (0).
*   The input char is NOT echoed back to the UART output stream.
*
*   Returns:  Byte from USART0 RX buffer (or 0, if buffer is empty).
*/
unsigned char  UART_GetByte(void)
{
    unsigned char  b = 0;

    if (TEST_BIT(UCSR0A, RXC0) != 0)  b = UDR0;  // RX data available

    return  b;
}

/*
*   UART Transmit Byte.
*   Writes a data byte into the UART TX Data register to be transmitted.
*   The function waits for the TX register to be cleared first.
*/
void  UART_PutByte(unsigned char b)
{
    while (TEST_BIT(UCSR0A, UDRE0) == 0)   // TX busy
    {
        ;  // Wait for TX ready
    }

    UDR0 = b;
}

/*
*   UART_PutString...  Transmit a NUL-terminated string.
*   Newline (ASCII 0x0A) is expanded to CR + LF (0x0D + 0x0A).
*   Note: This function delays the calling task by whatever time it takes to
*   transmit the string (= strlen(s) x 10000 / baudrate;  msec. approx.)
*/
void  UART_PutString(char *str)
{
    char c;

    while ( (c = *str++) != '\0' )
    {
        if ( c == '\n' )
        {
            UART_PutByte( '\r' );
            UART_PutByte( '\n' );
        }
        else   UART_PutByte( c );
    }
}


#ifdef UART_RX_INTERRUPT_DRIVEN
/*
*   UART0 receiver interrupt service routine.
*
*   The IRQ signals that one or more bytes have been received by the UART;
*   the byte(s) must be read out of the UART RX data register(s) and stored in a
*   buffer in RAM (circular FIFO) by the call-back function: UART_RX_IRQ_Handler().
*/
ISR( USART_RX_vect )
{
    if (TEST_BIT(UCSR0A, RXC0) != 0)    // RX data available
    {
        UART_RX_IRQ_Handler();
    }
    else
    {
        // Clear source of unexpected IRQ (todo)
    }
}
#endif


//========================   K E Y P A D    F U N C T I O N S  ==========================
//
//
char ASCII_Code[] = {'1','2','3','A',     // ASCII value matrix
                     '4','5','6','B',
                     '7','8','9','C',
                     '*','0','#','D'};

// static (private) variables
static BOOL  key_down_last_scan;     // TRUE if any key found pressed on last scan
static BOOL  is_key_hit;             // TRUE if a key hit was detected
static char  keycode_found;          // ASCII code of last key hit

/*
 * Function KeypadScan() must be called periodically from the application program
 * (main loop) at intervals of about 30 ~ 50ms for reliable de-bounce operation.
 * The function assumes only one key at a time (or none) is pressed.
 * Its purpose is to detect a "key hit" event, i.e. a transition from "no key pressed"
 * to "any key pressed" and to record the key code of the last pressed key.
 */
void  KeypadScan(void)
{
    static BYTE  PortB_reg_saved;

    char  row, col = 0;
    char  column_bits;
    int   key_location;
    BOOL  key_down_this_scan;

    PortB_reg_saved = PORTB;     // Save caller's PORTB output bits (e.g. 8 x LED)
    DDRB = 0xFF;                 // Set up port B pins PB0..PB3 as outputs
    DDRD = DDRD & 0b11000011;    // Set up port D pins PD2..PD5 as inputs
    PORTD = PORTD | 0b00111100;  // Activate pull-up resistors on PD2..PD5

    key_down_this_scan = FALSE;  // Just an assumption at this point

    for (row = 0 ; row < 4 ; row++)   // Activate (i.e. set LOW) one row at a time
    {
        if (row == 0)  PORTB = 0b11110111;    // Scan first row
        if (row == 1)  PORTB = 0b11111011;    // Scan second, etc.
        if (row == 2)  PORTB = 0b11111101;
        if (row == 3)  PORTB = 0b11111110;

        _delay_us(2);   // very short delay for outputs to settle

        column_bits = PIND;     // Read Port D pins (column bits)
        column_bits = (column_bits >> 2) & 0b00001111;  // Shift column bits to first 4

        if (column_bits != 0b1111)   // If a key is found pressed on the active row...
        {
            // Find out which column the pressed key is in...
            if (column_bits == 0b1110)  col = 0;
            if (column_bits == 0b1101)  col = 1;
            if (column_bits == 0b1011)  col = 2;
            if (column_bits == 0b0111)  col = 3;

            key_location = (row << 2) + col;   // find ASCII table index
            key_down_this_scan = TRUE;
        }
    }

    if (key_down_this_scan && !key_down_last_scan)  // key hit detected
    {
        is_key_hit = TRUE;
        keycode_found = ASCII_Code[key_location];
    }

    // Update keypad status for next scan...
    key_down_last_scan = key_down_this_scan;

    PORTB = PortB_reg_saved;   // restore caller's PORTB bits
}

/*
 * Function key_hit() returns the Boolean value (TRUE or FALSE) of a flag indicating
 * whether or not a "key hit" event has occurred since the previous call to the function.
 * The flag (internal static variable) is cleared "automatically" by the function so that
 * on subsequent calls the function will return FALSE (until the next key hit).
 */
BOOL key_hit()
{
    BOOL temp_hit = is_key_hit;
    is_key_hit = FALSE;
    return temp_hit;
}

/*
 * Function key_code() returns the ASCII character code corresponding to the key last
 * pressed, i.e. when the last "key hit" event was detected.
 * Note: The ASCII codes for digits '0' to '9' are 0x30 to 0x39 (resp).
 */
char  key_code()
{
    return keycode_found;
}


// end of file
