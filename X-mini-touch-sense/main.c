/**
 * Project:  X-mini-touch-sense
 *
 * Target:   ATmega328P 'X-plained' mini board, or Arduino Uno or Nano
 * Created:  16-APR-2023
 * Author :  M.J.Bauer  [www.mjbauer.biz]
 *
 * Overview:   The program demonstrates a technique to provide touch-sense capability
 *             on an AVR micro-controller without on-chip touch-sense hardware support.
 *
 * Operation:  Two push-button switches are wired to pins PD2 and PD3, labelled 'A' and 'B'.
 *             These buttons are used to start one of the two touch-pad tests.
 *             The X-mini on-board LED (driven by PB5) should pulse at frequency 1Hz.
 *
 * Reference:  [1] Document: "Touch Sense Technique for any MCU" (PDF)
 *             [2] Code library files: "lib_avrXmini.*"
 *
 * Note:  This application requires the library object file "lib_avrXmini.a" and associated
 *        header file "lib_avrXmini.h" to be included in the Microchip Studio project.
 */
#define  F_CPU  16000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "lib_avrXmini.h"

// Touch-pad on/off state is 'ON' if the ADC reading (MSB) is below this value...
#define TOUCH_ON_THRESHOLD  150  // minimum:50, typical:150, maximum:220

// Pin PD7 is a testpoint output used as a 'scope trigger signal (optional)
#define TESTPOINT_SET_HIGH()  SET_BIT(PORTD, 7)
#define TESTPOINT_SET_LOW()   CLEAR_BIT(PORTD, 7)  

/**
* This macro delays a specified number of CPU clock cycles (nc) using Timer/Counter_2.
* Delay time = nc x 16 us (approx.)  Maximum delay is 256 clocks => 16 microseconds.
* Minimum delay is 1 clock => 1 us (due to overhead in while-loop exit test, etc).
* Precise argument values:  nc = 12 for 2us, nc = 62 for 5us, nc = 142 for 10us.
* For time-critical delays, global interrupts must be disabled prior to execution.
*/
#define Delay_CPU_cycles(nc) { \
    TCNT2 = 0; \
    TCCR2B = 0x01; \
    while (TCNT2 < nc) {;} \
    TCCR2B = 0x00;  }     


// function prototypes...
void  TestRoutine();
void  Touch_Service();

// global variables...
unsigned long  elapsed_time; 
BYTE  test_step;
BYTE  touch_reading[8];  // touch-pad ADC readings (8 bits)
BOOL  pad_touched[8];    // touch-pad on/off states

//----------------------------------------------------------------------------------------

void  setup( void )
{
    SET_BIT(DDRB, 5);      // Set PB5 as output for on-board LED drive
    SET_BIT(DDRD, 7);      // Set PD7 as output for scope trigger
    
    // Clear PORT bits of touch-pad pins so that they pull LOW (0V) when set as outputs.
    // (Redundant, since this is the default state on MCU reset.)
    CLEAR_BIT(PORTC, 3);
    CLEAR_BIT(PORTC, 4);
    CLEAR_BIT(PORTC, 5);

    lcd_initialise();
    lcd_command(LCD_CURSOR_OFF);
    lcd_cursor_posn(0, 0);  // line 1
    lcd_print_string("AVR touch sense");
    lcd_cursor_posn(1, 0);  // line 2
    lcd_print_string("Test & demo...");
    
    // Set up Timer/Counter_0 for periodic task scheduling with 800us period.
    // At the end of each period, timer flag OCF0A will be set in register TIFR0;
    // the flag is cleared manually in software by writing a 1 into bit OCF0A.
    TCCR0A = 0x02;    // CTC mode (count up to OCR0A value, reset and repeat) 
    OCR0A = 49;       // Period = 50 x 16us = 800us
    TCCR0B = 0x04;    // Start TC0, Prescaler N = 256 (Tclk = 16us)

    TC1_initialise();     // Library timer functions use TC1
    GLOBAL_INT_ENABLE();    
}    


int  main( void )
{
    static  unsigned long  start_LED_duty;
    static  BYTE  count_to_4;
    static  BYTE  count_to_20;
    
    setup();

    while ( 1 )  // loop forever 
    {
        if (test_step < 2)  // if not running a touch-sense test...
        {
            // Generate pulses (Tpw(H) = 10us) to test delay macro: Delay_CPU_cycles(n)
            TESTPOINT_SET_HIGH();
            Delay_CPU_cycles(142);
            TESTPOINT_SET_LOW();
            Delay_CPU_cycles(50);   // Tpw(L) will vary due to execution of tasks below
        }        
        else if (TEST_BIT(TIFR0, OCF0A))  // flag raised every 800 microseconds...
        {
            SET_BIT(TIFR0, OCF0A);  // clear flag by writing 1!
            Touch_Service();
        }            
         
        if (isTaskPending_50ms())  // every 50ms...
        {
            ButtonScan(4);  // monitor push-button switches (A, B, C, D)
            
            if (++count_to_4 == 4)  // every 200ms...
            {
                count_to_4 = 0;
                TestRoutine();  
            }          
            if (++count_to_20 == 20)  // every second...
            {
                count_to_20 = 0;
                SET_BIT(PORTB, 5);  // Turn LED on
                start_LED_duty = milliseconds();
            }      
        }            

        if ((milliseconds() - start_LED_duty) >= 100)  // 100ms duty expired
            CLEAR_BIT(PORTB, 5);   // Turn LED off
            
        if (button_hit('A'))    // switch to "touch-pad raw ADC readings" test (#2)
        {
            lcd_command(LCD_CLR);
            lcd_cursor_posn(0, 0); 
            lcd_print_string("Pad1  Pad2  Pad3");
            test_step = 2;
        }
        if (button_hit('B'))    // switch to "stop-watch timer" test (#3)
        {
            lcd_command(LCD_CLR);
            lcd_cursor_posn(1, 0); 
            lcd_print_string("RUN  STOP  RESET");
            elapsed_time = 0;
            test_step = 3;
        }
    }
}


/*
* The test routine is called periodically, 5 times per second, to refresh displayed data.
*/
void  TestRoutine(void)
{
    static  BYTE  timer_state;
    char    buff[20];
    int     hours, minutes, seconds, tenths;

    if (test_step == 0)    // Showing startup message for 3 seconds...
    {
        if (++elapsed_time >= 15)  // ... then show prompt msg...
        {
            lcd_cursor_posn(1, 0);  // line 2
            lcd_print_string("  Press A or B");
            test_step = 1;
        }
    }
    else if (test_step == 1)    // Waiting for button press to start test #2 or #3...
    {
        ;
    }
    else if (test_step == 2)    // Showing touch-pad raw ADC readings
    {
        // LCD bottom line shows ADC counts for 3 touch inputs...
        lcd_cursor_posn(1, 0);  // erase existing data
        lcd_print_string("                ");
        sprintf(buff, "%4d  %4d  %4d", touch_reading[3], touch_reading[4], touch_reading[5]);
        lcd_cursor_posn(1, 0);
        lcd_print_string(buff); 
    }
    else if (test_step == 3)    // Running stop-watch timer demo
    {
        hours = elapsed_time / 36000;  // elapsed_time unit = 0.1 sec
        minutes = (elapsed_time - (long)hours * 36000) / 600;
        seconds = (elapsed_time - (long)hours * 36000 - (long)minutes * 600) / 10;
        tenths = elapsed_time % 10; 
        
        // LCD top line shows elapsed time, format [hh:mm:ss.t]
        lcd_cursor_posn(0, 3);  // erase existing data
        lcd_print_string("           ");
        sprintf(buff, "%02d:%02d:%02d.%1d", hours, minutes, seconds, tenths);
        lcd_cursor_posn(0, 3);
        lcd_print_string(buff); 
        
        // The 3 Touch-pads are used to RUN, STOP or RESET the stop-watch timer
        if (timer_state == 0)  // RESET state -- waiting for Run/Start
        {
            if (pad_touched[3])  timer_state = 1;  // 'RUN' pad touched
        }
        else if (timer_state == 1)  // RUN state -- waiting for Stop
        {
            elapsed_time += 2 ;  
            if (pad_touched[4])  timer_state = 2;  // 'STOP' pad touched
        }
        else  // STOP state -- waiting for Start or Reset
        {
            if (pad_touched[3])  timer_state = 1;  // 'RUN' pad touched
            if (pad_touched[5])  // 'RESET' pad touched
            {
                elapsed_time = 0;
                timer_state = 0;
            }                
        }
    }
}


/*
* TOUCH SENSE SERVICE ROUTINE
* Called periodically at intervals anywhere in the range 100us to 5ms.
* On each call, one touch input is serviced in sequence.
* Execution time is about 27 microseconds (for each touch input).
* Global IRQ is disabled during execution of this function.
* This example application reads 3 touch inputs on pins PC3, PC4 and PC5.
*/
void  Touch_Service()
{
    static  BYTE  pad_idx;  // Bit number (0..7) of Port C (ADC inputs)

    if (pad_idx == 0) pad_idx = 3; // begin with pad on PC3/ADC3
    
    TESTPOINT_SET_HIGH();  // 'Scope trigger
    GLOBAL_INT_DISABLE();
   
    // Select ADC input for pad to be serviced (pad_idx) and enable ADC
    CLEAR_BIT(DDRC, pad_idx);  // configure pad pin as input
    ADCSRA = 0x03;             // Set prescaler to F_SYS/8 (ADC clock = 2 MHz)
    ADMUX = 0x60 + pad_idx;    // set ADC channel; Vref=AVCC; left-align result   
    SET_BIT(ADCSRA, ADEN);     // Enable ADC

    // Switch on the current source (PB4) and delay for 6 microseconds (+/-1us)
    SET_BIT(PORTB, 4);
    SET_BIT(DDRB, 4);
    Delay_CPU_cycles(80);
    
    // Switch off the current source and read the ADC result
    CLEAR_BIT(PORTB, 4);
    CLEAR_BIT(DDRB, 4);
    SET_BIT(ADCSRA, ADSC);     // Start conversion (ADSC = 1)
    
    while (ADCSRA & (1<<ADSC))  { ; }  // wait until conversion done (ADSC == 0)
    
    touch_reading[pad_idx] = ADCH;  // get ADC result (MSB)
    SET_BIT(DDRC, pad_idx);    // Discharge pad -- configure pin as output (set LOW)
    
    // Determine on/off state of touch-pad
    if (touch_reading[pad_idx] < TOUCH_ON_THRESHOLD) 
        pad_touched[pad_idx] = TRUE; 
    else  pad_touched[pad_idx] = FALSE; 
    
    if (++pad_idx == 6) pad_idx = 3;  // next pad
    
    GLOBAL_INT_ENABLE();
    TESTPOINT_SET_LOW();
}    

