/*
 * Project:  "MIDI Rifficator" -- Melody and Riff Creator
 * ````````
 * Author:   M.J.Bauer  [ref. www.mjbauer.biz]
 * ```````
 * Hardware Platform:  Sparkfun 'Pro Micro' (ATmega32U4)
 * ```````````````````
 * Revision  History:  
 * ````````````````````````````````````````````````````````````````````````````
 *     v1.0 | 01-JUN-2024 | Originated -- MJB
 * 
 */ 
#include <avr/io.h>
#include "riff_defs.h"

// ------------  Global variables and data  ---------------------------------------------
//
struct NoteSelParams  ns_param;
//
unsigned long  TX_LED_period_start;
unsigned long  millisLast;
unsigned task1Freq;        // Call frequency of TouchService() task (Hz)
uint8  midi_chan;          // MIDI channel # (1..16)
uint8  notePlaying;        // MIDI note number (0 => No note playing)
uint8  noteOnOffState;     // Note ON/OFF/RPT state (0, 1, 2, resp.)
uint8  keypadState;        // Keypad state (pads touched)
uint8  playerState;        // Note ON/OFF/RPT and Keypad states combined
bool   isBlackNote;        // Last note selected was an 'accidental'
uint8  randNote;           // Note # last output from random nunber gen.

uint8  anyWhiteNote[] =    // Notes on the C-major scale
{
//   C    D    E    F    G    A    B  
    12,  14,  16,  17,  19,  21,  23,
    24,  26,  28,  29,  31,  33,  35,
    36,  38,  40,  41,  43,  45,  47,
    48,  50,  52,  53,  55,  57,  59,
    60,  62,  64,  65,  67,  69,  71,
    72,  74,  76,  77,  79,  81,  83,
    84,  86,  88,  89,  91,  93,  95,
    96,  98,  100, 101, 103, 105, 107,
    108, 110, 112, 113, 115, 117, 119, 
    120
};

uint8  anyBlackNote[] =    // Notes NOT on the C-major scale
{
//  C#   Eb   F#   Ab   Bb
    13,  15,  18,  20,  22,
    25,  27,  30,  32,  34,
    37,  39,  42,  44,  46,
    49,  51,  54,  56,  58,
    61,  63,  66,  68,  70,
    73,  75,  78,  80,  82,
    85,  87,  90,  92,  94,
    97,  99,  102, 104, 106,
    109, 111, 114, 116, 118
};

//============================================================================

void  setup(void)
{
    RX_LED_CONFIGURE();  // "system error" LED
    RX_LED_TURN_ON();
    TX_LED_CONFIGURE();  // "heartbeat" LED (debug option)
    TX_LED_TURN_OFF();
    SETUP_SWITCH_INPUT_PINS();
    
    SET_BIT(DDRC, 6);    // PC6 drives LED1
    SET_BIT(DDRD, 7);    // PD7 drives LED2
    SET_BIT(DDRE, 6);    // PE6 drives LED3
    SET_BIT(DDRD, 3);    // PD3 is TXD1 output
    CLEAR_BIT(DDRD, 4);  // PD4/ADC8 is Pot input
    
    TC0_Setup_SystemTimer();
    GLOBAL_INT_ENABLE();   // for timers
    UART_init(31250);

    I2C_MasterInit(400);   // F_SCL = 400 kHz
    if (I2C_MasterStart(SH1106_I2C_ADDRESS) != 0)
        { while (1) ;;; }  // fault -- hang!
    LED1_TURN_ON();
    if (I2C_MasterSend(SH1106_MESSAGETYPE_COMMAND) != 0)
        { while (1) ;;; }  // fault -- hang!
    LED2_TURN_ON();
    if (I2C_MasterSend(SH1106_DISPLAYOFF) != 0)
        { while (1) ;;; }  // fault -- hang!
    LED3_TURN_ON();
    I2C_MasterStop();

    LED1_TURN_OFF();  // indicate I2C init successful
    LED2_TURN_OFF();
    LED3_TURN_OFF();

    SH1106_Init();
    SH1106_SetContrast(20);
    while (milliseconds() < 100) ;;;  // delay 100ms for OLED init
        
    SH1106_Test_Pattern(); // OLED display test
    while (milliseconds() < 700) ;;;  // delay 500ms to view test pattern

    Disp_Init();
    Disp_ClearScreen();
    GoToNextScreen(0);   // 0 => INITIALIZING

    EEPROM_ReadArray((uint8 *) &ns_param, sizeof(ns_param));  // Fetch param's
    if (ns_param.check != 0xFEEDFACE)  // EEPROM data corrupted...
    {
        // Restore default parameter values
        midi_chan = MIDI_OUT_CHANNEL;
        ns_param.mid_note = 69;  // A4
        ns_param.range = 36;
        ns_param.opt_rand = 0;
        ns_param.spread = 100;
        ns_param.k_damp = 4;
        ns_param.p_accid = 0;
        ns_param.check = 0xFEEDFACE;
        EEPROM_WriteArray((uint8 *) &ns_param, sizeof(ns_param));
    }
    
    RX_LED_TURN_OFF();   // init successful
}
    

void  loop()
{  
    static unsigned  task1Count;
    static bool  touch_cal_done;
    
    if ((milliseconds() - TX_LED_period_start) >= 50)
        TX_LED_TURN_OFF();   // 50ms ON-time ended
        
    if (milliseconds() != millisLast)  // Do 1ms periodic task...
    {
        millisLast = milliseconds();
        TouchService();
        task1Count++;
    }        
    
    if (isTaskPending_5ms())  // Do 5ms periodic tasks...
    {
        KeypadScan();    // Look for keypad strikes
        PlayerTask();    // Play notes when keyed
        PotService();    // Update Pot position
    }        

    if (isTaskPending_50ms())  // Do 50ms periodic tasks...
    {
        ButtonScan(4);         // Look for button hits
        UserInterfaceTask();   // User interface
    }   

    if (isTaskPending_500ms())  // Do 500ms periodic task... 
    {
        // Comment out next line to disable annoying "heartbeat" LED!
//      TX_LED_TURN_ON();  // Start LED period
        TX_LED_period_start = milliseconds();
        task1Freq = task1Count * 2;
        task1Count = 0;
    }    
    
    if (!touch_cal_done && milliseconds() > 2000)  // once only after MCU reset
        { TouchCalibrate();  touch_cal_done = TRUE; } 
}


int  main(void)
{
    setup();
    
    while (TRUE)  loop();   // loop forever
}


/*
 * Called at periodic intervals of 5 milliseconds from the main loop,
 * the "Player Task" (function) looks for keypad touch-on and touch-off events and
 * initiates or terminates notes accordingly.
 */
void  PlayerTask()
{
    if (milliseconds() < 2020)  return;  // initializing -- wait
    
    switch (playerState)
    {
        case NOTE_OFF_KEYS_OFF_OFF_OFF:
        {
            if (KeyStruck(LEFT_KEY)) 
            {
                InitiateNote(GOING_DOWN);
                noteOnOffState = NOTE_ON;
            }    
            if (KeyStruck(CENTRE_KEY)) 
            {
                InitiateNote(ANY_DIRN);
                noteOnOffState = NOTE_ON;
            }  
            if (KeyStruck(RIGHT_KEY)) 
            {
                InitiateNote(GOING_UP);
                noteOnOffState = NOTE_ON;
            }          
            break;
        }        
        case NOTE_OFF_KEYS_OFF_OFF_ON:
        {
            if (KeyStruck(CENTRE_KEY)) 
            {
                InitiateNote(SAME_AGAIN);
                noteOnOffState = NOTE_ON;
            }  
            if (KeyReleased(RIGHT_KEY)) 
                noteOnOffState = NOTE_OFF;
            break;
        }        
        case NOTE_OFF_KEYS_OFF_ON_OFF:
        {
            if (KeyStruck(LEFT_KEY)) 
            {
                InitiateNote(SAME_AGAIN);
                noteOnOffState = NOTE_ON;
            }    
            if (KeyReleased(CENTRE_KEY)) 
                noteOnOffState = NOTE_OFF;
            if (KeyStruck(RIGHT_KEY)) 
            {
                InitiateNote(SAME_AGAIN);
                noteOnOffState = NOTE_ON;
            }          
            break;
        }        
        case NOTE_OFF_KEYS_OFF_ON_ON:
        {
            if (KeyReleased(CENTRE_KEY)) 
                noteOnOffState = NOTE_OFF;
            if (KeyReleased(RIGHT_KEY)) 
                noteOnOffState = NOTE_OFF;
            break;
        }        
        case NOTE_OFF_KEYS_ON_OFF_OFF:
        {
            if (KeyReleased(LEFT_KEY)) 
                noteOnOffState = NOTE_OFF;             
            if (KeyStruck(CENTRE_KEY)) 
            {
                InitiateNote(SAME_AGAIN);
                noteOnOffState = NOTE_ON;
            }          
            break;
        }        
        case NOTE_OFF_KEYS_ON_ON_OFF:
        {
            if (KeyReleased(LEFT_KEY)) 
                noteOnOffState = NOTE_OFF;
            if (KeyReleased(CENTRE_KEY)) 
                noteOnOffState = NOTE_OFF;
            break;
        }        
        case NOTE_ON_KEYS_OFF_OFF_ON:
        {
            if (KeyStruck(CENTRE_KEY)) 
            {
                TerminateNote();  // pending repeat
                noteOnOffState = NOTE_RPT;
            }  
            if (KeyReleased(RIGHT_KEY)) 
            {
                TerminateNote();
                noteOnOffState = NOTE_OFF;
            }            
            break;
        }        
        case NOTE_ON_KEYS_OFF_ON_OFF:
        {
            if (KeyStruck(LEFT_KEY)) 
            {
                TerminateNote();  // pending repeat
                noteOnOffState = NOTE_RPT;
            }    
            if (KeyReleased(CENTRE_KEY)) 
            {
                TerminateNote();
                noteOnOffState = NOTE_OFF;
            }            
            if (KeyStruck(RIGHT_KEY)) 
            {
                TerminateNote();  // pending repeat
                noteOnOffState = NOTE_RPT;
            }          
            break;
        }        
        case NOTE_ON_KEYS_OFF_ON_ON:
        {
            if (KeyReleased(CENTRE_KEY)) 
            {
                TerminateNote();
                noteOnOffState = NOTE_OFF;
            }            
            if (KeyReleased(RIGHT_KEY)) 
            {
                TerminateNote();
                noteOnOffState = NOTE_OFF;
            }            
            break;
        }        
        case NOTE_ON_KEYS_ON_OFF_OFF:
        {
            if (KeyReleased(LEFT_KEY)) 
            {
                TerminateNote();
                noteOnOffState = NOTE_OFF;
            }            
            if (KeyStruck(CENTRE_KEY)) 
            {
                TerminateNote();  // pending repeat
                noteOnOffState = NOTE_RPT;
            }    
            break;
        }        
        case NOTE_ON_KEYS_ON_ON_OFF:
        {
            if (KeyReleased(LEFT_KEY)) 
            {
                TerminateNote();
                noteOnOffState = NOTE_OFF;
            }            
            if (KeyReleased(CENTRE_KEY)) 
            {
                TerminateNote();
                noteOnOffState = NOTE_OFF;
            }            
            break;
        }        
        case NOTE_RPT_KEYS_OFF_ON_ON:
        {
            InitiateNote(SAME_AGAIN);
            noteOnOffState = NOTE_ON;
            break;
        }        
        case NOTE_RPT_KEYS_ON_ON_OFF:
        {
            InitiateNote(SAME_AGAIN);
            noteOnOffState = NOTE_ON;
            break;
        }        
        default:  // Invalid state -- an error occurred...
        {
            // Remain in this state until all keys released
            if (keypadState == 0)
            {
                TerminateNote();  // Kill note if playing
                KeypadClear();
                noteOnOffState = NOTE_OFF; 
            }                
            break;
        }            
    } // end switch
    
    keypadState = 0;       // Update player state...
    if (KeyTouched(LEFT_KEY)) keypadState |= 4;
    if (KeyTouched(CENTRE_KEY)) keypadState |= 2;
    if (KeyTouched(RIGHT_KEY)) keypadState |= 1;
    playerState = (noteOnOffState << 4) | keypadState;

    // Indicate keypad state on LEDs
    if (playerState & (1<<2)) LED3_TURN_ON();  else LED3_TURN_OFF();  // LEFT key
    if (playerState & (1<<1)) LED2_TURN_ON();  else LED2_TURN_OFF();  // CENTRE key
    if (playerState & (1<<0)) LED1_TURN_ON();  else LED1_TURN_OFF();  // RIGHT key
}    


void  InitiateNote(uint8 direction)
{
    if (notePlaying == 0)  // 0 => no note playing
    {
        notePlaying = NoteSequence(direction);  // Get new note
        MIDI_SendNoteOn(midi_chan, notePlaying, KEY_VELOCITY);
    }
}
    
    
void  TerminateNote()
{  
    if (notePlaying)
    {  
        MIDI_SendNoteOff(midi_chan, notePlaying);
        notePlaying = 0;
    }    
}


/*
 * Note Sequence Algorithm...
 *
 * Function generates a MIDI note number based on a pseudo-random sequence.
 * The sequence is moderated by a set of parameters which influence the statistical
 * distribution of notes. The argument (direction) determines whether the note
 * selected should be lower, higher, or either way, relative to the previous note.
 *
 * Entry arg:  direction = SAME_AGAIN, GOING_DOWN, GOING_UP or ANY_DIRN (either way)
 */
uint8  NoteSequence(uint8 direction)
{
    static uint8  previousNote;  // Last played note (if non-zero)
    uint8  i, note, half_range;
    uint8  selectedNote = previousNote;  // default (SAME_AGAIN)
    uint8  abs_min = ns_param.mid_note - ns_param.range / 2;
    uint8  abs_max = ns_param.mid_note + ns_param.range / 2 - 1;
    uint8  min_note = abs_min;  // default (ANY_DIRN)
    uint8  max_note = abs_max;  // default (ANY_DIRN)
    uint8  rnd_min = abs_min;   // default (ANY_DIRN)
    uint8  rnd_max = abs_max;   // default (ANY_DIRN)
    uint8  retryCount = 100;
    bool   onLimit = FALSE;
    bool   chooseBlackNote = FALSE;

    // One-time exception on reset... initialize previousNote to mid-range
    if (previousNote == 0) previousNote = ns_param.mid_note;
    
    // Choose white note or black note depending on probability param. (0..100 %)
    if ( (((FastRand(0) >> 8) * 100) >> 8) < ns_param.p_accid )
        chooseBlackNote = TRUE;

    if (direction == GOING_DOWN)  // allowable range is abs_min to (previousNote - 1)
    {
        max_note = (previousNote > abs_min) ? (previousNote - 1) : abs_min;
        if (previousNote == abs_min) onLimit = TRUE;
    }        

    if (direction == GOING_UP)  // allowable range is (previousNote + 1) to abs_max
    {
        min_note = (previousNote < abs_max) ? (previousNote + 1) : abs_max;  
        if (previousNote == abs_max) onLimit = TRUE;
    }    

    if (direction != SAME_AGAIN)  // Up, down or either way (regardless)...
    {
        // Adjust range of notes for random number generator so that it is
        // centered on the previous note, where possible; else approximate.
        half_range = ns_param.range / 2;
        rnd_min = previousNote - half_range;
        if (((int)previousNote - half_range) < 12) rnd_min = 12;  // lowest possible
        rnd_max = previousNote + half_range;
        if (rnd_max > 120) rnd_max = 120;  // highest possible

        // Try to find a note within allowable range and of preferred 'colour'
        while (retryCount--)  
        {
            if (onLimit)  // going up or down, no choice but to return previous note
            { 
                randNote = 0;  // Not random (for debug info)
                selectedNote = previousNote;
                break;
            }
            else  
            {
                if (ns_param.opt_rand == 0)
                    note = GaussRand(rnd_min, rnd_max, ns_param.spread);
                else  note = FilteRand(rnd_min, rnd_max, ns_param.k_damp);
                randNote = note;  // global for debug
                if (note < min_note) continue;  // out of range - try again
                if (note > max_note) continue;  // out of range - try again
            }
            if (chooseBlackNote)  // test if note is black...
            {
                for (i = 0;  i < sizeof(anyBlackNote);  i++) 
                {
                    if (note == anyBlackNote[i])  
                        { selectedNote = note;  retryCount = 0;  break; }  // success!
                }
            }
            else  // test if note is white...
            {
                for (i = 0;  i < sizeof(anyWhiteNote);  i++) 
                {
                    if (note == anyWhiteNote[i])  
                        { selectedNote = note;  retryCount = 0;  break; }  // success!
                }
            }                
        } // end while loop
    }    

    // Check selected note value again in case retries exhausted
    if (selectedNote < min_note) selectedNote = min_note;
    if (selectedNote > max_note) selectedNote = max_note;

    // Determine if selected note is black or white (for diagnostic info)
    for (isBlackNote = TRUE, i = 0;  i < sizeof(anyWhiteNote);  i++)
    {
        if (selectedNote == anyWhiteNote[i])  { isBlackNote = FALSE;  break; }
    }

    previousNote = selectedNote;  // Save for next call
    return  selectedNote;
}


/* 
 * Primitive but fast pseudo-random number generator...
 * Returns a 16-bit unsigned integer in the range 0..65535
 * with a uniform (equal probability) statistical distribution.
 *
 * Entry arg:  seed -- if odd number, initializes the random number series,
 *                     else, seed has no effect (ignored).
 */
unsigned  FastRand(unsigned long seed)
{
    static unsigned long rand_last = 1;  // seed must be odd number
    
    if (seed & 1) rand_last = seed;
    else  rand_last = (rand_last * 1103515245 + 12345);
    
    return  (unsigned int) (rand_last >> 8);
}


/* 
 * Primitive but fast "almost Gaussian" random number generator...
 * Returns an 8-bit unsigned integer in the range: minval to maxval (incl),
 * with roughly approximate Gaussian (Normal) distribution.
 * The mean of values returned is half-way between minval and maxval.
 *
 * Entry args:  minval = lower limit of return value (minval >= 1)
 *              maxval = upper limit of return value (maxval <= 127)
 *              spread = percent of standard deviation (10..250 %)
 */
uint8  GaussRand(uint8 minval, uint8 maxval, uint8 spread)
{
    int    ival, sum = 0;
    uint8  range = maxval - minval + 1;
    uint8  n;
    
    if ((maxval - minval) <= 1)  // borderline case...
        return   (maxval + minval) / 2;  // return mean value

    // Add 12 random numbers within the required range.  Maximum sum is (12 * range).
    for (n = 0;  n < 12;  n++)  
        { sum += ((long) FastRand(0) * range) / 65536; }
            
    ival = (sum / 12) - (range / 2);   // symmetrical on zero
    ival = (ival * spread) / 50;       // scale the variance
    ival += (minval + range / 2);      // offset to get actual mean value
    if (ival < minval) ival = minval;  // enforce lower limit
    if (ival > maxval) ival = maxval;  // enforce upper limit
    
    return  (uint8) ival;
}


/* 
 * Filtered random number series generator...
 * Returns an 8-bit unsigned integer in the range: minval to maxval (incl).
 * The return value is influenced by the history of previous numbers returned.
 * The amount of change between successive numbers in the series is determined by
 * a parameter (klag). Higher values of klag will result in smaller changes between
 * successive values returned.  Usable range of klag in this application is 1..10.
 * The mean of values returned is half-way between minval and maxval.
 *
 * Entry args:  minval = lower limit of return value (minval >= 1)
 *              maxval = upper limit of return value (maxval <= 127)
 *              klag   = parameter to control change between successive numbers
 */
uint8  FilteRand(uint8 minval, uint8 maxval, uint8 klag)
{
    static int  lastValue;  // fixed-point format, 8 LS bts = fractional part
    int  newValue;
    uint8  range = maxval - minval + 1;
    
    if (lastValue == 0)  lastValue = ((int)range / 2) << 8;  // mean value^^
    if (klag == 0)  klag = 1;
    
    // ^^NB: Here, the mean of newValue and lastValue is (range/2)
    newValue = (((long) FastRand(0) * range) / 65536) << 8;  // fixed-point
    lastValue -= (lastValue / klag);  // Apply 1st-order IIR filter agorithm
    lastValue += (newValue / klag);
    
    newValue = (lastValue >> 8) - (range / 2);  // integer part, mean = 0
	
    // Scale newValue x klag and offset result to get actual mean: minval + (range/2)
    newValue = newValue + minval + (range / 2);
    if (newValue < minval)  newValue = minval;  // enforce lower limit
    if (newValue > maxval)  newValue = maxval;  // enforce upper limit

    return  (uint8) newValue;
}    


//==================  T O U C H   K E Y - P A D   F U N C T I O N S   ===================
//
uint8  touch_onoff_thres[4];  // touch-pad on/off threshold values
uint8  touch_reading[4];      // touch-pad ADC readings (0..255)
bool   pad_touched[4];        // Pad on/off status (raw, from Touch Service routine)
bool   pad_touched_last[4];   // Pad on/off status (de-glitched, from Keypad Scan fn)
bool   key_struck[4], key_released[4];  // Pad on/off transition flags
/*
 * Function:  KeypadScan()
 *
 * Overview:  Touch keypad scanning routine, called periodically at 5 ms intervals.
 *
 *            Touchpad states are monitored for "touch-ON" and "touch-OFF" events.
 *            A "touch-ON" event is a transition from pad not touched to pad touched.
 *            A "touch-OFF" event is a transition from pad touched to pad not touched.
 *            Touchpad states are "de-glitched" using a 25ms "settling window".
 */
void  KeypadScan()
{
    static uint8  pad_touch_on_count[4];   // number of glitch-free touch-on samples
    static uint8  pad_touch_off_count[4];  // number of glitch-free touch-off samples
    uint8  pad;
    
    for (pad = 0;  pad < 3;  pad++)
    {
        if (!pad_touched[pad]) pad_touch_on_count[pad] = 0;
        if (pad_touched[pad]) pad_touch_off_count[pad] = 0;
        
        if (!pad_touched[pad] && pad_touch_off_count[pad] < 50)
            pad_touch_off_count[pad]++;  // capped at 50

        if (pad_touched[pad] && pad_touch_on_count[pad] < 50)
            pad_touch_on_count[pad]++;   // capped at 50
 
        if (pad_touched[pad] && !pad_touched_last[pad] && pad_touch_on_count[pad] >= 5) 
        {
            key_struck[pad] = TRUE;
            pad_touched_last[pad] = TRUE;
        }            
        
        if (!pad_touched[pad] && pad_touched_last[pad] && pad_touch_off_count[pad] >= 5) 
        {
            key_released[pad] = TRUE;
            pad_touched_last[pad] = FALSE;
        }            
    }
}    

/*
 * Function:  KeyTouched(pad)
 *
 * Overview:  Returns TRUE if the given key (pad) is currently being touched.
 *
 * Entry arg:   pad = digit identifying the pad to be checked...
 *                    0: Left pad, 1: Centre pad, 2: Right pad, 3: unused pad
 */
bool  KeyTouched(uint8 pad)
{
    return pad_touched_last[pad];
}
    
/*
 * Function:  KeyStruck(pad)
 *
 * Overview:  Returns TRUE if a "touch-ON event" has occurred since the previous call.
 *
 * Entry arg:   pad = digit identifying the pad to be checked...
 *                    0: Left pad, 1: Centre pad, 2: Right pad, 3: unused pad
 */
bool  KeyStruck(uint8 pad)
{
    bool result = FALSE;
    if (key_struck[pad]) result = TRUE;
    key_struck[pad] = FALSE;
    return result;
}
    
/*
 * Function:  KeyReleased(pad)
 *
 * Overview:  Returns TRUE if a "touch-OFF event" has occurred since the previous call.
 *
 * Entry arg:   pad = digit identifying the pad to be checked...
 *                    0: Left pad, 1: Centre pad, 2: Right pad, 3: unused pad
 */
bool  KeyReleased(uint8 pad)
{
    bool result = FALSE;
    if (key_released[pad]) result = TRUE;
    key_released[pad] = FALSE;
    return result;
}

/*
 * Function:  KeypadClear()
 *
 * Clear all key-struck and key-released flags to prevent false signaling of events.
 * To clear an individual flag, call respective function, e.g. KeyStruck(pad).
 */
void  KeypadClear()
{
    uint8  i;
    
    for (i = 0;  i < 4;  i++) 
    { 
        key_struck[i] = FALSE;  
        key_released[i] = FALSE; 
        pad_touched_last[i] = FALSE;
    }
}    

/*
 * Function:  TouchCalibrate()
 *
 * Determines touch-sense on/off threshold values which are written into array:
 * touch_onoff_thres[].
 * Function must be called at least once in the first second or two after MCU reset,
 * during which time the TouchService() routine must be executed periodically.
 * It is assumed the touch-pads are not being touched during the calibration period.
 */
void  TouchCalibrate()
{
    touch_onoff_thres[0] = touch_reading[0] - 10;
    touch_onoff_thres[1] = touch_reading[1] - 10;
    touch_onoff_thres[2] = touch_reading[2] - 10;
    touch_onoff_thres[3] = touch_reading[3] - 10;
    KeypadClear();
}    

/*
* TOUCH SENSE SERVICE ROUTINE
*
* Customized to read 4 touch inputs on PF4/ADC4, PF5/ADC5, PF6/ADC6 & PF7/ADC7.
* Current source is pin PB5.
*
* Called periodically at intervals of 1 millisec (approx).
* On each call, one touch input is serviced in sequence.
* Execution time is about 27 microseconds for each touch input.
* Global IRQ is disabled during execution of this function.
*/
void  TouchService()
{
    static  uint8  pad_idx;  // bit number (4..7) of Port F (ADC channel)
    uint8  pad;

    if (pad_idx == 0) pad_idx = 4; // begin with pad on PF4/ADC4
    
//  TESTPOINT_SET_HIGH();  // 'Scope trigger
    GLOBAL_INT_DISABLE();
   
    // Select ADC input for pad to be serviced (pad_idx) and enable ADC
    CLEAR_BIT(DDRF, pad_idx);  // configure pad pin as input
    ADCSRA = 0x03;             // Set prescaler to F_SYS/8 (ADC clock = 2 MHz)
    ADCSRB = 0x00;             // Set MUX5 bit = 0
    ADMUX = 0x60 + pad_idx;    // set ADC channel; Vref=AVCC; left-align result   
    SET_BIT(ADCSRA, ADEN);     // Enable ADC

    // Switch on the current source and delay for 6 microseconds (+/-1us)
    SET_BIT(PORTB, 5);
    SET_BIT(DDRB, 5);
    Delay_CPU_cycles(80);  // Macro defined in "pro-micro-periph.h"
    
    // Switch off the current source and read the ADC result
    CLEAR_BIT(PORTB, 5);
    CLEAR_BIT(DDRB, 5);
    SET_BIT(ADCSRA, ADSC);     // Start conversion (ADSC = 1)
    
    while (ADCSRA & (1<<ADSC))  { ; }  // wait until conversion done (ADSC == 0)
    
    pad = pad_idx - 4;         // pad ID number (0..3)
    touch_reading[pad] = ADCH; // get ADC result (MSB)
    SET_BIT(DDRF, pad_idx);    // Discharge pad -- configure pin as output (LOW)
    
    // Determine on/off state of touch-pad
    if (touch_reading[pad] < touch_onoff_thres[pad]) 
        pad_touched[pad] = TRUE; 
    else  pad_touched[pad] = FALSE; 

    if (++pad_idx == 8) pad_idx = 4;  // next pad
    
    GLOBAL_INT_ENABLE();
//  TESTPOINT_SET_LOW();
}    

// end-of-file
