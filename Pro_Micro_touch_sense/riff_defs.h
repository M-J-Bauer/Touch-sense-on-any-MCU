/*
 * File:  riff_defs.h
 *
 * Definitions and declarations for the Rifficator application
 * 
 */ 
#ifndef  RIFF_DEFS_H
#define  RIFF_DEFS_H
 
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "Pro_Micro_periph_lib.h"
#include "OLED_display_avr8_lib.h"
#include "MIDI_comms_lib.h"

#ifndef F_CPU
#define F_CPU  16000000UL    // CPU clock frequency
#endif

// Hard-coded constants -- may be changed only by re-compiling the code!
#define MIDI_OUT_CHANNEL  1     // 1..16
#define KEY_VELOCITY      80    // todo: make dynamic - (sample touch ADC value ???)

// Push-buttons 'A', 'B', 'C', 'D' are wired to port pins PB1, PB2, PB3, PB4, resp.
// Macro to configure switch input pins, enable internal pull-ups:
#define SETUP_SWITCH_INPUT_PINS()  { DDRB &= 0b11100001;  PORTB |= 0b00011110; }
    
#define BUTTON_A_PRESSED  ((PINB & (1<<1)) == 0)
#define BUTTON_B_PRESSED  ((PINB & (1<<2)) == 0)
#define BUTTON_C_PRESSED  ((PINB & (1<<3)) == 0)
#define BUTTON_D_PRESSED  ((PINB & (1<<4)) == 0)

#define LEFT_KEY    0     // Touch keypad identifiers
#define CENTRE_KEY  1
#define RIGHT_KEY   2

#define LED1_TURN_ON()    SET_BIT(PORTC, 6)
#define LED1_TURN_OFF()   CLEAR_BIT(PORTC, 6)
#define LED2_TURN_ON()    SET_BIT(PORTD, 7)
#define LED2_TURN_OFF()   CLEAR_BIT(PORTD, 7)
#define LED3_TURN_ON()    SET_BIT(PORTE, 6)
#define LED3_TURN_OFF()   CLEAR_BIT(PORTE, 6)

// Valid Note ON/OFF/Repeat states for 'PlayerTask' state-machine:
#define NOTE_OFF   0
#define NOTE_ON    1
#define NOTE_RPT   2

// Valid states for 'PlayerTask' state-machine...
// playerState bits 5-4 = Note On/Off/Rpt state, bits 2-1-0 = Keypad state (L-C-R)
#define NOTE_OFF_KEYS_OFF_OFF_OFF   0x00
#define NOTE_OFF_KEYS_OFF_OFF_ON    0x01
#define NOTE_OFF_KEYS_OFF_ON_OFF    0x02
#define NOTE_OFF_KEYS_OFF_ON_ON     0x03
#define NOTE_OFF_KEYS_ON_OFF_OFF    0x04
#define NOTE_OFF_KEYS_ON_ON_OFF     0x06
//
#define NOTE_ON_KEYS_OFF_OFF_ON     0x11
#define NOTE_ON_KEYS_OFF_ON_OFF     0x12
#define NOTE_ON_KEYS_OFF_ON_ON      0x13
#define NOTE_ON_KEYS_ON_OFF_OFF     0x14
#define NOTE_ON_KEYS_ON_ON_OFF      0x16
//
#define NOTE_RPT_KEYS_OFF_ON_ON     0x23
#define NOTE_RPT_KEYS_ON_ON_OFF     0x26
// All other state combinations are invalid

// Note sequence "direction" may be any one of these:
#define SAME_AGAIN        0x00    // Next note must be same as last (repeat)
#define ANY_DIRN          0x07    // Next note may be higher, lower or same
#define GOING_UP          0x1F    // Next note should be higher (or same)
#define GOING_DOWN        0xFF    // Next note should be lower (or same)


// ------------  Function prototype definitions  ------------------------------
void   PlayerTask();
void   InitiateNote(uint8 dirn);
void   TerminateNote();
uint8  GetCurrentScreenID();
void   GoToNextScreen(uint8 screenID);
void   UserInterfaceTask();
uint8  NoteSequence(uint8 direction);
unsigned  FastRand(unsigned long seed);
uint8  GaussRand(uint8 minval, uint8 maxval, uint8 spread);
uint8  FilteRand(uint8 minval, uint8 maxval, uint8 klag);
void   ButtonScan(unsigned char nb);
bool   ButtonHit(char  button_ID);
void   PotService();
bool   PotMoved();
uint8  PotPosition();
void   KeypadScan();
bool   KeyTouched(uint8 pad);
bool   KeyStruck(uint8 pad);
bool   KeyReleased(uint8 pad);
void   KeypadClear();
void   TouchCalibrate();
void   TouchService();

// Data structure holding parameters for the note selection algorithm
struct  NoteSelParams
{
    uint8  mid_note;   // note in centre of range (MIDI note number)
    uint8  range;      // allowed range of notes (semitones)
    uint8  opt_rand;   // random number gen. option (0: Gaussian, 1: Filter)
    uint8  spread;     // spread of Gaussian distribution (10..250 %)
    uint8  k_damp;     // damping level of filtered random series (1..10)
    uint8  p_accid;    // probability of hitting an accidental (0..100 %)
	//
	long   check;      // constant for EEPROM data integrity check
} ;   

// ------------  Global variables and data  -----------------------------------
//
extern struct NoteSelParams ns_param;  // (see above struct def'n)
extern unsigned  task1Freq;            // Call frequency of 1ms task (Hz)
extern uint8  midi_chan;               // MIDI channel # (0..15)
extern uint8  touch_onoff_thres[4];    // touch-pad on/off threshold values
extern uint8  touch_reading[4];        // touch-pad ADC readings (0..255)

extern uint8  notePlaying;        // MIDI note number (0 => No note playing)
extern uint8  noteOnOffState;     // Note ON/OFF/RPT state (0, 1, 2, resp.)
extern uint8  keypadState;        // Keypad state (pads touched)
extern uint8  playerState;        // Note ON/OFF/RPT and Keypad states combined
extern bool   isBlackNote;        // Last note selected was an 'accidental'
extern uint8  randNote;           // Note # last output from random nunber gen.
extern uint8  anyWhiteNote[];     // Table of note numbers in C-major scale

#endif  // RIFF_DEFS_H
