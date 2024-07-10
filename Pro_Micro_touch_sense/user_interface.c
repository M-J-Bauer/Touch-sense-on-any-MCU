/**
 * File:  riff_user_interface.c
 *
 * Rifficator 'User Interface' comprising IIC OLED display, 4 push-buttons
 * and Data Entry potentiometer.
 *
 */
#include <avr/io.h>
#include "riff_defs.h"

enum  User_Interface_Modes  // aka 'Screen identifiers'
{
    INITIALIZING = 0,
    HOME_SCREEN,
    SETTING_MID_NOTE,
    SETTING_RANGE,
    SETTING_PR_ACCID,
    SETTING_OPT_RAND,
    SETTING_SPREAD,
    SETTING_K_DAMP,
    SYSTEM_INFO,
    DIAGNOSTIC_INFO
};    

// ------------  Function prototype definitions  --------------------------------------
//
void   UserMode_Initializing();
void   UserMode_HomeScreen();
void   UserMode_SettingMidNote();
void   UserMode_SettingRange();
void   UserMode_SettingPrAccid();
void   UserMode_SettingOptRand();
void   UserMode_SettingSpread();
void   UserMode_SettingFiltDamp();
void   UserMode_SystemInfo();
void   UserMode_DiagnosticInfo();


//------------------- Private data ----------------------------------------------------
//
static bool   screenSwitchReq;       // flag: Request to switch to next screen
static bool   isNewScreen;           // flag: Screen switch has occurred
static uint8  currentScreen;         // ID number of current screen displayed
static uint8  previousScreen;        // ID number of previous screen displayed
static uint8  nextScreen;            // ID number of next screen to be displayed


/*
 * Display title bar text at top of screen in 12pt font with underline.
 * The title bar area is assumed to be already erased.
 * Maximum length of titleString is 16 ~ 17 chars.
 */
void  DisplayTitleBar(char *titleString)
{
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(4, 0);
    Disp_SetFont(PROP_12_NORM);
    Disp_PutText(titleString);
    Disp_PosXY(0, 12);
    Disp_DrawLineHoriz(128);
}    


/*
 * Display a text string (4 chars max) centred in a button (bar) of fixed width (30 px)
 * using 8pt mono-spaced font, at the specified screen position (x, y) = bar upper LHS.
 * On exit, the display write mode is restored to 'SET_PIXELS'.
 */
void  DisplayButtonLegend(uint8 x, uint8 y, char *str)
{
    int  len = strlen(str);
    int  i;
    
    if (len > 4) len = 4;

    Disp_Mode(SET_PIXELS);  // Draw the button
    Disp_PosXY(x, y);
    Disp_DrawBar(30, 9);
    Disp_Mode(CLEAR_PIXELS);
    Disp_SetFont(MONO_8_NORM);  // Write the string
    x += 3 + 3 * (4 - len);  
    Disp_PosXY(x, y+1);
    
    for (i = 0;  i < len;  i++)  { Disp_PutChar(*str++); }
   
    Disp_Mode(SET_PIXELS);
}    


/*
 * Function returns the screen ID number of the currently displayed screen.
 */
uint8  GetCurrentScreenID()
{
    return  currentScreen;
}


/*
 * Function GoToNextScreen() triggers a screen switch to a specified screen ID.
 * The real screen switch business is done by the UserInterfaceTask() function
 * when next executed following the call to GoToNextScreen().
 *
 * Entry arg:  screenID = ID number of next screen required.
 */
void  GoToNextScreen(uint8 screenID)
{
    nextScreen = screenID;
    screenSwitchReq = TRUE;
}


/*
 * Called at periodic intervals of 50 milliseconds from the main loop, the
 * User Interface task allows the user to view and adjust various operational
 * parameters using the Data Entry Pot, 4 push-buttons and IIC OLED display.
 */
void  UserInterfaceTask(void)
{
    if (screenSwitchReq)   // Screen switch requested
    {
        screenSwitchReq = FALSE;
        isNewScreen = TRUE;  // Signal to render a new screen
        previousScreen = currentScreen;     // Make the switch...
        currentScreen = nextScreen;         // next screen => current
        if (nextScreen != previousScreen)  Disp_ClearScreen();
        PotMoved();  // clear 'pot moved' flag
    }
      
    switch (currentScreen)
    {
        case INITIALIZING:      UserMode_Initializing();     break;
        case HOME_SCREEN:       UserMode_HomeScreen();       break;
        case SETTING_MID_NOTE:  UserMode_SettingMidNote();   break;
        case SETTING_RANGE:     UserMode_SettingRange();     break;
        case SETTING_PR_ACCID:  UserMode_SettingPrAccid();   break;
        case SETTING_OPT_RAND:  UserMode_SettingOptRand();   break;
        case SETTING_SPREAD:    UserMode_SettingSpread();    break;
        case SETTING_K_DAMP:    UserMode_SettingFiltDamp();  break;
        case SYSTEM_INFO:       UserMode_SystemInfo();       break;
        case DIAGNOSTIC_INFO:   UserMode_DiagnosticInfo();   break;
    }
    
    isNewScreen = FALSE;
}


void  UserMode_Initializing()
{
    if (isNewScreen)
    {
        Disp_Mode(SET_PIXELS);
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(3, 26);
        Disp_PutText("--- Initializing ---");
    }        
    
    // Exit this screen 2 sec. after reset
    if (milliseconds() > 2000)  GoToNextScreen(HOME_SCREEN);
}


void  UserMode_HomeScreen()
{
    if (isNewScreen)
    {
        // Prepare HOME screen -- todo: add graphic image(s)
        DisplayTitleBar("MIDI Rifficator");
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 20);
        Disp_PutText("Melody and Riff");
        Disp_PosXY(16, 30);
        Disp_PutText("    Creator");
            
        DisplayButtonLegend(0, 53, "Diag");
        DisplayButtonLegend(32, 53, "Info");
        DisplayButtonLegend(64, 53, "OPT");
        DisplayButtonLegend(96, 53, "SET");
    }       
    
    if (ButtonHit('A'))  GoToNextScreen(DIAGNOSTIC_INFO); 
    if (ButtonHit('B'))  GoToNextScreen(SYSTEM_INFO); 
    if (ButtonHit('C'))  GoToNextScreen(SETTING_OPT_RAND); 
    if (ButtonHit('D'))  GoToNextScreen(SETTING_MID_NOTE); 
}


void  UserMode_SettingMidNote()
{
    uint8  setting;
    bool  doRefresh = FALSE;
    
    if (isNewScreen)
    {
        DisplayTitleBar("Mid-range Note");
        DisplayButtonLegend(0, 53, "Home");
        DisplayButtonLegend(32, 53, "x");
        DisplayButtonLegend(64, 53, "Save");
        DisplayButtonLegend(96, 53, "Next");
        doRefresh = TRUE;
    }
    
    if (PotMoved())
    {
        setting = ((int)PotPosition() * 50) / 255 + 7;  // span is 7..57
        if (setting > 56)  setting = 56;  // limit is 56
        ns_param.mid_note = anyWhiteNote[setting];
        doRefresh = TRUE;
    }        
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('C'))  // Save param's in EEPROM
    {   
        EEPROM_WriteArray((uint8 *) &ns_param, sizeof(ns_param));
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 42);
        Disp_PutText("* Setting saved");
    }        
    if (ButtonHit('D'))  GoToNextScreen(SETTING_RANGE);

    if (doRefresh)
    {
        Disp_PosXY(16, 42);
        Disp_BlockClear(96, 8);  // clear "Saved..." msg area
        Disp_SetFont(MONO_16_NORM);
        Disp_PosXY(48, 24);
        Disp_BlockClear(48, 16);  // clear existing data
        Disp_PutDecimal(ns_param.mid_note, 2);
    }        
}   


void  UserMode_SettingRange()
{
    uint8  setting;
    bool  doRefresh = FALSE;
    
    if (isNewScreen)
    {
        DisplayTitleBar("Range of Notes");
        DisplayButtonLegend(0, 53, "Home");
        DisplayButtonLegend(32, 53, "Back");
        DisplayButtonLegend(64, 53, "Save");
        DisplayButtonLegend(96, 53, "Next");
        doRefresh = TRUE;
    }
    
    if (PotMoved())
    {
        setting = ((int)PotPosition() * 12) / 255;  // span 0..12
        if (setting > 11)  setting = 11;
        ns_param.range = (setting + 1) * 6;  // 6..72 in 12 steps
        doRefresh = TRUE;
    }        
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('B'))  GoToNextScreen(SETTING_MID_NOTE);
    if (ButtonHit('C'))  // Save param's in EEPROM
    {   
        EEPROM_WriteArray((uint8 *) &ns_param, sizeof(ns_param));
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 42);
        Disp_PutText("* Setting saved");
    }        
    if (ButtonHit('D'))  GoToNextScreen(SETTING_PR_ACCID);

    if (doRefresh)
    {
        Disp_PosXY(16, 42);
        Disp_BlockClear(96, 8);  // clear "Saved..." msg area
        Disp_SetFont(MONO_16_NORM);
        Disp_PosXY(48, 24);
        Disp_BlockClear(32, 16);  // clear existing data
        Disp_PutDecimal(ns_param.range, 2);
    }        
}   


void  UserMode_SettingPrAccid()
{
    uint8  setting;
    bool  doRefresh = FALSE;
    
    if (isNewScreen)
    {
        DisplayTitleBar("Pr (Accidental)");
        DisplayButtonLegend(0, 53, "Home");
        DisplayButtonLegend(32, 53, "Back");
        DisplayButtonLegend(64, 53, "Save");
        DisplayButtonLegend(96, 53, "Next");
        doRefresh = TRUE;
    }
    
    if (PotMoved())
    {
        setting = ((int)PotPosition() * 21) / 255;  // span 0..21
        if (setting > 20)  setting = 20;
        ns_param.p_accid = setting * 5;  // 0..100 in 21 steps
        doRefresh = TRUE;
    }        
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('B'))  GoToNextScreen(SETTING_RANGE);
    if (ButtonHit('C'))  // Save param's in EEPROM
    {   
        EEPROM_WriteArray((uint8 *) &ns_param, sizeof(ns_param));
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 42);
        Disp_PutText("* Setting saved");
    }        
    if (ButtonHit('D'))  GoToNextScreen(SETTING_OPT_RAND);

    if (doRefresh)
    {
        Disp_PosXY(16, 42);
        Disp_BlockClear(96, 8);  // clear "Saved..." msg area
        Disp_SetFont(MONO_16_NORM);
        Disp_PosXY(40, 24);
        Disp_BlockClear(64, 16);  // clear existing data
        Disp_PutDecimal(ns_param.p_accid, 2);
        Disp_PutText(" %");
    }        
}   


void   UserMode_SettingOptRand()
{
    uint8  setting;
    bool  doRefresh = FALSE;
    
    if (isNewScreen)
    {
        DisplayTitleBar("Random Num Gen");
        DisplayButtonLegend(0, 53, "Home");
        DisplayButtonLegend(32, 53, "Back");
        DisplayButtonLegend(64, 53, "Save");
        DisplayButtonLegend(96, 53, "Next");
        doRefresh = TRUE;
    }
    
    if (PotMoved())
    {
        setting = ((int)PotPosition() * 10) / 255;  // span 0..10
        if (setting & 1)  ns_param.opt_rand = 1;  // Filter
        else  ns_param.opt_rand = 0;  // Gaussian
        doRefresh = TRUE;
    }
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('B'))  GoToNextScreen(SETTING_PR_ACCID);
    if (ButtonHit('C'))  // Save param's in EEPROM  
    {   
        EEPROM_WriteArray((uint8 *) &ns_param, sizeof(ns_param));
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 42);
        Disp_BlockClear(96, 8);  // clear message area
        Disp_PutText("* Setting saved");  // show message
    }        
    if (ButtonHit('D'))  GoToNextScreen(SETTING_SPREAD);

    if (doRefresh)
    {
        Disp_PosXY(16, 42);
        Disp_BlockClear(96, 8);  // clear message area
        Disp_SetFont(PROP_8_NORM);
        Disp_PosXY(16, 42);
        Disp_PutText("Random Number Series");
        Disp_SetFont(PROP_12_BOLD);
        Disp_PosXY(24, 24);
        Disp_BlockClear(88, 12);  // clear existing data
        if (ns_param.opt_rand == 0)  Disp_PutText("Gaussian");
        else  Disp_PutText("Filtered");
    }
}    


void   UserMode_SettingSpread()
{
    uint8  setting;
    bool  doRefresh = FALSE;
    
    if (isNewScreen)
    {
        DisplayTitleBar("Gauss 'Spread'");
        if (ns_param.opt_rand != 0)
        {
            Disp_SetFont(PROP_8_NORM);
            Disp_PosXY(4, 14);
            Disp_PutText("Gauss RNG not enabled!");
        }      
        DisplayButtonLegend(0, 53, "Home");
        DisplayButtonLegend(32, 53, "Back");
        DisplayButtonLegend(64, 53, "Save");
        DisplayButtonLegend(96, 53, "Next");
        doRefresh = TRUE;
    }
    
    if (PotMoved())
    {
        setting = ((int)PotPosition() * 25) / 255;  // span 0..25
        if (setting > 24)  setting = 24;
        ns_param.spread = (setting + 1) * 10;  // 10..250 in 25 steps
        doRefresh = TRUE;
    }        
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('B'))  GoToNextScreen(SETTING_OPT_RAND);
    if (ButtonHit('C'))  // Save param's in EEPROM
    {   
        EEPROM_WriteArray((uint8 *) &ns_param, sizeof(ns_param));
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 42);
        Disp_PutText("* Setting saved");
    }        
    if (ButtonHit('D'))  GoToNextScreen(SETTING_K_DAMP);

    if (doRefresh)
    {
        Disp_PosXY(16, 42);
        Disp_BlockClear(96, 8);  // clear "Saved..." msg area
        Disp_SetFont(MONO_16_NORM);
        Disp_PosXY(40, 24);
        Disp_BlockClear(56, 16);  // clear existing data
        Disp_PutDecimal(ns_param.spread, 3);
        Disp_PutText(" %");
    }
}    


void   UserMode_SettingFiltDamp()
{
    uint8  setting;
    bool  doRefresh = FALSE;
    
    if (isNewScreen)
    {
        DisplayTitleBar("Filter K-Damp");
        if (ns_param.opt_rand == 0)
        {
            Disp_SetFont(PROP_8_NORM);
            Disp_PosXY(4, 14);
            Disp_PutText("Filter RNG not enabled!");
        }            
        DisplayButtonLegend(0, 53, "Home");
        DisplayButtonLegend(32, 53, "Back");
        DisplayButtonLegend(64, 53, "Save");
        DisplayButtonLegend(96, 53, "Next");
        doRefresh = TRUE;
    }
    
    if (PotMoved())
    {
        setting = ((int)PotPosition() * 10) / 255;  // span 0..10
        if (setting > 9)  setting = 9;  // 0..9
        ns_param.k_damp = setting + 1;  // 1..10 in 10 steps
        doRefresh = TRUE;
    }        
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('B'))  GoToNextScreen(SETTING_SPREAD);
    if (ButtonHit('C'))  // Save param's in EEPROM
    {   
        EEPROM_WriteArray((uint8 *) &ns_param, sizeof(ns_param));
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 42);
        Disp_PutText("* Setting saved");
    }        
    if (ButtonHit('D'))  GoToNextScreen(SETTING_MID_NOTE);  // wrap

    if (doRefresh)
    {
        Disp_PosXY(16, 42);
        Disp_BlockClear(96, 8);  // clear "Saved..." msg area
        Disp_SetFont(MONO_16_NORM);
        Disp_PosXY(48, 24);
        Disp_BlockClear(24, 16);  // clear existing data
        Disp_PutDecimal(ns_param.k_damp, 2);
    }         
}

    
void   UserMode_SystemInfo()
{
    if (isNewScreen)
    {
        Disp_Mode(SET_PIXELS);
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(30, 0);
        Disp_PutText("System Info");
        Disp_PosXY(0, 10);
        Disp_DrawLineHoriz(128);

        Disp_SetFont(PROP_12_NORM);
        Disp_PosXY(0, 13);
        Disp_PutText("FW version: 1.0");

        Disp_PosXY(0, 26);
        Disp_PutText("");

        Disp_PosXY(0, 39);
        Disp_PutText("");

        Disp_PosXY(4, 52);
        Disp_PutText("www.mjbauer.biz");
    }
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('D'))  GoToNextScreen(HOME_SCREEN);
}
    

void  UserMode_DiagnosticInfo()
{
    static uint8  count_to_4;
    static uint8  lastPlayerState;
    static unsigned  lastTask1Freq;
    
    if (isNewScreen)
    {
        Disp_Mode(SET_PIXELS);
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 0);
        Disp_PutText("Diagnostic Mode");
        Disp_PosXY(0, 10);
        Disp_DrawLineHoriz(128);

        Disp_SetFont(PROP_12_NORM);
        Disp_PosXY(0, 13);
        Disp_PutText("P-state: ");

        Disp_PosXY(0, 26);
        Disp_PutText("Rand #: ");

        Disp_PosXY(0, 39);
        Disp_PutText("Note on: ");

        Disp_PosXY(0, 52);
        Disp_PutText("Task1: ");
        
        lastPlayerState = 0xFF;  // force refresh
    }
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('D'))  GoToNextScreen(HOME_SCREEN);

    Disp_Mode(SET_PIXELS);
    Disp_SetFont(PROP_12_NORM);

    if (playerState != lastPlayerState) 
    {
        Disp_PosXY(80, 13);
        Disp_BlockClear(20, 12);
        Disp_PutHexByte(playerState);
        lastPlayerState = playerState;
    }        
       
    if (notePlaying && (++count_to_4 >= 4))  // every 200ms...
    {
        count_to_4 = 0;
        Disp_PosXY(80, 26);
        Disp_BlockClear(16, 12);
        Disp_PutDecimal(randNote, 2);
            
        Disp_PosXY(80, 39);
        Disp_BlockClear(32, 12);
        Disp_PutDecimal(notePlaying, 2);
        if (isBlackNote) Disp_PutText(" B ");
        else  Disp_PutText(" W ");
    }            

    if (task1Freq != lastTask1Freq)
    {
        Disp_PosXY(60, 52);
        Disp_BlockClear(40, 12);
        Disp_PutDecimal(task1Freq, 4);
        Disp_PutText("Hz");
        lastTask1Freq = task1Freq;
    }     
}     


/*****
void  UserMode_DiagnosticInfo()
{
    static uint8  count_to_4;
    
    if (isNewScreen)
    {
        Disp_Mode(SET_PIXELS);
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(16, 0);
        Disp_PutText("Diagnostic Mode");
        Disp_PosXY(0, 10);
        Disp_DrawLineHoriz(128);

        Disp_SetFont(PROP_12_NORM);
        Disp_PosXY(0, 13);
        Disp_PutText("Pad 1: ");

        Disp_PosXY(0, 26);
        Disp_PutText("Pad 2: ");

        Disp_PosXY(0, 39);
        Disp_PutText("Pad 3: ");
    }
    
    if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
    if (ButtonHit('D'))  GoToNextScreen(HOME_SCREEN);

    Disp_Mode(SET_PIXELS);
    Disp_SetFont(PROP_12_NORM);

    if (++count_to_4 >= 4)  // every 200ms...
    {
        count_to_4 = 0;
        Disp_PosXY(64, 13);
        Disp_BlockClear(40, 40);
        Disp_PosXY(64, 13);
        Disp_PutDecimal(touch_reading[0], 3);
        Disp_PosXY(64, 26);
        Disp_PutDecimal(touch_reading[1], 3);
        Disp_PosXY(64, 39);
        Disp_PutDecimal(touch_reading[2], 3);
    }            
}     
*****/

//===================   P U S H - B U T T O N   F U N C T I O N S  ======================
//
const char button_code[] = { 'A', 'B', 'C', 'D' };  // ASCII code table
static bool  is_button_hit[4];   // TRUE if button[x] hit detected

/*
 * Function ButtonScan() must be called periodically from the application program
 * (main loop) at intervals of about 30 ~ 50ms for reliable "de-bounce" operation.
 * Button inputs must have MCU internal pull-ups enabled if no external pull-ups.
 *
 * Entry argument 'nButts' specifies the number of buttons (1..4) to be serviced.
 * For example, if nButts is 1, only Button_A is serviced;  if nButts is 3, then 3
 * buttons (Button_A, Button_B and Button_C) will be serviced.
 */
void  ButtonScan(unsigned char nButts)
{
    static bool  button_down_last_scan[4];
    bool  is_button_down;   // TRUE if button is pressed
    unsigned char x;        // button array index (0..3)

    if (nButts > 4)  nButts = 4;

    for (x = 0 ; x < nButts ; x++)    // Scan buttons up to nButts
    {
        is_button_down = !TEST_BIT(PINB, (x + 1));  // Read button input state

        if (is_button_down && !button_down_last_scan[x])  // hit detected
            is_button_hit[x] = TRUE;
 
        button_down_last_scan[x] = is_button_down;  // update status
    }
}

/*
 * Function ButtonHit() returns the Boolean value (TRUE or FALSE) of a flag indicating
 * whether or not a "button hit" event occurred since the previous call to the function.
 *
 * Entry argument 'button_ID' is an ASCII code identifying one of 4 buttons to check,
 * which must be one of: 'A', 'B', 'C' or 'D', otherwise the function will return FALSE.
 */
bool  ButtonHit(char  button_ID)
{
    bool  result = FALSE;
    uint8  x;

    for (x = 0 ; x < 4 ; x++)
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


//----------------------------------------------------------------------------------------
// Functions to support data-entry potentiometer
//----------------------------------------------------------------------------------------
//
static unsigned long  potReadingAve;   // smoothed pot reading [24:8 fixed-pt]
//
/*
 * Function:  PotService()
 *
 * Overview:  Service Routine for front-panel data-entry pot.
 *            Non-blocking "task" called at 5ms intervals from main loop.
 *
 * Detail:    The routine reads the pot input and keeps a rolling average of the ADC
 *            readings in fixed-point format (24:8 bits);  range 0.0 ~ 1023.0
 *            The current pot position can be read by a call to function PotPosition().
 */
void  PotService()
{
    long  potReading = (long) ADC_ReadInput(8);  // 10-bit reading
    
    // Apply 1st-order IIR filter (K = 0.25)
    potReading = potReading << 8;  // convert to fixed-point (24:8 bits)
    potReadingAve -= (potReadingAve >> 2);
    potReadingAve += (potReading >> 2);
}

/*
 * Function:     PotMoved()
 *
 * Overview:     Returns TRUE if the pot position has changed by more than 2% since a
 *               previous call to the function which also returned TRUE.
 *
 * Return val:   (bool) status flag, value = TRUE or FALSE
 */
bool  PotMoved()
{
    static long  lastReading;
    bool  result = FALSE;

    if (labs(potReadingAve - lastReading) > (20 << 8))  // 20/1024 -> 2%
    {
        result = TRUE;
        lastReading = potReadingAve;  
    }
    return  result;
}

/*
 * Function:     PotPosition()
 *
 * Overview:     Returns the current position of the data-entry pot, averaged over 
 *               several ADC readings, as an 8-bit unsigned integer.
 *
 * Return val:   (uint8) Filtered Pot position reading, range 0..255.
 */
uint8  PotPosition()
{
    return  (uint8) (potReadingAve >> 10);  // = (Integer part) / 4
}

