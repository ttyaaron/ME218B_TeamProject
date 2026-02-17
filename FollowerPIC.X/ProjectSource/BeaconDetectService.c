/****************************************************************************
 Module
   BeaconDetectService.c

 Revision
   1.0.0

 Description
   This service handles input capture from a phototransistor circuit to
   measure beacon frequency. Uses interrupt-driven input capture to record
   signal edge timing and detect 1427Hz beacon signal.

 Notes
    When initializing the Beacon Detection service:
        Set the Input Capture pin to digital, input mode
        Configure Input Capture pin as digital input

        Configure a dedicated timer:
            Disable the timer
            Select internal PBCLK
            Set timer prescaler
            Clear TMRx
            Load PRx with maximum period
            Clear timer interrupt flag
            Enable the timer

        Configure Input Capture module:
            Select timer as time base
            Configure capture on rising (or desired) edge
            Clear input capture interrupt flag
            Enable input capture interrupt

        Set PRINT_FREQUENCY_INTERVAL timeout to 100 ms
        Initialize LastCapturedTime to invalid value

    Input Capture interrupt response routine:
        Read ICxBUF into a local variable CapturedTime
        Post an ES_NEW_SIGNAL_EDGE event
        Clear the capture interrupt flag

    On ES_NEW_SIGNAL_EDGE event:
        If LastCapturedTime is valid:
            Calculate time lapse:
                (CapturedTime − LastCapturedTime + TimerMaximum) mod TimerMaximum
            Calculate frequency from time lapse
            If frequency is close to 1427 Hz, post ES_BEACON_DETECTED event

        Store CapturedTime into LastCapturedTime

    On ES_TIMEOUT for PRINT_FREQUENCY_TIMER:
        Calculate frequency from (current time − LastCapturedTime)
        Print frequency value to screen

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 beacon detection
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "BeaconDetectService.h"
#include "MainLogicFSM.h"
#include "dbprintf.h"
#include "TimerConfig.h"
#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
// Timer definitions
#define PRINT_FREQUENCY_INTERVAL 100 // Print frequency every 100ms

// Input Capture pin configuration (IC1 on RB2/pin 6)
#define IC_PIN_TRIS TRISBbits.TRISB2
#define IC_PIN_ANSEL ANSELBbits.ANSB2


// Timer configuration
#define TIMER_PRESCALE 256
#define PRESCALE_CHOSEN PRESCALE_256
#define TIMER_MAX_PERIOD 0xFFFFFFFF // 32-bit timer maximum

// Frequency calculation constants
#define INVALID_TIME 0xFFFFFFFF // Marker for invalid/uninitialized time
#define IC_PRESCALE 16          // Input Capture prescale (captures every 16th edge)
#define PBCLK_FREQ 20000000     // 20 MHz peripheral bus clock

// Beacon detection parameters
#define TARGET_BEACON_FREQ 1427  // Target beacon frequency in Hz
#define BEACON_FREQ_TOLERANCE 50 // ±50 Hz tolerance for beacon detection

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service */
static void ConfigureICTimer(void);
static void ConfigureInputCapture(void);
static uint32_t CalculateFrequency(uint32_t timeLapse);

/*---------------------------- Module Variables ---------------------------*/
// Module level Priority variable
static uint8_t MyPriority;

// Time capture variables
static uint32_t LastCapturedTime = INVALID_TIME;
static volatile uint32_t CapturedTime = 0;
static volatile uint16_t RolloverCounter = 0; // Counts Timer3 rollovers for extended timing

// For time lapse smoothing
static uint32_t SmoothedTimeLapse = 0;
static bool FirstSample = true;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitBeaconDetectService

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the Input Capture module, phototransistor timer, and timing pin

 Author
     Tianyu, 02/03/26
****************************************************************************/
bool InitBeaconDetectService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  /********************************************
   Initialization code for Beacon Detection Service
   *******************************************/
  
  // Configure the Input Capture pin as digital input
  IC_PIN_TRIS = 1;   // Set as input
  IC_PIN_ANSEL = 0;

  IC1R = 0b0100; // Map IC1 to RB2
  
  
  // Configure the dedicated phototransistor timer
  ConfigureICTimer();
  
  // Configure the Input Capture module
  ConfigureInputCapture();
  
  // Configure timing pin for performance measurement
  ConfigureTimingPin();
  
  // Initialize timing variables
  LastCapturedTime = INVALID_TIME;
  CapturedTime = 0;
  
  // Start the frequency print timer
  ES_Timer_InitTimer(PRINT_FREQUENCY_TIMER, PRINT_FREQUENCY_INTERVAL);
  
  // Post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostBeaconDetectService

 Parameters
     ES_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Tianyu, 02/03/26
****************************************************************************/
bool PostBeaconDetectService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunBeaconDetectService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles signal edge events and timeout events for frequency printing
   and beacon detection

 Author
   Tianyu, 02/03/26
****************************************************************************/
ES_Event_t RunBeaconDetectService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
      // Initialization complete, nothing additional to do
      DB_printf("Beacon Detection Service Initialized\r\n");
      break;
      
    case ES_NEW_SIGNAL_EDGE:
    {
        
        // Latch current captured time
        uint32_t CurrentCapturedTime = CapturedTime;
      
      // Calculate time lapse if we have a valid previous capture
      if (LastCapturedTime != INVALID_TIME)
      {

        // Calculate time lapse with wraparound handling
        uint32_t timeLapse;
        if (CurrentCapturedTime >= LastCapturedTime)
        {
          timeLapse = CurrentCapturedTime - LastCapturedTime;
                  

        }
        else
        {
          // Handle timer wraparound
          timeLapse = (TIMER_MAX_PERIOD - LastCapturedTime) + CurrentCapturedTime + 1;
        }    
        
        if (FirstSample)
        {
          SmoothedTimeLapse = timeLapse;
          FirstSample = false;
        }
        else
        {
          SmoothedTimeLapse = (timeLapse + 5 * SmoothedTimeLapse) / 6;
        }
        
        // Calculate frequency and check if it matches beacon frequency
        uint32_t frequency = CalculateFrequency(SmoothedTimeLapse);
        
        // Check if frequency is close to target beacon frequency
        if ((frequency >= (TARGET_BEACON_FREQ - BEACON_FREQ_TOLERANCE)) &&
            (frequency <= (TARGET_BEACON_FREQ + BEACON_FREQ_TOLERANCE)))
        {
          // Post beacon detected event to MainLogicFSM
          ES_Event_t BeaconEvent;
          BeaconEvent.EventType = ES_BEACON_DETECTED;
          BeaconEvent.EventParam = frequency;
          PostMainLogicFSM(BeaconEvent);
        }
      }
      
      // Store current capture as the last capture for next calculation
      LastCapturedTime = CurrentCapturedTime;
      break;
    }
    
    case ES_TIMEOUT:
      // Check if this is the frequency print timer
      if (ThisEvent.EventParam == PRINT_FREQUENCY_TIMER)
      {
        // Calculate and print frequency
        TIMING_PIN_LAT = 1; // Raise timing pin
        uint32_t frequency = CalculateFrequency(SmoothedTimeLapse);
        TIMING_PIN_LAT = 0; // Lower timing pin
          
        // Print frequency to screen
        DB_printf("Frequency: %d Hz\r\n", frequency);
        
        // Restart the frequency print timer
        ES_Timer_InitTimer(PRINT_FREQUENCY_TIMER, PRINT_FREQUENCY_INTERVAL);
      }
      break;
      
    default:
      break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     InputCaptureISR

 Parameters
     None

 Returns
     None

 Description
     Input Capture interrupt response routine. Reads the captured time
     and posts an event to the beacon detection service.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL7SOFT) InputCaptureISR(void)
{

  // Read the captured timer value from IC buffer
  uint16_t capturedTimer16 = IC1BUF;
      
  // Clear the input capture interrupt flag
  IFS0CLR = _IFS0_IC1IF_MASK;

  // If T3IF is pending and captured value is after rollover (in lower half of timer range)
  if (IFS0bits.T3IF && (capturedTimer16 < 0x8000))
  {
      // Increment the roll-over counter
      RolloverCounter++;

      // Clear the roll-over interrupt flag
      IFS0CLR = _IFS0_T3IF_MASK;
  }

  // Combine roll-over counter with captured timer value to create a full 32-bit time
  CapturedTime = ((uint32_t)RolloverCounter << 16) | capturedTimer16;
  
  // Post event for the captured value
  ES_Event_t NewEvent;
  NewEvent.EventType = ES_NEW_SIGNAL_EDGE;
  PostBeaconDetectService(NewEvent);
}

/****************************************************************************
 Function
     Timer3ISR

 Parameters
     None

 Returns
     None

 Description
     Timer3 interrupt response routine. Increments the rollover counter
     to extend timing range beyond 16-bit timer. Works in collaboration
     with Input Capture ISR to provide extended 32-bit timing for long
     period measurements.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void __ISR(_TIMER_3_VECTOR, IPL6SOFT) Timer3ISR(void)
{
    // Disable interrupts globally to prevent race condition with IC ISR
    __builtin_disable_interrupts();
    
    // If T3IF is pending (timer has rolled over)
    if (IFS0bits.T3IF)
    {
        // Increment the roll-over counter to track timer wraparounds
        RolloverCounter++;

        // Clear the roll-over interrupt flag
        IFS0CLR = _IFS0_T3IF_MASK;
    }

    // Re-enable interrupts globally
    __builtin_enable_interrupts();
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
     ConfigureICTimer

 Parameters
     None

 Returns
     None

 Description
     Configures Timer3 as the time base for input capture with maximum period

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void ConfigureICTimer(void)
{
  // Disable the timer during configuration
  T3CONbits.ON = 0;
  
  // Select internal PBCLK as clock source (default)
  T3CONbits.TCS = 0;
  
  // Set timer prescaler using lookup table for readability
  T3CONbits.TCKPS = PrescaleLookup[PRESCALE_CHOSEN];
  
  // Clear timer register
  TMR3 = 0;
  
  // Load period register with maximum value for maximum range
  PR3 = TIMER_MAX_PERIOD;
  
  // Clear timer interrupt flag
  IFS0bits.T3IF = 0;

  // Configure the interrupt priority and subpriority levels in the IPCx register
  IPC3bits.T3IP = 6; // Priority 6
  IPC3bits.T3IS = 0; // Subpriority 0

  // Set the T3IE interrupt bit in the IEC0 register
  IEC0bits.T3IE = 1;
  
  // Enable the timer
  T3CONbits.ON = 1;
}

/****************************************************************************
 Function
     ConfigureInputCapture

 Parameters
     None

 Returns
     None

 Description
     Configures Input Capture module 1 to capture on rising edges using
     Timer3 as the time base

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void ConfigureInputCapture(void)
{
  // Disable base timer during configuration
  T3CONbits.ON = 0;

  // Disable Input Capture module during configuration
  IC1CONbits.ON = 0;
  
  // Select Timer3 as time base (ICTMR = 0 means Timer3 for IC1)
  IC1CONbits.ICTMR = 0;
  
  // Configure to capture on every 16th rising edge (ICM = 101)
  IC1CONbits.ICM = 0b101;
  
  // Clear the input capture interrupt flag
  IFS0CLR = _IFS0_IC1IF_MASK;

  // Clear IC buffer
  volatile uint32_t dummy;
  while (IC1CONbits.ICBNE) {
      dummy = IC1BUF;
  }
  
  // Set interrupt priority and subpriority
  IPC1bits.IC1IP = 7; // Priority 7
  IPC1bits.IC1IS = 0; // Subpriority 0
  
  // Enable input capture interrupt
  IEC0bits.IC1IE = 1;
  
  // Enable the Input Capture module
  IC1CONbits.ON = 1;

  // Enable the timer after IC configuration is complete
  T3CONbits.ON = 1;
}

/****************************************************************************
 Function
     CalculateFrequency

 Parameters
     uint32_t timeLapse - time between signal edges in timer ticks

 Returns
     uint32_t - calculated frequency in Hz

 Description
     Calculates frequency based on the time between signal edges

 Author
     Tianyu, 02/03/26
****************************************************************************/
static uint32_t CalculateFrequency(uint32_t timeLapse)
{
  // Prevent division by zero
  if (timeLapse == 0)
  {
    return 0;
  }
  
  // Calculate timer clock frequency
  uint32_t timerClock = PBCLK_FREQ / TIMER_PRESCALE;
  
  // Calculate frequency in Hz
  // frequency = timerClock / (timeLapse * IC_PRESCALE)
  uint32_t frequency = timerClock / (timeLapse * IC_PRESCALE);
  
  return frequency;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
