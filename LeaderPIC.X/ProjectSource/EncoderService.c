/****************************************************************************
 Module
   EncoderService.c

 Revision
   1.0.0

 Description
   This service handles input capture from an encoder to measure RPM.
   Uses interrupt-driven input capture to record encoder edge timing
   and calculates rotational speed.

 Notes
    When initializing the Encoder service:
              Map the selected Input Capture to Input Capture pin
    Set the Input Capture pin to digital, input mode
    Configure Input Capture pin as digital input

    Configure a dedicated encoder timer:
        Disable the timer
        Select internal PBCLK
        Set timer prescaler
        Clear TMRx
        Load PRx with maximum period
        Clear timer interrupt flag
        Configure the interrupt priority to 6 and subpriority to 0
    Set the T3IE interrupt bit to enable in the IEC0 register
        Enable the timer

    Configure Input Capture module:
        Disable the base timer
        Disable input capture module
        Select encoder timer as time base
        Configure capture on every 16th rising edge
        Clear input capture interrupt flag
        Clear input capture buffer
        Set interrupt priority to 7 and subpriority to 0
        Enable input capture interrupt
        Enable input capture module
        Enable the base timer

    Set PRINT_RPM_SPEED timeout to 100 ms
    Start the RPM print timer with PRINT_RPM_SPEED
    Initialize LastCapturedTime to invalid value

    Input Capture interrupt response routine:
    Read ICxBUF into a local variable CapturedTime
    Clear the input capture interrupt flag
    IF TxIF is pending and captured value is after rollover (in lower half of timer range)
        Increment the roll-over counter
        Clear the roll-over interrupt flag
    Combine roll-over counter with captured timer value to create a full 32-bit time
    Store the full 32-bit time in a module-level variable CapturedTime

    Post an ES_NEW_ENCODER_EDGE event

    IC Base Timer interrupt response routine:
        Disable interrupts globally to prevent race condition with IC ISR
        If TxIF is pending (timer has rolled over)
            Increment the roll-over counter to track timer wraparounds
            Clear the roll-over interrupt flag
        Re-enable interrupts globally

    On ES_NEW_ENCODER_EDGE event:
        Latch current captured time from the module-level CapturedTime
    If LastCapturedTime is valid:
        Calculate time lapse:
        IF (CurrentCapturedTime >= LastCapturedTime)
            timeLapse = CurrentCapturedTime – LastCapturedTime
        ELSE
            timeLapse = (TIMER_MAX_PERIOD – LastCapturedTime) + CurrentCapturedTime + 1
    Store current capture as the last capture for next calculation

    On ES_TIMEOUT for PRINT_RPM_SPEED:
      Calculate RPM from (timeLapse)
      Print RPM value to screen
      Restart the RPM print timer



 History
 When           Who     What/Why
 -------------- ---     --------
 01/21/26       Tianyu  Initial creation for Lab 6
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "EncoderService.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"
#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
// Timer definitions
#define PRINT_RPM_INTERVAL 100 // Print RPM every 100ms

// Input Capture pin configuration (IC1 on RB2/pin 6)
#define IC_PIN_TRIS TRISBbits.TRISB2
#define IC_PIN_ANSEL ANSELBbits.ANSB2

// Timing pin for performance measurement (using RB15/pin 26)
#define TIMING_PIN_TRIS TRISBbits.TRISB15
#define TIMING_PIN_ANSEL ANSELBbits.ANSB15


// Timer configuration
#define TIMER_PRESCALE 256
#define PRESCALE_CHOSEN PRESCALE_256

// RPM calculation constants
#define INVALID_TIME 0xFFFFFFFF // Marker for invalid/uninitialized time


/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service */
static void ConfigureEncoderTimer(void);
static void ConfigureInputCapture(void);
static void ConfigureTimingPin(void);

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
static uint32_t timeLapse = 0;

static bool ICFiredFlag = false;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitEncoderService

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the Input Capture module, encoder timer, and timing pin

 Author
     Tianyu, 01/21/26
****************************************************************************/
bool InitEncoderService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  /********************************************
   Initialization code for Encoder Service
   *******************************************/
  
  // Configure the Input Capture pin as digital input
  IC_PIN_TRIS = 1;   // Set as input
  IC_PIN_ANSEL = 0;

  IC1R = 0b0100; // Map IC1 to RB2

  // Configure timing pin for performance measurement
  ConfigureTimingPin();
  
  // Configure the dedicated encoder timer
  ConfigureEncoderTimer();
  
  // Configure the Input Capture module
  ConfigureInputCapture();
  
  // Initialize timing variables
  LastCapturedTime = INVALID_TIME;
  CapturedTime = 0;
  
  // Start the RPM print timer
  ES_Timer_InitTimer(PRINT_RPM_TIMER, PRINT_RPM_INTERVAL);
  
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
     PostEncoderService

 Parameters
     ES_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Tianyu, 01/21/26
****************************************************************************/
bool PostEncoderService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunEncoderService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles encoder edge events and timeout events for RPM printing

 Author
   Tianyu, 01/21/26
****************************************************************************/
ES_Event_t RunEncoderService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
      // Initialization complete, nothing additional to do
      DB_printf("Encoder Service Initialized\r\n");
      break;
      
    case ES_NEW_ENCODER_EDGE:
    {
        
        // Latch current captured time
        uint32_t CurrentCapturedTime = CapturedTime;
      
      // Calculate time lapse if we have a valid previous capture
      if (LastCapturedTime != INVALID_TIME)
      {

        // Calculate time lapse with wraparound handling
        
        if (CurrentCapturedTime >= LastCapturedTime)
        {
          timeLapse = CurrentCapturedTime - LastCapturedTime;
                  

        }
        else
        {
          // Handle timer wraparound
          timeLapse = (0xFFFFFFFF - LastCapturedTime) + CurrentCapturedTime + 1;
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
      }
      
      // Store current capture as the last capture for next calculation
      LastCapturedTime = CurrentCapturedTime;
      break;
    }
    
    case ES_TIMEOUT:
      // Check if this is the RPM print timer
      if (ThisEvent.EventParam == PRINT_RPM_TIMER)
      {
        // Calculate and print RPM
        float rpm = PeriodToRPM(timeLapse);
          
        // Convert to integer (with 2 decimal place)
        uint32_t rpm_hundreds = (uint32_t)(rpm * 100);
        if (ICFiredFlag){
            DB_printf("RPM: %d.%d\r\n", rpm_hundreds / 100, rpm_hundreds % 100);
            ICFiredFlag = false;
        }
        else{
            timeLapse = 0;
            DB_printf("RPM: 0\r\n");
        }
        
        
        // Restart the RPM print timer
        ES_Timer_InitTimer(PRINT_RPM_TIMER, PRINT_RPM_INTERVAL);
      }
      break;
      
    default:
      break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     Encoder_GetLatestPeriod

 Parameters
     None

 Returns
     uint32_t - the latest measured period in timer ticks

 Description
     Query function that returns the smoothed time period between encoder
     edges. Other services can call this to get the current encoder period
     measurement for speed control or monitoring purposes.

 Author
     Tianyu, 01/28/26
****************************************************************************/
uint32_t Encoder_GetLatestPeriod(void)
{
  return timeLapse;
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
     and posts an event to the encoder service.

 Author
     Tianyu, 01/21/26
****************************************************************************/
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL7SOFT) InputCaptureISR(void)
{

  // Read the captured timer value from IC buffer
  uint16_t capturedTimer16 = IC1BUF;
      
  // Clear the input capture interrupt flag
  IFS0CLR = _IFS0_IC1IF_MASK;
  
  ICFiredFlag = true;

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
  NewEvent.EventType = ES_NEW_ENCODER_EDGE;
  PostEncoderService(NewEvent);
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
     Tianyu, 01/22/26
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
     ConfigureEncoderTimer

 Parameters
     None

 Returns
     None

 Description
     Configures Timer3 as the time base for input capture with maximum period

 Author
     Tianyu, 01/21/26
****************************************************************************/
static void ConfigureEncoderTimer(void)
{
  // Disable the timer during configuration
  T3CONbits.ON = 0;
  
  // Select internal PBCLK as clock source (default)
  T3CONbits.TCS = 0;
  
  // Set timer prescaler using lookup table for readability
  T3CONbits.TCKPS = PrescaleLookup[PRESCALE_CHOSEN];
  
  // Clear timer register
  TMR3 = 0;
  
  // Load period register with maximum value for maximum range (PR3 is 16-bit)
  PR3 = 0xFFFF;
  
  // Clear timer interrupt flag
  IFS0bits.T3IF = 0;

  // Configure the interrupt priority and subpriority levels in the IPCx register
  IPC3bits.T3IP = 6; // Priority 6
  IPC3bits.T3IS = 0; // Subpriority 0

  // Set (enable) the T3IE interrupt bit in the IEC0 register
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
     Tianyu, 01/21/26
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
     ConfigureTimingPin

 Parameters
     None

 Returns
     None

 Description
     Configures a GPIO pin for timing/performance measurement

 Author
     Tianyu, 01/21/26
****************************************************************************/
static void ConfigureTimingPin(void)
{
  // Configure timing pin as digital output
  TIMING_PIN_TRIS = 0; // Output
  TIMING_PIN_LAT = 0;  // Initialize low
  TIMING_PIN_ANSEL = 0; // Disable analog functionw
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
