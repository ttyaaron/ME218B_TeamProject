/****************************************************************************
 Module
   BeaconDetectFSM.c

 Revision
   2.0.0

 Description
   Flat state machine for beacon frequency detection using Input Capture.
   Timer3 and IC1 measure time between every 16th rising edge from a
   phototransistor, and the resulting frequency is compared against several
   target beacons' frequency.

   State machine:

     InitPState
       |--[ES_INIT]--> NoSignal (entry: start PRINT_FREQUENCY_TIMER)

     NoSignal
       |--[ES_NEW_SIGNAL_EDGE]--> SignalDetected
       |       entry actions: reset timing history, start SIGNAL_WATCHDOG_TIMER
       |--[ES_TIMEOUT / PRINT_FREQUENCY_TIMER]--> NoSignal (self)
               action: print 0 Hz, restart PRINT_FREQUENCY_TIMER

     SignalDetected
       |--[ES_NEW_SIGNAL_EDGE]--> SignalDetected (self)
       |       action: compute smoothed frequency, restart SIGNAL_WATCHDOG_TIMER,
       |               post ES_BEACON_DETECTED if within tolerance
       |--[ES_TIMEOUT / SIGNAL_WATCHDOG_TIMER]--> NoSignal
       |       entry actions: reset timing history
       |--[ES_TIMEOUT / PRINT_FREQUENCY_TIMER]--> SignalDetected (self)
               action: print current frequency, restart PRINT_FREQUENCY_TIMER

 Notes
   IC_PRESCALE = 16: IC1 fires on every 16th rising edge, so each time lapse
   spans 16 signal periods. The correct frequency formula is therefore:
       frequency = (timerClock * IC_PRESCALE) / timeLapse

   RolloverCounter extends 16-bit Timer3 to a 32-bit virtual timestamp.
   The IC ISR (priority 7) and Timer3 ISR (priority 6) coordinate to avoid
   double-counting rollovers at the 0xFFFF boundary.

   Both PRINT_FREQUENCY_TIMER and SIGNAL_WATCHDOG_TIMER must be declared
   in ES_Configure.h.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 beacon detection
 02/19/26       Tianyu  Refactored into BeaconDetectFSM with NoSignal /
                        SignalDetected states and signal watchdog timer
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "BeaconDetectFSM.h"
#include "MainLogicFSM.h"
#include "dbprintf.h"
#include "CommonDefinitions.h"
#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
// Periodic debug-print interval
#define PRINT_FREQUENCY_INTERVAL  100   // ms

// Watchdog: if no edge arrives within this window, declare signal lost.
// At 1427 Hz with IC_PRESCALE=16 an edge arrives every ~11 ms, so 50 ms
// gives roughly 4 missed captures before declaring signal lost.
#define SIGNAL_WATCHDOG_INTERVAL  50    // ms

// Input Capture pin (IC1 mapped to RB2)
#define IC_PIN_TRIS   TRISBbits.TRISB2
#define IC_PIN_ANSEL  ANSELBbits.ANSB2

// Timer3 configuration
#define TIMER_PRESCALE   256
#define PRESCALE_CHOSEN  PRESCALE_256
#define TIMER_MAX_PERIOD 0xFFFF   // PR3 = 0xFFFF (16-bit timer, full range)

// Timestamp and frequency constants
#define INVALID_TIME     0xFFFFFFFF  // Sentinel for uninitialized LastCapturedTime
#define IC_PRESCALE      16          // IC fires on every 16th rising edge
#define PBCLK_FREQ       20000000    // 20 MHz peripheral bus clock

// Beacon detection window
#define BEACON_G_FREQ    3333   // Hz
#define BEACON_B_FREQ    1427   // Hz
#define BEACON_R_FREQ    909    // Hz
#define BEACON_L_FREQ    2000   // Hz

typedef struct {
    uint32_t freq;
    char     id;
} BeaconDef_t;

static const BeaconDef_t BeaconTable[] = {
    { BEACON_G_FREQ, 'g' },
    { BEACON_B_FREQ, 'b' },
    { BEACON_R_FREQ, 'r' },
    { BEACON_L_FREQ, 'l' },
};
#define NUM_BEACONS (sizeof(BeaconTable) / sizeof(BeaconTable[0]))
#define BEACON_FREQ_TOLERANCE 100     // �50 Hz

// Debouncing: require this many consecutive detections of the same beacon
// before locking onto it (prevents false positives from noise/bouncing)
#define BEACON_DEBOUNCE_THRESHOLD 1

/*---------------------------- Module Functions ---------------------------*/
static void     ConfigureICTimer(void);
static void     ConfigureInputCapture(void);
static uint32_t CalculateFrequency(uint32_t timeLapse);
static void     ResetSignalHistory(void);
static uint32_t UpdateSmoothingFilter(uint32_t currentCapturedTime);
static int8_t   FindMatchingBeacon(uint32_t frequency);
static void     HandleCommonTimeout(ES_Event_t ThisEvent);

/*---------------------------- Module Variables ---------------------------*/
// FSM state and priority
static BeaconState_t CurrentState;
static uint8_t       MyPriority;

// Shared between ISRs and the FSM (must be volatile)
static volatile uint32_t CapturedTime    = 0;
static volatile uint16_t RolloverCounter = 0;

// Timing history (only touched in task context, not in ISRs)
static uint32_t LastCapturedTime  = INVALID_TIME;
static uint32_t SmoothedTimeLapse = 0;
static bool     FirstSample       = true;

// Tracks which beacon is currently locked (set on BeaconLocked entry)
static char LockedBeaconId = 0;

// Debouncing variables
static char     CandidateBeaconId = 0;      // Beacon ID currently being validated
static uint8_t  BeaconMatchCount  = 0;      // Consecutive detections of CandidateBeaconId

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitBeaconDetectFSM

 Parameters
     uint8_t Priority : the priority of this service

 Returns
     bool - false if initialization failed, true otherwise

 Description
     Saves priority, configures the IC pin, Timer3, and IC1 hardware,
     initialises all timing variables, starts the periodic print timer,
     and posts ES_INIT to enter the initial pseudo-state.

 Author
     Tianyu, 02/19/26
****************************************************************************/
bool InitBeaconDetectFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority   = Priority;
  CurrentState = InitPState;

  // Configure the Input Capture pin as a digital input
  IC_PIN_TRIS  = 1;   // input direction
  IC_PIN_ANSEL = 0;   // disable analog
  IC1R         = 0b0100; // PPS: map IC1 input to RB2

  // Configure Timer3 as the IC time base
  ConfigureICTimer();

  // Configure Input Capture module 1
  ConfigureInputCapture();

  // Initialise timing variables to known-invalid state
  ResetSignalHistory();
  CapturedTime = 0;

  // Start the periodic frequency-print timer
  ES_Timer_InitTimer(PRINT_FREQUENCY_TIMER, PRINT_FREQUENCY_INTERVAL);

  // Post the initial transition event to kick the FSM
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
     PostBeaconDetectFSM

 Parameters
     ES_Event_t ThisEvent : the event to post to this FSM's queue

 Returns
     bool - false if the enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue.

 Author
     Tianyu, 02/19/26
****************************************************************************/
bool PostBeaconDetectFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     RunBeaconDetectFSM

 Parameters
     ES_Event_t ThisEvent : the event to process

 Returns
     ES_Event_t - ES_NO_EVENT if no error, ES_ERROR otherwise

 Description
     Implements the two-state beacon detection FSM using nested
     switch/case per the Gen2 template pattern.

     NoSignal state
       ES_NEW_SIGNAL_EDGE  --> transition to SignalDetected; reset timing
                               history and start the signal watchdog timer.
       ES_TIMEOUT (PRINT)  --> stay in NoSignal; print 0 Hz and restart
                               the print timer.

     SignalDetected state
       ES_NEW_SIGNAL_EDGE  --> stay in SignalDetected; compute smoothed
                               frequency, restart watchdog, post
                               ES_BEACON_DETECTED if within tolerance.
       ES_TIMEOUT (WATCHDOG)--> transition to NoSignal; reset timing history.
       ES_TIMEOUT (PRINT)  --> stay in SignalDetected; print current
                               frequency and restart the print timer.

 Author
     Tianyu, 02/19/26
****************************************************************************/
ES_Event_t RunBeaconDetectFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    /*--------------------------------------------------------------------
      InitPState: initial pseudo-state, only responds to ES_INIT
    --------------------------------------------------------------------*/
    case InitPState:
    {
      if (ThisEvent.EventType == ES_INIT)
      {
        // Transition into the actual initial state
        CurrentState = NoSignal;
        DB_printf("BeaconDetectFSM Initialized -> NoSignal\r\n");
      }
    }
    break;

    /*--------------------------------------------------------------------
      NoSignal: no valid edges received; frequency is 0 Hz
    --------------------------------------------------------------------*/
    case NoSignal:
    {
      switch (ThisEvent.EventType)
      {
        case ES_NEW_SIGNAL_EDGE:
        {
          // First edge after a quiet period — begin tracking
          // Reset all history so the first inter-edge measurement is clean
          ResetSignalHistory();
          
          // Clear any stale beacon ID from previous detections
          LockedBeaconId = 0;

          // Latch the timestamp recorded by the ISR
          LastCapturedTime = CapturedTime;

          // Arm the watchdog; it will fire if edges stop arriving
          ES_Timer_InitTimer(SIGNAL_WATCHDOG_TIMER, SIGNAL_WATCHDOG_INTERVAL);

          // Transition to SignalDetected
          CurrentState = SignalDetected;
          DB_printf("NoSignal -> SignalDetected\r\n");
        }
        break;

        case ES_TIMEOUT:
        {
          if (ThisEvent.EventParam == PRINT_FREQUENCY_TIMER)
          {
            // No signal - report 0 Hz and restart the print timer
            DB_printf("Frequency: 0 Hz (no signal)\r\n");
            ES_Timer_InitTimer(PRINT_FREQUENCY_TIMER, PRINT_FREQUENCY_INTERVAL);
          }
          // Ignore SIGNAL_WATCHDOG_TIMER timeouts while already in NoSignal
        }
        break;

        default:
          break;
      } // end switch on event in NoSignal
    }
    break;

    /*--------------------------------------------------------------------
      SignalDetected: edges are arriving; compute and report frequency
    --------------------------------------------------------------------*/
    case SignalDetected:
    {
      switch (ThisEvent.EventType)
      {
        case ES_NEW_SIGNAL_EDGE:
        {
          uint32_t CurrentCapturedTime = CapturedTime;

          // Kick the watchdog on every arriving edge
          ES_Timer_InitTimer(SIGNAL_WATCHDOG_TIMER, SIGNAL_WATCHDOG_INTERVAL);

          // Update smoothing filter; returns 0 if not enough data yet
          uint32_t frequency = UpdateSmoothingFilter(CurrentCapturedTime);

          if (frequency > 0)
          {
            int8_t beaconIdx = FindMatchingBeacon(frequency);
            if (beaconIdx >= 0)
            {
              char detectedId = BeaconTable[beaconIdx].id;
              
              // Debouncing: check if it's the same beacon as before
              if (detectedId == CandidateBeaconId)
              {
                // Same beacon — increment match count
                BeaconMatchCount++;
                DB_printf("Beacon '%c' match count: %d/%d\r\n", 
                    detectedId, BeaconMatchCount, BEACON_DEBOUNCE_THRESHOLD);
                
                // Check if we've reached the debounce threshold
                if (BeaconMatchCount >= BEACON_DEBOUNCE_THRESHOLD)
                {
                  // Lock onto this beacon and notify exactly once
                  LockedBeaconId = detectedId;
                  ES_Event_t BeaconEvent;
                  BeaconEvent.EventType  = ES_BEACON_DETECTED;
                  BeaconEvent.EventParam = LockedBeaconId;
                  PostMainLogicFSM(BeaconEvent);
                  CurrentState = BeaconLocked;
                  DB_printf("SignalDetected -> BeaconLocked ('%c') after %d confirmations\r\n", 
                      LockedBeaconId, BeaconMatchCount);
                }
              }
              else
              {
                // Different beacon detected: restart debounce process
                CandidateBeaconId = detectedId;
                BeaconMatchCount  = 1;
                DB_printf("New candidate beacon '%c', starting debounce count\r\n", detectedId);
              }
            }
            else
            {
              // No beacon matched, reset debounce state
              DB_printf("Resetting debounce state");
              CandidateBeaconId = 0;
              BeaconMatchCount  = 0;
            }
          }
        }
        break;

        case ES_TIMEOUT:
          HandleCommonTimeout(ThisEvent);
          break;

        default:
          break;
      } // end switch on event in SignalDetected
    }
    break;

    /*--------------------------------------------------------------------
      BeaconLocked: a specific beacon is confirmed; suppress re-posting
      unless a *different* beacon frequency is detected.
      Watchdog expiry is the only way back to NoSignal.
    --------------------------------------------------------------------*/
    case BeaconLocked:
    {
      switch (ThisEvent.EventType)
      {
        case ES_NEW_SIGNAL_EDGE:
        {
          uint32_t CurrentCapturedTime = CapturedTime;

          uint32_t frequency = UpdateSmoothingFilter(CurrentCapturedTime);

          if (frequency > 0)
          {
            int8_t beaconIdx = FindMatchingBeacon(frequency);
            if (beaconIdx >= 0)
            {
              // Valid beacon signal — keep the watchdog alive
              ES_Timer_InitTimer(SIGNAL_WATCHDOG_TIMER, SIGNAL_WATCHDOG_INTERVAL);
              char detectedId = BeaconTable[beaconIdx].id;
              if (detectedId != LockedBeaconId)
              {
                // Different beacon — re-lock and notify once
                LockedBeaconId = detectedId;
                ES_Event_t BeaconEvent;
                BeaconEvent.EventType  = ES_BEACON_DETECTED;
                BeaconEvent.EventParam = LockedBeaconId;
                PostMainLogicFSM(BeaconEvent);
                DB_printf("BeaconLocked: re-locked to '%c'\r\n", LockedBeaconId);
              }
              // else: same beacon — watchdog already kicked, nothing else to do
            }
            // else: no beacon matched — watchdog will expire → NoSignal
          }
        }
        break;

        case ES_TIMEOUT:
          HandleCommonTimeout(ThisEvent);
          break;

        default:
          break;
      } // end switch on event in BeaconLocked
    }
    break;

    default:
      break;
  } // end switch on CurrentState

  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryBeaconDetectFSM

 Parameters
     None

 Returns
     BeaconState_t - the current state of the FSM

 Description
     Returns the current state so other modules can query beacon status
     without coupling to internal events.

 Author
     Tianyu, 02/19/26
****************************************************************************/
BeaconState_t QueryBeaconDetectFSM(void)
{
  return CurrentState;
}

/****************************************************************************
 Function
     QueryLockedBeaconId

 Parameters
     None

 Returns
     char - the locked beacon ID ('g', 'b', 'r', 'l'), or 0 if no beacon
            is currently locked

 Description
     Returns the currently locked beacon identifier character. Returns 0
     if the FSM is not in BeaconLocked state or no beacon has been detected.

 Author
     Tianyu, 02/24/26
****************************************************************************/
char QueryLockedBeaconId(void)
{
  return LockedBeaconId;
}

/***************************************************************************
 Interrupt Service Routines
 ***************************************************************************/

/****************************************************************************
 Function
     InputCaptureISR

 Parameters
     None

 Returns
     None

 Description
     IC1 interrupt response routine (priority 7, higher than Timer3 ISR).
     Reads IC1BUF, handles the Timer3 rollover boundary race condition,
     assembles a 32-bit virtual timestamp, and posts ES_NEW_SIGNAL_EDGE.

     Race condition handling: if Timer3 has rolled over (T3IF set) AND the
     captured 16-bit value is in the lower half of the range (i.e., the
     capture occurred just after the rollover), the rollover is counted here
     and the Timer3 ISR will see T3IF already cleared and skip it.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL7SOFT) InputCaptureISR(void)
{
  // Read captured timer value from the IC FIFO buffer
  uint16_t capturedTimer16 = IC1BUF;

  // Clear the IC interrupt flag
  IFS0CLR = _IFS0_IC1IF_MASK;

  // If Timer3 rolled over and the capture is in the post-rollover region,
  // handle the rollover here before the Timer3 ISR gets a chance to
  if (IFS0bits.T3IF && (capturedTimer16 < 0x8000))
  {
    RolloverCounter++;
    IFS0CLR = _IFS0_T3IF_MASK;
  }

  // Assemble the 32-bit virtual timestamp
  CapturedTime = ((uint32_t)RolloverCounter << 16) | capturedTimer16;

  // Notify the FSM that a new edge has been captured
  ES_Event_t NewEvent;
  NewEvent.EventType = ES_NEW_SIGNAL_EDGE;
  PostBeaconDetectFSM(NewEvent);
}

/****************************************************************************
 Function
     Timer3ISR

 Parameters
     None

 Returns
     None

 Description
     Timer3 overflow interrupt response routine (priority 6, lower than
     IC ISR). Increments RolloverCounter to extend the 16-bit Timer3 into
     a 32-bit virtual timestamp used by the IC ISR.

     Interrupts are disabled briefly to prevent a race where both ISRs
     would increment RolloverCounter for the same rollover event.

 Author
     Tianyu, 02/03/26
****************************************************************************/
//void __ISR(_TIMER_3_VECTOR, IPL6SOFT) Timer3ISR(void)
//{
//
//    // Disable interrupts globally to prevent race condition with IC ISR
//    __builtin_disable_interrupts();
//    
//    // If T3IF is pending (timer has rolled over)
//    if (IFS0bits.T3IF)
//    {
//        // Increment the roll-over counter to track timer wraparounds
//        RolloverCounter++;
//
//        // Clear the roll-over interrupt flag
//        IFS0CLR = _IFS0_T3IF_MASK;
//    }
//
//    // Re-enable interrupts globally
//    __builtin_enable_interrupts();
//}
// Already handled in DCMotorService

/***************************************************************************
 Private Functions
 ***************************************************************************/

/****************************************************************************
 Function
     ConfigureICTimer

 Parameters
     None

 Returns
     None

 Description
     Configures Timer3 as the time base for IC1. Runs at PBCLK/256 =
     78,125 Hz, giving a 16-bit range of ~0.84 s before rollover.
     The Timer3 ISR and RolloverCounter extend this to a 32-bit range.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void ConfigureICTimer(void)
{
  T3CONbits.ON    = 0;                          // disable during config
  T3CONbits.TCS   = 0;                          // internal PBCLK source
  T3CONbits.TCKPS = PrescaleLookup[PRESCALE_CHOSEN]; // 1:256 prescale
  TMR3            = 0;                          // clear timer register
  PR3             = TIMER_MAX_PERIOD;           // maximum 16-bit period
  IFS0bits.T3IF   = 0;                          // clear interrupt flag
  IPC3bits.T3IP   = 6;                          // priority 6
  IPC3bits.T3IS   = 0;                          // subpriority 0
  IEC0bits.T3IE   = 1;                          // enable Timer3 interrupt
  T3CONbits.ON    = 1;                          // start timer
}

/****************************************************************************
 Function
     ConfigureInputCapture

 Parameters
     None

 Returns
     None

 Description
     Configures IC1 to capture on every 16th rising edge (ICM = 0b101),
     using Timer3 as its time base (ICTMR = 0). IC1 interrupt priority
     is set higher than Timer3 so the boundary race can be resolved
     deterministically in the IC ISR.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void ConfigureInputCapture(void)
{
  T3CONbits.ON    = 0;    // disable timer during IC config
  IC1CONbits.ON   = 0;    // disable IC module during config
  IC1CONbits.ICTMR = 0;   // use Timer3 as time base
  IC1CONbits.ICM  = 0b101; // capture on every 16th rising edge

  // Clear any stale IC interrupt flag and drain the FIFO
  IFS0CLR = _IFS0_IC1IF_MASK;
  volatile uint32_t dummy;
  while (IC1CONbits.ICBNE)
  {
    dummy = IC1BUF;
  }

  IPC1bits.IC1IP  = 7;    // priority 7 (higher than Timer3)
  IPC1bits.IC1IS  = 0;    // subpriority 0
  IEC0bits.IC1IE  = 1;    // enable IC1 interrupt
  IC1CONbits.ON   = 1;    // enable IC module
  T3CONbits.ON    = 1;    // re-enable timer
}

/****************************************************************************
 Function
     CalculateFrequency

 Parameters
     uint32_t timeLapse - timer ticks between IC captures

 Returns
     uint32_t - frequency in Hz, or 0 if timeLapse is 0

 Description
     Converts a tick count to Hz. Because IC1 fires every IC_PRESCALE-th
     rising edge, each timeLapse spans IC_PRESCALE signal periods:
         frequency = (timerClock * IC_PRESCALE) / timeLapse
     where timerClock = PBCLK_FREQ / TIMER_PRESCALE = 78,125 Hz.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static uint32_t CalculateFrequency(uint32_t timeLapse)
{
  if (timeLapse == 0)
  {
    return 0;
  }

  uint32_t timerClock = PBCLK_FREQ / TIMER_PRESCALE; // 78,125 Hz
  return (timerClock * IC_PRESCALE) / timeLapse;
}

/****************************************************************************
 Function
     ResetSignalHistory

 Parameters
     None

 Returns
     None

 Description
     Clears all timing history variables to their initial invalid state.
     Called on entry to NoSignal (from SignalDetected via watchdog expiry)
     and on first entry to SignalDetected (from NoSignal) so that the
     smoothed average is not contaminated by stale data across transitions.

 Author
     Tianyu, 02/19/26
****************************************************************************/
static void ResetSignalHistory(void)
{
  LastCapturedTime  = INVALID_TIME;
  SmoothedTimeLapse = 0;
  FirstSample       = true;
  
  // Reset debouncing state
  CandidateBeaconId = 0;
  BeaconMatchCount  = 0;
}

/****************************************************************************
 Function
     UpdateSmoothingFilter

 Parameters
     uint32_t currentCapturedTime - the timestamp just latched from CapturedTime

 Returns
     uint32_t - the smoothed frequency in Hz, or 0 if LastCapturedTime was
                still INVALID (i.e. this is the very first edge after a reset)

 Description
     Computes the time lapse from the previous captured edge, updates the
     exponential moving average (5:1 history weighting), advances
     LastCapturedTime, and returns the resulting frequency.
     Extracted to eliminate identical code in SignalDetected and BeaconLocked.

 Author
     Tianyu, 02/19/26
****************************************************************************/
static uint32_t UpdateSmoothingFilter(uint32_t currentCapturedTime)
{
  uint32_t frequency = 0;

  if (LastCapturedTime != INVALID_TIME)
  {
    uint32_t timeLapse;
    if (currentCapturedTime >= LastCapturedTime)
    {
      timeLapse = currentCapturedTime - LastCapturedTime;
    }
    else
    {
      // Handle 32-bit virtual-timestamp wraparound
      timeLapse = (0xFFFFFFFF - LastCapturedTime) + currentCapturedTime + 1;
    }

    if (FirstSample)
    {
      SmoothedTimeLapse = timeLapse;
      FirstSample       = false;
    }
    else
    {
      SmoothedTimeLapse = (timeLapse + 5 * SmoothedTimeLapse) / 6;
    }

    frequency = CalculateFrequency(SmoothedTimeLapse);
  }

  // Advance the timing window regardless
  LastCapturedTime = currentCapturedTime;
  return frequency;
}

/****************************************************************************
 Function
     FindMatchingBeacon

 Parameters
     uint32_t frequency - frequency in Hz to look up in BeaconTable

 Returns
     int8_t - index into BeaconTable of the first match within
               BEACON_FREQ_TOLERANCE, or -1 if no beacon matches

 Description
     Searches BeaconTable for a frequency within tolerance. Returns -1
     rather than a sentinel char so that callers can cleanly distinguish
     "no match" from any valid beacon id character.

 Author
     Tianyu, 02/19/26
****************************************************************************/
static int8_t FindMatchingBeacon(uint32_t frequency)
{
  for (uint8_t i = 0; i < NUM_BEACONS; i++)
  {
    if ((frequency >= BeaconTable[i].freq - BEACON_FREQ_TOLERANCE) &&
        (frequency <= BeaconTable[i].freq + BEACON_FREQ_TOLERANCE))
    {
      return (int8_t)i;
    }
  }
  return -1;
}

/****************************************************************************
 Function
     HandleCommonTimeout

 Parameters
     ES_Event_t ThisEvent - the ES_TIMEOUT event to handle

 Returns
     None

 Description
     Handles the two timeout events shared by SignalDetected and
     BeaconLocked:
       SIGNAL_WATCHDOG_TIMER expiry  → reset history, go to NoSignal
       PRINT_FREQUENCY_TIMER expiry  → debug-print frequency, restart timer

 Author
     Tianyu, 02/19/26
****************************************************************************/
static void HandleCommonTimeout(ES_Event_t ThisEvent)
{
  if (ThisEvent.EventParam == SIGNAL_WATCHDOG_TIMER)
  {
    ResetSignalHistory();
    LockedBeaconId    = 0;
    CandidateBeaconId = 0;
    BeaconMatchCount  = 0;
    CurrentState      = NoSignal;
    DB_printf("-> NoSignal (watchdog expired)\r\n");
  }
//  else if (ThisEvent.EventParam == PRINT_FREQUENCY_TIMER)
//  {
//    uint32_t frequency = CalculateFrequency(SmoothedTimeLapse);
//    DB_printf("Frequency: %d Hz\r\n", frequency);
//    DB_printf("Time lapse: %d ticks\r\n", SmoothedTimeLapse);
//    ES_Timer_InitTimer(PRINT_FREQUENCY_TIMER, PRINT_FREQUENCY_INTERVAL);
//  }
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/