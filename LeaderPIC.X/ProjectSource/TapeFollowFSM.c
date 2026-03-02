/****************************************************************************
 Module
   TapeFollowFSM.c

 Revision
   1.0.0

 Description
   This module implements a tape-following state machine that manages
   tape sensor reading, line following PID control, and intersection detection.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 03/01/26 00:00 Team    Initial implementation
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "TapeFollowFSM.h"
#include "MainLogicFSM.h"
#include "DCMotorService.h"
#include "CommonDefinitions.h"
#include "Ports.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define TAPE_FOLLOW_INTERVAL_MS   20u     // sensor poll rate: 50 Hz
#define BASE_FOLLOW_SPEED_MM_S    200u    // straight-line speed target in mm/s
#define LINE_KP                   0.08f   // proportional gain, tune upward from here
#define LINE_KD                   0.15f   // derivative gain, tune after KP settled
#define LINE_LOST_THRESHOLD       5u      // consecutive no-tape cycles before ES_LINE_LOST
#define TAPE_ANALOG_PINS          (BIT12HI | BIT11HI | BIT5HI)  // AN12,11,5
#define THRESH_DIV                2       // Threshold divisor for tape detection

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine */
static void ReadTapeSensors(void);
static void CheckFollowConditions(void);

/*---------------------------- Module Variables ---------------------------*/
// State variable
static TapeFollowState_t CurrentState;

// Priority variable
static uint8_t MyPriority;

// Tape Sensor Variables (from DCMotorService)
static uint32_t ADValues[3] = {0, 0, 0};
static uint32_t leftVal = 0;      // AN12
static uint32_t rightVal = 0;     // AN11
static uint32_t centerVal = 0;    // AN5
static bool leftTState = false;
static bool rightTState = false;

// Min/Max tracking for analog sensors
static uint32_t MinLeftC = 1023;
static uint32_t MaxLeftC = 0;
static uint32_t MinRightC = 1023;
static uint32_t MaxRightC = 0;
static uint32_t MinCenterC = 1023;
static uint32_t MaxCenterC = 0;

// Line following control variables
static int32_t lastError = 0;

// Intersection detection flags
static bool lastLeftTState = false;
static bool lastRightTState = false;
static bool intersectionPublished = false;

// Line lost counter
static uint8_t lineLostCount = 0;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTapeFollowFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the TapeFollowFSM state machine and configures tape sensors

 Author
     Team, 03/01/26
****************************************************************************/
bool InitTapeFollowFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  /********************************************
   Tape Sensor Hardware Initialization
   *******************************************/
  
  // Initialize digital tape sensor pins
  InitLeftTapeInputPin();
  InitRightTapeInputPin();
  
  // Configure ADC to scan AN12 (RB12, pin23), AN11 (RB13, pin24) and AN5 (RB3, pin7)
  // Note: RB3 needs to be configured as analog for AN5
  TRISBbits.TRISB12 = 1;   // Set as input
  ANSELBbits.ANSB12 = 1;   // Enable analog function
  
  TRISBbits.TRISB13 = 1;   // Set as input
  ANSELBbits.ANSB13 = 1;   // Enable analog function
  
  TRISBbits.TRISB3 = 1;    // Set as input  
  ANSELBbits.ANSB3 = 1;    // Enable analog function
  
  ADC_ConfigAutoScan(TAPE_ANALOG_PINS);
  
  // Read initial ADC values
  ADC_MultiRead(ADValues);
  
  centerVal = ADValues[0];   // AN5 (RB3) - listed first because lower bit number
  rightVal = ADValues[1];    // AN11 (RB13)
  leftVal = ADValues[2];     // AN12 (RB12)
  
  // Read initial digital sensor states
  leftTState = ReadLeftTapeInputPin();
  rightTState = ReadRightTapeInputPin();
  
  // Initialize min/max values
  MinLeftC = leftVal;
  MaxLeftC = leftVal;
  MinRightC = rightVal;
  MaxRightC = rightVal;
  MinCenterC = centerVal;
  MaxCenterC = centerVal;
  
  DB_printf("TapeFollowFSM: Tape Sensors Initialized\r\n");
  
  // put us into the Initial PseudoState
  CurrentState = TapeIdle;
  
  // post the initial transition event
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
     PostTapeFollowFSM

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Team, 03/01/26
****************************************************************************/
bool PostTapeFollowFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTapeFollowFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Implements the TapeFollowFSM state machine

 Author
   Team, 03/01/26
****************************************************************************/
ES_Event_t RunTapeFollowFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case TapeIdle:
    {
      switch (ThisEvent.EventType)
      {
        case ES_INIT:
        {
          // Already in Idle state, nothing to do
        }
        break;
        
        case ES_START_LINE_FOLLOW:
        {
          DB_printf("TapeFollowFSM: Starting line following\r\n");
          
          // Reset control variables
          lastError = 0;
          lineLostCount = 0;
          intersectionPublished = false;
          
          // Start the tape follow timer
          ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
          
          // Transition to TapeFollowing state
          CurrentState = TapeFollowing;
        }
        break;
        
        default:
          ;
      }
    }
    break;

    case TapeFollowing:
    {
      switch (ThisEvent.EventType)
      {
        case ES_STOP_LINE_FOLLOW:
        {
          DB_printf("TapeFollowFSM: Stopping line following\r\n");
          
          // Stop motors
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          
          // Transition to Idle
          CurrentState = TapeIdle;
        }
        break;
        
        case ES_TIMEOUT:
        {
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            // 1. Read all sensors
            ReadTapeSensors();
            
            // 2. Compute signed error from analog sensors
            // Positive error = robot drifted left, steer right (slow left, speed up right)
            int32_t error = (int32_t)rightVal - (int32_t)leftVal;
            
            // 3. PD correction
            float correction = LINE_KP * (float)error + LINE_KD * (float)(error - lastError);
            lastError = error;
            
            // 4. Differential speed targets
            float leftSpeed_f  = (float)BASE_FOLLOW_SPEED_MM_S + correction;
            float rightSpeed_f = (float)BASE_FOLLOW_SPEED_MM_S - correction;
            
            // 5. Clamp to physical range [0, SPEED_FULL_MM_S]
            if (leftSpeed_f  < 0.0f) leftSpeed_f  = 0.0f;
            if (rightSpeed_f < 0.0f) rightSpeed_f = 0.0f;
            if (leftSpeed_f  > (float)SPEED_FULL_MM_S) leftSpeed_f  = (float)SPEED_FULL_MM_S;
            if (rightSpeed_f > (float)SPEED_FULL_MM_S) rightSpeed_f = (float)SPEED_FULL_MM_S;
            
            // 6. Command motors through closed-loop PI
            DCMotor_SetSpeed_mm_s((uint16_t)leftSpeed_f, (uint16_t)rightSpeed_f,
                                  FORWARD, FORWARD);
            
            // 7. Debug print — NO %f allowed. Use integer hundredths method:
            uint32_t lSpd_h = (uint32_t)(leftSpeed_f  * 100.0f);
            uint32_t rSpd_h = (uint32_t)(rightSpeed_f * 100.0f);
            uint32_t corr_h = (uint32_t)((correction > 0.0f ? correction : -correction) * 100.0f);
            DB_printf("err=%d corr=%s%u.%u L=%u.%u R=%u.%u mm/s\r\n",
                      (int)error,
                      (correction < 0.0f ? "-" : "+"),
                      (unsigned)(corr_h / 100), (unsigned)(corr_h % 100),
                      (unsigned)(lSpd_h / 100), (unsigned)(lSpd_h % 100),
                      (unsigned)(rSpd_h / 100), (unsigned)(rSpd_h % 100));
            
            // 8. Restart timer
            ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
            
            // 9. Check for line lost and intersection conditions
            CheckFollowConditions();
          }
        }
        break;
        
        default:
          ;
      }
    }
    break;

    default:
      ;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTapeFollowFSM

 Parameters
     None

 Returns
     TapeFollowState_t The current state of the TapeFollow state machine

 Description
     returns the current state of the TapeFollow state machine

 Author
     Team, 03/01/26
****************************************************************************/
TapeFollowState_t QueryTapeFollowFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
     ReadTapeSensors

 Parameters
     None

 Returns
     None

 Description
     Reads all tape sensors and updates min/max tracking

 Author
     Tianyu, 02/25/26 (moved from DCMotorService)
****************************************************************************/
static void ReadTapeSensors(void)
{
  // Read analog sensors
  ADC_MultiRead(ADValues);
  centerVal = ADValues[0];   // AN5 (RB3)
  rightVal = ADValues[1];    // AN11 (RB13)
  leftVal = ADValues[2];     // AN12 (RB12)
  
  // Update min/max for center analog sensor
  if (centerVal < MinCenterC)
  {
    MinCenterC = centerVal;
  }
  if (centerVal > MaxCenterC)
  {
    MaxCenterC = centerVal;
  }
  
  // Update min/max for left analog sensor
  if (leftVal < MinLeftC)
  {
    MinLeftC = leftVal;
  }
  if (leftVal > MaxLeftC)
  {
    MaxLeftC = leftVal;
  }
  
  // Update min/max for right analog sensor
  if (rightVal < MinRightC)
  {
    MinRightC = rightVal;
  }
  if (rightVal > MaxRightC)
  {
    MaxRightC = rightVal;
  }
  
  // Read digital sensors
  leftTState = ReadLeftTapeInputPin();
  rightTState = ReadRightTapeInputPin();
}

/****************************************************************************
 Function
     CheckFollowConditions

 Parameters
     None

 Returns
     None

 Description
     Checks for intersection detection and line lost conditions.
     Posts events to MainLogicFSM as needed.

 Author
     Team, 03/01/26 (moved from DCMotorService)
****************************************************************************/
static void CheckFollowConditions(void)
{
  uint32_t LeftThreshC  = (MaxLeftC  - MinLeftC)  / THRESH_DIV + MinLeftC;
  uint32_t RightThreshC = (MaxRightC - MinRightC) / THRESH_DIV + MinRightC;

  // T-intersection: both outer analog sensors see tape
  if (leftVal > LeftThreshC && rightVal > RightThreshC)
  {
    if (!intersectionPublished)
    {
      DB_printf("TapeFollow: T-Intersection detected\r\n");
      ES_Event_t ev;
      ev.EventType  = ES_INTERSECTION_DETECTED;
      ev.EventParam = 0;
      PostMainLogicFSM(ev);
      intersectionPublished = true;
    }
  }
  else
  {
    intersectionPublished = false;
  }

  // Line lost: center sensor below threshold for N consecutive cycles
  if (centerVal < (MinCenterC + (MaxCenterC - MinCenterC) / THRESH_DIV))
  {
    lineLostCount++;
    if (lineLostCount >= LINE_LOST_THRESHOLD)
    {
      DB_printf("TapeFollow: Line lost\r\n");
      ES_Event_t ev;
      ev.EventType  = ES_LINE_LOST;
      ev.EventParam = 0;
      PostMainLogicFSM(ev);
      lineLostCount = 0;
    }
  }
  else
  {
    lineLostCount = 0;
  }
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
