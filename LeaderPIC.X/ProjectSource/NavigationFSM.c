/****************************************************************************
 Module
   NavigationFSM.c

 Revision
   1.0.0

 Description
   This module implements a navigation state machine that manages
   tape sensor reading, line following PID control, and intersection detection.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 03/02/26       Team    Renamed from TapeFollowFSM to NavigationFSM
 03/01/26 00:00 Team    Initial implementation
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "NavigationFSM.h"
#include "MainLogicFSM.h"
#include "DCMotorService.h"
#include "CommonDefinitions.h"
#include "Ports.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define TAPE_FOLLOW_INTERVAL_MS   20u     // sensor poll rate: 50 Hz
#define BASE_FOLLOW_SPEED_MM_S    100u    // straight-line speed target in mm/s
#define BASE_FOLLOW_SPEED_REV_MM_S    70u    // straight-line speed target in mm/s
#define LINE_KP                   5.0f   // proportional gain, tune upward from here
#define LINE_KD                   1.5f   // derivative gain, tune after KP settled
#define LINE_KP_REV               0.5f   // proportional gain, tune upward from here
#define LINE_KD_REV               0.5f   // derivative gain, tune after KP settled
#define LINE_LOST_THRESHOLD       50u      // consecutive no-tape cycles before ES_LINE_LOST
#define TAPE_ANALOG_PINS          (BIT12HI | BIT11HI | BIT5HI)  // AN12,11,5
#define THRESH_DIV                2       // Threshold divisor for tape detection

// Fixed threshold: analog value > 600 = black tape detected
#define TAPE_THRESHOLD            600u

#define SEARCH_ROTATE_SPEED_MM_S  120u   // slow rotation during search

// Speed used for all open-loop rotation maneuvers
// Lower = more accurate (less overshoot from motor inertia)
// Must match what DCMotor_SetSpeed_mm_s can reliably track
#define ROTATE_SPEED_MM_S       100u
#define RADIUS_ROTATE_SPEED_MM_S 70u

// Duration of startup calibration rotation in milliseconds.
// Robot sweeps sensors over floor (and hopefully tape) to build min/max range.
// At SEARCH_ROTATE_SPEED_MM_S, this gives roughly 200-270 degrees of rotation.
#define CALIB_ROTATION_MS         5000u

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine */
static void ReadTapeSensors(void);
static void CheckFollowConditions(void);
static bool IsOnTape(uint32_t val, uint32_t minC, uint32_t maxC);
static void StartRotation(uint8_t leftDir, uint8_t rightDir, uint32_t targetArc_mm);

/*---------------------------- Module Variables ---------------------------*/
// State variable
static NavigationState_t CurrentState;

// Priority variable
static uint8_t MyPriority;

// Tape Sensor Variables (from DCMotorService)
static uint32_t ADValues[3] = {0, 0, 0};
static uint32_t leftVal = 0;      // AN12
static uint32_t rightVal = 0;     // AN11
static uint32_t centerVal = 0;    // AN5
static bool leftTState = false;
static bool rightTState = false;

// Calibration values
static uint32_t MinLeftC = 1023u, MaxLeftC = 0u;
static uint32_t MinRightC = 1023u, MaxRightC = 0u;
static uint32_t MinCenterC = 1023u, MaxCenterC = 0u;

// Boolean tape detection states
static bool centerOnTape = false;
static bool leftOnTape   = false;
static bool rightOnTape  = false;

// Line following control variables
static int32_t lastError = 0;

// Intersection detection flags
static bool lastLeftTState = false;
static bool lastRightTState = false;
static bool tIntersectionPublished    = false;
static bool leftTurnPublished         = false;
static bool rightTurnPublished        = false;

// Line lost counter
static uint8_t lineLostCount = 0;

// Odometer-based rotation tracking
static uint32_t RotateStartDistLeft_mm  = 0u;
static uint32_t RotateStartDistRight_mm = 0u;
static uint32_t RotateTargetArc_mm      = 0u;

// Odometer-based radius rotation tracking
static uint32_t RotateRadiusStartDistLeft_mm  = 0u;
static uint32_t RotateRadiusStartDistRight_mm = 0u;

static uint32_t RotateRadiusTargetLeft_mm  = 0u;
static uint32_t RotateRadiusTargetRight_mm = 0u;

// Odometer-based forward movement tracking
static uint32_t MoveTargetDist_mm      = 0u;
static uint32_t MoveStartDistLeft_mm   = 0u;
static uint32_t MoveStartDistRight_mm  = 0u;

// Odometer-based backward movement tracking
static uint32_t MoveBackTargetDist_mm     = 0u;
static uint32_t MoveBackStartDistLeft_mm  = 0u;
static uint32_t MoveBackStartDistRight_mm = 0u;

// Odometer-based forward follow-with-distance tracking
static uint32_t FollowForwardTargetDist_mm     = 0u;
static uint32_t FollowForwardStartDistLeft_mm  = 0u;
static uint32_t FollowForwardStartDistRight_mm = 0u;

// Odometer-based backward follow-with-distance tracking
static uint32_t FollowReverseTargetDist_mm     = 0u;
static uint32_t FollowReverseStartDistLeft_mm  = 0u;
static uint32_t FollowReverseStartDistRight_mm = 0u;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitNavigationFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the NavigationFSM state machine and configures tape sensors

 Author
     Team, 03/01/26
****************************************************************************/
bool InitNavigationFSM(uint8_t Priority)
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

  MinLeftC = 1023u;   MaxLeftC = 0u;
  MinRightC = 1023u;  MaxRightC = 0u;
  MinCenterC = 1023u; MaxCenterC = 0u;
  
  DB_printf("NavigationFSM: Tape Sensors Initialized\r\n");
  
  // put us into the Initial PseudoState
  CurrentState = NavIdle;
  
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
     PostNavigationFSM

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Team, 03/01/26
****************************************************************************/
bool PostNavigationFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunNavigationFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Implements the NavigationFSM state machine

 Author
   Team, 03/01/26
****************************************************************************/
ES_Event_t RunNavigationFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case NavIdle:
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
          DB_printf("NavigationFSM: Starting line following\r\n");
          
          // Reset control variables
          lastError = 0;
          lineLostCount = 0;
          tIntersectionPublished = false;
          leftTurnPublished = false;
          rightTurnPublished = false;
          
          // Start the tape follow timer
          ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
          
          // Transition to NavFollowingForward state
          CurrentState = NavFollowingForward;
        }
        break;
        
        default:
          ;
      }
    }
    break;

    case NavFollowingForward:
    {
      switch (ThisEvent.EventType)
      {
        case ES_STOP_LINE_FOLLOW:
        {
          DB_printf("NavigationFSM: Stopping line following\r\n");
          
          // Stop motors
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          
          // Transition to Idle
          CurrentState = NavIdle;
        }
        break;
        
        case ES_TIMEOUT:
        {
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            // 1. Read all sensors
            ReadTapeSensors();

            // Debug print: raw analog values, boolean states, digital sensors, and error
            DB_printf("L=%u(%d) R=%u(%d) C=%u(%d) LT=%d RT=%d err=%d\r\n",
                      (unsigned)leftVal,   leftOnTape   ? 1 : 0,
                      (unsigned)rightVal,  rightOnTape  ? 1 : 0,
                      (unsigned)centerVal, centerOnTape ? 1 : 0,
                      leftTState  ? 1 : 0,
                      rightTState ? 1 : 0,
                      (int)lastError);
            
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

    case NavCalibrating:
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT:
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            // Poll sensors to build min/max — do not act on tape detection
            ReadTapeSensors();

            // Debug: show current min/max ranges building up
            DB_printf("CALIB C=%u[%u,%u] L=%u[%u,%u] R=%u[%u,%u]\r\n",
                      (unsigned)centerVal, (unsigned)MinCenterC, (unsigned)MaxCenterC,
                      (unsigned)leftVal,   (unsigned)MinLeftC,   (unsigned)MaxLeftC,
                      (unsigned)rightVal,  (unsigned)MinRightC,  (unsigned)MaxRightC);

            // Restart sensor poll timer
            ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
          }
          else if (ThisEvent.EventParam == CALIB_TIMER)
          {
            // Calibration rotation complete — stop motors and notify MainLogicFSM
            DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);

            DB_printf("Nav: Calibration done. C[%u,%u] L[%u,%u] R[%u,%u]\r\n",
                      (unsigned)MinCenterC, (unsigned)MaxCenterC,
                      (unsigned)MinLeftC,   (unsigned)MaxLeftC,
                      (unsigned)MinRightC,  (unsigned)MaxRightC);

            ES_Event_t ev;
            ev.EventType  = ES_CALIB_DONE;
            ev.EventParam = 0;
            PostMainLogicFSM(ev);

            CurrentState = NavIdle;
          }
          break;

        case ES_STOP_LINE_FOLLOW:
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          ES_Timer_StopTimer(CALIB_TIMER);
          CurrentState = NavIdle;
          break;

        default:
          break;
      }
    }
    break;

    case NavSearching:
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT:
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            ReadTapeSensors();  // updates min/max and centerOnTape during rotation
            
            if (centerOnTape)
            {
              // Tape found — stop and notify MainLogicFSM
              DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
              DB_printf("Nav: Tape found during search\r\n");
              ES_Event_t ev;
              ev.EventType  = ES_TAPE_FOUND;
              ev.EventParam = 0;
              PostMainLogicFSM(ev);
              CurrentState = NavIdle;
            }
            else
            {
              ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
            }
          }
          break;
          
        case ES_STOP_LINE_FOLLOW:
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          CurrentState = NavIdle;
          break;
          
        default:
          break;
      }
    }
    break;

    case NavFollowingReverse:
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT:
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            ReadTapeSensors();
            
            int32_t error = (int32_t)rightVal - (int32_t)leftVal;
            float correction = LINE_KP_REV * (float)error + LINE_KD_REV * (float)(error - lastError);
            lastError = error;
            
            // For reverse: left wheel = base - correction, right = base + correction
            // Both commanded in REVERSE direction
            float leftSpeed_f  = (float)BASE_FOLLOW_SPEED_REV_MM_S - correction;
            float rightSpeed_f = (float)BASE_FOLLOW_SPEED_REV_MM_S + correction;
            
            if (leftSpeed_f  < 0.0f) leftSpeed_f  = 0.0f;
            if (rightSpeed_f < 0.0f) rightSpeed_f = 0.0f;
            if (leftSpeed_f  > (float)SPEED_FULL_MM_S) leftSpeed_f  = (float)SPEED_FULL_MM_S;
            if (rightSpeed_f > (float)SPEED_FULL_MM_S) rightSpeed_f = (float)SPEED_FULL_MM_S;
            
            DCMotor_SetSpeed_mm_s((uint16_t)leftSpeed_f, (uint16_t)rightSpeed_f,
                                  REVERSE, REVERSE);
            
            uint32_t lSpd_h = (uint32_t)(leftSpeed_f  * 100.0f);
            uint32_t rSpd_h = (uint32_t)(rightSpeed_f * 100.0f);
            DB_printf("REV err=%d L=%u.%u R=%u.%u mm/s\r\n",
                      (int)error,
                      (unsigned)(lSpd_h / 100), (unsigned)(lSpd_h % 100),
                      (unsigned)(rSpd_h / 100), (unsigned)(rSpd_h % 100));
            
            CheckFollowConditions();
            ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
          }
          break;
          
        case ES_STOP_LINE_FOLLOW:
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          CurrentState = NavIdle;
          break;
          
        default:
          break;
      }
    }
    break;

    case NavRotating:
    {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT:
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            uint32_t curL = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
            uint32_t curR = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
            uint32_t dL   = curL - RotateStartDistLeft_mm;
            uint32_t dR   = curR - RotateStartDistRight_mm;
            uint32_t avg  = (dL + dR) / 2u;

            if (avg >= RotateTargetArc_mm)
            {
              DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
              ES_Timer_StopTimer(ROTATE_SAFETY_TIMER);
              DB_printf("Nav: Rotation done, avg=%u mm\r\n", (unsigned)avg);
              ES_Event_t ev = { ES_BEHAVIOR_COMPLETE, 0 };
              PostMainLogicFSM(ev);
              CurrentState = NavIdle;
            }
            else
            {
              ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);
            }
          }
          else if (ThisEvent.EventParam == ROTATE_SAFETY_TIMER)
          {
            DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
            DB_printf("Nav: Rotation safety timeout\r\n");
            ES_Event_t ev = { ES_BEHAVIOR_COMPLETE, 0 };
            PostMainLogicFSM(ev);
            CurrentState = NavIdle;
          }
          break;

        case ES_STOP_LINE_FOLLOW:
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          ES_Timer_StopTimer(ROTATE_SAFETY_TIMER);
          CurrentState = NavIdle;
          break;

        default:
          break;
      }
    }
    break;

    case NavRotatingRadius:
    {
      if (ThisEvent.EventType == ES_TIMEOUT &&
          ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
      {
        uint32_t curL = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
        uint32_t curR = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));

        uint32_t dL = curL - RotateRadiusStartDistLeft_mm;
        uint32_t dR = curR - RotateRadiusStartDistRight_mm;
        uint32_t avg2 = (dL + dR);

        // Use average distance to target for completion check
        if (avg2 >= (RotateRadiusTargetLeft_mm + RotateRadiusTargetRight_mm))
        {
          DCMotor_SetSpeed_mm_s(0,0,FORWARD,FORWARD);

          ES_Event_t ev = { ES_BEHAVIOR_COMPLETE, 0 };
          PostMainLogicFSM(ev);

          CurrentState = NavIdle;
        }
        else
        {
          ES_Timer_InitTimer(TAPE_FOLLOW_TIMER,
                            ROTATE_POLL_INTERVAL_MS);
        }
      }
    }
    break;

    case NavMovingForward:
    {
      if (ThisEvent.EventType == ES_TIMEOUT &&
          ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
      {
        uint32_t curL = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
        uint32_t curR = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
        uint32_t avg  = ((curL - MoveStartDistLeft_mm) +
                         (curR - MoveStartDistRight_mm)) / 2u;

        if (avg >= MoveTargetDist_mm)
        {
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          DB_printf("Nav: MoveForward done, avg=%u mm\r\n", (unsigned)avg);
          ES_Event_t ev = { ES_BEHAVIOR_COMPLETE, 0 };
          PostMainLogicFSM(ev);
          CurrentState = NavIdle;
        }
        else
        {
          ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);
        }
      }
      else if (ThisEvent.EventType == ES_STOP_LINE_FOLLOW)
      {
        DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
        CurrentState = NavIdle;
      }
    }
    break;

    case NavMovingBackward:
    {
      if (ThisEvent.EventType == ES_TIMEOUT &&
          ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
      {
        uint32_t curL = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
        uint32_t curR = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
        uint32_t avg  = ((curL - MoveBackStartDistLeft_mm) +
                         (curR - MoveBackStartDistRight_mm)) / 2u;

        if (avg >= MoveBackTargetDist_mm)
        {
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          DB_printf("Nav: MoveBackward done, avg=%u mm\r\n", (unsigned)avg);
          ES_Event_t ev = { ES_BEHAVIOR_COMPLETE, 0 };
          PostMainLogicFSM(ev);
          CurrentState = NavIdle;
        }
        else
        {
          ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);
        }
      }
      else if (ThisEvent.EventType == ES_STOP_LINE_FOLLOW)
      {
        DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
        CurrentState = NavIdle;
      }
    }
    break;

    case NavRotatingContinuous:
    {
      if (ThisEvent.EventType == ES_STOP_LINE_FOLLOW)
      {
        DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
        DB_printf("Nav: Continuous rotate stopped\r\n");
        CurrentState = NavIdle;
      }
    }
    break;

    case NavFollowingForwardDistance:
    {
      switch (ThisEvent.EventType)
      {
        case ES_STOP_LINE_FOLLOW:
        {
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          CurrentState = NavIdle;
        }
        break;

        case ES_TIMEOUT:
        {
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            // 1. Read all sensors
            ReadTapeSensors();

            // 2. Compute signed error from analog sensors
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

            // 6. Command motors
            DCMotor_SetSpeed_mm_s((uint16_t)leftSpeed_f, (uint16_t)rightSpeed_f,
                                  FORWARD, FORWARD);

            // 7. Check odometer for completion
            uint32_t curL = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
            uint32_t curR = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
            uint32_t avg  = ((curL - FollowForwardStartDistLeft_mm) +
                             (curR - FollowForwardStartDistRight_mm)) / 2u;

            if (avg >= FollowForwardTargetDist_mm)
            {
              // Target distance reached — stop and complete
              DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
              DB_printf("Nav: FollowForwardDistance done, avg=%u mm\r\n", (unsigned)avg);
              ES_Event_t ev = { ES_BEHAVIOR_COMPLETE, 0 };
              PostMainLogicFSM(ev);
              CurrentState = NavIdle;
            }
            else
            {
              // Continue following
              ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
              CheckFollowConditions();
            }
          }
        }
        break;

        default:
          ;
      }
    }
    break;

    case NavFollowingReverseDistance:
    {
      switch (ThisEvent.EventType)
      {
        case ES_STOP_LINE_FOLLOW:
        {
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          CurrentState = NavIdle;
        }
        break;

        case ES_TIMEOUT:
        {
          if (ThisEvent.EventParam == TAPE_FOLLOW_TIMER)
          {
            // 1. Read all sensors
            ReadTapeSensors();

            // 2. Compute signed error from analog sensors
            int32_t error = (int32_t)rightVal - (int32_t)leftVal;

            // 3. PD correction
            float correction = LINE_KP_REV * (float)error + LINE_KD_REV * (float)(error - lastError);
            lastError = error;

            // 4. Differential speed targets (both in REVERSE)
            float leftSpeed_f  = (float)BASE_FOLLOW_SPEED_REV_MM_S - correction;
            float rightSpeed_f = (float)BASE_FOLLOW_SPEED_REV_MM_S + correction;

            // 5. Clamp to physical range [0, SPEED_FULL_MM_S]
            if (leftSpeed_f  < 0.0f) leftSpeed_f  = 0.0f;
            if (rightSpeed_f < 0.0f) rightSpeed_f = 0.0f;
            if (leftSpeed_f  > (float)SPEED_FULL_MM_S) leftSpeed_f  = (float)SPEED_FULL_MM_S;
            if (rightSpeed_f > (float)SPEED_FULL_MM_S) rightSpeed_f = (float)SPEED_FULL_MM_S;

            // 6. Command motors in REVERSE
            DCMotor_SetSpeed_mm_s((uint16_t)leftSpeed_f, (uint16_t)rightSpeed_f,
                                  REVERSE, REVERSE);

            // 7. Check odometer for completion
            uint32_t curL = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
            uint32_t curR = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
            uint32_t avg  = ((curL - FollowReverseStartDistLeft_mm) +
                             (curR - FollowReverseStartDistRight_mm)) / 2u;

            if (avg >= FollowReverseTargetDist_mm)
            {
              // Target distance reached — stop and complete
              DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
              DB_printf("Nav: FollowReverseDistance done, avg=%u mm\r\n", (unsigned)avg);
              ES_Event_t ev = { ES_BEHAVIOR_COMPLETE, 0 };
              PostMainLogicFSM(ev);
              CurrentState = NavIdle;
            }
            else
            {
              // Continue following
              ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
              CheckFollowConditions();
            }
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
     QueryNavigationFSM

 Parameters
     None

 Returns
     NavigationState_t The current state of the Navigation state machine

 Description
     returns the current state of the Navigation state machine

 Author
     Team, 03/01/26
****************************************************************************/
NavigationState_t QueryNavigationFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
     IsOnTape

 Parameters
     uint32_t val - current analog sensor reading

 Returns
     bool - true if sensor detects black tape, false if white floor

 Description
     Returns true if the given analog reading indicates black tape using
     a fixed threshold of 600.

 Author
     Team, 03/01/26
****************************************************************************/
static bool IsOnTape(uint32_t val, uint32_t minC, uint32_t maxC)
{
  uint32_t threshold = minC + (maxC - minC) / THRESH_DIV;
  return (val > threshold);
}

/****************************************************************************
 Function
     ReadTapeSensors

 Parameters
     None

 Returns
     None

 Description
     Reads all tape sensors (analog ADC and digital pins) and updates
     boolean tape detection states using fixed threshold.

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

  if (centerVal > 0u && centerVal < MinCenterC) MinCenterC = centerVal;
  if (centerVal > MaxCenterC)                   MaxCenterC = centerVal;

  if (leftVal > 0u && leftVal < MinLeftC)   MinLeftC = leftVal;
  if (leftVal > MaxLeftC)                   MaxLeftC = leftVal;

  if (rightVal > 0u && rightVal < MinRightC) MinRightC = rightVal;
  if (rightVal > MaxRightC)                  MaxRightC = rightVal;
  
  // Read digital sensors
  leftTState = ReadLeftTapeInputPin();
  rightTState = ReadRightTapeInputPin();
  
  // Compute boolean tape states using fixed threshold
  centerOnTape = IsOnTape(centerVal, MinCenterC, MaxCenterC);
  leftOnTape   = IsOnTape(leftVal, MinLeftC, MaxLeftC);
  rightOnTape  = IsOnTape(rightVal, MinRightC, MaxRightC);
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
  // Priority order: T > left > right
  // Mutually exclusive — only the highest-priority match fires

  if (leftTState && rightTState)
  {
    // T-intersection: both digital sensors on tape
    if (!tIntersectionPublished)
    {
      DB_printf("Nav: T-Intersection detected\r\n");
      // Stop motors — navigation is done
      DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
      // Notify MainLogicFSM that this behavior is complete
      ES_Event_t ev;
      ev.EventType  = ES_BEHAVIOR_COMPLETE;
      ev.EventParam = 0;
      PostMainLogicFSM(ev);
      CurrentState = NavIdle;
      tIntersectionPublished = true;
    }
    // Reset lower-priority flags so they re-arm after T clears
    leftTurnPublished  = false;
    rightTurnPublished = false;
  }
  else if (leftTState && centerOnTape)
  {
    // Left turn intersection
    if (!leftTurnPublished)
    {
      DB_printf("Nav: Left turn intersection detected\r\n");
      ES_Event_t ev;
      ev.EventType  = ES_INTERSECTION_DETECTED;
      ev.EventParam = 1;
      PostMainLogicFSM(ev);
      leftTurnPublished = true;
    }
    tIntersectionPublished = false;
    rightTurnPublished     = false;
  }
  else if (rightTState && centerOnTape)
  {
    // Right turn intersection
    if (!rightTurnPublished)
    {
      DB_printf("Nav: Right turn intersection detected\r\n");
      ES_Event_t ev;
      ev.EventType  = ES_INTERSECTION_DETECTED;
      ev.EventParam = 2;
      PostMainLogicFSM(ev);
      rightTurnPublished = true;
    }
    tIntersectionPublished = false;
    leftTurnPublished      = false;
  }
  else
  {
    // No intersection — reset all flags so next detection fires fresh
    tIntersectionPublished = false;
    leftTurnPublished      = false;
    rightTurnPublished     = false;
  }

  // Line lost: center off tape for N consecutive cycles
  if (!centerOnTape)
  {
    lineLostCount++;
    if (lineLostCount >= LINE_LOST_THRESHOLD)
    {
      DB_printf("Nav: Line lost\r\n");
      ES_Event_t ev;
      ev.EventType  = ES_LINE_LOST;
      ev.EventParam = 0;
      PostMainLogicFSM(ev);
      ES_Timer_StopTimer(TAPE_FOLLOW_TIMER);
      CurrentState = NavIdle;
      lineLostCount = 0;
    }
  }
  else
  {
    lineLostCount = 0;
  }
}

/***************************************************************************
 Public helper functions
 ***************************************************************************/

/****************************************************************************
 Function
     Nav_StartRotateSearch

 Parameters
     bool clockwise - true for CW rotation, false for CCW

 Returns
     None

 Description
     Begins rotating CW or CCW at low speed until center sensor detects tape.
     When tape is found, posts ES_TAPE_FOUND to MainLogicFSM and stops rotation.
     Caller should transition their state to wait for ES_TAPE_FOUND.

 Author
     Team, 03/01/26
****************************************************************************/
void Nav_StartRotateSearch(bool clockwise)
{
  lastError     = 0;
  lineLostCount = 0;
  
  // Set rotation direction: CW = left forward, right reverse
  uint8_t leftDir  = clockwise ? FORWARD : REVERSE;
  uint8_t rightDir = clockwise ? REVERSE : FORWARD;
  DCMotor_SetSpeed_mm_s(SEARCH_ROTATE_SPEED_MM_S, SEARCH_ROTATE_SPEED_MM_S,
                        leftDir, rightDir);
  
  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
  CurrentState = NavSearching;
  
  DB_printf("Nav: Rotate search started clockwise=%d\r\n", clockwise ? 1 : 0);
}

/****************************************************************************
 Function
     Nav_StartCalibration

 Parameters
     None

 Returns
     None

 Description
     Begins a timed CW calibration rotation to build sensor min/max ranges.
     Does NOT check for tape — purely for ADC range calibration.
     After CALIB_ROTATION_MS, posts ES_CALIB_DONE to MainLogicFSM and stops.

 Author
     Team, 03/01/26
****************************************************************************/
void Nav_StartCalibration(void)
{
  lastError              = 0;
  lineLostCount          = 0;
  tIntersectionPublished = false;
  leftTurnPublished      = false;
  rightTurnPublished     = false;

  // Rotate CW: left forward, right reverse
  DCMotor_SetSpeed_mm_s(SEARCH_ROTATE_SPEED_MM_S, SEARCH_ROTATE_SPEED_MM_S,
                        FORWARD, REVERSE);

  // Use a one-shot timer for total calibration duration
  ES_Timer_InitTimer(CALIB_TIMER, CALIB_ROTATION_MS);

  // Also start tape follow timer for sensor polling during rotation
  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);

  CurrentState = NavCalibrating;

  DB_printf("Nav: Calibration rotation started (%u ms)\r\n",
            (unsigned)CALIB_ROTATION_MS);
}

/****************************************************************************
 Function
     Nav_StartDriveSearch

 Parameters
     bool forward - true for forward drive, false for backward

 Returns
     None

 Description
     Drives forward (or backward) at a fixed speed until center sensor
     detects tape, then posts ES_TAPE_FOUND to MainLogicFSM and stops.

 Author
     Team, 03/01/26
****************************************************************************/
void Nav_StartDriveSearch(bool forward)
{
  lastError     = 0;
  lineLostCount = 0;
  
  uint8_t dir = forward ? FORWARD : REVERSE;
  DCMotor_SetSpeed_mm_s(SEARCH_ROTATE_SPEED_MM_S, SEARCH_ROTATE_SPEED_MM_S,
                        dir, dir);
  
  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
  CurrentState = NavSearching;
  
  DB_printf("Nav: Drive search started forward=%d\r\n", forward ? 1 : 0);
}

/****************************************************************************
 Function
     Nav_StartFollowReverse

 Parameters
     None

 Returns
     None

 Description
     Starts reverse line following using the same PD control as forward
     following but with both motors in REVERSE direction.

 Author
     Team, 03/01/26
****************************************************************************/
void Nav_StartFollowReverse(void)
{
  lastError     = 0;
  lineLostCount = 0;
  tIntersectionPublished = false;
  leftTurnPublished      = false;
  rightTurnPublished     = false;
  
  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
  CurrentState = NavFollowingReverse;
  
  DB_printf("Nav: Reverse line following started\r\n");
}

/****************************************************************************
 Function
     Nav_StartFollowForward

 Parameters
     None

 Returns
     None

 Description
     Starts forward line following with PD control.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_StartFollowForward(void)
{
  lastError              = 0;
  lineLostCount          = 0;
  tIntersectionPublished = false;
  leftTurnPublished      = false;
  rightTurnPublished     = false;

  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
  CurrentState = NavFollowingForward;

  DB_printf("Nav: Forward tape follow started\r\n");
}

/****************************************************************************
 Function
     StartRotation

 Parameters
     uint8_t leftDir, uint8_t rightDir - FORWARD or REVERSE for each wheel
     uint32_t targetArc_mm - arc distance each wheel must travel

 Returns
     None

 Description
     Starts a point turn at ROTATE_SPEED_MM_S and records odometer start values.

 Author
     Team, 03/01/26
****************************************************************************/
static void StartRotation(uint8_t leftDir, uint8_t rightDir, uint32_t targetArc_mm)
{
  // Record odometer baseline before motion begins
  RotateStartDistLeft_mm  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
  RotateStartDistRight_mm = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
  RotateTargetArc_mm      = targetArc_mm;

  // Start motors
  DCMotor_SetSpeed_mm_s(ROTATE_SPEED_MM_S, ROTATE_SPEED_MM_S, leftDir, rightDir);

  // Start short polling timer to check odometer
  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);

  // Start safety timeout in case encoders fail
  ES_Timer_InitTimer(ROTATE_SAFETY_TIMER, ROTATE_SAFETY_TIMEOUT_MS);

  uint32_t arc_h = targetArc_mm;  // already integer mm, no float needed
  DB_printf("Nav: StartRotation target=%u mm L=%u R=%u\r\n",
            (unsigned)arc_h,
            (unsigned)RotateStartDistLeft_mm,
            (unsigned)RotateStartDistRight_mm);
}

/****************************************************************************
 Function
     Nav_RotateCW

 Parameters
     uint8_t degrees - angle to rotate clockwise

 Returns
     None

 Description
     Rotates clockwise by the specified degrees using odometer feedback.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_RotateCW(uint8_t degrees)
{
  DB_printf("Nav: RotateCW %u deg, arc=%u mm\r\n",
            (unsigned)degrees, (unsigned)ROTATE_ARC_MM(degrees));
  StartRotation(FORWARD, REVERSE, ROTATE_ARC_MM(degrees));
  CurrentState = NavRotating;
}

/****************************************************************************
 Function
     Nav_RotateCWRadius

 Parameters
     uint8_t degrees - angle to rotate clockwise
     uint32_t radius_mm - radius of turn in millimeters

 Returns
     None

 Description
     Rotates clockwise by the specified degrees using odometer feedback, and maintains a turn radius by setting different speeds on each wheel.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_RotateCWRadius(uint8_t degrees, uint32_t radius_mm)
{
  uint32_t halfW = TRACK_WIDTH_MM / 2u;

  // CW turn: right wheel is inner, left wheel is outer
  // When radius_mm < halfW, inner wheel must go backward
  // Compute wheel radii from turn center (unsigned magnitudes only)
  uint32_t R_right;  // inner wheel radius magnitude
  uint32_t R_left;   // outer wheel radius magnitude
  uint8_t rightDir;  // direction for right (inner) wheel
  
  if (radius_mm < halfW)
  {
    // Tight turn: center between wheels, inner wheel goes backward
    R_right = halfW - radius_mm;  // magnitude
    rightDir = REVERSE;
  }
  else
  {
    // Wide turn: both wheels forward
    R_right = radius_mm - halfW;
    rightDir = FORWARD;
  }
  
  R_left = radius_mm + halfW;  // outer wheel always farther from center

  // Compute arc lengths: arc = degrees * π * R / 180, using π ≈ 314/100
  uint32_t theta_num = (uint32_t)degrees * 314u;
  uint32_t theta_den = 180u * 100u;

  uint32_t arcRight_mm = (theta_num * R_right) / theta_den;
  uint32_t arcLeft_mm  = (theta_num * R_left) / theta_den;

  // Save starting odometer
  RotateRadiusStartDistLeft_mm  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
  RotateRadiusStartDistRight_mm = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));

  RotateRadiusTargetLeft_mm  = arcLeft_mm;
  RotateRadiusTargetRight_mm = arcRight_mm;

  // Speed ratio: outer/inner, avoid division by zero
  uint16_t leftSpeed, rightSpeed;
  
  if (R_right == 0u)
  {
    // Pivot around right wheel
    rightSpeed = 0u;
    leftSpeed = RADIUS_ROTATE_SPEED_MM_S;
  }
  else
  {
    float ratio = (float)R_left / (float)R_right;
    rightSpeed = RADIUS_ROTATE_SPEED_MM_S;
    leftSpeed = (uint16_t)(RADIUS_ROTATE_SPEED_MM_S * ratio);
    
    if (leftSpeed > SPEED_FULL_MM_S)
      leftSpeed = SPEED_FULL_MM_S;
  }

  // CW turn: left outer (forward), right inner (forward or reverse)
  DCMotor_SetSpeed_mm_s(leftSpeed, rightSpeed,
                        FORWARD, rightDir);

  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);
  CurrentState = NavRotatingRadius;

  DB_printf("Nav: CW radius turn %u deg R=%u L=%u R=%u\r\n",
            (unsigned)degrees,
            (unsigned)radius_mm,
            (unsigned)arcLeft_mm,
            (unsigned)arcRight_mm);
}

/****************************************************************************
 Function
     Nav_RotateCCW

 Parameters
     uint8_t degrees - angle to rotate counter-clockwise

 Returns
     None

 Description
     Rotates counter-clockwise by the specified degrees using odometer feedback.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_RotateCCW(uint8_t degrees)
{
  DB_printf("Nav: RotateCCW %u deg, arc=%u mm\r\n",
            (unsigned)degrees, (unsigned)ROTATE_ARC_MM(degrees));
  StartRotation(REVERSE, FORWARD, ROTATE_ARC_MM(degrees));
  CurrentState = NavRotating;
}

/****************************************************************************
 Function
     Nav_RotateCCWRadius

 Parameters
     uint8_t degrees - angle to rotate counter-clockwise
     uint32_t radius_mm - radius of turn in millimeters

 Returns
     None

 Description
     Rotates counter-clockwise by the specified degrees using odometer feedback, and maintains a turn radius by setting different speeds on each wheel.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_RotateCCWRadius(uint8_t degrees, uint32_t radius_mm)
{
  uint32_t halfW = TRACK_WIDTH_MM / 2u;

  // CCW turn: left wheel is inner, right wheel is outer
  // When radius_mm < halfW, inner wheel must go backward
  // Compute wheel radii from turn center (unsigned magnitudes only)
  uint32_t R_left;   // inner wheel radius magnitude
  uint32_t R_right;  // outer wheel radius magnitude
  uint8_t leftDir;   // direction for left (inner) wheel
  
  if (radius_mm < halfW)
  {
    // Tight turn: center between wheels, inner wheel goes backward
    R_left = halfW - radius_mm;  // magnitude
    leftDir = REVERSE;
  }
  else
  {
    // Wide turn: both wheels forward
    R_left = radius_mm - halfW;
    leftDir = FORWARD;
  }
  
  R_right = radius_mm + halfW;  // outer wheel always farther from center

  // Compute arc lengths: arc = degrees * π * R / 180, using π ≈ 314/100
  uint32_t theta_num = (uint32_t)degrees * 314u;
  uint32_t theta_den = 180u * 100u;

  uint32_t arcLeft_mm  = (theta_num * R_left) / theta_den;
  uint32_t arcRight_mm = (theta_num * R_right) / theta_den;

  // Save starting odometer
  RotateRadiusStartDistLeft_mm  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
  RotateRadiusStartDistRight_mm = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));

  RotateRadiusTargetLeft_mm  = arcLeft_mm;
  RotateRadiusTargetRight_mm = arcRight_mm;

  // Speed ratio: outer/inner, avoid division by zero
  uint16_t leftSpeed, rightSpeed;
  
  if (R_left == 0u)
  {
    // Pivot around left wheel
    leftSpeed = 0u;
    rightSpeed = RADIUS_ROTATE_SPEED_MM_S;
  }
  else
  {
    float ratio = (float)R_right / (float)R_left;
    leftSpeed = RADIUS_ROTATE_SPEED_MM_S;
    rightSpeed = (uint16_t)(RADIUS_ROTATE_SPEED_MM_S * ratio);
    
    if (rightSpeed > SPEED_FULL_MM_S)
      rightSpeed = SPEED_FULL_MM_S;
  }

  // CCW turn: left inner (forward or reverse), right outer (forward)
  DCMotor_SetSpeed_mm_s(leftSpeed, rightSpeed,
                        leftDir, FORWARD);

  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);
  CurrentState = NavRotatingRadius;

  DB_printf("Nav: CCW radius turn %u deg R=%u L=%u R=%u\r\n",
            (unsigned)degrees,
            (unsigned)radius_mm,
            (unsigned)arcLeft_mm,
            (unsigned)arcRight_mm);
}

/****************************************************************************
 Function
     Nav_MoveForward_mm

 Parameters
     uint32_t dist_mm - distance to move forward in millimeters

 Returns
     None

 Description
     Drives forward in a straight line for the specified distance using
     odometer feedback.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_MoveForward_mm(uint32_t dist_mm)
{
  MoveStartDistLeft_mm  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
  MoveStartDistRight_mm = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
  MoveTargetDist_mm     = dist_mm;

  DCMotor_SetSpeed_mm_s(BASE_FOLLOW_SPEED_MM_S, BASE_FOLLOW_SPEED_MM_S,
                        FORWARD, FORWARD);
  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);
  CurrentState = NavMovingForward;

  DB_printf("Nav: MoveForward %u mm\r\n", (unsigned)dist_mm);
}

/****************************************************************************
 Function
     Nav_MoveForward_mm_Follow

 Parameters
     uint32_t dist_mm - distance to move forward while following tape

 Returns
     None

 Description
     Follows the tape forward using PD control and stops after traveling
     the specified distance. Posts ES_BEHAVIOR_COMPLETE when done.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_MoveForward_mm_Follow(uint32_t dist_mm)
{
  FollowForwardStartDistLeft_mm  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
  FollowForwardStartDistRight_mm = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
  FollowForwardTargetDist_mm     = dist_mm;

  lastError     = 0;
  lineLostCount = 0;
  tIntersectionPublished = false;
  leftTurnPublished      = false;
  rightTurnPublished     = false;

  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
  CurrentState = NavFollowingForwardDistance;

  DB_printf("Nav: FollowForward %u mm\r\n", (unsigned)dist_mm);
}

/****************************************************************************
 Function
     Nav_MoveBackward_mm_Follow

 Parameters
     uint32_t dist_mm - distance to move backward while following tape

 Returns
     None

 Description
     Follows the tape backward using PD control and stops after traveling
     the specified distance. Posts ES_BEHAVIOR_COMPLETE when done.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_MoveBackward_mm_Follow(uint32_t dist_mm)
{
  FollowReverseStartDistLeft_mm  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
  FollowReverseStartDistRight_mm = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
  FollowReverseTargetDist_mm     = dist_mm;

  lastError     = 0;
  lineLostCount = 0;
  tIntersectionPublished = false;
  leftTurnPublished      = false;
  rightTurnPublished     = false;

  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, TAPE_FOLLOW_INTERVAL_MS);
  CurrentState = NavFollowingReverseDistance;

  DB_printf("Nav: FollowReverse %u mm\r\n", (unsigned)dist_mm);
}

/****************************************************************************
 Function
     Nav_MoveBackward_mm

 Parameters
     uint32_t dist_mm - distance to move backward in millimeters

 Returns
     None

 Description
     Drives backward in a straight line for the specified distance using
     odometer feedback.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_MoveBackward_mm(uint32_t dist_mm)
{
  MoveBackStartDistLeft_mm  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
  MoveBackStartDistRight_mm = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));
  MoveBackTargetDist_mm     = dist_mm;

  DCMotor_SetSpeed_mm_s(BASE_FOLLOW_SPEED_MM_S, BASE_FOLLOW_SPEED_MM_S,
                        REVERSE, REVERSE);
  ES_Timer_InitTimer(TAPE_FOLLOW_TIMER, ROTATE_POLL_INTERVAL_MS);
  CurrentState = NavMovingBackward;

  DB_printf("Nav: MoveBackward %u mm\r\n", (unsigned)dist_mm);
}

/****************************************************************************
 Function
     Nav_StartRotateContinuous

 Parameters
     bool clockwise - true for CW rotation, false for CCW

 Returns
     None

 Description
     Starts continuous rotation at ROTATE_SPEED_MM_S until Nav_Stop() or
     ES_STOP_LINE_FOLLOW is received.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_StartRotateContinuous(bool clockwise)
{
  if (clockwise)
  {
    DCMotor_SetSpeed_mm_s(ROTATE_SPEED_MM_S, ROTATE_SPEED_MM_S,
                          FORWARD, REVERSE);
    DB_printf("Nav: Continuous rotate CW\r\n");
  }
  else
  {
    DCMotor_SetSpeed_mm_s(ROTATE_SPEED_MM_S, ROTATE_SPEED_MM_S,
                          REVERSE, FORWARD);
    DB_printf("Nav: Continuous rotate CCW\r\n");
  }
  CurrentState = NavRotatingContinuous;
}

/****************************************************************************
 Function
     Nav_Stop

 Parameters
     None

 Returns
     None

 Description
     Stops all motors and timers, returns to NavIdle.

 Author
     Team, 03/02/26
****************************************************************************/
void Nav_Stop(void)
{
  DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
  ES_Timer_StopTimer(TAPE_FOLLOW_TIMER);
  ES_Timer_StopTimer(ROTATE_SAFETY_TIMER);
  CurrentState = NavIdle;
  DB_printf("Nav: Stop\r\n");
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
