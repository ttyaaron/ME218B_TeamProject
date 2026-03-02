/****************************************************************************
 Module
   MainLogicFSM.c

 Revision
   0.1

 Description
   Main logic state machine for command-driven robot behavior.

 Notes
   States:
     - Stopped
     - SimpleMoving
     - CalibrationRotating
     - TapeSearchingRotate
     - FollowingForward
     - SearchingForTape
     - AligningWithBeacon
     - DrivingToBeacon

   Field Test Sequence (STAGE1_SENSOR_TEST_ONLY = 0):
     Step 1: Calibration rotation (2s CW spin)
     Step 2: Rotate search for tape
     Step 3: Forward line follow
     Step 4: T-intersection → rotate CW 90°
     Step 5: Forward line follow again
     Step 6: Line lost → stop

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 main logic
 03/01/26       Team    Added calibration and deterministic test sequence
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "MainLogicFSM.h"
#include "BeaconDetectFSM.h"
#include "DCMotorService.h"
#include "TapeFollowFSM.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"
#include "Ports.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
// Set to 1 to run workstation sensor-only test (motors will not move, only print)
// Set to 0 for field test with full sequence
#define STAGE1_SENSOR_TEST_ONLY 1

/*---------------------------- Module Functions ---------------------------*/
static void RotateCW90(void);
static void RotateCW45(void);
static void RotateCCW90(void);
static void RotateCCW45(void);
static void DriveForwardHalf(void);
static void DriveForwardFull(void);
static void DriveReverseHalf(void);
static void DriveReverseFull(void);
static void SearchForTape(void);
static void AlignWithBeacon(void);

/*---------------------------- Module Variables ---------------------------*/
static MainLogicState_t CurrentState;
static uint8_t MyPriority;

// Odometer-based rotation tracking
static uint32_t RotateStartDistLeft_mm  = 0u;
static uint32_t RotateStartDistRight_mm = 0u;
static uint32_t RotateTargetArc_mm      = 0u;

/*------------------------------ Module Code ------------------------------*/
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
  ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, ROTATE_POLL_INTERVAL_MS);

  // Start safety timeout in case encoders fail
  ES_Timer_InitTimer(ROTATE_SAFETY_TIMER, ROTATE_SAFETY_TIMEOUT_MS);

  uint32_t arc_h = targetArc_mm;  // already integer mm, no float needed
  DB_printf("StartRotation: target=%u mm L=%u R=%u\r\n",
            (unsigned)arc_h,
            (unsigned)RotateStartDistLeft_mm,
            (unsigned)RotateStartDistRight_mm);
}

/****************************************************************************
 Function
     InitMainLogicFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the main logic state machine.

 Author
     Tianyu, 02/03/26
****************************************************************************/
bool InitMainLogicFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  /********************************************
   Initialization code for ports and sensors
   *******************************************/
  // TODO: Initialize ports via Ports.c/Ports.h
  InitBeaconInputPin();
  InitTapeSensorPin();
  InitCommandSPIPins();
  InitDebugOutputPin();

  CurrentState = Stopped;

  // Stop motors on startup
  MotorCommandWrapper(0, 0, FORWARD, FORWARD);

  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  return false;
}

/****************************************************************************
 Function
     PostMainLogicFSM

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     bool, false if the enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Tianyu, 02/03/26
****************************************************************************/
bool PostMainLogicFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMainLogicFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   State machine for command-driven robot behavior.

 Author
     Tianyu, 02/03/26
****************************************************************************/
ES_Event_t RunMainLogicFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;
  
//  DB_printf("Current State is %d \r\n", CurrentState);

  switch (CurrentState)
  {
    case CalibrationRotating:
      if (ThisEvent.EventType == ES_CALIB_DONE)
      {
        DB_printf("MainLogic: Step 2 - Rotate search for tape\r\n");
        TapeFollow_StartRotateSearch(true);
        CurrentState = TapeSearchingRotate;
      }
      break;

    case Stopped:
      if (ThisEvent.EventType == ES_INIT)
      {
#if STAGE1_SENSOR_TEST_ONLY
        DB_printf("MainLogic: STAGE 1 sensor test — motors zeroed\r\n");
        TapeFollow_StartCalibrationRotate();
        DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);  // override: no motion
        CurrentState = Stopped;
#else
        DB_printf("MainLogic: Step 1 - Calibration rotation\r\n");
        TapeFollow_StartCalibrationRotate();
        CurrentState = CalibrationRotating;
#endif
        break;
      }
      if (ThisEvent.EventType == ES_COMMAND_RETRIEVED)
      {
        switch (ThisEvent.EventParam)
        {
          case CMD_STOP:
            MotorCommandWrapper(0, 0, FORWARD, FORWARD);
            break;
          case CMD_ROTATE_CW_90:
            DB_printf("State: Rotating CW 90 deg\r\n");

            RotateCW90();
            CurrentState = SimpleMoving;
            break;
          case CMD_ROTATE_CW_45:
            DB_printf("State:  Rotating CW 45 deg\r\n");

            RotateCW45();
            CurrentState = SimpleMoving;
            break;
          case CMD_ROTATE_CCW_90:
              DB_printf("State: Rotating CCW 90 deg\r\n");
            RotateCCW90();
            CurrentState = SimpleMoving;
            break;
          case CMD_ROTATE_CCW_45:
              DB_printf("State: Rotating CCW 45 deg\r\n");
            RotateCCW45();
            CurrentState = SimpleMoving;
            break;
          case CMD_DRIVE_FWD_HALF:
              DB_printf("State: drive forwards half speed\r\n");
            DriveForwardHalf();
            CurrentState = SimpleMoving;
            break;
          case CMD_DRIVE_FWD_FULL:
              DB_printf("State: drive forwards full speed\r\n");
            DriveForwardFull();
            CurrentState = SimpleMoving;
            break;
          case CMD_DRIVE_REV_HALF:
              DB_printf("State: drive reverse half speed\r\n");
            DriveReverseHalf();
            CurrentState = SimpleMoving;
            break;
          case CMD_DRIVE_REV_FULL:
              DB_printf("State: drive reverse full speed\r\n");
            DriveReverseFull();
            CurrentState = SimpleMoving;
            break;
          case CMD_ALIGN_BEACON:
              DB_printf("State: aligning with beacon\r\n");
            // Check if a specific beacon is already locked
            BeaconState_t beaconState = QueryBeaconDetectFSM();
            if (beaconState == BeaconLocked) {
              // Beacon already locked - post event immediately with beacon ID
              ES_Event_t BeaconEvent;
              BeaconEvent.EventType = ES_BEACON_DETECTED;
              BeaconEvent.EventParam = QueryLockedBeaconId();
              PostMainLogicFSM(BeaconEvent);
            }
            else {
              // No beacon locked yet (NoSignal or SignalDetected) - start rotating to search
              AlignWithBeacon();
            }
            CurrentState = AligningWithBeacon;
            break;
          case CMD_SEARCH_TAPE:
              DB_printf("State: searching for tape \r\n");
            // If already HIGH, the ES_TAPE_DETECTED event will be posted immediately
            if( ReadTapeSensorPin() == true ) {
              ES_Event_t TapeEvent;
              TapeEvent.EventType = ES_TAPE_DETECTED;
              TapeEvent.EventParam = 0;
              PostMainLogicFSM(TapeEvent);
            }
            else{
            // If not detected, act to look for line detect signal
              SearchForTape();
            }
            CurrentState = SearchingForTape;
            break;
          default:
            break;
        }
      }
      break;

    case SimpleMoving:
    {
      if (ThisEvent.EventType == ES_TIMEOUT &&
          ThisEvent.EventParam == SIMPLE_MOVE_TIMER)
      {
        // Odometer poll: check how far each wheel has traveled since rotation start
        uint32_t currentLeft  = ICCountToDistance_mm(DCMotor_GetICEventCount(LEFT_MOTOR));
        uint32_t currentRight = ICCountToDistance_mm(DCMotor_GetICEventCount(RIGHT_MOTOR));

        uint32_t deltaLeft  = currentLeft  - RotateStartDistLeft_mm;
        uint32_t deltaRight = currentRight - RotateStartDistRight_mm;
        uint32_t avgDelta   = (deltaLeft + deltaRight) / 2u;

        DB_printf("Rot poll: L=%u R=%u avg=%u target=%u\r\n",
                  (unsigned)deltaLeft, (unsigned)deltaRight,
                  (unsigned)avgDelta, (unsigned)RotateTargetArc_mm);

        if (avgDelta >= RotateTargetArc_mm)
        {
          // Target arc reached — stop rotation
          DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
          ES_Timer_StopTimer(ROTATE_SAFETY_TIMER);
          DB_printf("MainLogic: Rotation done (odometer), starting forward line follow\r\n");
          ES_Event_t ev;
          ev.EventType  = ES_START_LINE_FOLLOW;
          ev.EventParam = 0;
          PostTapeFollowFSM(ev);
          CurrentState = FollowingForward;
        }
        else
        {
          // Not done yet — poll again
          ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, ROTATE_POLL_INTERVAL_MS);
        }
      }
      else if (ThisEvent.EventType == ES_TIMEOUT &&
               ThisEvent.EventParam == ROTATE_SAFETY_TIMER)
      {
        // Safety fallback: encoder may have failed, stop anyway
        DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
        DB_printf("MainLogic: Rotation safety timeout fired — stopping\r\n");
        ES_Event_t ev;
        ev.EventType  = ES_START_LINE_FOLLOW;
        ev.EventParam = 0;
        PostTapeFollowFSM(ev);
        CurrentState = FollowingForward;
      }
    }
    break;

    case TapeSearchingRotate:
      if (ThisEvent.EventType == ES_TAPE_FOUND)
      {
        DB_printf("MainLogic: Step 3 - Forward line follow\r\n");
        ES_Event_t ev;
        ev.EventType  = ES_START_LINE_FOLLOW;
        ev.EventParam = 0;
        PostTapeFollowFSM(ev);
        CurrentState = FollowingForward;
      }
      break;

    case FollowingForward:
      if (ThisEvent.EventType == ES_INTERSECTION_DETECTED)
      {
        if (ThisEvent.EventParam == 0)  // T-intersection only
        {
          DB_printf("MainLogic: Step 4 - T-intersection, rotating CW 90\r\n");
          ES_Event_t stopEv;
          stopEv.EventType  = ES_STOP_LINE_FOLLOW;
          stopEv.EventParam = 0;
          PostTapeFollowFSM(stopEv);
          // Post the rotating event to the same state so the robot will not be stopped by 
          // TapeFollowFSM until the rotation is done. 
          ES_Event_t rotateEv;
          rotateEv.EventType  = ES_COMMAND_RETRIEVED;
          rotateEv.EventParam = CMD_ROTATE_CW_90;
          PostMainLogicFSM(rotateEv);
          CurrentState = Stopped;
        }
        // Left (param=1) or right (param=2) intersections: ignore, keep following
      }
      else if (ThisEvent.EventType == ES_LINE_LOST)
      {
        DB_printf("MainLogic: Step 6 - Line lost, stopping\r\n");
        ES_Event_t stopEv;
        stopEv.EventType  = ES_STOP_LINE_FOLLOW;
        stopEv.EventParam = 0;
        PostTapeFollowFSM(stopEv);
        CurrentState = Stopped;
      }
      break;

    case SearchingForTape:
      if (ThisEvent.EventType == ES_TAPE_DETECTED) // detected tape
      {
          DB_printf("Tape detected\r\n");
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        CurrentState = Stopped;
      }
      else if (ThisEvent.EventType == ES_TIMEOUT &&
               ThisEvent.EventParam == TAPE_SEARCH_TIMER) // stop looking for tape after set time
      {
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        DB_printf("Tape Search Failed: Timeout");
        CurrentState = Stopped;
      }
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED)    // new command received while searching for tape
      {
        DB_printf("New command received while searching for tape\r\n");
        CurrentState = Stopped;
        PostMainLogicFSM(ThisEvent);
      }
      break;

    case AligningWithBeacon:
      if (ThisEvent.EventType == ES_BEACON_DETECTED) // found direction of beacon
      {
        DB_printf("Found beacon: driving towards it\r\n");
        MotorCommandWrapper(QUARTER_SPEED, QUARTER_SPEED, FORWARD, FORWARD);
        ES_Timer_InitTimer(DRIVE_TO_BEACON_TIMER, DRIVE_TO_BEACON_MS);
        CurrentState = DrivingToBeacon;
      }
      /*
      else if (ThisEvent.EventType == ES_TIMEOUT &&
               ThisEvent.EventParam == BEACON_ALIGN_TIMER) // set time passed, stop aligning towards beacon
      {
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        DB_printf("Beacon Search Failed: Timeout");
        CurrentState = Stopped;
      } */
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED) // new command received while aligning for beacon
      {
        DB_printf("New command received while aligning with beacon\r\n");
        CurrentState = Stopped;
        PostMainLogicFSM(ThisEvent);
      }
      break;
    
    case DrivingToBeacon:
      if (ThisEvent.EventType == ES_TIMEOUT &&
          ThisEvent.EventParam == DRIVE_TO_BEACON_TIMER)
      {
        DB_printf("Some time passed. Search for beacon again. \r\n");
        // Send align with beacon command in the same state
        ES_Event_t BeaconCommand;
        BeaconCommand.EventType = ES_COMMAND_RETRIEVED;
        BeaconCommand.EventParam = CMD_ALIGN_BEACON;
        PostMainLogicFSM(BeaconCommand);
      }
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED)
      {
        DB_printf("New command received while driving to beacon\r\n");
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        ES_Timer_StopTimer(DRIVE_TO_BEACON_TIMER);
        CurrentState = Stopped;
        PostMainLogicFSM(ThisEvent);
      }
      break;

    default:
      break;
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryMainLogicFSM

 Parameters
     None

 Returns
     MainLogicState_t: the current state of the main logic FSM

 Description
     Returns the current state of the main logic FSM

 Author
     Tianyu, 02/03/26
****************************************************************************/
MainLogicState_t QueryMainLogicFSM(void)
{
  return CurrentState;
}

/*----------------------------- Helper Functions --------------------------*/
/****************************************************************************
 Function
     RotateCW90

 Parameters
     None

 Returns
     None

 Description
     Open-loop 90 degree clockwise rotation.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void RotateCW90(void)
{
  DB_printf("RotateCW90: arc=%u mm\r\n", (unsigned)ROTATE_ARC_MM(90u));
  StartRotation(FORWARD, REVERSE, ROTATE_ARC_MM(90u));
}

/****************************************************************************
 Function
     RotateCW45

 Parameters
     None

 Returns
     None

 Description
     Open-loop 45 degree clockwise rotation.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void RotateCW45(void)
{
  DB_printf("RotateCW45: arc=%u mm\r\n", (unsigned)ROTATE_ARC_MM(45u));
  StartRotation(FORWARD, REVERSE, ROTATE_ARC_MM(45u));
}

/****************************************************************************
 Function
     RotateCCW90

 Parameters
     None

 Returns
     None

 Description
     Open-loop 90 degree counter-clockwise rotation.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void RotateCCW90(void)
{
  DB_printf("RotateCCW90: arc=%u mm\r\n", (unsigned)ROTATE_ARC_MM(90u));
  StartRotation(REVERSE, FORWARD, ROTATE_ARC_MM(90u));
}

/****************************************************************************
 Function
     RotateCCW45

 Parameters
     None

 Returns
     None

 Description
     Open-loop 45 degree counter-clockwise rotation.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void RotateCCW45(void)
{
  DB_printf("RotateCCW45: arc=%u mm\r\n", (unsigned)ROTATE_ARC_MM(45u));
  StartRotation(REVERSE, FORWARD, ROTATE_ARC_MM(45u));
}

/****************************************************************************
 Function
     DriveForwardHalf

 Parameters
     None

 Returns
     None

 Description
     Drive forward at half speed (open-loop).

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void DriveForwardHalf(void)
{
  // Pseudocode:
  // MotorCommandWrapper(HALF_SPEED, HALF_SPEED, FORWARD, FORWARD)
  // Optionally set SIMPLE_MOVE_TIMER
  MotorCommandWrapper(HALF_SPEED, HALF_SPEED, FORWARD, FORWARD);
}

/****************************************************************************
 Function
     DriveForwardFull

 Parameters
     None

 Returns
     None

 Description
     Drive forward at full speed (open-loop).

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void DriveForwardFull(void)
{
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, FORWARD)
  // Optionally set SIMPLE_MOVE_TIMER
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, FORWARD);
}

/****************************************************************************
 Function
     DriveReverseHalf

 Parameters
     None

 Returns
     None

 Description
     Drive reverse at half speed (open-loop).

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void DriveReverseHalf(void)
{
  // Pseudocode:
  // MotorCommandWrapper(HALF_SPEED, HALF_SPEED, REVERSE, REVERSE)
  // Optionally set SIMPLE_MOVE_TIMER
  MotorCommandWrapper(HALF_SPEED, HALF_SPEED, REVERSE, REVERSE);
}

/****************************************************************************
 Function
     DriveReverseFull

 Parameters
     None

 Returns
     None

 Description
     Drive reverse at full speed (open-loop).

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void DriveReverseFull(void)
{
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, REVERSE, REVERSE)
  // Optionally set SIMPLE_MOVE_TIMER
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, REVERSE, REVERSE);
}

/****************************************************************************
 Function
     SearchForTape

 Parameters
     None

 Returns
     None

 Description
     Drive forward until tape detected or timeout.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void SearchForTape(void)
{
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, FORWARD)
  // Initialize TAPE_SEARCH_TIMER
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, FORWARD);
//  ES_Timer_InitTimer(TAPE_SEARCH_TIMER, TAPE_SEARCH_MS);
}

/****************************************************************************
 Function
     AlignWithBeacon

 Parameters
     None

 Returns
     None

 Description
     Spin until beacon detected or timeout.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static void AlignWithBeacon(void)
{
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, REVERSE)
  // Initialize BEACON_ALIGN_TIMER
  MotorCommandWrapper(HALF_SPEED, HALF_SPEED, FORWARD, REVERSE);
  ES_Timer_InitTimer(BEACON_ALIGN_TIMER, BEACON_ALIGN_MS);
}
