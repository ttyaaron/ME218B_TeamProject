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
#include "SPILeaderFSM.h"
#include "NavigationFSM.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"
#include "Ports.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
// Navigation intent types for tape lost recovery
#define NAV_INTENT_FORWARD    0
#define NAV_INTENT_REVERSE    1
#define NAV_INTENT_ROTATE_CW  2
#define NAV_INTENT_ROTATE_CCW 3

/*---------------------------- Module Functions ---------------------------*/
// Top-level behaviors
static void Behavior_Calibrate(void);
static void Behavior_SearchTapeCCW(void);
static void Behavior_SearchBeacon(void);
static void Behavior_IndicateSide(void);
static void Behavior_TapeFollowToT(void);
static void Behavior_MoveForward110mm(void);
static void Behavior_RotateCW90(void);
static void Behavior_MoveBackwardToNode(void);
static void Behavior_BallCollection(void);

// Ball collection sub-behaviors
static void BallCollection_Dock(void);
static void BallCollection_PushAndScoop(void);
static void BallCollection_Retract(void);

// Private helpers
static void AdvanceMainSequence(void);
static void AdvanceCollectionSequence(void);
static void Behavior_RecoverTapeLost(void);

/*---------------------------- Module Variables ---------------------------*/
static MainLogicState_t CurrentState;
static uint8_t MyPriority;

// Tape lost recovery state
static bool IsRecovering = false;
static uint8_t LastNavIntent = NAV_INTENT_FORWARD;

// Behavior function pointer type.
// Each behavior starts one async action and returns immediately.
// The action must eventually post ES_BEHAVIOR_COMPLETE to MainLogicFSM.
typedef void (*BehaviorFn_t)(void);

// ---------------------------------------------------------------
// TOP-LEVEL GAME SEQUENCE
// To change the game strategy: reorder, add, or comment out entries.
// ---------------------------------------------------------------
static const BehaviorFn_t BehaviorSequence[] = {
  Behavior_Calibrate,
  Behavior_SearchTapeCCW,
  Behavior_SearchBeacon,
  Behavior_IndicateSide,
  Behavior_TapeFollowToT,
  Behavior_MoveForward110mm,
  Behavior_RotateCW90,
  Behavior_MoveBackwardToNode,
  Behavior_BallCollection,
};
#define NUM_BEHAVIORS (sizeof(BehaviorSequence) / sizeof(BehaviorSequence[0]))
static uint8_t BehaviorIdx = 0;

// ---------------------------------------------------------------
// BALL COLLECTION SUB-SEQUENCE
// Add, remove, or reorder entries to change collection steps.
// ---------------------------------------------------------------
static const BehaviorFn_t CollectionSequence[] = {
  BallCollection_Dock,
  BallCollection_PushAndScoop,
  BallCollection_Retract,
};
#define NUM_COLLECTION_BEHAVIORS \
  (sizeof(CollectionSequence) / sizeof(CollectionSequence[0]))
static uint8_t CollectionIdx = 0;
/*------------------------------ Module Code ------------------------------*/
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

  CurrentState = ML_Running;

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
    case ML_Running:
    {
      if (ThisEvent.EventType == ES_INIT)
      {
        BehaviorIdx = 0;
        DB_printf("MainLogic: Starting sequence, behavior 0\r\n");
        BehaviorSequence[0]();
      }
      else if (ThisEvent.EventType == ES_BEHAVIOR_COMPLETE)
      {
        AdvanceMainSequence();
      }
      else if (ThisEvent.EventType == ES_CALIB_DONE)
      {
        DB_printf("MainLogic: Calibration done\r\n");
        AdvanceMainSequence();
      }
      else if (ThisEvent.EventType == ES_TAPE_FOUND)
      {
        if (IsRecovering)
        {
          IsRecovering = false;
          DB_printf("MainLogic: Recovery done, resuming behavior %u\r\n",
                    (unsigned)BehaviorIdx);
          BehaviorSequence[BehaviorIdx]();  // restart interrupted behavior
        }
        else
        {
          DB_printf("MainLogic: Tape found\r\n");
          AdvanceMainSequence();
        }
      }
      else if (ThisEvent.EventType == ES_LINE_LOST)
      {
        DB_printf("MainLogic: Line lost, starting recovery\r\n");
        IsRecovering = true;
        Behavior_RecoverTapeLost();
        // Do NOT call AdvanceMainSequence \u2014 BehaviorIdx stays the same
      }
      else if (ThisEvent.EventType == ES_BEACON_DETECTED)
      {
        // Only relevant during Behavior_SearchBeacon.
        // Record the beacon and complete the behavior.
        DB_printf("MainLogic: Beacon detected during search, id=%c\r\n",
                  (char)ThisEvent.EventParam);
        // Stop navigation rotation
        DCMotor_SetSpeed_mm_s(0, 0, FORWARD, FORWARD);
        // Advance sequence
        AdvanceMainSequence();
      }
    }
    break;

    case ML_BallCollecting:
    {
      if (ThisEvent.EventType == ES_BEHAVIOR_COMPLETE)
      {
        AdvanceCollectionSequence();
      }
    }
    break;

    case ML_Done:
      // Sequence complete — do nothing
      break;

    case ML_Stopped:
      // Manual override or error state — do nothing until reset
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
     AdvanceMainSequence

 Parameters
     None

 Returns
     None

 Description
     Advances to the next behavior in the main sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void AdvanceMainSequence(void)
{
  BehaviorIdx++;
  if (BehaviorIdx < NUM_BEHAVIORS)
  {
    DB_printf("MainLogic: behavior %u starting\r\n", (unsigned)BehaviorIdx);
    BehaviorSequence[BehaviorIdx]();
  }
  else
  {
    DB_printf("MainLogic: sequence complete\r\n");
    CurrentState = ML_Done;
  }
}

/****************************************************************************
 Function
     AdvanceCollectionSequence

 Parameters
     None

 Returns
     None

 Description
     Advances to the next behavior in the ball collection sub-sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void AdvanceCollectionSequence(void)
{
  CollectionIdx++;
  if (CollectionIdx < NUM_COLLECTION_BEHAVIORS)
  {
    DB_printf("MainLogic: collection step %u starting\r\n",
              (unsigned)CollectionIdx);
    CollectionSequence[CollectionIdx]();
  }
  else
  {
    CollectionIdx = 0;
    DB_printf("MainLogic: ball collection complete\r\n");
    CurrentState = ML_Running;
    AdvanceMainSequence();
  }
}

/****************************************************************************
 Function
     Behavior_Calibrate

 Parameters
     None

 Returns
     None

 Description
     Starts calibration rotation. ES_CALIB_DONE will advance the sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_Calibrate(void)
{
  DB_printf("Behavior: Calibrate\r\n");
  Nav_StartCalibration();
  // ES_CALIB_DONE will be received by MainLogicFSM.
  // It is mapped to AdvanceMainSequence() in ML_Running case.
}

/****************************************************************************
 Function
     Behavior_SearchTapeCCW

 Parameters
     None

 Returns
     None

 Description
     Starts CCW rotation to search for tape. ES_TAPE_FOUND will advance.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_SearchTapeCCW(void)
{
  DB_printf("Behavior: SearchTapeCCW\r\n");
  Nav_StartRotateSearch(false);  // false = CCW
  // NavigationFSM posts ES_TAPE_FOUND → MainLogicFSM
}

/****************************************************************************
 Function
     Behavior_SearchBeacon

 Parameters
     None

 Returns
     None

 Description
     Rotates CW to find beacon. ES_BEACON_DETECTED will advance.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_SearchBeacon(void)
{
  DB_printf("Behavior: SearchBeacon\r\n");
  // Check if beacon already locked from calibration rotation
  if (QueryBeaconDetectFSM() == BeaconLocked)
  {
    DB_printf("Behavior: Beacon already locked, id=%c\r\n",
              QueryLockedBeaconId());
    // Complete immediately
    PostMainLogicFSM((ES_Event_t){ES_BEHAVIOR_COMPLETE, 0});
    return;
  }
  // Otherwise start CW rotation and wait for ES_BEACON_DETECTED
  // BeaconDetectFSM posts ES_BEACON_DETECTED
  // ML_Running handles ES_BEACON_DETECTED → AdvanceMainSequence
  DCMotor_SetSpeed_mm_s(ROTATE_SPEED_MM_S, ROTATE_SPEED_MM_S,
                        FORWARD, REVERSE);
  DB_printf("Behavior: Rotating CW to find beacon\r\n");
}

/****************************************************************************
 Function
     Behavior_IndicateSide

 Parameters
     None

 Returns
     None

 Description
     Sends SPI command to indicate which field side we detected.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_IndicateSide(void)
{
  char side = QueryLockedBeaconId();
  ES_Event_t ev;
  ev.EventType  = ES_NEW_COMMAND;
  ev.EventParam = (side == 'g') ? CMD_SIDE_GREEN :
                  (side == 'b') ? CMD_SIDE_BLUE  : CMD_SIDE_MIDDLE;
  PostSPILeaderFSM(ev);
  DB_printf("Behavior: IndicateSide side=%c\r\n", side ? side : '?');
  // Fire-and-forget SPI command — complete immediately
  PostMainLogicFSM((ES_Event_t){ ES_BEHAVIOR_COMPLETE, 0 });
}

/****************************************************************************
 Function
     Behavior_TapeFollowToT

 Parameters
     None

 Returns
     None

 Description
     Follows tape until T-intersection detected.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_TapeFollowToT(void)
{
  DB_printf("Behavior: TapeFollowToT\r\n");
  LastNavIntent = NAV_INTENT_FORWARD;
  Nav_StartFollowForward();
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when T-intersection detected
}

/****************************************************************************
 Function
     Behavior_MoveForward110mm

 Parameters
     None

 Returns
     None

 Description
     Moves forward 110mm past T-intersection using odometer.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_MoveForward110mm(void)
{
  DB_printf("Behavior: MoveForward110mm\r\n");
  LastNavIntent = NAV_INTENT_FORWARD;
  Nav_MoveForward_mm(110u);
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when odometer dist reached
}

/****************************************************************************
 Function
     Behavior_RotateCW90

 Parameters
     None

 Returns
     None

 Description
     Rotates clockwise 90 degrees using odometer.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_RotateCW90(void)
{
  DB_printf("Behavior: RotateCW90\r\n");
  LastNavIntent = NAV_INTENT_ROTATE_CW;
  Nav_RotateCW(90u);
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when odometer arc reached
}

/****************************************************************************
 Function
     Behavior_RecoverTapeLost

 Parameters
     None

 Returns
     None

 Description
     Recovers from line lost condition by reverting to the last navigation intent.
     For example, if we lost the line during forward following, we start a reverse search. 
     If we lost the line during a rotate search, we start a rotate search in the opposite direction.
     Completion comes via ES_TAPE_FOUND, not ES_BEHAVIOR_COMPLETE.

 Author
     Team, 03/02/26
**********************************************************************/
static void Behavior_RecoverTapeLost(void)
{
  DB_printf("Behavior: RecoverTapeLost, LastIntent=%u\r\n", (unsigned)LastNavIntent);
  
  if (LastNavIntent == NAV_INTENT_FORWARD)
  {
    Nav_StartDriveSearch(false);  // search reverse
  }
  else if (LastNavIntent == NAV_INTENT_REVERSE)
  {
    Nav_StartDriveSearch(true); // search forward
  }
  else if (LastNavIntent == NAV_INTENT_ROTATE_CW)
  {
    Nav_StartRotateSearch(false); // CCW search
  }
  else  // NAV_INTENT_ROTATE_CCW
  {
    Nav_StartRotateSearch(true); // CW search
  }
  // NavigationFSM will post ES_TAPE_FOUND when recovery succeeds
}

/****************************************************************************
 Function
     Behavior_MoveBackwardToNode

 Parameters
     None

 Returns
     None

 Description
     Stub: moves backward to collection node.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_MoveBackwardToNode(void)
{
  DB_printf("Behavior: MoveBackwardToNode (stub)\r\n");
  PostMainLogicFSM((ES_Event_t){ES_BEHAVIOR_COMPLETE, 0});
}

/****************************************************************************
 Function
     Behavior_BallCollection

 Parameters
     None

 Returns
     None

 Description
     Starts ball collection sub-sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_BallCollection(void)
{
  DB_printf("Behavior: BallCollection starting sub-sequence\r\n");
  CollectionIdx = 0;
  CurrentState = ML_BallCollecting;
  CollectionSequence[0]();
}

/****************************************************************************
 Function
     BallCollection_Dock

 Parameters
     None

 Returns
     None

 Description
     Stub: docks at collection station.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Dock(void)
{
  DB_printf("BallCollection: Dock (stub)\r\n");
  PostMainLogicFSM((ES_Event_t){ES_BEHAVIOR_COMPLETE, 0});
}

/****************************************************************************
 Function
     BallCollection_PushAndScoop

 Parameters
     None

 Returns
     None

 Description
     Stub: pushes and scoops balls.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_PushAndScoop(void)
{
  DB_printf("BallCollection: PushAndScoop (stub)\r\n");
  PostMainLogicFSM((ES_Event_t){ES_BEHAVIOR_COMPLETE, 0});
}

/****************************************************************************
 Function
     BallCollection_Retract

 Parameters
     None

 Returns
     None

 Description
     Stub: retracts from collection station.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Retract(void)
{
  DB_printf("BallCollection: Retract (stub)\r\n");
  PostMainLogicFSM((ES_Event_t){ES_BEHAVIOR_COMPLETE, 0});
}
