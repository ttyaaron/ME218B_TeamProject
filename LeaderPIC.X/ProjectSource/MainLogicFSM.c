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
static void Behavior_SearchBeaconBG(void);
static void Behavior_IndicateSide(void);
static void Behavior_TapeFollowToT(void);
static void Behavior_MoveForward110mm(void);
static void Behavior_RotateCW90(void);
static void Behavior_RotateCW90R110mm(void);
static void Behavior_MoveBackwardToNode(void);
static void Behavior_BallCollection(void);
static void Behavior_TapeFollowBackward(void);
static void Behavior_MoveForwardFollow100mm(void);

// Ball collection sub-behaviors
static void BallCollection_InitSweepServo(void);
static void BallCollection_InitScoopServo(void);
static void BallCollection_Dock(void);
static void BallCollection_Sweep1(void);
static void BallCollection_Scoop1(void);
static void BallCollection_Sweep2(void);
static void BallCollection_Scoop2(void);
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
  Behavior_SearchBeaconBG,
  Behavior_IndicateSide,
  Behavior_TapeFollowToT,
  Behavior_RotateCW90R110mm,
  Behavior_MoveForwardFollow100mm,
  Behavior_MoveBackwardToNode,
  Behavior_BallCollection,
  // Behavior_RotateCW90R110mm,
  // Behavior_TapeFollowBackward,
};
#define NUM_BEHAVIORS (sizeof(BehaviorSequence) / sizeof(BehaviorSequence[0]))
static uint8_t BehaviorIdx = 0;

// ---------------------------------------------------------------
// BALL COLLECTION SUB-SEQUENCE
// Add, remove, or reorder entries to change collection steps.
// ---------------------------------------------------------------
static const BehaviorFn_t CollectionSequence[] = {
  BallCollection_InitSweepServo,    // send CMD_SWEEP, short delay
  BallCollection_InitScoopServo,    // send CMD_SCOOP, short delay
  // BallCollection_Dock,          // Nav_MoveBackward_mm(BALL_DOCK_DISTANCE_MM)
  BallCollection_Sweep1,        // send CMD_SWEEP, wait BALL_SWEEP_DURATION_MS
  BallCollection_Scoop1,        // send CMD_SCOOP, wait BALL_SCOOP_DURATION_MS
  BallCollection_Sweep2,        // send CMD_SWEEP, wait BALL_SWEEP_DURATION_MS
  BallCollection_Scoop2,        // send CMD_SCOOP, wait BALL_SCOOP_DURATION_MS
  BallCollection_Retract,       // Nav_ ,M M,M MoveForward_mm(BALL_RETRACT_DISTANCE_MM)
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
          // TODO: need to check if current behavior needs such condition
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
        // Only relevant during Behavior_SearchBeaconBG.
        // TODO: Add a check to ensure we only handle this event during the correct behavior.
        if (BehaviorIdx == 2)  // index of Behavior_SearchBeaconBG in BehaviorSequence
        {
          DB_printf("MainLogic: Beacon detected during search, id=%c\r\n",
                    (char)ThisEvent.EventParam);

          // Only completes if the beacon is b or g — if it's neither, we still want to run the rotate search to find it
          if (ThisEvent.EventParam == 'b' || ThisEvent.EventParam == 'g')
          {
            Nav_Stop();     
            AdvanceMainSequence();
          }
        }
      }
    }
    break;

    case ML_BallCollecting:
    {
      if (ThisEvent.EventType == ES_BEHAVIOR_COMPLETE)
      {
        AdvanceCollectionSequence();
      }
      else if (ThisEvent.EventType == ES_TIMEOUT &&
               ThisEvent.EventParam == BALL_COLLECTION_TIMER)
      {
        DB_printf("MainLogic: BallCollection timer fired, advancing\r\n");
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
     Behavior_SearchBeaconBG

 Parameters
     None

 Returns
     None

 Description
     Rotates CW to find beacon. ES_BEACON_DETECTED will advance.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_SearchBeaconBG(void)
{
  DB_printf("Behavior: SearchBeacon\r\n");
  // Check if beacon already locked from calibration rotation
  if (QueryBeaconDetectFSM() == BeaconLocked)
  {
    uint8_t beaconId = QueryLockedBeaconId();
    DB_printf("Behavior: Beacon already locked, id=%c\r\n",
              beaconId);
    // Only completes if the beacon is b or g — if it's neither, we still want to run the rotate search to find it
    if (beaconId == 'b' || beaconId == 'g')
    {
      PostMainLogicFSM((ES_Event_t){ES_BEHAVIOR_COMPLETE, 0});
      return;
    }
  }
  // Otherwise start CW rotation and wait for ES_BEACON_DETECTED
  // BeaconDetectFSM posts ES_BEACON_DETECTED
  // ML_Running handles ES_BEACON_DETECTED → AdvanceMainSequence
  Nav_StartRotateContinuous(true);
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
  ev.EventParam = (side == 1) ? CMD_SIDE_GREEN :
                  (side == 0) ? CMD_SIDE_BLUE  : CMD_SIDE_MIDDLE;
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
     Behavior_RotateCW90R110mm

 Parameters
     None

 Returns
     None

 Description
     Rotates clockwise 90 degrees using odometer, with a turn radius that results in a 110mm forward displacement.

 Author
     Team, 03/02/26
****************************************************************************/

static void Behavior_RotateCW90R110mm(void)
{
  DB_printf("Behavior: RotateCW90R110mm\r\n");
  LastNavIntent = NAV_INTENT_ROTATE_CW;
  Nav_RotateCWRadius(90u, 110u);
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when odometer arc reached
}

/****************************************
  Function
      Behavior_MoveForwardFollow
  Parameters

*/
static void Behavior_MoveForwardFollow100mm(void)
{
  DB_printf("Behavior: MoveForwardFollow100mm\r\n");
  LastNavIntent = NAV_INTENT_FORWARD;
  Nav_MoveForward_mm_Follow(100u);
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when T-intersection detected
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
     Behavior_TapeFollowBackward

 Parameters
     None

 Returns
     None

 Description
     Follows tape backward until line lost.

 Author
     Team, 03/02/26 
****************************************************************************/
static void Behavior_TapeFollowBackward(void)
{
  DB_printf("Behavior: TapeFollowBackward\r\n");
  LastNavIntent = NAV_INTENT_REVERSE;
  Nav_StartFollowReverse();
  // NavigationFSM posts ES_LINE_LOST when tape lost
}

/****************************************************************************
 Function
     Behavior_MoveBackwardToNode

 Parameters
     None

 Returns
     None

 Description
     Moves backward 50mm to collection node using odometer.

 Author
     Team, 03/02/26
****************************************************************************/
static void Behavior_MoveBackwardToNode(void)
{
  DB_printf("Behavior: MoveBackwardToNode\r\n");
  LastNavIntent = NAV_INTENT_REVERSE;
  Nav_MoveBackward_mm_Follow(350u);
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when odometer dist reached
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
     BallCollection_InitSweepServo

 Parameters
     None

 Returns
     None

 Description
     Sends CMD_INIT_SERVOS to follower PIC and waits for servo initialization.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_InitSweepServo(void)
{
  DB_printf("BallCollection: InitSweepServo\r\n");
  ES_Event_t ev;
  ev.EventType  = ES_NEW_COMMAND;
  ev.EventParam = CMD_SWEEP; // Now it is simply a sweep command to initialize the servo to idle position
  PostSPILeaderFSM(ev);
  // Short delay before docking so servos reach idle position
  ES_Timer_InitTimer(BALL_COLLECTION_TIMER, BALL_INIT_SERVO_DELAY_MS);
  // Completion via BALL_COLLECTION_TIMER timeout in ML_BallCollecting
}

/****************************************************************************
 Function
  BallCollection_InitScoopServo

 Parameters
  None

 Returns
  None

 Description
  Sends CMD_SCOOP to follower PIC and waits for servo initialization.

 Author
  Team, 03/02/26
****************************************************************************/
static void BallCollection_InitScoopServo(void)
{
  DB_printf("BallCollection: InitScoopServo\r\n");
  ES_Event_t ev;
  ev.EventType  = ES_NEW_COMMAND;
  ev.EventParam = CMD_SCOOP;
  PostSPILeaderFSM(ev);
  // Short delay before docking so servos reach idle position
  ES_Timer_InitTimer(BALL_COLLECTION_TIMER, BALL_INIT_SERVO_DELAY_MS);
  // Completion via BALL_COLLECTION_TIMER timeout in ML_BallCollecting
}

/****************************************************************************
 Function
     BallCollection_Dock

 Parameters
     None

 Returns
     None

 Description
     Moves backward to dock at collection station.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Dock(void)
{
  DB_printf("BallCollection: Dock %u mm\r\n",
            (unsigned)BALL_DOCK_DISTANCE_MM);
  LastNavIntent = NAV_INTENT_REVERSE;
  Nav_MoveBackward_mm(BALL_DOCK_DISTANCE_MM);
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when odometer dist reached
}

/****************************************************************************
 Function
     BallCollection_Sweep1

 Parameters
     None

 Returns
     None

 Description
     First sweep command in collection sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Sweep1(void)
{
  DB_printf("BallCollection: Sweep 1\r\n");
  ES_Event_t ev;
  ev.EventType  = ES_NEW_COMMAND;
  ev.EventParam = CMD_SWEEP;
  PostSPILeaderFSM(ev);
  ES_Timer_InitTimer(BALL_COLLECTION_TIMER, BALL_SWEEP_DURATION_MS);
}

/****************************************************************************
 Function
     BallCollection_Scoop1

 Parameters
     None

 Returns
     None

 Description
     First scoop command in collection sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Scoop1(void)
{
  DB_printf("BallCollection: Scoop 1\r\n");
  ES_Event_t ev;
  ev.EventType  = ES_NEW_COMMAND;
  ev.EventParam = CMD_SCOOP;
  PostSPILeaderFSM(ev);
  ES_Timer_InitTimer(BALL_COLLECTION_TIMER, BALL_SCOOP_DURATION_MS);
}

/****************************************************************************
 Function
     BallCollection_Sweep2

 Parameters
     None

 Returns
     None

 Description
     Second sweep command in collection sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Sweep2(void)
{
  DB_printf("BallCollection: Sweep 2\r\n");
  ES_Event_t ev;
  ev.EventType  = ES_NEW_COMMAND;
  ev.EventParam = CMD_SWEEP;
  PostSPILeaderFSM(ev);
  ES_Timer_InitTimer(BALL_COLLECTION_TIMER, BALL_SWEEP_DURATION_MS);
}

/****************************************************************************
 Function
     BallCollection_Scoop2

 Parameters
     None

 Returns
     None

 Description
     Second scoop command in collection sequence.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Scoop2(void)
{
  DB_printf("BallCollection: Scoop 2\r\n");
  ES_Event_t ev;
  ev.EventType  = ES_NEW_COMMAND;
  ev.EventParam = CMD_SCOOP;
  PostSPILeaderFSM(ev);
  ES_Timer_InitTimer(BALL_COLLECTION_TIMER, BALL_SCOOP_DURATION_MS);
}

/****************************************************************************
 Function
     BallCollection_Retract

 Parameters
     None

 Returns
     None

 Description
     Moves forward to retract from collection station.

 Author
     Team, 03/02/26
****************************************************************************/
static void BallCollection_Retract(void)
{
  DB_printf("BallCollection: Retract %u mm\r\n",
            (unsigned)BALL_RETRACT_DISTANCE_MM);
  LastNavIntent = NAV_INTENT_FORWARD;
  Nav_MoveForward_mm(BALL_RETRACT_DISTANCE_MM);
  // NavigationFSM posts ES_BEHAVIOR_COMPLETE when odometer dist reached
}
