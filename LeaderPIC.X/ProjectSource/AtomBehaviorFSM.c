/****************************************************************************
 Module
   AtomBehaviorFSM.c

 Revision
   0.2

 Description
   Atomic Behavior state machine for executing individual robot behaviors.
   Renamed from MainLogicFSM to better reflect its role as executor of
   atomic behaviors commanded by MainStrategyHSM.

 Notes
   States:
     - Stopped
     - SimpleMoving
     - SearchingForTape
     - AligningWithBeacon
     - DrivingToBeacon

 History
 When           Who     What/Why
 -------------- ---     --------
 02/28/26       Tianyu  Renamed from MainLogicFSM to AtomBehaviorFSM
 02/03/26       Tianyu  Initial creation for Lab 8 main logic
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "AtomBehaviorFSM.h"
#include "MainStrategyHSM.h"
#include "BeaconDetectFSM.h"
#include "DCMotorService.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"
#include "Ports.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/

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
static AtomBehaviorState_t CurrentState;
static uint8_t MyPriority;
static AtomBehavior_t CurrentBehavior = ATOM_IDLE;  // Track current atom behavior for completion events

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitAtomBehaviorFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the atomic behavior state machine.

 Author
     Tianyu, 02/28/26
****************************************************************************/
bool InitAtomBehaviorFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  /********************************************
   Initialization code for ports and sensors
   *******************************************/
  // Pin initialization is now handled by respective services/FSMs:
  // - Beacon pin (RB2): initialized by BeaconDetectFSM
  // - Tape sensors: initialized by DCMotorService
  // - SPI pins: initialized by SPILeaderFSM
  // - Debug pin: initialized below (AtomBehavior-specific debugging)
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
     PostAtomBehaviorFSM

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     bool, false if the enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Tianyu, 02/28/26
****************************************************************************/
bool PostAtomBehaviorFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunAtomBehaviorFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   State machine for executing atomic robot behaviors.

 Author
     Tianyu, 02/28/26
****************************************************************************/
ES_Event_t RunAtomBehaviorFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;
  
//  DB_printf("Current State is %d \r\n", CurrentState);

  switch (CurrentState)
  {
    case Stopped:
      if (ThisEvent.EventType == ES_INIT)
      {
        // On startup, automatically start searching for beacon
        DB_printf("Startup: Auto-initiating beacon search\r\n");
        ES_Event_t BeaconCommand;
        BeaconCommand.EventType = ES_COMMAND_RETRIEVED;
        BeaconCommand.EventParam = CMD_ALIGN_BEACON;
        PostAtomBehaviorFSM(BeaconCommand);
        break;
      }
      
      // Handle ES_START_ATOM_BEHAVIOR from MainStrategyHSM
      if (ThisEvent.EventType == ES_START_ATOM_BEHAVIOR)
      {
        AtomBehavior_t behavior = (AtomBehavior_t)ThisEvent.EventParam;
        CurrentBehavior = behavior;  // Track current behavior for completion events
        DB_printf("AtomBehaviorFSM: Starting atom behavior %d\r\n", behavior);
        
        switch (behavior)
        {
          case ATOM_IDLE:
            // Just stop - already in Stopped state
            MotorCommandWrapper(0, 0, FORWARD, FORWARD);
            // Post completion event back to strategy
            ES_Event_t CompleteEvent;
            CompleteEvent.EventType = ES_ATOM_BEHAVIOR_COMPLETE;
            CompleteEvent.EventParam = ATOM_IDLE;
            PostMainStrategyHSM(CompleteEvent);
            break;
            
          case ATOM_ROTATE_CW_90:
            RotateCW90();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_ROTATE_CW_45:
            RotateCW45();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_ROTATE_CCW_90:
            RotateCCW90();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_ROTATE_CCW_45:
            RotateCCW45();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_DRIVE_FWD_HALF:
            DriveForwardHalf();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_DRIVE_FWD_FULL:
            DriveForwardFull();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_DRIVE_REV_HALF:
            DriveReverseHalf();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_DRIVE_REV_FULL:
            DriveReverseFull();
            CurrentState = SimpleMoving;
            break;
            
          case ATOM_BEACON_ALIGN:
            // Check if a specific beacon is already locked
            BeaconState_t beaconState = QueryBeaconDetectFSM();
            if (beaconState == BeaconLocked) {
              // Beacon already locked - post event immediately with beacon ID
              ES_Event_t BeaconEvent;
              BeaconEvent.EventType = ES_BEACON_DETECTED;
              BeaconEvent.EventParam = QueryLockedBeaconId();
              PostAtomBehaviorFSM(BeaconEvent);
            }
            else {
              // No beacon locked yet - start rotating to search
              AlignWithBeacon();
            }
            CurrentState = AligningWithBeacon;
            break;
            
          case ATOM_TAPE_SEARCH:
            // If already HIGH, post event immediately
            if( ReadTapeSensorPin() == true ) {
              ES_Event_t TapeEvent;
              TapeEvent.EventType = ES_TAPE_DETECTED;
              TapeEvent.EventParam = 0;
              PostAtomBehaviorFSM(TapeEvent);
            }
            else {
              // Start searching
              SearchForTape();
            }
            CurrentState = SearchingForTape;
            break;
            
          default:
            DB_printf("Unknown atom behavior: %d\r\n", behavior);
            // Post failure event
            ES_Event_t FailEvent;
            FailEvent.EventType = ES_ATOM_BEHAVIOR_FAILED;
            FailEvent.EventParam = behavior;
            PostMainStrategyHSM(FailEvent);
            break;
        }
        break;
      }
      
      // Legacy ES_COMMAND_RETRIEVED support (for backward compatibility)
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
              PostAtomBehaviorFSM(BeaconEvent);
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
              PostAtomBehaviorFSM(TapeEvent);
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
      if (ThisEvent.EventType == ES_TIMEOUT &&
          ThisEvent.EventParam == SIMPLE_MOVE_TIMER) // movement timer expired after a set amount of time
      {
          DB_printf("Motor Timeout Received while moving\r\n");
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        CurrentState = Stopped;
        
        // Notify strategy of completion
        ES_Event_t CompleteEvent;
        CompleteEvent.EventType = ES_ATOM_BEHAVIOR_COMPLETE;
        CompleteEvent.EventParam = CurrentBehavior;
        PostMainStrategyHSM(CompleteEvent);
        CurrentBehavior = ATOM_IDLE;  // Reset to idle after completion
      }
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED) // while simple moving, new command received
      {
        DB_printf("New command received while moving\r\n");
        CurrentState = Stopped;
        PostAtomBehaviorFSM(ThisEvent); //go back to stopped list to take action on new command
      }
      break;

    case SearchingForTape:
      if (ThisEvent.EventType == ES_TAPE_DETECTED) // detected tape
      {
          DB_printf("Tape detected\r\n");
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        CurrentState = Stopped;
        
        // Notify strategy of successful tape search completion
        ES_Event_t CompleteEvent;
        CompleteEvent.EventType = ES_ATOM_BEHAVIOR_COMPLETE;
        CompleteEvent.EventParam = ATOM_TAPE_SEARCH;
        PostMainStrategyHSM(CompleteEvent);
      }
      else if (ThisEvent.EventType == ES_TIMEOUT &&
               ThisEvent.EventParam == TAPE_SEARCH_TIMER) // stop looking for tape after set time
      {
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        DB_printf("Tape Search Failed: Timeout");
        CurrentState = Stopped;
        
        // Notify strategy of failed tape search
        ES_Event_t FailEvent;
        FailEvent.EventType = ES_ATOM_BEHAVIOR_FAILED;
        FailEvent.EventParam = ATOM_TAPE_SEARCH;
        PostMainStrategyHSM(FailEvent);
      }
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED)    // new command received while searching for tape
      {
        DB_printf("New command received while searching for tape\r\n");
        CurrentState = Stopped;
        PostAtomBehaviorFSM(ThisEvent);
      }
      break;

    case AligningWithBeacon:
      if (ThisEvent.EventType == ES_BEACON_DETECTED) // found direction of beacon
      {
        DB_printf("Found beacon: driving towards it\r\n");
        MotorCommandWrapper(QUARTER_SPEED, QUARTER_SPEED, FORWARD, FORWARD);
        ES_Timer_InitTimer(DRIVE_TO_BEACON_TIMER, DRIVE_TO_BEACON_MS);
        CurrentState = DrivingToBeacon;
        
        // Note: Completion event will be sent when DrivingToBeacon finishes
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
        PostAtomBehaviorFSM(ThisEvent);
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
        PostAtomBehaviorFSM(BeaconCommand);
      }
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED)
      {
        DB_printf("New command received while driving to beacon\r\n");
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        ES_Timer_StopTimer(DRIVE_TO_BEACON_TIMER);
        CurrentState = Stopped;
        PostAtomBehaviorFSM(ThisEvent);
      }
      break;

    default:
      break;
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryAtomBehaviorFSM

 Parameters
     None

 Returns
     AtomBehaviorState_t: the current state of the atom behavior FSM

 Description
     Returns the current state of the atom behavior FSM

 Author
     Tianyu, 02/28/26
****************************************************************************/
AtomBehaviorState_t QueryAtomBehaviorFSM(void)
{
  return CurrentState;
}

/****************************************************************************
 Function
     StartAtomBehaviorFSM

 Parameters
     ES_Event_t CurrentEvent

 Returns
     None

 Description
     Starts/restarts the Atom Behavior FSM. Used by MainStrategyHSM
     to initialize or resume the lower level state machine.

 Author
     Tianyu, 02/28/26
****************************************************************************/
void StartAtomBehaviorFSM(ES_Event_t CurrentEvent)
{
  // For now, we maintain current state on Start
  // (allows history restoration)
  // Can be enhanced to reset to Stopped if needed
  
  // Call Run to process the entry event
  RunAtomBehaviorFSM(CurrentEvent);
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
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, REVERSE)
  // Initialize SIMPLE_MOVE_TIMER to 6000 ms
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, REVERSE);
  ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, SIMPLE_MOVE_90_MS);
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
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, REVERSE)
  // Initialize SIMPLE_MOVE_TIMER to 3000 ms
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, REVERSE);
  ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, SIMPLE_MOVE_45_MS);
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
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, REVERSE, FORWARD)
  // Initialize SIMPLE_MOVE_TIMER to 6000 ms
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, REVERSE, FORWARD);
  ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, SIMPLE_MOVE_90_MS);
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
  // Pseudocode:
  // MotorCommandWrapper(FULL_SPEED, FULL_SPEED, REVERSE, FORWARD)
  // Initialize SIMPLE_MOVE_TIMER to 3000 ms
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, REVERSE, FORWARD); 
  ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, SIMPLE_MOVE_45_MS);
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
  MotorCommandWrapper(QUARTER_SPEED, QUARTER_SPEED, FORWARD, REVERSE);
  ES_Timer_InitTimer(BEACON_ALIGN_TIMER, BEACON_ALIGN_MS);
}
