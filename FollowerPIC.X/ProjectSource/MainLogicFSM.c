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
     - SearchingForTape
     - AligningWithBeacon

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 main logic
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "MainLogicFSM.h"
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
static MainLogicState_t CurrentState;
static uint8_t MyPriority;

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
    case Stopped:
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
            // If already HIGH, the ES_BEACON_DETECTED event will be posted immediately
            if( ReadBeaconInputPin() == true ) {
              ES_Event_t BeaconEvent;
              BeaconEvent.EventType = ES_BEACON_DETECTED;
              BeaconEvent.EventParam = 0;
              PostMainLogicFSM(BeaconEvent);
            }
            else{
              // If not detected, act to look for beacon signal
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
      if (ThisEvent.EventType == ES_TIMEOUT &&
          ThisEvent.EventParam == SIMPLE_MOVE_TIMER) // movement timer expired after a set amount of time
      {
          DB_printf("Motor Timeout Received while moving\r\n");
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        CurrentState = Stopped;
      }
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED) // while simple moving, new command received
      {
        DB_printf("New command received while moving\r\n");
        CurrentState = Stopped;
        PostMainLogicFSM(ThisEvent); //go back to stopped list to take action on new command
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
        DB_printf("Found beacon\r\n");
        MotorCommandWrapper(0, 0, FORWARD, FORWARD); // change speed
        CurrentState = Stopped;
      }
      else if (ThisEvent.EventType == ES_TIMEOUT &&
               ThisEvent.EventParam == BEACON_ALIGN_TIMER) // set time passed, stop aligning towards beacon
      {
        MotorCommandWrapper(0, 0, FORWARD, FORWARD);
        DB_printf("Beacon Search Failed: Timeout");
        CurrentState = Stopped;
      }
      else if (ThisEvent.EventType == ES_COMMAND_RETRIEVED) // new command received while aligning for beacon
      {
        DB_printf("New command received while aligning with beacon\r\n");
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
  MotorCommandWrapper(FULL_SPEED, FULL_SPEED, FORWARD, REVERSE);
  ES_Timer_InitTimer(BEACON_ALIGN_TIMER, BEACON_ALIGN_MS);
}
