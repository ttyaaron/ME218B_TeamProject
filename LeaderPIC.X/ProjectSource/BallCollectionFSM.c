/****************************************************************************
 Module
   BallCollectionFSM.c

 Revision
   1.0.0

 Description
   Ball Collection state machine implementation.
   Coordinates Leader-Follower communication to collect balls through
   a sequence of servo actions: sweep pusher, then scoop collector.

 Notes
   This is a sub-state machine called by AtomBehaviorFSM when ATOM_BALL_COLLECT
   is requested. It sends commands to the Follower PIC via SPILeaderFSM and
   waits for ES_FOLLOWER_STATUS events indicating completion.
   
   NOT registered as a service - runs as child state machine of AtomBehaviorFSM.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/28/26       Tianyu  Initial creation for ball collection coordination
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "BallCollectionFSM.h"
#include "SPILeaderFSM.h"
#include "AtomBehaviorFSM.h"
#include "MainStrategyHSM.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define COLLECTION_TIMEOUT_MS 5000  // 5 second timeout for each servo action

/*---------------------------- Module Functions ---------------------------*/
/* None */

/*---------------------------- Module Variables ---------------------------*/
static BallCollectionState_t CurrentState;
static uint8_t BallsCollected = 0;
static uint8_t TargetBallCount = 3;  // Default target number of balls

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitBallCollectionFSM

 Parameters
     None

 Returns
     None

 Description
     Initializes the Ball Collection FSM to idle state.

 Author
     Tianyu, 02/28/26
****************************************************************************/
void InitBallCollectionFSM(void)
{
  CurrentState = BallCollectionIdle;
  BallsCollected = 0;
  DB_printf("BallCollectionFSM Init\r\n");
}

/****************************************************************************
 Function
     StartBallCollection

 Parameters
     None

 Returns
     None

 Description
     Starts a new ball collection sequence.

 Author
     Tianyu, 02/28/26
****************************************************************************/
void StartBallCollection(void)
{
  DB_printf("Ball collection started\r\n");
  BallsCollected = 0;
  
  // Send CMD_SWEEP to Follower PIC
  ES_Event_t SweepCommand;
  SweepCommand.EventType = ES_NEW_COMMAND;
  SweepCommand.EventParam = CMD_SWEEP;
  PostSPILeaderFSM(SweepCommand);
  
  // Start timeout timer
  ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, COLLECTION_TIMEOUT_MS);
  
  CurrentState = WaitingSweep;
}

/****************************************************************************
 Function
    RunBallCollectionFSM

 Parameters
   ES_Event_t CurrentEvent - the event to process

 Returns
   ES_Event_t - ES_NO_EVENT if no error, ES_ERROR otherwise

 Description
   Implements the Ball Collection FSM.

 Author
     Tianyu, 02/28/26
****************************************************************************/
ES_Event_t RunBallCollectionFSM(ES_Event_t CurrentEvent)
{
  ES_Event_t ReturnEvent = CurrentEvent;
  
  switch (CurrentState)
  {
    case BallCollectionIdle:
    {
      // This state should not receive events - use StartBallCollection() instead
      // If we get here, just ignore the event
    }
    break;

    case WaitingSweep:
    {
      if (CurrentEvent.EventType == ES_FOLLOWER_STATUS)
      {
        // Follower completed sweep action
        DB_printf("Sweep action completed\r\n");
        ES_Timer_StopTimer(SIMPLE_MOVE_TIMER);
        
        // Send CMD_SCOOP to Follower PIC
        ES_Event_t ScoopCommand;
        ScoopCommand.EventType = ES_NEW_COMMAND;
        ScoopCommand.EventParam = CMD_SCOOP;
        PostSPILeaderFSM(ScoopCommand);
        
        // Start timeout timer
        ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, COLLECTION_TIMEOUT_MS);
        
        CurrentState = WaitingScoop;
      }
      else if (CurrentEvent.EventType == ES_TIMEOUT &&
               CurrentEvent.EventParam == SIMPLE_MOVE_TIMER)
      {
        // Timeout waiting for sweep completion - report failure
        DB_printf("Sweep action timeout\r\n");
        CurrentState = BallCollectionIdle;
        
        ES_Event_t FailEvent;
        FailEvent.EventType = ES_ATOM_BEHAVIOR_FAILED;
        FailEvent.EventParam = ATOM_BALL_COLLECT;
        PostMainStrategyHSM(FailEvent);
      }
    }
    break;

    case WaitingScoop:
    {
      if (CurrentEvent.EventType == ES_FOLLOWER_STATUS)
      {
        // Follower completed scoop action
        DB_printf("Scoop action completed\r\n");
        ES_Timer_StopTimer(SIMPLE_MOVE_TIMER);
        
        BallsCollected++;
        
        if (BallsCollected >= TargetBallCount)
        {
          // Collection complete
          DB_printf("Ball collection complete: %d balls collected\r\n", BallsCollected);
          CurrentState = BallCollectionIdle;
          BallsCollected = 0;  // Reset for next collection
          
          ES_Event_t CompleteEvent;
          CompleteEvent.EventType = ES_ATOM_BEHAVIOR_COMPLETE;
          CompleteEvent.EventParam = ATOM_BALL_COLLECT;
          PostMainStrategyHSM(CompleteEvent);
        }
        else
        {
          // Need to collect more balls - restart sweep
          DB_printf("Ball %d collected, collecting next ball\r\n", BallsCollected);
          
          ES_Event_t SweepCommand;
          SweepCommand.EventType = ES_NEW_COMMAND;
          SweepCommand.EventParam = CMD_SWEEP;
          PostSPILeaderFSM(SweepCommand);
          
          ES_Timer_InitTimer(SIMPLE_MOVE_TIMER, COLLECTION_TIMEOUT_MS);
          CurrentState = WaitingSweep;
        }
      }
      else if (CurrentEvent.EventType == ES_TIMEOUT &&
               CurrentEvent.EventParam == SIMPLE_MOVE_TIMER)
      {
        // Timeout waiting for scoop completion - report failure
        DB_printf("Scoop action timeout\r\n");
        CurrentState = BallCollectionIdle;
        BallsCollected = 0;  // Reset counter
        
        ES_Event_t FailEvent;
        FailEvent.EventType = ES_ATOM_BEHAVIOR_FAILED;
        FailEvent.EventParam = ATOM_BALL_COLLECT;
        PostMainStrategyHSM(FailEvent);
      }
    }
    break;

    default:
      break;
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryBallCollectionFSM

 Parameters
     None

 Returns
     BallCollectionState_t - current state of the FSM

 Description
     Returns the current state of the Ball Collection FSM.

 Author
     Tianyu, 02/28/26
****************************************************************************/
BallCollectionState_t QueryBallCollectionFSM(void)
{
  return CurrentState;
}
