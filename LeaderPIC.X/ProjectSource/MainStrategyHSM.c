/****************************************************************************
 Module
   MainStrategyHSM.c

 Revision
   1.0.0

 Description
   Main Strategy Hierarchical State Machine implementation.
   Manages high-level game strategies and coordinates AtomBehaviorFSM.

 Notes
   This HSM implements a layered control architecture:
   - MainStrategyHSM: High-level strategy sequencing
   - AtomBehaviorFSM: Low-level atomic behavior execution
   
   Supports preemption for debugging/testing with optional history restoration.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/28/26       Tianyu  Initial creation for Phase 2
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "MainStrategyHSM.h"
#include "AtomBehaviorFSM.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
static ES_Event_t DuringIdleStrategy(ES_Event_t Event);
static ES_Event_t DuringExecutingStrategy(ES_Event_t Event);
static ES_Event_t DuringPreemptedMode(ES_Event_t Event);

/*---------------------------- Module Variables ---------------------------*/
static StrategyState_t CurrentState;
static uint8_t MyPriority;

// Strategy execution state
static Strategy_t CurrentStrategy = STRATEGY_NONE;
static uint8_t StrategyStep = 0;

// Preemption state preservation
static StrategyState_t PreemptedState;
static Strategy_t PreemptedStrategy;
static uint8_t PreemptedStrategyStep;
static PreemptMode_t PreemptRecoveryMode = PREEMPT_ABORT;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitMainStrategyHSM

 Parameters
     uint8_t Priority - priority of this service

 Returns
     bool - false if error in initialization, true otherwise

 Description
     Initializes the Main Strategy HSM.

 Author
     Tianyu, 02/28/26
****************************************************************************/
bool InitMainStrategyHSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  CurrentState = InitStrategyState;
  CurrentStrategy = STRATEGY_NONE;
  StrategyStep = 0;

  DB_printf("MainStrategyHSM Init\r\n");

  // Post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  return false;
}

/****************************************************************************
 Function
     PostMainStrategyHSM

 Parameters
     ES_Event_t ThisEvent - event to post

 Returns
     bool - false if enqueue failed, true otherwise

 Description
     Posts an event to this state machine's queue.

 Author
     Tianyu, 02/28/26
****************************************************************************/
bool PostMainStrategyHSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMainStrategyHSM

 Parameters
   ES_Event_t CurrentEvent - the event to process

 Returns
   ES_Event_t - ES_NO_EVENT if no error, ES_ERROR otherwise

 Description
   Implements the Main Strategy HSM using nested switch/case
   following the Gen2 HSM pattern.

 Author
     Tianyu, 02/28/26
****************************************************************************/
ES_Event_t RunMainStrategyHSM(ES_Event_t CurrentEvent)
{
  bool MakeTransition = false;
  StrategyState_t NextState = CurrentState;
  ES_Event_t EntryEventKind = { ES_ENTRY, 0 };
  ES_Event_t ReturnEvent = CurrentEvent;

  switch (CurrentState)
  {
    case InitStrategyState:
    {
      if (CurrentEvent.EventType == ES_INIT)
      {
        NextState = IdleStrategy;
        MakeTransition = true;
        DB_printf("MainStrategyHSM: Init -> Idle\r\n");
      }
    }
    break;

    case IdleStrategy:
    {
      ReturnEvent = CurrentEvent = DuringIdleStrategy(CurrentEvent);
      
      if (ES_NO_EVENT != CurrentEvent.EventType)
      {
        switch (CurrentEvent.EventType)
        {
          case ES_STRATEGY_START:
          {
            // Start a new strategy
            CurrentStrategy = (Strategy_t)CurrentEvent.EventParam;
            StrategyStep = 0;
            NextState = ExecutingStrategy;
            MakeTransition = true;
            DB_printf("Strategy started: %d\r\n", CurrentStrategy);
          }
          break;
          
          case ES_COMMAND_RETRIEVED:
          {
            // For now, just forward commands to AtomBehaviorFSM
            // (maintaining backward compatibility during Phase 2)
            PostAtomBehaviorFSM(CurrentEvent);
          }
          break;
          
          case ES_FORCE_ATOM_BEHAVIOR:
          {
            // Direct preemption from Idle - just execute the behavior
            ES_Event_t AtomEvent;
            AtomEvent.EventType = ES_START_ATOM_BEHAVIOR;
            AtomEvent.EventParam = CurrentEvent.EventParam;
            PostAtomBehaviorFSM(AtomEvent);
            
            // Stay in Idle (or transition to PreemptedMode if desired)
            DB_printf("Force atom behavior from Idle: %d\r\n", CurrentEvent.EventParam);
          }
          break;
          
          default:
            break;
        }
      }
    }
    break;

    case ExecutingStrategy:
    {
      ReturnEvent = CurrentEvent = DuringExecutingStrategy(CurrentEvent);
      
      if (ES_NO_EVENT != CurrentEvent.EventType)
      {
        switch (CurrentEvent.EventType)
        {
          case ES_FORCE_ATOM_BEHAVIOR:
          {
            // Preempt current strategy
            PreemptedState = CurrentState;
            PreemptedStrategy = CurrentStrategy;
            PreemptedStrategyStep = StrategyStep;
            
            // Determine recovery mode (could be from EventParam high byte)
            // For now, default to PREEMPT_ABORT
            PreemptRecoveryMode = PREEMPT_ABORT;
            
            // Command AtomBehaviorFSM to execute the preempting behavior
            ES_Event_t AtomEvent;
            AtomEvent.EventType = ES_START_ATOM_BEHAVIOR;
            AtomEvent.EventParam = CurrentEvent.EventParam & 0xFF;
            PostAtomBehaviorFSM(AtomEvent);
            
            NextState = PreemptedMode;
            MakeTransition = true;
            DB_printf("Strategy preempted. Executing atom behavior: %d\r\n", 
                     AtomEvent.EventParam);
          }
          break;
          
          case ES_ATOM_BEHAVIOR_COMPLETE:
          {
            // Atom behavior completed - advance strategy
            // For now, just acknowledge completion
            DB_printf("Atom behavior completed during strategy\r\n");
            
            // TODO: Implement strategy step advancement in Phase 4
            // For now, return to Idle
            CurrentStrategy = STRATEGY_NONE;
            NextState = IdleStrategy;
            MakeTransition = true;
          }
          break;
          
          case ES_ATOM_BEHAVIOR_FAILED:
          {
            // Atom behavior failed - handle error
            DB_printf("Atom behavior FAILED during strategy\r\n");
            
            // TODO: Implement retry/recovery logic in Phase 6
            // For now, abort strategy
            CurrentStrategy = STRATEGY_NONE;
            NextState = IdleStrategy;
            MakeTransition = true;
          }
          break;
          
          case ES_COMMAND_RETRIEVED:
          {
            // New command during strategy execution - could be abort
            if (CurrentEvent.EventParam == CMD_STOP)
            {
              DB_printf("Strategy aborted by STOP command\r\n");
              CurrentStrategy = STRATEGY_NONE;
              PostAtomBehaviorFSM(CurrentEvent);
              NextState = IdleStrategy;
              MakeTransition = true;
            }
            // Otherwise ignore or handle as preempt
          }
          break;
          
          default:
            break;
        }
      }
    }
    break;

    case PreemptedMode:
    {
      ReturnEvent = CurrentEvent = DuringPreemptedMode(CurrentEvent);
      
      if (ES_NO_EVENT != CurrentEvent.EventType)
      {
        switch (CurrentEvent.EventType)
        {
          case ES_ATOM_BEHAVIOR_COMPLETE:
          {
            // Preempting behavior completed - decide recovery
            if (PreemptRecoveryMode == PREEMPT_RESUME)
            {
              // Restore previous strategy state
              CurrentStrategy = PreemptedStrategy;
              StrategyStep = PreemptedStrategyStep;
              NextState = PreemptedState;
              
              // Use history entry if going back to ExecutingStrategy
              if (NextState == ExecutingStrategy)
              {
                EntryEventKind.EventType = ES_ENTRY_HISTORY;
              }
              
              MakeTransition = true;
              DB_printf("Resuming strategy after preempt\r\n");
            }
            else  // PREEMPT_ABORT
            {
              // Abort previous strategy, return to Idle
              CurrentStrategy = STRATEGY_NONE;
              NextState = IdleStrategy;
              MakeTransition = true;
              DB_printf("Preempt complete, returning to Idle\r\n");
            }
          }
          break;
          
          case ES_ATOM_BEHAVIOR_FAILED:
          {
            // Preempting behavior failed - always abort
            CurrentStrategy = STRATEGY_NONE;
            NextState = IdleStrategy;
            MakeTransition = true;
            DB_printf("Preempt behavior failed, returning to Idle\r\n");
          }
          break;
          
          default:
            break;
        }
      }
    }
    break;
  }

  // Handle state transitions
  if (true == MakeTransition)
  {
    // Execute exit function for current state
    CurrentEvent.EventType = ES_EXIT;
    RunMainStrategyHSM(CurrentEvent);

    CurrentState = NextState;

    // Execute entry function for new state
    RunMainStrategyHSM(EntryEventKind);
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
     StartMainStrategyHSM

 Parameters
     ES_Event_t CurrentEvent

 Returns
     None

 Description
     Starts the Main Strategy HSM, initializes to Idle state.

 Author
     Tianyu, 02/28/26
****************************************************************************/
void StartMainStrategyHSM(ES_Event_t CurrentEvent)
{
  CurrentState = IdleStrategy;
  RunMainStrategyHSM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryMainStrategyHSM

 Parameters
     None

 Returns
     StrategyState_t - current state of the HSM

 Description
     Returns the current state of the Main Strategy HSM.

 Author
     Tianyu, 02/28/26
****************************************************************************/
StrategyState_t QueryMainStrategyHSM(void)
{
  return CurrentState;
}

/***************************************************************************
 Private Functions
 ***************************************************************************/

/****************************************************************************
 Function
     DuringIdleStrategy

 Parameters
     ES_Event_t Event

 Returns
     ES_Event_t - modified event (usually unchanged)

 Description
     During function for IdleStrategy state.
     Processes ES_ENTRY, ES_EXIT events.

 Author
     Tianyu, 02/28/26
****************************************************************************/
static ES_Event_t DuringIdleStrategy(ES_Event_t Event)
{
  if (ES_ENTRY == Event.EventType)
  {
    // Entry actions for Idle state
    // Stop any motors as safety measure
    // (Atom behaviors should handle this, but safety first)
  }
  else if (ES_EXIT == Event.EventType)
  {
    // Exit actions for Idle state
  }
  else
  {
    // During function
    // No continuous actions needed in Idle
  }
  
  return Event;
}

/****************************************************************************
 Function
     DuringExecutingStrategy

 Parameters
     ES_Event_t Event

 Returns
     ES_Event_t - modified event after processing by sub-machines

 Description
     During function for ExecutingStrategy state.
     Delegates to AtomBehaviorFSM for execution.

 Author
     Tianyu, 02/28/26
****************************************************************************/
static ES_Event_t DuringExecutingStrategy(ES_Event_t Event)
{
  if ((ES_ENTRY == Event.EventType) ||
      (ES_ENTRY_HISTORY == Event.EventType))
  {
    // Entry to ExecutingStrategy
    // Start/Resume AtomBehaviorFSM
    StartAtomBehaviorFSM(Event);
    
    if (ES_ENTRY_HISTORY == Event.EventType)
    {
      DB_printf("Resuming strategy execution with history\r\n");
    }
    else
    {
      DB_printf("Starting strategy execution\r\n");
    }
  }
  else if (ES_EXIT == Event.EventType)
  {
    // Give sub-machines chance to clean up
    Event = RunAtomBehaviorFSM(Event);
  }
  else
  {
    // During function - run lower level state machine
    Event = RunAtomBehaviorFSM(Event);
  }
  
  return Event;
}

/****************************************************************************
 Function
     DuringPreemptedMode

 Parameters
     ES_Event_t Event

 Returns
     ES_Event_t - modified event after processing

 Description
     During function for PreemptedMode state.
     Waits for preempting atom behavior to complete.

 Author
     Tianyu, 02/28/26
****************************************************************************/
static ES_Event_t DuringPreemptedMode(ES_Event_t Event)
{
  if (ES_ENTRY == Event.EventType)
  {
    // Entry to PreemptedMode
    DB_printf("Entered Preempted Mode\r\n");
  }
  else if (ES_EXIT == Event.EventType)
  {
    // Exit from PreemptedMode
    DB_printf("Exiting Preempted Mode\r\n");
  }
  else
  {
    // During function
    // Forward events to AtomBehaviorFSM
    Event = RunAtomBehaviorFSM(Event);
  }
  
  return Event;
}
