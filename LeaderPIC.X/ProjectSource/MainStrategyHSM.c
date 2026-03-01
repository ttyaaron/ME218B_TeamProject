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

// Strategy step definitions for STRATEGY_MAIN_GAME
typedef enum
{
  MAIN_GAME_SEARCH_TAPE_CCW = 0,      // Search for tape rotating CCW
  MAIN_GAME_SEARCH_BEACON,            // Search for beacon (G or B)
  MAIN_GAME_INDICATE_SIDE,            // Indicate detected side
  MAIN_GAME_SEARCH_INTERSECTION_1,    // Find first intersection
  MAIN_GAME_SEARCH_INTERSECTION_2,    // Find second intersection
  MAIN_GAME_SEARCH_T,                 // Find T-intersection
  MAIN_GAME_ROTATE_90_CCW,            // Rotate 90 degrees CCW
  MAIN_GAME_MOVE_FORWARD,             // Move forward to collection area
  MAIN_GAME_BALL_COLLECT,             // Collect balls
  MAIN_GAME_COMPLETE                  // Strategy complete
} MainGameStep_t;

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
static uint8_t DetectedBeaconColor = 0;  // Store detected beacon color (0=none, 'g'=green, 'b'=blue)

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
            // Atom behavior completed - advance strategy based on current strategy type
            DB_printf("Atom behavior completed. Strategy: %d, Step: %d\r\n", 
                     CurrentStrategy, StrategyStep);
            
            if (CurrentStrategy == STRATEGY_MAIN_GAME)
            {
              // Handle main game strategy progression
              switch (StrategyStep)
              {
                case MAIN_GAME_SEARCH_TAPE_CCW:
                  // Tape found, now search for beacon
                  DB_printf("Tape found, searching for beacon\r\n");
                  StrategyStep = MAIN_GAME_SEARCH_BEACON;
                  {
                    ES_Event_t BeaconEvent;
                    BeaconEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    BeaconEvent.EventParam = ATOM_BEACON_ALIGN;
                    PostAtomBehaviorFSM(BeaconEvent);
                  }
                  break;
                  
                case MAIN_GAME_SEARCH_BEACON:
                  // Beacon detected, check which color (from EventParam)
                  // EventParam should contain beacon ID: 'g' or 'b'
                  DetectedBeaconColor = CurrentEvent.EventParam;
                  DB_printf("Beacon detected: %c, indicating side\r\n", DetectedBeaconColor);
                  StrategyStep = MAIN_GAME_INDICATE_SIDE;
                  {
                    ES_Event_t SideEvent;
                    SideEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    SideEvent.EventParam = ATOM_SIDE_INDICATION;
                    PostAtomBehaviorFSM(SideEvent);
                  }
                  break;
                  
                case MAIN_GAME_INDICATE_SIDE:
                  // Side indicated, search for first intersection
                  DB_printf("Side indicated, searching for first intersection\r\n");
                  StrategyStep = MAIN_GAME_SEARCH_INTERSECTION_1;
                  {
                    // TODO: Need to implement ATOM_SEARCH_INTERSECTION
                    // For now, use line following or forward movement
                    ES_Event_t MoveEvent;
                    MoveEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    MoveEvent.EventParam = ATOM_DRIVE_FWD_HALF;
                    PostAtomBehaviorFSM(MoveEvent);
                  }
                  break;
                  
                case MAIN_GAME_SEARCH_INTERSECTION_1:
                  // First intersection found, search for second
                  DB_printf("First intersection found, searching for second\r\n");
                  StrategyStep = MAIN_GAME_SEARCH_INTERSECTION_2;
                  {
                    ES_Event_t MoveEvent;
                    MoveEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    MoveEvent.EventParam = ATOM_DRIVE_FWD_HALF;
                    PostAtomBehaviorFSM(MoveEvent);
                  }
                  break;
                  
                case MAIN_GAME_SEARCH_INTERSECTION_2:
                  // Second intersection found, search for T
                  DB_printf("Second intersection found, searching for T\r\n");
                  StrategyStep = MAIN_GAME_SEARCH_T;
                  {
                    ES_Event_t TEvent;
                    TEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    TEvent.EventParam = ATOM_T_INTERSECTION_FIND;
                    PostAtomBehaviorFSM(TEvent);
                  }
                  break;
                  
                case MAIN_GAME_SEARCH_T:
                  // T found, rotate 90 CCW
                  DB_printf("T found, rotating 90 CCW\r\n");
                  StrategyStep = MAIN_GAME_ROTATE_90_CCW;
                  {
                    ES_Event_t RotateEvent;
                    RotateEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    RotateEvent.EventParam = ATOM_ROTATE_CCW_90;
                    PostAtomBehaviorFSM(RotateEvent);
                  }
                  break;
                  
                case MAIN_GAME_ROTATE_90_CCW:
                  // Rotation complete, move forward
                  DB_printf("Rotation complete, moving forward\r\n");
                  StrategyStep = MAIN_GAME_MOVE_FORWARD;
                  {
                    ES_Event_t MoveEvent;
                    MoveEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    // TODO: This should use encoder-based distance control
                    MoveEvent.EventParam = ATOM_DRIVE_FWD_FULL;
                    PostAtomBehaviorFSM(MoveEvent);
                  }
                  break;
                  
                case MAIN_GAME_MOVE_FORWARD:
                  // Movement complete, start ball collection
                  DB_printf("Position reached, starting ball collection\r\n");
                  StrategyStep = MAIN_GAME_BALL_COLLECT;
                  {
                    ES_Event_t CollectEvent;
                    CollectEvent.EventType = ES_START_ATOM_BEHAVIOR;
                    CollectEvent.EventParam = ATOM_BALL_COLLECT;
                    PostAtomBehaviorFSM(CollectEvent);
                  }
                  break;
                  
                case MAIN_GAME_BALL_COLLECT:
                  // Ball collection complete, strategy finished
                  DB_printf("Ball collection complete, main game strategy finished!\r\n");
                  StrategyStep = MAIN_GAME_COMPLETE;
                  CurrentStrategy = STRATEGY_NONE;
                  NextState = IdleStrategy;
                  MakeTransition = true;
                  // TODO: Post ES_STRATEGY_COMPLETE event if needed
                  break;
                  
                default:
                  DB_printf("Unknown strategy step: %d\r\n", StrategyStep);
                  CurrentStrategy = STRATEGY_NONE;
                  NextState = IdleStrategy;
                  MakeTransition = true;
                  break;
              }
            }
            else
            {
              // Other strategies (test sequences, etc.)
              // For now, just return to Idle
              DB_printf("Strategy behavior completed, returning to Idle\r\n");
              CurrentStrategy = STRATEGY_NONE;
              NextState = IdleStrategy;
              MakeTransition = true;
            }
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
      // History entry - state should already be preserved
    }
    else
    {
      DB_printf("Starting strategy execution: %d\r\n", CurrentStrategy);
      
      // Fresh entry - start from step 0
      // Launch the first atom behavior based on strategy type
      if (CurrentStrategy == STRATEGY_MAIN_GAME)
      {
        StrategyStep = MAIN_GAME_SEARCH_TAPE_CCW;
        DB_printf("Main game: Starting with tape search CCW\r\n");
        
        // Start searching for tape rotating CCW
        ES_Event_t TapeEvent;
        TapeEvent.EventType = ES_START_ATOM_BEHAVIOR;
        TapeEvent.EventParam = ATOM_TAPE_SEARCH;
        PostAtomBehaviorFSM(TapeEvent);
      }
      else if (CurrentStrategy == STRATEGY_BALL_COLLECT)
      {
        // Ball collection only strategy
        DB_printf("Ball collect strategy: Starting collection\r\n");
        ES_Event_t CollectEvent;
        CollectEvent.EventType = ES_START_ATOM_BEHAVIOR;
        CollectEvent.EventParam = ATOM_BALL_COLLECT;
        PostAtomBehaviorFSM(CollectEvent);
      }
      else if (CurrentStrategy == STRATEGY_BALL_SHOOT)
      {
        // Ball shooting only strategy
        DB_printf("Ball shoot strategy: Starting shooting\r\n");
        ES_Event_t ShootEvent;
        ShootEvent.EventType = ES_START_ATOM_BEHAVIOR;
        ShootEvent.EventParam = ATOM_BALL_SHOOT;
        PostAtomBehaviorFSM(ShootEvent);
      }
      else
      {
        // Test sequence or unknown strategy
        DB_printf("Test/Unknown strategy: %d\r\n", CurrentStrategy);
      }
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
