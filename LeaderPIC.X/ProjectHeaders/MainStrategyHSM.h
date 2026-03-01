/****************************************************************************
 Module
     MainStrategyHSM.h

 Revision
     1.0.0

 Description
     Header file for the Main Strategy Hierarchical State Machine.
     This HSM manages high-level game strategies and coordinates
     AtomBehaviorFSM to execute atomic behaviors.

 Notes
     This state machine implements:
     - Strategy execution sequences
     - Preemption mechanism for debugging/testing
     - History state restoration after preemption
     
     States:
       Idle             - Waiting for strategy start command
       ExecutingStrategy- Executing a planned strategy sequence
       PreemptedMode    - Temporarily preempted by external command

 History
 When           Who     What/Why
 -------------- ---     --------
 02/28/26       Tianyu  Initial creation for Phase 2
*****************************************************************************/

#ifndef MainStrategyHSM_H
#define MainStrategyHSM_H

#include "ES_Configure.h"
#include "ES_Types.h"

/*----------------------------- Module Defines ----------------------------*/

// Strategy types
typedef enum
{
  STRATEGY_NONE,          // No strategy active
  STRATEGY_MAIN_GAME,     // Main game strategy sequence
  STRATEGY_TEST_SEQUENCE, // Test sequence for debugging
  STRATEGY_BALL_COLLECT,  // Ball collection only
  STRATEGY_BALL_SHOOT,    // Ball shooting only
} Strategy_t;

// Preempt recovery modes
typedef enum
{
  PREEMPT_ABORT,          // Abort strategy, return to Idle
  PREEMPT_RESUME          // Resume previous strategy after preempt
} PreemptMode_t;

// State machine states
typedef enum
{
  IdleStrategy,           // Idle, waiting for commands
  ExecutingStrategy,      // Executing a strategy sequence
  PreemptedMode           // Temporarily preempted
} StrategyState_t;

/*----------------------- Public Function Prototypes ----------------------*/

bool InitMainStrategyHSM(uint8_t Priority);
bool PostMainStrategyHSM(ES_Event_t ThisEvent);
ES_Event_t RunMainStrategyHSM(ES_Event_t ThisEvent);
StrategyState_t QueryMainStrategyHSM(void);
void StartMainStrategyHSM(ES_Event_t CurrentEvent);

#endif /* MainStrategyHSM_H */
