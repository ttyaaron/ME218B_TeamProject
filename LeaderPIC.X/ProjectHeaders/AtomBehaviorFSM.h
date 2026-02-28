/****************************************************************************
 Module
     AtomBehaviorFSM.h

 Revision
     0.2

 Description
     Header file for the Atomic Behavior state machine.
     Renamed from MainLogicFSM to better reflect its role as executor of
     atomic behaviors commanded by MainStrategyHSM.

 Notes
     This FSM executes individual atomic behaviors such as:
     - Simple movements (rotate, drive)
     - Line following / tape searching
     - Beacon alignment
     - Ball collection/shooting
     - Side indication
     etc.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/28/26       Tianyu  Renamed from MainLogicFSM to AtomBehaviorFSM
 02/03/26       Tianyu  Initial creation for Lab 8 main logic
*****************************************************************************/

#ifndef AtomBehaviorFSM_H
#define AtomBehaviorFSM_H

#include "ES_Configure.h"
#include "ES_Types.h"

typedef enum
{
  Stopped,
  SimpleMoving,
  SearchingForTape,
  AligningWithBeacon,
  DrivingToBeacon
} AtomBehaviorState_t;

bool InitAtomBehaviorFSM(uint8_t Priority);
bool PostAtomBehaviorFSM(ES_Event_t ThisEvent);
ES_Event_t RunAtomBehaviorFSM(ES_Event_t ThisEvent);
AtomBehaviorState_t QueryAtomBehaviorFSM(void);
void StartAtomBehaviorFSM(ES_Event_t CurrentEvent);

#endif /* AtomBehaviorFSM_H */
