/****************************************************************************
 Module
     BallCollectionFSM.h

 Revision
     1.0.0

 Description
     Header file for the Ball Collection state machine.
     This FSM coordinates the Leader and Follower PICs to collect balls
     through a sequence of servo actions (sweep pusher, scoop collector).

 Notes
     States:
       Idle          - Waiting for collection to start
       WaitingSweep  - CMD_SWEEP sent, waiting for Follower confirmation
       WaitingScoop  - CMD_SCOOP sent, waiting for Follower confirmation
       CollectionComplete - All actions completed

 History
 When           Who     What/Why
 -------------- ---     --------
 02/28/26       Tianyu  Initial creation for ball collection coordination
*****************************************************************************/

#ifndef BallCollectionFSM_H
#define BallCollectionFSM_H

#include "ES_Configure.h"
#include "ES_Types.h"

/*----------------------------- Module Defines ----------------------------*/

// Ball collection states
typedef enum
{
  BallCollectionIdle,
  WaitingSweep,
  WaitingScoop,
  CollectionComplete
} BallCollectionState_t;

/*----------------------- Public Function Prototypes ----------------------*/

void InitBallCollectionFSM(void);
ES_Event_t RunBallCollectionFSM(ES_Event_t ThisEvent);
BallCollectionState_t QueryBallCollectionFSM(void);
void StartBallCollection(void);

#endif /* BallCollectionFSM_H */
