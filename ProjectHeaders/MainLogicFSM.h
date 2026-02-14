/****************************************************************************
 Module
     MainLogicFSM.h

 Revision
     0.1

 Description
     Header file for the Main Logic state machine.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 main logic
*****************************************************************************/

#ifndef MainLogicFSM_H
#define MainLogicFSM_H

#include "ES_Configure.h"
#include "ES_Types.h"

typedef enum
{
  Stopped,
  SimpleMoving,
  SearchingForTape,
  AligningWithBeacon
} MainLogicState_t;

bool InitMainLogicFSM(uint8_t Priority);
bool PostMainLogicFSM(ES_Event_t ThisEvent);
ES_Event_t RunMainLogicFSM(ES_Event_t ThisEvent);
MainLogicState_t QueryMainLogicFSM(void);

#endif /* MainLogicFSM_H */
