/****************************************************************************
 Module
     SPIFollowerFSM.h

 Revision
     0.1

 Description
     Header file for the SPI Follower FSM that receives keyboard commands
     and transmits them to the Leader PIC via SPI.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/17/26       Tianyu  Initial creation for Leader-Follower architecture
*****************************************************************************/

#ifndef SPIFollowerFSM_H
#define SPIFollowerFSM_H

#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Typedefs for state machine states
typedef enum
{
  InitSPIFollowerState,
  WaitingForCommand,
  SendingNewFlag,
  SendingCommand
} SPIFollowerState_t;

// Public Function Prototypes
bool InitSPIFollowerFSM(uint8_t Priority);
bool PostSPIFollowerFSM(ES_Event_t ThisEvent);
ES_Event_t RunSPIFollowerFSM(ES_Event_t ThisEvent);
SPIFollowerState_t QuerySPIFollowerFSM(void);

#endif /* SPIFollowerFSM_H */
