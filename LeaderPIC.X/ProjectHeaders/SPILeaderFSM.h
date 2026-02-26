/****************************************************************************
 Module
     SPILeaderFSM.h

 Revision
     0.1

 Description
     Header file for the SPI Leader FSM that sends commands to the Follower PIC.

 Notes
     This FSM sends commands to the Follower PIC via SPI communication.
     Commands come from other services or state machines in the Leader code.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/26/26       Tianyu  Renamed from CommandRetrieveService to SPILeaderFSM
 02/03/26       Tianyu  Initial creation for Lab 8 command retrieval
*****************************************************************************/

#ifndef SPILeaderFSM_H
#define SPILeaderFSM_H

#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Typedefs for state machine states
typedef enum
{
  InitSPILeaderState,
  WaitingToSend,
  SendingCommand
} SPILeaderState_t;

// Public Function Prototypes
bool InitSPILeaderFSM(uint8_t Priority);
bool PostSPILeaderFSM(ES_Event_t ThisEvent);
ES_Event_t RunSPILeaderFSM(ES_Event_t ThisEvent);
SPILeaderState_t QuerySPILeaderFSM(void);

#endif /* SPILeaderFSM_H */
