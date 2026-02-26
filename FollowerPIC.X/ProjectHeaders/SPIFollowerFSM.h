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
  WaitingForStatus,
  SendingNewFlag,
  SendingCommand
} SPIFollowerState_t;

// Public Function Prototypes
bool InitSPIFollowerFSM(uint8_t Priority);
bool PostSPIFollowerFSM(ES_Event_t ThisEvent);
ES_Event_t RunSPIFollowerFSM(ES_Event_t ThisEvent);
SPIFollowerState_t QuerySPIFollowerFSM(void);

// SPI Servo Command Codes (received from Leader)
// CMD_INIT_SERVOS      0x01
// CMD_SWEEP            0x50
// CMD_SCOOP            0x51
// CMD_RELEASE          0x52
// CMD_SHOOT            0x53
// CMD_RETRACT_SWEEP    0x54
// CMD_RETRACT_RELEASE  0x55

#endif /* SPIFollowerFSM_H */
