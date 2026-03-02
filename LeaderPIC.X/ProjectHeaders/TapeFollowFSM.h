/****************************************************************************

  Header file for TapeFollowFSM
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef TapeFollowFSM_H
#define TapeFollowFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  TapeIdle,
  TapeFollowing
} TapeFollowState_t;

// Public Function Prototypes
bool InitTapeFollowFSM(uint8_t Priority);
bool PostTapeFollowFSM(ES_Event_t ThisEvent);
ES_Event_t RunTapeFollowFSM(ES_Event_t ThisEvent);
TapeFollowState_t QueryTapeFollowFSM(void);

#endif /* TapeFollowFSM_H */


