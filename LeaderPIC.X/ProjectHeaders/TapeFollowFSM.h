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
  TapeCalibrating,       // rotating to build sensor min/max ranges
  TapeSearching,         // spinning or driving to find tape
  TapeFollowingF,
  TapeFollowingR
} TapeFollowState_t;

// Public Function Prototypes
bool InitTapeFollowFSM(uint8_t Priority);
bool PostTapeFollowFSM(ES_Event_t ThisEvent);
ES_Event_t RunTapeFollowFSM(ES_Event_t ThisEvent);
TapeFollowState_t QueryTapeFollowFSM(void);

// Public search and control functions
void TapeFollow_StartCalibrationRotate(void);
void TapeFollow_StartRotateSearch(bool clockwise);
void TapeFollow_StartDriveSearch(bool forward);
void TapeFollow_StartReverse(void);

#endif /* TapeFollowFSM_H */


