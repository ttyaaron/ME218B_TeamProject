/****************************************************************************

  Header file for NavigationFSM
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef NavigationFSM_H
#define NavigationFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  NavIdle,
  NavCalibrating,       // rotating to build sensor min/max ranges
  NavSearching,         // spinning or driving to find tape
  NavFollowingForward,
  NavFollowingReverse,
  NavRotating,          // point turn at target angle
  NavMovingForward,     // straight drive forward to target distance
  NavMovingBackward,    // straight drive backward to target distance
  NavRotatingContinuous // continuous rotation until stopped
} NavigationState_t;

// Public Function Prototypes
bool InitNavigationFSM(uint8_t Priority);
bool PostNavigationFSM(ES_Event_t ThisEvent);
ES_Event_t RunNavigationFSM(ES_Event_t ThisEvent);
NavigationState_t QueryNavigationFSM(void);

// Public search and control functions
void Nav_StartCalibration(void);
void Nav_StartRotateSearch(bool clockwise);
void Nav_StartDriveSearch(bool forward);
void Nav_StartFollowReverse(void);
void Nav_StartFollowForward(void);
void Nav_RotateCW(uint8_t degrees);
void Nav_RotateCCW(uint8_t degrees);
void Nav_MoveForward_mm(uint32_t dist_mm);
void Nav_MoveBackward_mm(uint32_t dist_mm);
void Nav_StartRotateContinuous(bool clockwise);
void Nav_Stop(void);

#endif /* NavigationFSM_H */


