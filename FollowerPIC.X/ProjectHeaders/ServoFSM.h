/****************************************************************************
 Header file for ServoFSM.h
 
 Description
     Unified state machine for controlling four servos:
     - Sweep servo (with retract capability)
     - Scoop servo
     - Release servo (with retract capability)
     - Shoot servo

 Notes
     Each servo operates independently with its own state tracking.
     Sweep and Release servos have retracted states for volume reduction.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/26/26       Tianyu  Created from GearMotorFSM for unified servo control
****************************************************************************/

#ifndef ServoFSM_H
#define ServoFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Typedefs for servo identifiers
typedef enum
{
  SERVO_SWEEP = 0,
  SERVO_SCOOP,
  SERVO_RELEASE,
  SERVO_SHOOT,
  NUM_SERVOS
} ServoID_t;

// State definitions for each servo
typedef enum
{
  SERVO_INIT,
  SERVO_IDLE,
  SERVO_ACTING,
  SERVO_RETRACTED
} ServoState_t;

// Public Function Prototypes
bool InitServoFSM(uint8_t Priority);
bool PostServoFSM(ES_Event_t ThisEvent);
ES_Event_t RunServoFSM(ES_Event_t ThisEvent);
ServoState_t QueryServoState(ServoID_t whichServo);

// Helper function to initialize all servos to default positions
void InitializeAllServos(void);

#endif /* ServoFSM_H */
