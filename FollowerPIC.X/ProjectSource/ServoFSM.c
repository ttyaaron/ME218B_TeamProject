/****************************************************************************
 Module
   ServoFSM.c

 Revision
   0.1

 Description
   Unified state machine for controlling four angle servos:
   - Sweep servo: Normal position ↔ Sweep action ↔ Retracted
   - Scoop servo: Normal position ↔ Scoop action ↔ Retracted
   - Release servo: Idle ↔ Release action
   - Shoot servo: Idle ↔ Shoot action
   
   All servos are controlled via PWM pulse widths and operate independently.

 Notes
   Converted from GearMotorFSM to support multiple servos with unified control.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/26/26       Tianyu  Created from GearMotorFSM for unified servo control
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ServoFSM.h"
#include "Ports.h"
#include "terminal.h"
#include "dbprintf.h"
#include "ES_Types.h"
#include "PWM_PIC32.h"
#include "SPIFollowerFSM.h"
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>

/*----------------------------- Module Defines ----------------------------*/
// PWM Channel assignments (using channels 1-5)
#define SWEEP_CHANNEL 1
#define SCOOP_CHANNEL 2
#define RELEASE_CHANNEL  3
#define SHOOT_CHANNEL 4
#define SIDE_CHANNEL 5

// Pin assignments (from your original code)
#define SWEEP_SERVO_PIN PWM_RPB4
#define SCOOP_SERVO_PIN PWM_RPB5
#define RELEASE_SERVO_PIN  PWM_RPB9
#define SHOOT_SERVO_PIN PWM_RPA2
#define SIDE_SERVO_PIN PWM_RPA4

// Servo pulse widths in timer ticks (TICS_PER_MS = 2500 from PWM library)
// These are example values - adjust based on your specific servos
// Bigger number -> bigger CCW angle with output axis pointing at you
#define SWEEP_IDLE_PW       (2.5 * 2500)  // 1.5ms - neutral/idle position
#define SWEEP_ACTION_PW     (1.6 * 2500)  // 2.0ms - sweep position
#define SWEEP_RETRACT_PW    (0.8 * 2500)  // 1.0ms - retracted position

#define SCOOP_IDLE_PW       (2.62 * 2500)  // 1.5ms - idle/open position
#define SCOOP_ACTION_PW     (1.6 * 2500)  // 2.0ms - scoop/closed position
#define SCOOP_RETRACT_PW    (1.6 * 2500)

#define RELEASE_IDLE_PW        (1.5 * 2500)  // 1.5ms - neutral/stored position
#define RELEASE_ACTION_PW      (2.0 * 2500)  // 2.0ms - release/dispense position
#define RELEASE_RETRACT_PW     (1.0 * 2500)  // 1.0ms - retracted position

#define SHOOT_IDLE_PW       (0.5 * 2500)  // 1.5ms - idle/ready position
#define SHOOT_ACTION_PW     (2.5 * 2500)  // 2.0ms - shoot/release position

#define SIDE_MIDDLE_PW      (1.5 * 2500)  // 1.5ms - middle/neutral position
#define SIDE_BLUE_PW        (0.5 * 2500)  // 1.0ms - blue field indicator position
#define SIDE_GREEN_PW       (2.5 * 2500)  // 2.0ms - green field indicator position

// Action durations in milliseconds
#define SWEEP_ACTION_TIME   500   // Time to complete sweep action
#define SCOOP_ACTION_TIME   600   // Time to complete scoop action
#define RELEASE_ACTION_TIME    700   // Time to complete release action
#define SHOOT_ACTION_TIME   1000   // Time to complete shoot action
#define SIDE_ACTION_TIME    300    // Time to move side servo to position

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static ServoState_t ServoStates[NUM_SERVOS];

// Mapping arrays for easier servo control
static const uint8_t ServoChannels[NUM_SERVOS] = {
  SWEEP_CHANNEL, SCOOP_CHANNEL, RELEASE_CHANNEL, SHOOT_CHANNEL, SIDE_CHANNEL
};

static const uint8_t ServoTimers[NUM_SERVOS] = {
  SWEEP_TIMER, SCOOP_TIMER, RELEASE_TIMER, SHOOT_TIMER, SIDE_TIMER
};

static const PWM_PinMap_t ServoPins[NUM_SERVOS] = {
  SWEEP_SERVO_PIN, SCOOP_SERVO_PIN, RELEASE_SERVO_PIN, SHOOT_SERVO_PIN, SIDE_SERVO_PIN
};

/*---------------------------- Module Functions ---------------------------*/
static void MoveServoToPosition(ServoID_t servo, uint16_t pulseWidth);
static void StartServoAction(ServoID_t servo);
static void ReturnServoToIdle(ServoID_t servo);
static void RetractServo(ServoID_t servo);

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
   InitServoFSM

 Parameters
   uint8_t Priority - priority of this service

 Returns
   bool - true if initialization successful

 Description
   Initializes the Servo FSM and all four servos
****************************************************************************/
bool InitServoFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;
  
  // Initialize all servo states to INIT
  for (uint8_t i = 0; i < NUM_SERVOS; i++)
  {
    ServoStates[i] = SERVO_INIT;
  }

  // Initialize PWM system for all 5 servo channels
  PWMSetup_BasicConfig(5);  // 5 channels for 5 servos
  
  // Set 50Hz frequency (20ms period) for servo control on Timer3
  PWMSetup_SetFreqOnTimer(50, _Timer3_);
  
  // Configure each servo channel
  for (uint8_t i = 0; i < NUM_SERVOS; i++)
  {
    // Assign channel to Timer3
    PWMSetup_AssignChannelToTimer(ServoChannels[i], _Timer3_);
    
    // Map channel to output pin
    PWMSetup_MapChannelToOutputPin(ServoChannels[i], ServoPins[i]);
  }
  
  // Initialize all servos to their idle/neutral positions
  InitializeAllServos();
  
  DB_printf("ServoFSM: All servos initialized\r\n");
  
  // Post the initial event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
   PostServoFSM

 Parameters
   ES_Event_t ThisEvent - event to post

 Returns
   bool - true if post successful
****************************************************************************/
bool PostServoFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
   RunServoFSM

 Parameters
   ES_Event_t ThisEvent - event to process

 Returns
   ES_Event_t - return event
****************************************************************************/
ES_Event_t RunServoFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
      // Transition all servos to IDLE state
      for (uint8_t i = 0; i < NUM_SERVOS; i++)
      {
        if (i == SERVO_SWEEP || i == SERVO_SCOOP)
        {
          // Sweep and Scoop start in retracted state
          ServoStates[i] = SERVO_RETRACTED;
        }
        else
        {
          ServoStates[i] = SERVO_IDLE;
        }
      }
      DB_printf("ServoFSM: Ready\r\n");
      break;
    }
    
    case EV_SWEEP_ACTION:
    {
      if (ServoStates[SERVO_SWEEP] == SERVO_IDLE || 
          ServoStates[SERVO_SWEEP] == SERVO_RETRACTED)
      {
        StartServoAction(SERVO_SWEEP);
        ServoStates[SERVO_SWEEP] = SERVO_ACTING;
        ES_Timer_InitTimer(SWEEP_TIMER, SWEEP_ACTION_TIME);
        DB_printf("Sweep servo: Action started\r\n");
      }
      break;
    }
    
    case EV_SCOOP_ACTION:
    {
      if (ServoStates[SERVO_SCOOP] == SERVO_IDLE || 
          ServoStates[SERVO_SCOOP] == SERVO_RETRACTED)
      {
        StartServoAction(SERVO_SCOOP);
        ServoStates[SERVO_SCOOP] = SERVO_ACTING;
        ES_Timer_InitTimer(SCOOP_TIMER, SCOOP_ACTION_TIME);
        DB_printf("Scoop servo: Action started\r\n");
      }
      break;
    }
    
    case EV_RELEASE_ACTION:
    {
      if (ServoStates[SERVO_RELEASE] == SERVO_IDLE)
      {
        StartServoAction(SERVO_RELEASE);
        ServoStates[SERVO_RELEASE] = SERVO_ACTING;
        ES_Timer_InitTimer(RELEASE_TIMER, RELEASE_ACTION_TIME);
        DB_printf("Release servo: Action started\r\n");
      }
      break;
    }
    
    case EV_SHOOT_ACTION:
    {
      if (ServoStates[SERVO_SHOOT] == SERVO_IDLE)
      {
        StartServoAction(SERVO_SHOOT);
        ServoStates[SERVO_SHOOT] = SERVO_ACTING;
        ES_Timer_InitTimer(SHOOT_TIMER, SHOOT_ACTION_TIME);
        DB_printf("Shoot servo: Action started\r\n");
      }
      break;
    }
    
    case EV_SWEEP_RETRACT:
    {
      if (ServoStates[SERVO_SWEEP] == SERVO_IDLE)
      {
        RetractServo(SERVO_SWEEP);
        ServoStates[SERVO_SWEEP] = SERVO_RETRACTED;
        DB_printf("Sweep servo: Retracted\r\n");
        
        // Post completion event
        ES_Event_t CompleteEvent;
        CompleteEvent.EventType = ES_SERVO_ACTION_COMPLETE;
        CompleteEvent.EventParam = SERVO_SWEEP;
        PostSPIFollowerFSM(CompleteEvent);
      }
      break;
    }
    
    case EV_SCOOP_RETRACT:
    {
      if (ServoStates[SERVO_SCOOP] == SERVO_IDLE)
      {
        RetractServo(SERVO_SCOOP);
        ServoStates[SERVO_SCOOP] = SERVO_RETRACTED;
        DB_printf("Scoop servo: Retracted\r\n");
        
        // Post completion event
        ES_Event_t CompleteEvent;
        CompleteEvent.EventType = ES_SERVO_ACTION_COMPLETE;
        CompleteEvent.EventParam = SERVO_SCOOP;
        PostSPIFollowerFSM(CompleteEvent);
      }
      break;
    }
    
    case EV_SIDE_BLUE:
    {
      if (ServoStates[SERVO_SIDE] == SERVO_IDLE)
      {
        MoveServoToPosition(SERVO_SIDE, SIDE_BLUE_PW);
        ServoStates[SERVO_SIDE] = SERVO_ACTING;
        ES_Timer_InitTimer(SIDE_TIMER, SIDE_ACTION_TIME);
        DB_printf("Side servo: Moving to BLUE position\r\n");
      }
      break;
    }
    
    case EV_SIDE_GREEN:
    {
      if (ServoStates[SERVO_SIDE] == SERVO_IDLE)
      {
        MoveServoToPosition(SERVO_SIDE, SIDE_GREEN_PW);
        ServoStates[SERVO_SIDE] = SERVO_ACTING;
        ES_Timer_InitTimer(SIDE_TIMER, SIDE_ACTION_TIME);
        DB_printf("Side servo: Moving to GREEN position\r\n");
      }
      break;
    }
    
    case EV_SIDE_MIDDLE:
    {
      if (ServoStates[SERVO_SIDE] == SERVO_IDLE)
      {
        MoveServoToPosition(SERVO_SIDE, SIDE_MIDDLE_PW);
        ServoStates[SERVO_SIDE] = SERVO_ACTING;
        ES_Timer_InitTimer(SIDE_TIMER, SIDE_ACTION_TIME);
        DB_printf("Side servo: Moving to MIDDLE position\r\n");
      }
      break;
    }
    
    case ES_TIMEOUT:
    {
      // Handle timeout for each servo
      if (ThisEvent.EventParam == SWEEP_TIMER)
      {
        if (ServoStates[SERVO_SWEEP] == SERVO_ACTING)
        {
          ReturnServoToIdle(SERVO_SWEEP);
          ServoStates[SERVO_SWEEP] = SERVO_IDLE;
          DB_printf("Sweep servo: Action complete\r\n");
          
          // Post completion event to SPIFollowerFSM
          ES_Event_t CompleteEvent;
          CompleteEvent.EventType = ES_SERVO_ACTION_COMPLETE;
          CompleteEvent.EventParam = SERVO_SWEEP;
          PostSPIFollowerFSM(CompleteEvent);
        }
      }
      else if (ThisEvent.EventParam == SCOOP_TIMER)
      {
        if (ServoStates[SERVO_SCOOP] == SERVO_ACTING)
        {
          ReturnServoToIdle(SERVO_SCOOP);
          ServoStates[SERVO_SCOOP] = SERVO_IDLE;
          DB_printf("Scoop servo: Action complete\r\n");
          
          ES_Event_t CompleteEvent;
          CompleteEvent.EventType = ES_SERVO_ACTION_COMPLETE;
          CompleteEvent.EventParam = SERVO_SCOOP;
          PostSPIFollowerFSM(CompleteEvent);
        }
      }
      else if (ThisEvent.EventParam == RELEASE_TIMER)
      {
        if (ServoStates[SERVO_RELEASE] == SERVO_ACTING)
        {
          ReturnServoToIdle(SERVO_RELEASE);
          ServoStates[SERVO_RELEASE] = SERVO_IDLE;
          DB_printf("Release servo: Action complete\r\n");
          
          ES_Event_t CompleteEvent;
          CompleteEvent.EventType = ES_SERVO_ACTION_COMPLETE;
          CompleteEvent.EventParam = SERVO_RELEASE;
          PostSPIFollowerFSM(CompleteEvent);
        }
      }
      else if (ThisEvent.EventParam == SHOOT_TIMER)
      {
        if (ServoStates[SERVO_SHOOT] == SERVO_ACTING)
        {
          ReturnServoToIdle(SERVO_SHOOT);
          ServoStates[SERVO_SHOOT] = SERVO_IDLE;
          DB_printf("Shoot servo: Action complete\r\n");
          
          ES_Event_t CompleteEvent;
          CompleteEvent.EventType = ES_SERVO_ACTION_COMPLETE;
          CompleteEvent.EventParam = SERVO_SHOOT;
          PostSPIFollowerFSM(CompleteEvent);
        }
      }
      else if (ThisEvent.EventParam == SIDE_TIMER)
      {
        if (ServoStates[SERVO_SIDE] == SERVO_ACTING)
        {
          ServoStates[SERVO_SIDE] = SERVO_IDLE;
          DB_printf("Side servo: Position reached\r\n");
          
          ES_Event_t CompleteEvent;
          CompleteEvent.EventType = ES_SERVO_ACTION_COMPLETE;
          CompleteEvent.EventParam = SERVO_SIDE;
          PostSPIFollowerFSM(CompleteEvent);
        }
      }
      break;
    }
    
    default:
      break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
   QueryServoState

 Parameters
   ServoID_t whichServo - which servo to query

 Returns
   ServoState_t - current state of the specified servo
****************************************************************************/
ServoState_t QueryServoState(ServoID_t whichServo)
{
  if (whichServo < NUM_SERVOS)
  {
    return ServoStates[whichServo];
  }
  return SERVO_INIT;  // Invalid servo ID
}

/****************************************************************************
 Function
   InitializeAllServos

 Description
   Sets all servos to their default/neutral positions
****************************************************************************/
void InitializeAllServos(void)
{
  // Sweep and Scoop start in retracted positions
  PWMOperate_SetPulseWidthOnChannel(SWEEP_RETRACT_PW, SWEEP_CHANNEL);
  PWMOperate_SetPulseWidthOnChannel(SCOOP_RETRACT_PW, SCOOP_CHANNEL);
  
  // Release and Shoot start in idle positions
  PWMOperate_SetPulseWidthOnChannel(RELEASE_IDLE_PW, RELEASE_CHANNEL);
  PWMOperate_SetPulseWidthOnChannel(SHOOT_IDLE_PW, SHOOT_CHANNEL);
  
  // Side servo starts in middle/neutral position
  PWMOperate_SetPulseWidthOnChannel(SIDE_MIDDLE_PW, SIDE_CHANNEL);
  
  DB_printf("All servos initialized to default positions\r\n");
}

/***************************************************************************
 Private Functions
 ***************************************************************************/

/****************************************************************************
 Function
   MoveServoToPosition

 Description
   Moves specified servo to a given pulse width position
****************************************************************************/
static void MoveServoToPosition(ServoID_t servo, uint16_t pulseWidth)
{
  if (servo < NUM_SERVOS)
  {
    PWMOperate_SetPulseWidthOnChannel(pulseWidth, ServoChannels[servo]);
  }
}

/****************************************************************************
 Function
   StartServoAction

 Description
   Moves servo to its action position
****************************************************************************/
static void StartServoAction(ServoID_t servo)
{
  switch (servo)
  {
    case SERVO_SWEEP:
      MoveServoToPosition(SERVO_SWEEP, SWEEP_ACTION_PW);
      break;
    case SERVO_SCOOP:
      MoveServoToPosition(SERVO_SCOOP, SCOOP_ACTION_PW);
      break;
    case SERVO_RELEASE:
      MoveServoToPosition(SERVO_RELEASE, RELEASE_ACTION_PW);
      break;
    case SERVO_SHOOT:
      MoveServoToPosition(SERVO_SHOOT, SHOOT_ACTION_PW);
      break;
    default:
      break;
  }
}

/****************************************************************************
 Function
   ReturnServoToIdle

 Description
   Returns servo to its idle/neutral position
****************************************************************************/
static void ReturnServoToIdle(ServoID_t servo)
{
  switch (servo)
  {
    case SERVO_SWEEP:
      MoveServoToPosition(SERVO_SWEEP, SWEEP_IDLE_PW);
      break;
    case SERVO_SCOOP:
      MoveServoToPosition(SERVO_SCOOP, SCOOP_IDLE_PW);
      break;
    case SERVO_RELEASE:
      MoveServoToPosition(SERVO_RELEASE, RELEASE_IDLE_PW);
      break;
    case SERVO_SHOOT:
      MoveServoToPosition(SERVO_SHOOT, SHOOT_IDLE_PW);
      break;
    default:
      break;
  }
}

/****************************************************************************
 Function
   RetractServo

 Description
   Moves servo to retracted position (only applicable to Sweep and Scoop)
****************************************************************************/
static void RetractServo(ServoID_t servo)
{
  switch (servo)
  {
    case SERVO_SWEEP:
      MoveServoToPosition(SERVO_SWEEP, SWEEP_RETRACT_PW);
      break;
    case SERVO_SCOOP:
      MoveServoToPosition(SERVO_SCOOP, SCOOP_RETRACT_PW);
      break;
    default:
      // Other servos don't have retract positions
      break;
  }
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
