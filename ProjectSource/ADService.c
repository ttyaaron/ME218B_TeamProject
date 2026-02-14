/****************************************************************************
 Module
   ADService.c

 Revision
   1.0.0

 Description
   This service handles reading the A/D converter from the potentiometer
   and updating the desired speed of the DC motor based on the
   analog reading.

 Notes
    When initializing the ADC service:
        Configure ADC for potentiometer input
        Set desiredSpeed = DEFAULT_SPEED
        Store desiredSpeed at module level
        Configure a dedicated ADC sampling timer to expire at 10 Hz
    Start the ADC sampling timer

    When the ADC checking timer TMR_ADC expires:
        Read ADC value
        Convert ADC value → desiredSpeed
        Clamp desiredSpeed to [MIN_SPEED, MAX_SPEED]

        If |desiredSpeed − lastDesiredSpeed| > SPEED_DEADBAND:
        Update module-level desiredSpeed
        Post ES_MOTOR_ACTION_CHANGE(desiredSpeed)
        Update lastDesiredSpeed

    Restart the ADC sampling timer

    To provide the current desired speed to other services:
    Return the current value of the module-level speed variable


 History
 When           Who     What/Why
 -------------- ---     --------
 01/21/26       Tianyu    Updated for Lab 6 motor speed control
 01/14/26       Tianyu    Initial creation for Lab 5
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "ADService.h"
#include "DCMotorService.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"
#include <stdlib.h>

/*----------------------------- Module Defines ----------------------------*/
// Timer interval for checking the potentiometer (10Hz = 100ms)
#define ADC_CHECK_INTERVAL 100

// A/D converter configuration
#define POT_PIN_MASK BIT10HI  // Configure for AN10 (RB14) potentiometer input, pin 25 (Datasheet P4 Pin Diagrams)

#define SPEED_DEADBAND 0  // Minimum change in speed to trigger update

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service */

/*---------------------------- Module Variables ---------------------------*/
// Module level Priority variable
static uint8_t MyPriority;

// Module level variable to store the current desired speed
// This can be accessed by other services via GetDesiredSpeed()
static uint16_t CurrentDesiredSpeed;
static uint16_t LastDesiredSpeed;

// Buffer for A/D readings
static uint32_t ADCResults[1];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitADService

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the A/D converter and starts the ADC checking timer

 Author
     Tianyu, 01/14/26
****************************************************************************/
bool InitADService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  /********************************************
   Initialization code for ADC service
   *******************************************/
  
  // Configure the analog-to-digital converter for the potentiometer input
  if (!ADC_ConfigAutoScan(POT_PIN_MASK))
  {
    return false; // Return false if ADC configuration fails
  }

  ADC_MultiRead(ADCResults);
  
  // Set desiredSpeed = result read from ADC
  // Store desiredSpeed at module level

  CurrentDesiredSpeed = ADCResults[0];
  LastDesiredSpeed = CurrentDesiredSpeed;
  
  // Configure a timer to check potentiometer regularly
  // Start the ADC checking timer
  ES_Timer_InitTimer(AD_TIMER, ADC_CHECK_INTERVAL);
  
  // Post the initial transition event
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
     PostADService

 Parameters
     ES_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Tianyu, 01/14/26
****************************************************************************/
bool PostADService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunADService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles timer events to read the A/D converter and update desired speed
   when the ADC checking timer expires.

 Author
   Tianyu, 01/14/26
****************************************************************************/
ES_Event_t RunADService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // Assume no errors
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
      // No additional initialization needed here
      break;
      
    case ES_TIMEOUT:
      // Check if this timeout is for our ADC timer
      if (ThisEvent.EventParam == AD_TIMER)
      {
        // When the ADC checking timer expires:
        
        // Read the current voltage from the potentiometer
        ADC_MultiRead(ADCResults);
        
        // DB_printf("Current ADC is %d\r\n",
        // ADCResults[0]);

        CurrentDesiredSpeed = ADCResults[0];
        
//        DB_printf("Current Desired Speed is %d\r\n",
//          CurrentDesiredSpeed);

        // Post a ES_MOTOR_ACTION_CHANGE event with the new desired speed if it changed
        int32_t speedDelta = (int32_t)CurrentDesiredSpeed - (int32_t)LastDesiredSpeed;
        if (abs(speedDelta) > SPEED_DEADBAND)
        {
          // Update lastDesiredSpeed
          LastDesiredSpeed = CurrentDesiredSpeed;
          
          // Post ES_MOTOR_ACTION_CHANGE event
          ES_Event_t SpeedChangedEvent;
          SpeedChangedEvent.EventType = ES_MOTOR_ACTION_CHANGE;
          SpeedChangedEvent.EventParam = CurrentDesiredSpeed;
          PostDCMotorService(SpeedChangedEvent);
        }
        
        // Restart the ADC checking timer
        ES_Timer_InitTimer(AD_TIMER, ADC_CHECK_INTERVAL);
      }
      break;
      
    default:
      break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     GetDesiredSpeed

 Parameters
     None

 Returns
     uint16_t, the current desired speed

 Description
     Access function to provide the current desired speed to other services
     Returns the current value of the module-level desired speed variable
     
 Notes

 Author
     Tianyu, 01/14/26
****************************************************************************/
uint16_t GetDesiredSpeed(void)
{
  // To provide the current desired speed to other services:
  // Return the current value of the module-level desired speed variable
  return CurrentDesiredSpeed;
}

/***************************************************************************
 Private Functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
