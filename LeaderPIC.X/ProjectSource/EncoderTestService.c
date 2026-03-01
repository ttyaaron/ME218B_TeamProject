/****************************************************************************
 Module
   EncoderTestService.c

 Revision
   1.0.0

 Description
   Standalone encoder hardware verification service. Periodically prints
   raw encoder period values in timer ticks (no float math, no RPM conversion)
   to verify IC3 and IC2 interrupts are firing correctly on real hardware.

 Notes
   This service is for Phase 1 hardware verification only.
   It reads raw tick counts from DCMotorService via the public getter
   function and prints them every 200ms.

   DO NOT print RPM. DO NOT use floats. Print raw tick counts only.

 History
 When           Who     What/Why
 -------------- ---     --------
 03/01/26       Tianyu  Initial creation for Phase 1 encoder verification
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "EncoderTestService.h"
#include "DCMotorService.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
#define ENCODER_PRINT_INTERVAL 200  // Print every 200ms

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitEncoderTestService

 Parameters
     uint8_t Priority : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the Encoder Test Service and starts the periodic print timer

 Author
     Tianyu, 03/01/26
****************************************************************************/
bool InitEncoderTestService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  // Start the periodic print timer
  ES_Timer_InitTimer(ENCODER_PRINT_TIMER, ENCODER_PRINT_INTERVAL);
  
  DB_printf("Encoder Test Service Initialized\r\n");
  
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
     PostEncoderTestService

 Parameters
     ES_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this service's queue

 Author
     Tianyu, 03/01/26
****************************************************************************/
bool PostEncoderTestService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunEncoderTestService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles timeout events for periodic encoder period printing

 Author
   Tianyu, 03/01/26
****************************************************************************/
ES_Event_t RunEncoderTestService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
      // Initialization complete
      break;
      
    case ES_TIMEOUT:
      // Check if this is the encoder print timer
      if (ThisEvent.EventParam == ENCODER_PRINT_TIMER)
      {
        // Read raw encoder periods from DCMotorService
        uint32_t leftPeriod = DCMotor_GetEncoderPeriod(LEFT_MOTOR);
        uint32_t rightPeriod = DCMotor_GetEncoderPeriod(RIGHT_MOTOR);
        
        // Print raw tick counts only (no float math, no RPM)
        DB_printf("ENC L ticks:%u  R ticks:%u\r\n",
                  (unsigned int)leftPeriod,
                  (unsigned int)rightPeriod);
        
        // Restart the print timer
        ES_Timer_InitTimer(ENCODER_PRINT_TIMER, ENCODER_PRINT_INTERVAL);
      }
      break;
      
    default:
      break;
  }
  
  return ReturnEvent;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
