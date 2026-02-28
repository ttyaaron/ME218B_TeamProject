/****************************************************************************
 Module
   MW_MapKeys.c

 Revision
   1.0.1

 Description
   This service maps keystrokes to events for the microwave oven.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/07/12 00:00 jec      converted to service for use with E&S Gen2
 02/20/07 21:37 jec      converted to use enumerated type for events
 02/21/05 15:38 jec      Began coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <stdio.h>
#include <ctype.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MW_MapKeys.h"
#include "MW_MasterMachine.h"
#include "MW_Display.h"

/*----------------------------- Module Defines ----------------------------*/
#define CLOSED 1
#define OPEN 0

// at the 5ms/tick this is about a 3Hz update rate
#define DISPLAY_UPDATE_TIME 60

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t        MyPriority;

static uint8_t  DoorState = CLOSED;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMapKeys

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 02/07/12, 00:04
****************************************************************************/
bool InitMapKeys(uint8_t Priority)
{
  MyPriority = Priority;
  // start up the display timer
  ES_Timer_InitTimer(DISPLAY_TIMER, DISPLAY_UPDATE_TIME);

  return true;
}

/****************************************************************************
 Function
     PostMapKeys

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostMapKeys(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMapKeys

 Parameters
   ES_Event_t: the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   maps keys to Events for HierMuWave Example
 Notes

 Author
   J. Edward Carryer, 02/07/12, 00:08
****************************************************************************/
ES_Event_t RunMapKeys(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  if (EV_NEW_KEY == ThisEvent.EventType)    // there was a key pressed
  {
    switch (toupper(ThisEvent.EventParam))
    {
      case 'O':
      {
        ThisEvent.EventType = EV_DOOR_OPEN;
        DoorState = OPEN;
      }
      break;
      case 'D':
      {
        DoorState = CLOSED;
      }
      break;
      case 'C':
      {
        ThisEvent.EventType = EV_CLEAR;
      }
      break;
      case 'S':
      {
        ThisEvent.EventType = EV_START;
      }
      break;
      case 'T':
      {
        ThisEvent.EventType = EV_SET;
      }
      break;
      case 'L':
      {
        ThisEvent.EventType = EV_DEF_LIGHT;
      }
      break;
      case 'P':
      {
        ThisEvent.EventType = EV_POPCORN;
      }
      break;
    }
    PostMasterSM(ThisEvent);
  }

  if (ES_TIMEOUT == ThisEvent.EventType)    // time to update display
  {
    UpdateDisplay();
    ES_Timer_InitTimer(DISPLAY_TIMER, DISPLAY_UPDATE_TIME);
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
    ChkDoorClosed

 Parameters
   None

 Returns
   bool, true if door is closed false otherwise

 Description
   provides access to module level door state variable
 Notes

 Author
   J. Edward Carryer, 02/07/12, 00:29
****************************************************************************/
bool ChkDoorClosed(void)
{
  if (CLOSED == DoorState)
  {
    return true;
  }
  else
  {
    return false;
  }
}

