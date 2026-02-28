/****************************************************************************
 Module
   d:\me218bTNG\Lectures\Lecture31\HierMuWave\MW_PowControl.c

 Revision
   1.0.1

 Description
   This is the power state machine for the microwave oven.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/07/12 08:03 jec      converted to Events and Services Framework Gen2
 02/13/10 11:52 jec      converted During functions to return Event_t
                         so that they match the template
 02/21/07 17:04 jec      converted to pass Event_t to Start...()
 02/20/07 21:37 jec      converted to use enumerated type for events
 02/21/05 13:25 jec      Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MW_PowControl.h"
#include <stdio.h>

/*----------------------------- Module Defines ----------------------------*/
// define constants for this machine
#define STATE_ONE 1
#define STATE_TWO 2

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringFullState(ES_Event_t Event);
static ES_Event_t DuringHalfState(ES_Event_t Event);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static PowerState CurrentState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunPowControlSM

 Parameters
   unsigned char: the event to process

 Returns
   unsigned char: an event to return

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 2/21/05, 1:34PM
****************************************************************************/
ES_Event_t RunPowControlSM(ES_Event_t CurrentEvent)
{
  bool        MakeTransition = false;  /* are we making a state transition? */
  PowerState  NextState = CurrentState;
  ES_Event_t  ReturnEvent = CurrentEvent;       // assume we are not consuming event

  switch (CurrentState)
  {
    case FullState:
    {
      // Execute During function for Full power State. ES_ENTRY & ES_EXIT are
      // processed here
      ReturnEvent = CurrentEvent = DuringFullState(CurrentEvent);
      //process any events
      if (ES_NO_EVENT != CurrentEvent.EventType )             //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case EV_DEF_LIGHT:      //If event is event one
          {
            NextState = HalfState;        //Decide what the next state will be
            MakeTransition = true;        //mark that we are taking a transition
          }
          break;
            // repeat cases as required for relevant events
          default:
          {
          }
          break;
        }
      }
    }
    break;
    case HalfState:
    {
      // Execute During function for Half power State. ES_ENTRY & ES_EXIT are
      // processed here
      ReturnEvent = CurrentEvent = DuringHalfState(CurrentEvent);
      // we don't process any events other than ES_ENTRY in this state
    }
    break;
  }
  //   If we are making a state transition
  if (true == MakeTransition)
  {
    //   Execute exit function for current state
    CurrentEvent.EventType = ES_EXIT;
    RunPowControlSM(CurrentEvent);

    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    CurrentEvent.EventType = ES_ENTRY;
    RunPowControlSM(CurrentEvent);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartPowControlSM

 Parameters
     Event_t CurrentEvent

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 2/21/05, 1:40PM
****************************************************************************/
void StartPowControlSM(ES_Event_t CurrentEvent)
{
  if (ES_ENTRY_HISTORY != CurrentEvent.EventType)
  {
    CurrentState = FullState;
  }
  // call the entry function (if any) for the ENTRY_STATE
    RunPowControlSM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryPowControlSM

 Parameters
     None

 Returns
     unsigned char The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/
PowerState QueryPowControlSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringFullState(ES_Event_t Event)
{
  // process ES_ENTRY & ES_EXIT events
  if (ES_ENTRY == Event.EventType)
  {
    // implement any entry actions required for this state machine
    // in a real muWave this is where we would set the power level to 100%
  }
  else if (ES_EXIT == Event.EventType)
  {
    // on exit, give the lower levels a chance to clean up first
    // now do any local exit functionality, there is none here
  }
  else
  {
    // do the 'during' function for this state
    // nothing to do here in this state machine
  }
  return Event;       // Don't remap event
}

static ES_Event_t DuringHalfState(ES_Event_t Event)
{
  // process ES_ENTRY & ES_EXIT events
  if (ES_ENTRY == Event.EventType)
  {
    // implement any entry actions required for this state machine
    // in a real muWave this is where we would set the power level to 50%
  }
  else if (ES_EXIT == Event.EventType)
  {
    // on exit, give the lower levels a chance to clean up first
    // now do any local exit functionality, there is none here
  }
  else
  {
    // do the 'during' function for this state
    // nothing to do here in this state machine
  }
  return Event;      // Don't remap this event
}

