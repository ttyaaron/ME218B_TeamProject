/****************************************************************************
 Module
   d:\me218bTNG\Lectures\Lecture31\HierMuWave\MW_MagControl.c

 Revision
   1.0.1

 Description
   This is a MagControl file for implementing state machines.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/07/12 00:45 jec      converted to Events and Services Gen2
 02/13/10 11:53 jec      converted During functions to return Event_t
                         so that they match the template
 02/21/07 17:04 jec      converted to pass Event_t to Start...()
 02/20/07 21:37 jec      converted to use enumerated type for events
 02/21/95 14:20 jec      Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MW_MagControl.h"
#include "MW_PowControl.h"
#include "MW_Timer.h"       // so we can test the state
#include "MW_MapKeys.h"     // to test door state
#include <stdio.h>

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringCookingState(ES_Event_t Event);
static ES_Event_t DuringNotCookingState(ES_Event_t Event);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static MagControlState CurrentState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunMagControlSM

 Parameters
   unsigned char: the event to process

 Returns
   unsigned char: an event to return

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 2/21/05, 2:25PM
****************************************************************************/
ES_Event_t RunMagControlSM(ES_Event_t CurrentEvent)
{
  bool            MakeTransition = false;  /* are we making a state transition? */
  MagControlState NextState = CurrentState;
  ES_Event_t      EntryEventKind = { ES_ENTRY, 0 }; // default to normal entry to new state
  ES_Event_t      ReturnEvent = CurrentEvent;       // assume we are not consuming event

  switch (CurrentState)
  {
    case CookingState:  // If current state is state one
    {                   // Execute During function for state one. ES_ENTRY & ES_EXIT are
      // processed here
      ReturnEvent = CurrentEvent = DuringCookingState(CurrentEvent);
      //process any events
      if (ES_NO_EVENT != CurrentEvent.EventType)             //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case ES_TIMEOUT:
          {
            // Execute action function for state one : event one
            NextState = NotCookingState;        //Decide what the next state will be
            MakeTransition = true;              //mark that we are taking a transition
          }
          break;
          case EV_DOOR_OPEN:
          {
            // We need to process this one specially, since the NotCooking
            // state has a lower level state machine that contains a
            // history state.
            EntryEventKind.EventType = ES_ENTRY_HISTORY;
            NextState = NotCookingState;
            MakeTransition = true;            //mark that we are taking a transition
          }
          break;
          default:
          {
          }
          break;
        }
      }
    }
    break;
    case NotCookingState: // If current state is state one
    {                     // Execute During function for state one. ES_ENTRY & ES_EXIT are
      // processed here
      ReturnEvent = CurrentEvent = DuringNotCookingState(CurrentEvent);
      //process any events
      if (ES_NO_EVENT != CurrentEvent.EventType)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case EV_START:
          {
            // implement Guard Function tests
            if (((CountingState == QueryTimerSM()) ||
                  (SetState == QueryTimerSM())) &&
                (true == ChkDoorClosed()))
            {
              // Execute action function
              NextState = CookingState;       //Decide what the next state will be
              MakeTransition = true;          //mark that we are taking a transition
            }
          }
          break;
          default:
          {
          }
          break;
        }
      }
    }
    break;
  }
  //   If we are making a state transition
  if (true == MakeTransition)
  {
    //   Execute exit function for current state
    CurrentEvent.EventType = ES_EXIT;
    RunMagControlSM(CurrentEvent);

    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    RunMagControlSM(EntryEventKind);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartMagControlSM

 Parameters
     Event_t CurrentEvent

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 2/21/05, 2:38PM
****************************************************************************/
void StartMagControlSM(ES_Event_t CurrentEvent)
{
  CurrentState = NotCookingState;
  // call the entry function (if any) for the ENTRY_STATE
    RunMagControlSM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryMagControlSM

 Parameters
     None

 Returns
     MagControlState The current state of the MagControl state machine

 Description
     returns the current state of the MagControl state machine
 Notes

 Author
     J. Edward Carryer, 2/21/05, 2:50PM
****************************************************************************/
MagControlState QueryMagControlSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringCookingState(ES_Event_t Event)
{
  // process ES_ENTRY & ES_EXIT events
  if (ES_ENTRY == Event.EventType)
  {
    // Here's where we would start the magnetron running
  }
  else if (ES_EXIT == Event.EventType)
  {
    // Here's where we would stop the magnetron running
  }
  else
  {
    // do the 'during' function for this state
    // no during function in this state
  }
  return Event;    // don't remap event
}

static ES_Event_t DuringNotCookingState(ES_Event_t Event)
{
  // process ES_ENTRY & ES_EXIT events
  if ((ES_ENTRY == Event.EventType) ||
      (ES_ENTRY_HISTORY == Event.EventType))
  {
    StartPowControlSM(Event);
  }
  else if (ES_EXIT == Event.EventType)
  {
    // on exit, give the lower levels a chance to clean up first
    Event = RunPowControlSM(Event);
    // repeat for any concurrently running state machines
    // now do any local exit functionality
  }
  else
  {
    // do the 'during' function for this state
    // run any lower level state machine
    Event = RunPowControlSM(Event);
  }
  return Event;
}

