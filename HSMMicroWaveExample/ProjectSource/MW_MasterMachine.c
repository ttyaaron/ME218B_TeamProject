/****************************************************************************
 Module
   MW_MasterMachine.c

 Revision
   2.0.1

 Description
   This is the master state machine for the microwave oven.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/06/12 22:02 jec      converted to Gen 2 Events and Services Framework
 02/13/10 11:54 jec      converted During functions to return Event_t
                         so that they match the template
 02/21/07 17:04 jec      converted to pass Event_t to Start...()
 02/20/07 21:37 jec      converted to use enumerated type for events
 02/21/05 15:03 jec      Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MW_MasterMachine.h"
#include "MW_Timer.h"
#include "MW_MagControl.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
static ES_Event_t DuringMaster(ES_Event_t Event);
//static void Dummy(ES_Event_t NewEvent);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMasterSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority,  and starts
     the top level state machine
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:06
****************************************************************************/
bool InitMasterSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;  // save our priority

  ThisEvent.EventType = ES_ENTRY;
  // Start the Master State machine

  StartMasterSM(ThisEvent);

  return true;
}

/****************************************************************************
 Function
     PostMasterSM

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     bool false if the post operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostMasterSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMasterSM

 Parameters
   ES_Event: the event to process

 Returns
   ES_Event: an event to return

 Description
   the run function for the top level state machine of the HierMuWave
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 02/06/12, 22:09
****************************************************************************/
ES_Event_t RunMasterSM(ES_Event_t CurrentEvent)
{
  // start by passing events to the lower level machines
  // this is the 'during' function for this machine
  CurrentEvent = DuringMaster(CurrentEvent);
  // there is only 1 state at this level and it has 2 concurrent regions
  // running the Timer & Magnetron Control state machines.
  // we only process 1 event at this level, the Clear Button
  if (EV_CLEAR == CurrentEvent.EventType)
  {
    // First pass exit messages to the lower level machines
    CurrentEvent.EventType = ES_EXIT;
    RunMasterSM(CurrentEvent);
    // Now pass entry messages, since this is a self transition
    CurrentEvent.EventType = ES_ENTRY;
    RunMasterSM(CurrentEvent);
  }
  // in the absence of an error the top level state machine should
  // always return ES_NO_EVENT
  CurrentEvent.EventType = ES_NO_EVENT;
  return CurrentEvent;
}

/****************************************************************************
 Function
     StartMasterSM

 Parameters
     ES_Event CurrentEvent

 Returns
     nothing

 Description
     Does any required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:15
****************************************************************************/
void StartMasterSM(ES_Event_t CurrentEvent)
{
  // there is only 1 state to the top level machine so no need for a state
  // variable.
  // all we need to do is to let the Run function init the lower level
  // state machines
  // use LocalEvent to keep the compiler from complaining about unused var
    RunMasterSM(CurrentEvent);
  return;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringMaster(ES_Event_t Event)
{
  // process ES_ENTRY & ES_EXIT events
  if (ES_ENTRY == Event.EventType)
  {
    // implement any entry actions required for this state machine
    // after that start any lower level machines that run in this state
    StartTimerSM(Event);
    StartMagControlSM(Event);
  }
  else if (ES_EXIT == Event.EventType)
  {
    // on exit, give the lower levels a chance to clean up first
    RunTimerSM(Event);
    // repeat for any concurrently running state machines
    RunMagControlSM(Event);
  }
  else
  {
    // do the 'during' function for this state
    // run any lower level state machine
    // since we have concurrent state machines below this level
    // we can not alter the Event that is passed
    RunTimerSM(Event);
    // repeat for any concurrent lower level machines
    RunMagControlSM(Event);
  }
  return Event;
}

