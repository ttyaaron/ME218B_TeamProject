/****************************************************************************
 Module
   MW_Timer.c

 Revision
   1.0.1

 Description
   Timer state machine for HierMuWave example.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/06/12 23:13 jec      converted to Gen2 Events and Services Framework
 02/13/10 11:50 jec      converted During functions to return Event_t
                         so that they match the template
 02/21/07 17:04 jec      converted to pass Event_t to Start...()
 02/20/07 21:37 jec      converted to use enumerated type for events
 02/21/05 12:21 jec      Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MW_Timer.h"
#include "MW_MapKeys.h"
#include "PIC32_AD_Lib.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

// assumes 5ms tick rate
#define TICKS_PER_SEC 200
#define POPCORN_TIME (10 * TICKS_PER_SEC)
#define COOK_TIME (15 * TICKS_PER_SEC)

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringCountingState(ES_Event_t Event);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static TimerState CurrentState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunTimerSM

 Parameters
   ES_Event_t CurrentEvent: the event to process

 Returns
   ES_Event

 Description
   Implements the timer state machine for the microwave oven
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 2/21/05, 12:30PM
****************************************************************************/
ES_Event_t RunTimerSM(ES_Event_t CurrentEvent)
{
  bool        MakeTransition = false; /* are we making a state transition? */
  TimerState  NextState = CurrentState;
  uint16_t    RequestedTime;
  uint32_t    ADResults[1];               //New A/D library returns results into an array
  ES_Event_t  ReturnEvent = CurrentEvent; // assume we are not consuming event

  switch (CurrentState)
  {
    case ExpiredState:
    {
      // No During function for this state
      //process any events
      if (ES_NO_EVENT != CurrentEvent.EventType)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case EV_SET:  //set button Pressed
          {             // Execute action function, set the cooking timer
            // uncomment these lines to get real analog input
            //ADC_MultiRead(ADResults);
            //RequestedTime = ((ADResults[0] / 32) * TICKS_PER_SEC);
            // fake out A/D for debugger demo and force a cook time
            //comment out this line to enable real input
            RequestedTime = COOK_TIME;
            if (0 != RequestedTime)
            {
              ES_Timer_SetTimer(OVEN_TIMER, RequestedTime);
              NextState = SetState;                 //Decide what the next state will be
              MakeTransition = true;                //mark that we are taking a transition
              ReturnEvent.EventType = ES_NO_EVENT;  // consume this event
            }
          }
          break;
          case EV_POPCORN:  //Popcorn button Pressed
          {                 // Execute action function, set the cooking timer
            ES_Timer_SetTimer(OVEN_TIMER, POPCORN_TIME);
            NextState = SetState;                 //Decide what the next state will be
            MakeTransition = true;                //mark that we are taking a transition
            ReturnEvent.EventType = ES_NO_EVENT;  // consume this event
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

    case SetState:
    {
      // No During function for this state
      //process any events
      if (ES_NO_EVENT != CurrentEvent.EventType)     //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case EV_START:  //start button Pressed
          {               // implement Guard Function tests
            if (true == ChkDoorClosed())
            {
              NextState = CountingState;            //Decide what the next state will be
              MakeTransition = true;                //mark that we are taking a transition
              ReturnEvent.EventType = ES_NO_EVENT;  // consume this event
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

    case CountingState:
    {
      // Execute During function for counting state. ES_ENTRY & ES_EXIT are
      // processed here
      CurrentEvent = DuringCountingState(CurrentEvent);
      //process any events
      if (ES_NO_EVENT != CurrentEvent.EventType)       //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case ES_TIMEOUT:       //Timer Expired
          {
            NextState = ExpiredState;     //Decide what the next state will be
            MakeTransition = true;        //mark that we are taking a transition
          }
          break;
          case EV_DOOR_OPEN:       //Door Opened
          {
            NextState = SetState;       //Decide what the next state will be
            MakeTransition = true;      //mark that we are taking a transition
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
    RunTimerSM(CurrentEvent);

    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    CurrentEvent.EventType = ES_ENTRY;
    RunTimerSM(CurrentEvent);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartTimerSM

 Parameters
     ES_Event_t CurrentEvent

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 2/21/05, 12:54PM
****************************************************************************/
void StartTimerSM(ES_Event_t CurrentEvent)
{
  CurrentState = ExpiredState;   // always start in Expired
  // call the entry function (if any) for the ENTRY_STATE
  // there is no entry function for Expired
}

/****************************************************************************
 Function
     QueryTimerSM

 Parameters
     None

 Returns
     TimerState The current state of the Timer state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 2/21/05, 12:59PM
****************************************************************************/
TimerState QueryTimerSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringCountingState(ES_Event_t Event)
{
  // process ES_ENTRY & ES_EXIT events
  if (ES_ENTRY == Event.EventType)
  {
    // implement any entry actions required for this state machine
    ES_Timer_StartTimer(OVEN_TIMER);
  }
  else if (ES_EXIT == Event.EventType)
  {
    // now do any local exit functionality
    ES_Timer_StopTimer(OVEN_TIMER);
  }
  else
  {
    // do the 'during' function for this state
    // no during function for this machine
  }
  return Event;       // Don't remap this event
}

