/****************************************************************************
 Module
   TemplateFSM.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "SPIFSM.h"

#include "PIC32_SPI_HAL.h"
#include "Commands.h"
#include "dbprintf.h"

#include "ES_DeferRecall.h"

/*----------------------------- Module Defines ----------------------------*/
#define SPI_RATE 1000
#define SPI_DELAY_TIME 10
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static SPIState_t CurrentState;
static ES_Event_t DeferralQueue[3 + 1];
// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitSPIFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitSPIState;
  // post the initial transition event
  ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue));

   EnterCritical();
  SPISetup_BasicConfig(SPI_SPI1);
  SPISetup_SetLeader(SPI_SPI1, SPI_SMP_MID);
  SPISetup_MapSDOutput(SPI_SPI1, SPI_RPA1);
  SPISetup_MapSSOutput(SPI_SPI1, SPI_RPA0);

  SPISetup_SetXferWidth(SPI_SPI1, SPI_8BIT);

  SPI1BUF;
  SPI1STATbits.SPIROV = 0; //clear overflow bit

  SPISetEnhancedBuffer(SPI_SPI1, false);
  
  SPISetup_SetActiveEdge(SPI_SPI1, SPI_SECOND_EDGE);
  SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_HI);
  SPISetup_SetBitTime(SPI_SPI1, SPI_RATE);

  SPISetup_EnableSPI(SPI_SPI1);
  ExitCritical();

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
     PostTemplateFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostSPIFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunSPIFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitSPIState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = Ready4Cmd;
      }
    }
    break;

    case Ready4Cmd:        // If current state is state one
    {
      switch (ThisEvent.EventType) {
        case ES_NEW_KEY: {
          switch (ThisEvent.EventParam) {
            case 'F': {
              SPIOperate_SPI1_Send8Wait(TODD_FWD);
              DB_printf("FWD_CMD \n");
            }
            break;

            case 'R': {
              SPIOperate_SPI1_Send8Wait(TODD_REV);
              DB_printf("REV_CMD \n");
            }
            break;
            
            case 'S': {
              SPIOperate_SPI1_Send8Wait(TODD_SHUTDOWN);
              DB_printf("SD_CMD \n");
            }
            break;
            
            case 'U': {
              SPIOperate_SPI1_Send8Wait(BELT_UP);
              DB_printf("BELT UP \n");
            }
            break;
            
            case 'D': {
              SPIOperate_SPI1_Send8Wait(BELT_DOWN);
              DB_printf("BELT DOWN \n");
            }
            break;
          }
          CurrentState = Sending;
          ES_Timer_InitTimer(SPI_TIMER, SPI_DELAY_TIME);
        }
        break;

        case ES_SPI_CMD: {
          SPIOperate_SPI1_Send8Wait(ThisEvent.EventParam);
          CurrentState = Sending;
          ES_Timer_InitTimer(SPI_TIMER, SPI_DELAY_TIME);
        }
        break;

        default:
        ;
      }
    }
    break;

    case Sending: {
      switch (ThisEvent.EventType)
      {
        case ES_TIMEOUT: {
          CurrentState = Ready4Cmd;
          ES_RecallEvents(MyPriority, DeferralQueue);
        }
        break;

        case ES_NEW_KEY:
        case ES_SPI_CMD: {
          ES_DeferEvent(DeferralQueue, ThisEvent);
        }
        break;
      
      default:
        break;
      }
    }
    break;
    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
SPIState_t QuerySPIFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/
