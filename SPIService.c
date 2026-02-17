/****************************************************************************
 Module
   TemplateService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "SPIService.h"
#include "DriverHSM.h"
#include "PIC32_SPI_HAL.h"
#include <sys/attribs.h>
#include "dbprintf.h"
#include "Commands.h"
#include "BeltService.h"


/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
void __ISR(_SPI_1_VECTOR,IPL7SOFT) SPI_ISR(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitSPIService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
 TRISAbits.TRISA0 = 0;
    
    ////      SPI initialization       ////
    DB_printf("Pic2Pic Follower Init\n");
    
    SPISetup_BasicConfig(SPI_SPI1);
    //SPISetup_SetFollower(SPI_SPI1);
    SPI1CONbits.MSTEN = 0;
    ANSELBbits.ANSB14 = 0; // clock
    TRISBbits.TRISB14 = 1;
    //TRISBbits.TRISB8 = 0; // SDO to output
    
    // Setup SS Input on RPB15
    TRISBbits.TRISB3 = 1;
    ANSELBbits.ANSB3 = 0;
    SS1R = 0b0001;

    // Setup SDI on RB11
    TRISBbits.TRISB5 = 1;
    SDI1R = 0b0001;
    
    IFS0CLR = _IFS0_INT4IF_MASK;
    //SPISetup_MapSDOutput(SPI_SPI1, SPI_RPB8);
    SPI1CONbits.SSEN = 1;
    SPI1CONbits.DISSDO = 1;
    SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_HI);
    SPISetup_SetActiveEdge(SPI_SPI1, SPI_SECOND_EDGE);
    SPISetup_SetXferWidth(SPI_SPI1, SPI_8BIT);
    SPISetup_Interrupts(SPI_SPI1);
    SPISetEnhancedBuffer(SPI_SPI1, false);
    SPISetup_EnableSPI(SPI_SPI1);
    
    __builtin_enable_interrupts();
  
  // post the initial transition event
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
     PostTemplateService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostSPIService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunSPIService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  if (ThisEvent.EventType == ES_CMD) {
      DB_printf("%d \n", ThisEvent.EventParam);
  }
  return ReturnEvent;
}

void __ISR(_SPI_1_VECTOR,IPL7SOFT) SPI_ISR(void){
  uint8_t CurrentCmd;
  CurrentCmd = (uint8_t)SPI1BUF;
  IFS1CLR = _IFS1_SPI1RXIF_MASK;
  ES_Event_t CMD_Event;
  CMD_Event.EventType = ES_CMD;
  CMD_Event.EventParam = CurrentCmd;
	PostSPIService(CMD_Event);
  if (CurrentCmd > 0x00 && CurrentCmd <= 0x1F) {
    PostDriverSM(CMD_Event);
  } else if (CurrentCmd > 0x1F && CurrentCmd <= 0x2F) {
    PostBeltService(CMD_Event);
  } else {
    PostDriverSM(CMD_Event);
    PostBeltService(CMD_Event);
  }
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
