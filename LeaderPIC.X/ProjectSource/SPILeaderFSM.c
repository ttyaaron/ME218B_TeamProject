/****************************************************************************
 Module
   SPILeaderFSM.c

 Revision
   0.1

 Description
   SPI Leader FSM that sends commands to the Follower PIC via SPI communication.

 Notes
   This Leader sends commands to the Follower via SPI:
   - Commands come from other services/SMs in the Leader code
   - Leader periodically polls Follower for status updates
   - When Follower has new status (0xFF flag), Leader reads the status byte
   
   Query behavior:
     - Leader sends command bytes to Follower
     - When Follower has new status ready, the next query returns 0xFF
     - The query following that 0xFF returns the status byte
     - Subsequent queries return the same status until a new one arrives

 History
 When           Who     What/Why
 -------------- ---     --------
 02/26/26       Tianyu  Renamed from CommandRetrieveService to SPILeaderFSM
 02/03/26       Tianyu  Initial creation for Lab 8 command retrieval
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "SPILeaderFSM.h"
#include "CommonDefinitions.h"
#include "PIC32_SPI_HAL.h"
#include "AtomBehaviorFSM.h"
#include "dbprintf.h"
#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
#define SPI_POLL_INTERVAL_MS 500
SPI_Module_t Module = SPI_SPI1;

/*---------------------------- Module Functions ---------------------------*/
static uint8_t QueryFollower(uint8_t commandToSend);
static bool IsValidCommandByte(uint8_t commandByte);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static SPILeaderState_t CurrentState;
static bool SawNewStatusFlag;
static uint8_t CurrentCommand;
static uint8_t LastStatus;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitSPILeaderFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the SPI Leader FSM.

 Author
     Tianyu, 02/26/26
****************************************************************************/
bool InitSPILeaderFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  CurrentState = InitSPILeaderState;
  SawNewStatusFlag = false;
  CurrentCommand = CMD_STOP;  // Default command
  LastStatus = 0x00;

  /********************************************
   SPI Leader Initialization
   *******************************************/
  DB_printf("SPI Leader Init\\n");
  
  SPI_SamplePhase_t SamplePhase = SPI_SMP_MID;
  uint32_t DesiredClock_ns = 10000;
  SPI_Clock_t ClockIdle = SPI_CLK_HI;
  SPI_ActiveEdge_t ChosenEdge = SPI_FIRST_EDGE;
  SPI_XferWidth_t DataWidth = SPI_8BIT;
  
  SPI_PinMap_t SSPin = SPI_RPA0;   // Chip Select Pin
  SPI_PinMap_t SDOPin = SPI_RPA1;  // Chip Output Pin
  SPI_PinMap_t SDIPin = SPI_RPB8;  // Chip Input Pin

  // Configure pins
  TRISAbits.TRISA0 = 0;
  TRISAbits.TRISA1 = 0;
  
  ANSELAbits.ANSA0 = 0;
  ANSELAbits.ANSA1 = 0;
  
  SPI1CONbits.SRXISEL = 0b01;
  
  // Configure SPI as Leader
  SPISetup_BasicConfig(Module);
  SPISetup_SetLeader(Module, SamplePhase);
  SPISetup_SetBitTime(Module, DesiredClock_ns);
  SPISetup_MapSSOutput(Module, SSPin);
  SPISetup_MapSDOutput(Module, SDOPin);
  
  // Map SDI input
  SDI1R = 0b0100;
  TRISBbits.TRISB8 = 1;

  SPISetup_SetClockIdleState(Module, ClockIdle);
  SPISetup_SetActiveEdge(Module, ChosenEdge);
  SPISetup_SetXferWidth(Module, DataWidth);
  SPISetEnhancedBuffer(Module, true);
  
  SPISetup_EnableSPI(Module);

  // Start periodic poll timer
  ES_Timer_InitTimer(COMMAND_SPI_TIMER, SPI_POLL_INTERVAL_MS);
  
  __builtin_enable_interrupts();
  
  DB_printf("SPI Leader configured\\n");

  ThisEvent.EventType = ES_ENTRY;
  // Start the SPI Leader State machine
  StartSPILeaderFSM(ThisEvent);

  return true;
}

/****************************************************************************
 Function
     PostSPILeaderFSM

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     bool, false if the enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Tianyu, 02/26/26
****************************************************************************/
bool PostSPILeaderFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSPILeaderFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   State machine for SPI Leader communication with Follower.

 Author
     Tianyu, 02/26/26
****************************************************************************/
ES_Event_t RunSPILeaderFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  switch (CurrentState)
  {
    case WaitingToSend:
    {
      // Handle new command to send to Follower
      if (ThisEvent.EventType == ES_NEW_COMMAND)
      {
        uint8_t newCommand = ThisEvent.EventParam;
        if (IsValidCommandByte(newCommand))
        {
          CurrentCommand = newCommand;
          DB_printf("New command to send: 0x%x\\n", CurrentCommand);
        }
        else
        {
          DB_printf("Invalid command: 0x%x\\n", newCommand);
        }
      }
      
      // Periodic polling
      else if (ThisEvent.EventType == ES_TIMEOUT && 
               ThisEvent.EventParam == COMMAND_SPI_TIMER)
      {
        // Query Follower - send current command and read status
        uint8_t commandToSend = CurrentCommand;
        uint8_t statusByte = QueryFollower(commandToSend);
        
        // After sending a non-idle command, switch to CMD_STOP to avoid repeated triggers
        if (commandToSend != CMD_STOP)
        {
          CurrentCommand = CMD_STOP;
        }
        
        if (statusByte == 0xFF)
        {
          // Follower has new status ready
          SawNewStatusFlag = true;
          DB_printf("Follower has new status\\n");
        }
        else if (SawNewStatusFlag == true)
        {
          // This is the actual status byte
          if (statusByte != LastStatus)
          {
            DB_printf("Follower status: 0x%x\\n", statusByte);
            LastStatus = statusByte;
            
            // Could post event to other services if needed
            // ES_Event_t StatusEvent;
            // StatusEvent.EventType = ES_FOLLOWER_STATUS;
            // StatusEvent.EventParam = statusByte;
            // PostAtomBehaviorFSM(StatusEvent);
          }
          SawNewStatusFlag = false;
        }
        // else: repeated old status, ignore
        
        // Restart timer
        ES_Timer_InitTimer(COMMAND_SPI_TIMER, SPI_POLL_INTERVAL_MS);
      }
    }
    break;

    default:
      break;
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
     StartSPILeaderFSM

 Parameters
     ES_Event_t CurrentEvent

 Returns
     None

 Description
     Starts the SPI Leader FSM, initializes to WaitingToSend state.

 Author
     Tianyu, 02/28/26
****************************************************************************/
void StartSPILeaderFSM(ES_Event_t CurrentEvent)
{
  // Set the initial state to WaitingToSend
  CurrentState = WaitingToSend;
  DB_printf("SPILeader: Ready\\n");
  
  // Call Run to initialize the state machine
  RunSPILeaderFSM(CurrentEvent);
}

/****************************************************************************
 Function
     QuerySPILeaderFSM

 Parameters
     None

 Returns
     SPILeaderState_t The current state of the SPI Leader state machine

 Description
     Returns the current state of the SPI Leader state machine

 Author
     Tianyu, 02/26/26
****************************************************************************/
SPILeaderState_t QuerySPILeaderFSM(void)
{
  return CurrentState;
}

/*----------------------------- Module Helpers ----------------------------*/
/****************************************************************************
 Function
     QueryFollower

 Parameters
     uint8_t commandToSend - command byte to send to Follower

 Returns
     uint8_t - status byte read from Follower

 Description
     Sends a command byte to the Follower and reads the response.

 Author
     Tianyu, 02/26/26
****************************************************************************/
static uint8_t QueryFollower(uint8_t commandToSend)
{
  // Send command byte and read response
  if (!SPI1STATbits.SPITBF)
  {
    SPIOperate_SPI1_Send8Wait(commandToSend);
  }
  uint8_t data = (uint8_t)SPIOperate_ReadData(Module);
  return data;
}

/****************************************************************************
 Function
     IsValidCommandByte

 Parameters
     uint8_t commandByte

 Returns
     bool, true if the command byte is valid

 Description
     Validates a command byte against the lookup table.

 Author
     Tianyu, 02/03/26
****************************************************************************/
static bool IsValidCommandByte(uint8_t commandByte)
{
  bool returnVal = false;
  uint8_t index = 0;
  
  for (index = 0; index < sizeof(validCommandBytes)/sizeof(validCommandBytes[0]); index++)
  {
    if (commandByte == validCommandBytes[index])
    {
      returnVal = true;
      break;
    }
  }
  return returnVal;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
