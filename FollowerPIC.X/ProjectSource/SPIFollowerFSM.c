/****************************************************************************
 Module
   SPIFollowerFSM.c

 Revision
   0.1

 Description
   SPI Follower FSM that receives keyboard commands and transmits them
   to the Leader PIC via SPI communication.

 Notes
   This follower responds to SPI queries from the Leader:
   - Maintains a command byte to send
   - First query after new command returns 0xFF (flag)
   - Second query returns the actual command byte
   - Subsequent queries return the same command until updated

 History
 When           Who     What/Why
 -------------- ---     --------
 02/17/26       Tianyu  Initial creation for Leader-Follower architecture
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "SPIFollowerFSM.h"
#include "CommonDefinitions.h"
#include "PIC32_SPI_HAL.h"
#include <sys/attribs.h>
#include "dbprintf.h"
#include <xc.h>

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
void __ISR(_SPI_1_VECTOR, IPL7SOFT) SPI_ISR(void);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static SPIFollowerState_t CurrentState;
static uint8_t CurrentCommand = 0x00;  // Current command to send
static bool NewCommandFlag = false;    // Flag indicating new command pending

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitSPIFollowerFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the SPI Follower FSM and configures SPI as follower.

 Author
     Tianyu, 02/17/26
****************************************************************************/
bool InitSPIFollowerFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  CurrentState = InitSPIFollowerState;

  /********************************************
   SPI Follower Initialization
   *******************************************/
  DB_printf("SPI Follower Init\n");

  // Configure SPI1 as Follower (based on SPIService.c)
  
  // Set RPA0 as output for debugging/status if needed
  TRISAbits.TRISA0 = 0;
  
  // Basic SPI configuration
  SPISetup_BasicConfig(SPI_SPI1);
  
  // Set as follower (clear MSTEN bit)
  SPI1CONbits.MSTEN = 0;
  
  // Configure Clock input (SCK1) on RB14
  ANSELBbits.ANSB14 = 0; // Disable analog
  TRISBbits.TRISB14 = 1;  // Set as input
  
  // Configure SS (Slave Select) input on RB3
  TRISBbits.TRISB3 = 1;   // Set as input
  ANSELBbits.ANSB3 = 0;   // Disable analog
  SS1R = 0b0001;          // Map SS1 to RPB3
  
  // Configure SDI (Serial Data Input) on RB5
  TRISBbits.TRISB5 = 1;   // Set as input
  SDI1R = 0b0001;         // Map SDI1 to RPB5
  
  // Configure SDO (Serial Data Output) on RB8
  TRISBbits.TRISB8 = 0;   // Set as output
  ANSELBbits.ANSB8 = 0;   // Disable analog
  RPB8R = 0b0011;         // Map RPB8 to SDO1
  
  // Clear interrupt flag
  IFS1CLR = _IFS1_SPI1RXIF_MASK;
  
  // Enable SS pin control
  SPI1CONbits.SSEN = 1;
  
  // We need SDO for output
  SPI1CONbits.DISSDO = 0;
  
  // Clock and data settings
  SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_HI);
  SPISetup_SetActiveEdge(SPI_SPI1, SPI_FIRST_EDGE);
  SPISetup_SetXferWidth(SPI_SPI1, SPI_8BIT);
  
  // Configure interrupts
  SPISetup_Interrupts(SPI_SPI1);
  
  // Disable enhanced buffer
  SPISetEnhancedBuffer(SPI_SPI1, false);
  
  // Enable SPI
  SPISetup_EnableSPI(SPI_SPI1);
  
  // Load initial value (0x00 = STOP command)
  SPI1BUF = CurrentCommand;
  
  // Enable global interrupts
  __builtin_enable_interrupts();
  
  DB_printf("SPI Follower configured\n");

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
     PostSPIFollowerFSM

 Parameters
     ES_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Tianyu, 02/17/26
****************************************************************************/
bool PostSPIFollowerFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSPIFollowerFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles keyboard input events and updates command to send via SPI

 Author
     Tianyu, 02/17/26
****************************************************************************/
ES_Event_t RunSPIFollowerFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  switch (CurrentState)
  {
    case InitSPIFollowerState:
    {
      if (ThisEvent.EventType == ES_INIT)
      {
        CurrentState = WaitingForCommand;
        DB_printf("SPIFollower: Ready for keyboard input\n");
      }
    }
    break;

    case WaitingForCommand:
    {
      if (ThisEvent.EventType == ES_NEW_KEY)
      {
        uint8_t newCommand = 0x00; // Default STOP
        char key = (char)ThisEvent.EventParam;
        
        // Map keyboard input to commands (based on CommonDefinitions.h)
        switch (key)
        {
          case 'S':
          case 's':
            newCommand = CMD_STOP;
            DB_printf("Cmd: STOP\n");
            break;
            
          case 'W':
          case 'w':
            newCommand = CMD_DRIVE_FWD_FULL;
            DB_printf("Cmd: FWD FULL\n");
            break;
            
          case 'Q':
          case 'q':
            newCommand = CMD_DRIVE_FWD_HALF;
            DB_printf("Cmd: FWD HALF\n");
            break;
            
          case 'X':
          case 'x':
            newCommand = CMD_DRIVE_REV_FULL;
            DB_printf("Cmd: REV FULL\n");
            break;
            
          case 'Z':
          case 'z':
            newCommand = CMD_DRIVE_REV_HALF;
            DB_printf("Cmd: REV HALF\n");
            break;
            
          case 'D':
          case 'd':
            newCommand = CMD_ROTATE_CW_90;
            DB_printf("Cmd: CW 90\n");
            break;
            
          case 'E':
          case 'e':
            newCommand = CMD_ROTATE_CW_45;
            DB_printf("Cmd: CW 45\n");
            break;
            
          case 'A':
          case 'a':
            newCommand = CMD_ROTATE_CCW_90;
            DB_printf("Cmd: CCW 90\n");
            break;
            
          case 'R':
          case 'r':
            newCommand = CMD_ROTATE_CCW_45;
            DB_printf("Cmd: CCW 45\n");
            break;
            
          case 'B':
          case 'b':
            newCommand = CMD_ALIGN_BEACON;
            DB_printf("Cmd: ALIGN BEACON\n");
            break;
            
          case 'T':
          case 't':
            newCommand = CMD_SEARCH_TAPE;
            DB_printf("Cmd: SEARCH TAPE\n");
            break;
            
          default:
            DB_printf("Unknown key: %c\n", key);
            break;
        }
        
        // Update command and set flag for new command
        if (newCommand != CurrentCommand || key == 'S' || key == 's')
        {
          CurrentCommand = newCommand;
          NewCommandFlag = true;
          CurrentState = SendingNewFlag;
          DB_printf("New command ready: 0x%02X\n", CurrentCommand);
        }
      }
    }
    break;

    case SendingNewFlag:
      // Waiting for ISR to send 0xFF flag
      // State will change in ISR
      break;

    case SendingCommand:
      // Waiting for ISR to send actual command
      // State will change in ISR
      break;

    default:
      break;
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
     QuerySPIFollowerFSM

 Parameters
     None

 Returns
     SPIFollowerState_t The current state of the SPI Follower state machine

 Description
     Returns the current state of the SPI Follower state machine

 Author
     Tianyu, 02/17/26
****************************************************************************/
SPIFollowerState_t QuerySPIFollowerFSM(void)
{
  return CurrentState;
}

/****************************************************************************
 Function
     SPI_ISR

 Parameters
     None

 Returns
     None

 Description
     SPI interrupt service routine - responds to queries from Leader PIC
     Implements protocol:
     - If new command pending: send 0xFF flag first, then command
     - Otherwise: send last command

 Author
     Tianyu, 02/17/26
****************************************************************************/
void __ISR(_SPI_1_VECTOR, IPL7SOFT) SPI_ISR(void)
{
  uint8_t receivedData;
  uint8_t dataToSend;
  
  // Read received data (query from leader)
  receivedData = (uint8_t)SPI1BUF;
  
  // Clear interrupt flag
  IFS1CLR = _IFS1_SPI1RXIF_MASK;
  
  // Determine what to send based on current state
  if (CurrentState == SendingNewFlag)
  {
    // Send new command flag
    dataToSend = 0xFF;
    CurrentState = SendingCommand;
  }
  else if (CurrentState == SendingCommand)
  {
    // Send actual command
    dataToSend = CurrentCommand;
    NewCommandFlag = false;
    CurrentState = WaitingForCommand;
  }
  else
  {
    // No new command, send current command
    dataToSend = CurrentCommand;
  }
  
  // Load data for next transmission
  SPI1BUF = dataToSend;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
