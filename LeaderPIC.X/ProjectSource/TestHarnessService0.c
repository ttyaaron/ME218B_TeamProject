/****************************************************************************
 Module
   TestHarnessService0.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/26/17 18:26 jec     moves definition of ALL_BITS to ES_Port.h
 10/19/17 21:28 jec     meaningless change to test updating
 10/19/17 18:42 jec     removed referennces to driverlib and programmed the
                        ports directly
 08/21/17 21:44 jec     modified LED blink routine to only modify bit 3 so that
                        I can test the new new framework debugging lines on PF1-2
 08/16/17 14:13 jec      corrected ONE_SEC constant to match Tiva tick rate
 11/02/13 17:21 jec      added exercise of the event deferral/recall module
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// This module
#include "../ProjectHeaders/TestHarnessService0.h"

// debugging printf()

// Hardware
#include <xc.h>
//#include <proc/p32mx170f256b.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"

// Project modules
#include "MainStrategyHSM.h"
#include "AtomBehaviorFSM.h"
#include "DCMotorService.h"
#include "SPILeaderFSM.h"
#include "CommonDefinitions.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 10.000mS/tick timing
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)

#define ENTER_POST     ((MyPriority<<3)|0)
#define ENTER_RUN      ((MyPriority<<3)|1)
#define ENTER_TIMEOUT  ((MyPriority<<3)|2)


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService0

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
bool InitTestHarnessService0(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  // When doing testing, it is useful to announce just which program
  // is running.
  clrScrn();
  puts("\rStarting Test Harness for \r");
  DB_printf( "the 2nd Generation Events & Services Framework V2.4\r\n");
  DB_printf( "compiled at %s on %s\n", __TIME__, __DATE__);
  DB_printf( "\n\r\n");
  DB_printf( "=== Strategy & Atom Behavior Control ===\r\n");
  DB_printf( "G - Start main game strategy\r\n");
  DB_printf( "P - Force STOP (preempt, abort mode)\r\n");
  DB_printf( "1-9 - Force specific atom behavior\r\n");
  DB_printf( "  1: Rotate CW 90   2: Rotate CW 45\r\n");
  DB_printf( "  3: Rotate CCW 90  4: Rotate CCW 45\r\n");
  DB_printf( "  5: Drive Fwd Half 6: Drive Fwd Full\r\n");
  DB_printf( "  7: Align Beacon   8: Search Tape\r\n");
  DB_printf( "b/g/l - Force beacon detected event\r\n");
  DB_printf( "\r\n=== Servo Control (send via SPI to Follower) ===\r\n");
  DB_printf( "w - Sweep servo    W - Retract sweep\r\n");
  DB_printf( "s - Scoop servo    S - Retract scoop\r\n");
  DB_printf( "r - Release servo  f - Shoot servo\r\n");
  DB_printf( "i - Initialize servos\r\n");
  DB_printf( "\r\n=== Other ===\r\n");
  DB_printf( "h - Display this help\r\n");
  DB_printf( "t - Read tape sensors\r\n");
  DB_printf( "================================================\r\n\n");

  /********************************************
   in here you write your initialization code
   *******************************************/


  // initialize the Short timer system for channel A
  //ES_ShortTimerInit(MyPriority, SHORT_TIMER_UNUSED);

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
     PostTestHarnessService0

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostTestHarnessService0(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService0

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunTestHarnessService0(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

#ifdef _INCLUDE_BYTE_DEBUG_
  _HW_ByteDebug_SetValueWithStrobe( ENTER_RUN );
#endif  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
      ES_Timer_InitTimer(SERVICE0_TIMER, HALF_SEC);
      puts("Service 00:");
      DB_printf("\rES_INIT received in Service %d\r\n", MyPriority);
      DB_printf("Starting tape sensor monitoring (0.5 sec interval)\r\n");
    }
    break;
    case ES_TIMEOUT:   // re-start timer & announce
    {
      // Read all tape sensors
      TapeSensor_Read();
      
      // Get analog sensor values
      uint32_t leftAnalog = TapeSensor_GetLeftAnalog();
      uint32_t rightAnalog = TapeSensor_GetRightAnalog();
      uint32_t centerAnalog = TapeSensor_GetCenterAnalog();
      
      // Get digital sensor states
      bool leftDigital = TapeSensor_GetLeftDigital();
      bool rightDigital = TapeSensor_GetRightDigital();
      
      // Print tape sensor status
      DB_printf("\r\n=== Tape Sensors ===\r\n");
      DB_printf("Analog: L=%d  C=%d  R=%d\r\n", leftAnalog, centerAnalog, rightAnalog);
      DB_printf("Digital: L=%d  R=%d\r\n", 
                leftDigital ? 1 : 0, 
                rightDigital ? 1 : 0);
      
      // Restart timer for next reading
      ES_Timer_InitTimer(SERVICE0_TIMER, HALF_SEC);
    }
    break;
    case ES_SHORT_TIMEOUT:   // lower the line & announce
    {
      puts("\rES_SHORT_TIMEOUT received\r\n");
    }
    break;
    case ES_NEW_KEY:   // announce and handle servo control keys
    {
      DB_printf("ES_NEW_KEY received with -> %c <- in Service 0\r\n",
          (char)ThisEvent.EventParam);

      // Handle key mappings for servo control via SPI
      ES_Event_t CommandEvent;
      char key = (char)ThisEvent.EventParam;
      
      switch(key)
      {
        //=== Strategy Control ===
        case 'G':  // Start main game strategy
        {
          ES_Event_t StrategyEvent;
          StrategyEvent.EventType = ES_STRATEGY_START;
          StrategyEvent.EventParam = STRATEGY_MAIN_GAME;
          PostMainStrategyHSM(StrategyEvent);
          DB_printf(">>> Starting MAIN GAME STRATEGY <<<\r\n");
        }
        break;
        
        case 'P':  // Force stop/preempt (abort mode)
        {
          ES_Event_t PreemptEvent;
          PreemptEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          PreemptEvent.EventParam = ATOM_IDLE;
          PostMainStrategyHSM(PreemptEvent);
          DB_printf(">>> FORCE STOP (Preempt) <<<\r\n");
        }
        break;
        
        //=== Force Atom Behaviors ===
        case '1':  // Rotate CW 90
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_ROTATE_CW_90;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Rotate CW 90 <<<\r\n");
        }
        break;
        
        case '2':  // Rotate CW 45
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_ROTATE_CW_45;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Rotate CW 45 <<<\r\n");
        }
        break;
        
        case '3':  // Rotate CCW 90
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_ROTATE_CCW_90;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Rotate CCW 90 <<<\r\n");
        }
        break;
        
        case '4':  // Rotate CCW 45
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_ROTATE_CCW_45;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Rotate CCW 45 <<<\r\n");
        }
        break;
        
        case '5':  // Drive forward half
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_DRIVE_FWD_HALF;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Drive Fwd Half <<<\r\n");
        }
        break;
        
        case '6':  // Drive forward full
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_DRIVE_FWD_FULL;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Drive Fwd Full <<<\r\n");
        }
        break;
        
        case '7':  // Align with beacon
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_BEACON_ALIGN;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Align Beacon <<<\r\n");
        }
        break;
        
        case '8':  // Search for tape
        {
          ES_Event_t AtomEvent;
          AtomEvent.EventType = ES_FORCE_ATOM_BEHAVIOR;
          AtomEvent.EventParam = ATOM_TAPE_SEARCH;
          PostMainStrategyHSM(AtomEvent);
          DB_printf(">>> Force Atom: Search Tape <<<\r\n");
        }
        break;
        
        case 't':  // Read tape sensors manually
        {
          TapeSensor_Read();
          uint32_t leftAnalog = TapeSensor_GetLeftAnalog();
          uint32_t rightAnalog = TapeSensor_GetRightAnalog();
          uint32_t centerAnalog = TapeSensor_GetCenterAnalog();
          bool leftDigital = TapeSensor_GetLeftDigital();
          bool rightDigital = TapeSensor_GetRightDigital();
          
          DB_printf("\r\n=== Tape Sensors (Manual Read) ===\r\n");
          DB_printf("Analog: L=%d  C=%d  R=%d\r\n", leftAnalog, centerAnalog, rightAnalog);
          DB_printf("Digital: L=%d  R=%d\r\n", 
                    leftDigital ? 1 : 0, 
                    rightDigital ? 1 : 0);
        }
        break;
        
        //=== Servo control commands - send to Follower via SPI ===
        case 'w':  // Sweep action
          CommandEvent.EventType = ES_NEW_COMMAND;
          CommandEvent.EventParam = CMD_SWEEP;
          PostSPILeaderFSM(CommandEvent);
          DB_printf("Posted CMD_SWEEP to SPILeaderFSM\r\n");
          break;
          
        case 'W':  // Sweep retract
          CommandEvent.EventType = ES_NEW_COMMAND;
          CommandEvent.EventParam = CMD_RETRACT_SWEEP;
          PostSPILeaderFSM(CommandEvent);
          DB_printf("Posted CMD_RETRACT_SWEEP to SPILeaderFSM\r\n");
          break;
          
        case 's':  // Scoop action
          CommandEvent.EventType = ES_NEW_COMMAND;
          CommandEvent.EventParam = CMD_SCOOP;
          PostSPILeaderFSM(CommandEvent);
          DB_printf("Posted CMD_SCOOP to SPILeaderFSM\r\n");
          break;
          
        case 'S':  // Scoop retract
          CommandEvent.EventType = ES_NEW_COMMAND;
          CommandEvent.EventParam = CMD_RETRACT_SCOOP;
          PostSPILeaderFSM(CommandEvent);
          DB_printf("Posted CMD_RETRACT_SCOOP to SPILeaderFSM\r\n");
          break;
          
        case 'r':  // Release action
          CommandEvent.EventType = ES_NEW_COMMAND;
          CommandEvent.EventParam = CMD_RELEASE;
          PostSPILeaderFSM(CommandEvent);
          DB_printf("Posted CMD_RELEASE to SPILeaderFSM\r\n");
          break;
          
        case 'f':  // Shoot action (f for fire)
          CommandEvent.EventType = ES_NEW_COMMAND;
          CommandEvent.EventParam = CMD_SHOOT;
          PostSPILeaderFSM(CommandEvent);
          DB_printf("Posted CMD_SHOOT to SPILeaderFSM\r\n");
          break;
          
        case 'i':  // Initialize all servos
          CommandEvent.EventType = ES_NEW_COMMAND;
          CommandEvent.EventParam = CMD_INIT_SERVOS;
          PostSPILeaderFSM(CommandEvent);
          DB_printf("Posted CMD_INIT_SERVOS to SPILeaderFSM\r\n");
          break;
          
        case 'h':  // Help - display key mappings
          DB_printf("\r\n=== Strategy & Atom Behavior Control ===\r\n");
          DB_printf("G - Start main game strategy\r\n");
          DB_printf("P - Force STOP (preempt, abort mode)\r\n");
          DB_printf("1-9 - Force specific atom behavior\r\n");
          DB_printf("  1: Rotate CW 90   2: Rotate CW 45\r\n");
          DB_printf("  3: Rotate CCW 90  4: Rotate CCW 45\r\n");
          DB_printf("  5: Drive Fwd Half 6: Drive Fwd Full\r\n");
          DB_printf("  7: Align Beacon   8: Search Tape\r\n");
          DB_printf("b/g/l - Force beacon detected event\r\n");
          DB_printf("\r\n=== Servo Control (send via SPI to Follower) ===\r\n");
          DB_printf("w - Sweep servo    W - Retract sweep\r\n");
          DB_printf("s - Scoop servo    S - Retract scoop\r\n");
          DB_printf("r - Release servo  f - Shoot servo\r\n");
          DB_printf("i - Initialize servos\r\n");
          DB_printf("\r\n=== Other ===\r\n");
          DB_printf("h - Display this help\r\n");
          DB_printf("t - Read tape sensors\r\n");
          DB_printf("================================================\r\n\n");
          break;
      }

      // If the key is 'b''g''r''l', post an ES_BEACON_DETECTED event with the key as a parameter
      switch (ThisEvent.EventParam)
      {
          case 'b':
          case 'g':
          case 'l':
          {
              ES_Event_t BeaconEvent;
              DB_printf("Posting ES_BEACON_DETECTED with param -> %c <- to AtomBehaviorFSM\r\n",
                  (char)ThisEvent.EventParam);
              BeaconEvent.EventType = ES_BEACON_DETECTED;
              BeaconEvent.EventParam = ThisEvent.EventParam;
              PostAtomBehaviorFSM(BeaconEvent);
          }
          break;
          default:
            break;
      }
        
    }
    break;
    case ES_BEACON_DETECTED:
    {
      DB_printf("ES_BEACON_DETECTED received with -> %c <- in Service 0\r\n",
          (char)ThisEvent.EventParam);
    }
    break;
    default:
    {}
     break;
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

