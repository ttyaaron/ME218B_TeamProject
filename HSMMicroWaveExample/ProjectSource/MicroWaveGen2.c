/****************************************************************************
 Module
   MicrowaveGen2.c

 Revision
   1.0.1

 Description
   This is the main() for the microwave oven.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/06/12 21:49 jec      converted to the Gen2 Events & Services Framework
 02/21/07 17:04 jec      converted to pass Event_t to Start...()
 02/20/07 21:37 jec      converted to use enumerated type for events
 02/21/05 15:38 jec      Began coding
****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "PIC32_AD_Lib.h"



#define clrScrn() puts("\x1b[2J")
#define goHome() puts("\x1b[H")
#define clrLine() printf("\x1b[K")

int main(void)
{
  ES_Return_t ErrorType;

 _HW_PIC32Init();

  clrScrn();
  goHome();
  puts( "Starting HierMicroWaveOven Gen2 PIC32\r");
  printf( "Compiled at %s on %s\n\r\n", __TIME__, __DATE__);
  puts( "Keys simulate events:\r");
  puts( "'O'= EV_DOOR_OPEN\r");
  puts( "'D'= Door CLOSED\r");
  puts( "'C'= EV_CLEAR\r");
  puts( "'S'= EV_START\r");
  puts( "'T'= EV_SET\r");
  puts( "'L'= EV_DEF_LIGHT\r");
  puts( "'P'= EV_POPCORN\r\n");

  puts( " Mag Control  Power Timer\r");

// Your hardware initialization function calls go here
// do local initialization for A/D port, where we get cook time value
// AD lib stuff commented out for Demo  
//  ADC_ConfigAutoScan( BIT4HI, 1);

// do not enable interrupts in ES_TimerInit to make it possible to single step
// for demo/debug purposes
// this ruins the timing functionality, so put it back to run for real

// now initialize the Events and Services Framework and start it running
  ErrorType = ES_Initialize(ES_Timer_RATE_5mS);
  if (Success == ErrorType)
  {
    ErrorType = ES_Run();
  }
//if we got to here, there was an error
  switch (ErrorType)
  {
    case FailedPointer:
    {
      puts("Failed on NULL pointer");
    }
    break;
    case FailedInit:
    {
      puts("Failed Initialization");
    }
    break;
    default:
    {
      puts("Other Failure");
    }
    break;
  }
  for ( ; ;)   // hang after reporting error
  {
    ;
  }
}

