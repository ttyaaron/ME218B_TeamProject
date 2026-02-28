/****************************************************************************
 Module
   MW_Display.c

 Revision
   1.0.1

 Description
   This is the dispaly Module for the microwave oven.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/07/12 00:36 jec      converted to Events & Services Framework Gen2
 02/21/05 16:20 jec      Began coding
****************************************************************************/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MW_MagControl.h"
#include "MW_PowControl.h"
#include "MW_Timer.h"
#include "MW_MapKeys.h"
#include <stdio.h>

void UpdateDisplay(void)
{
  MagControlState MagControlState = QueryMagControlSM();
  PowerState      PowControlState = QueryPowControlSM();
  TimerState      TimState = QueryTimerSM();

  // now update the display
      printf("\r");
  switch (MagControlState)
  {
    case CookingState:
    {
      printf("     Cooking ");
    }
    break;
    case NotCookingState:
    {
      printf(" Not Cooking ");
    }
    break;
  }

  switch (PowControlState)
  {
    case FullState:
    {
      printf(" Full ");
    }
    break;
    case HalfState:
    {
      printf(" Half ");
    }
    break;
  }

  switch (TimState)
  {
    case SetState:
    {
      printf(" Set      ");
    }
    break;
    case ExpiredState:
    {
      printf(" Expired  ");
    }
    break;
    case CountingState:
    {
      printf(" Counting ");
    }
    break;
  }

  if (ChkDoorClosed() == true)
  {
      printf(" Door Closed ");
  }
  else
  {
      printf(" Door Open   ");
  }
}

