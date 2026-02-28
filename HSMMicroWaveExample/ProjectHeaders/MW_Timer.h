/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef MW_Timer_H
#define MW_Timer_H

// State definitions for use with the query function
typedef enum
{
  CountingState, SetState, ExpiredState
}TimerState;

// Public Function Prototypes

ES_Event_t RunTimerSM(ES_Event_t CurrentEvent);
void StartTimerSM(ES_Event_t CurrentEvent);
TimerState QueryTimerSM(void);
unsigned char QueryOvenTimer(void);
void ClearOvenTimer(void);

#endif /*MW_Timer_H */

