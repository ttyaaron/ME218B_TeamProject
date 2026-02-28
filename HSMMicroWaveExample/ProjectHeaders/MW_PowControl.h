/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef MW_PowControl_H
#define MW_PowControl_H

// State definitions for use with the query function
typedef enum
{
  FullState, HalfState
}PowerState;

// Public Function Prototypes

ES_Event_t RunPowControlSM(ES_Event_t CurrentEvent);
void StartPowControlSM(ES_Event_t CurrentEvent);
PowerState QueryPowControlSM(void);

#endif /*MW_PowControl_H */

