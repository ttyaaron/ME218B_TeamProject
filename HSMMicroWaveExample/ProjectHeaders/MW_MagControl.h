/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef MW_MagControl_H
#define MW_MagControl_H

// State definitions for use with the query function
typedef enum
{
  CookingState, NotCookingState
}MagControlState;

// Public Function Prototypes

ES_Event_t RunMagControlSM(ES_Event_t CurrentEvent);
void StartMagControlSM(ES_Event_t CurrentEvent);
MagControlState QueryMagControlSM(void);

#endif /*MW_MagControl_H */

