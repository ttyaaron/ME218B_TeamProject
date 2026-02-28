/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef MW_MasterMachine_H
#define MW_MasterMachine_H

// Public Function Prototypes

ES_Event_t RunMasterSM(ES_Event_t CurrentEvent);
void StartMasterSM(ES_Event_t CurrentEvent);
bool PostMasterSM(ES_Event_t ThisEvent);
bool InitMasterSM(uint8_t Priority);

#endif /*MW_MasterMachine_H */

