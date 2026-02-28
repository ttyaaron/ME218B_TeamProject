/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef MW_MapKeys_H
#define MW_MapKeys_H

// Public Function Prototypes

bool InitMapKeys(uint8_t Priority);
bool PostMapKeys(ES_Event_t ThisEvent);
ES_Event_t RunMapKeys(ES_Event_t ThisEvent);
bool ChkDoorClosed(void);

#endif /*MW_MapKeys_H */

