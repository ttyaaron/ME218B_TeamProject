/****************************************************************************
 Header file for ADService
 
 Description
     This service handles reading the A/D converter from the potentiometer
     and updating the interval between stepper motor steps based on the
     analog reading.
     
 Notes
     The A/D reading is scaled to provide motor speed control from zero
     up to slightly faster than the maximum safe speed.

 ****************************************************************************/

#ifndef ADService_H
#define ADService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitADService(uint8_t Priority);
bool PostADService(ES_Event_t ThisEvent);
ES_Event_t RunADService(ES_Event_t ThisEvent);

// Access function to query the scaled step interval
uint16_t GetStepInterval(void);

#endif /* ADService_H */
