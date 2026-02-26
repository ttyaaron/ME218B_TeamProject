/****************************************************************************
 Header file for SpeedControlService
 
 Description
     This service implements a PI (Proportional-Integral) controller for
     DC motor speed control. Runs on a dedicated timer interrupt at 2ms
     intervals to maintain tight control loop timing.
     
 Notes
     The controller reads desired speed from the ADC service, measures
     actual speed from the encoder service, and outputs duty cycle commands
     to the DC motor service.

 ****************************************************************************/

#ifndef SpeedControlService_H
#define SpeedControlService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitSpeedControlService(uint8_t Priority);
bool PostSpeedControlService(ES_Event_t ThisEvent);
ES_Event_t RunSpeedControlService(ES_Event_t ThisEvent);

#endif /* SpeedControlService_H */
