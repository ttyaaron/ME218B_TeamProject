/****************************************************************************
 Header file for EncoderService
 
 Description
     This service handles input capture from an encoder to measure RPM.
     Uses interrupt-driven input capture to record encoder edge timing
     and calculates rotational speed.
     
 Notes
     The encoder input is captured on rising edges and the time between
     edges is used to drive an LED bar display and calculate RPM.

 ****************************************************************************/

#ifndef EncoderService_H
#define EncoderService_H

#include "ES_Types.h"
#include <xc.h>

// Public Function Prototypes

bool InitEncoderService(uint8_t Priority);
bool PostEncoderService(ES_Event_t ThisEvent);
ES_Event_t RunEncoderService(ES_Event_t ThisEvent);
uint32_t Encoder_GetLatestPeriod(void);


#endif /* EncoderService_H */
