/****************************************************************************
 Module
     EncoderTestService.h

 Revision
     1.0.0

 Description
     Header file for EncoderTestService - standalone encoder hardware
     verification service

 Notes
     This service periodically prints raw encoder period values in timer
     ticks to verify IC3 and IC2 hardware operation. For Phase 1 testing only.

 History
 When           Who     What/Why
 -------------- ---     --------
 03/01/26       Tianyu  Initial creation for Phase 1 encoder verification
*****************************************************************************/

#ifndef EncoderTestService_H
#define EncoderTestService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitEncoderTestService(uint8_t Priority);
bool PostEncoderTestService(ES_Event_t ThisEvent);
ES_Event_t RunEncoderTestService(ES_Event_t ThisEvent);

#endif /* EncoderTestService_H */
