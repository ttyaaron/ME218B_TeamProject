/****************************************************************************
 Module
     DCMotorService.h

 Revision
     0.1

 Description
     Header file for DCMotorService (two-motor wrapper).

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Updated for Lab 8 two-motor control
*****************************************************************************/

#ifndef DCMotorService_H
#define DCMotorService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitDCMotorService(uint8_t Priority);
bool PostDCMotorService(ES_Event_t ThisEvent);
ES_Event_t RunDCMotorService(ES_Event_t ThisEvent);
void MotorCommandWrapper(uint16_t speedLeft, uint16_t speedRight,
                         uint8_t dirLeft, uint8_t dirRight);

#endif /* DCMotorService_H */
