/****************************************************************************
 Module
     CommandRetrieveService.h

 Revision
     0.1

 Description
     Header file for the SPI Command Retrieve Service.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 command retrieval
*****************************************************************************/

#ifndef CommandRetrieveService_H
#define CommandRetrieveService_H

#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */


bool InitCommandRetrieveService(uint8_t Priority);
bool PostCommandRetrieveService(ES_Event_t ThisEvent);
ES_Event_t RunCommandRetrieveService(ES_Event_t ThisEvent);

#endif /* CommandRetrieveService_H */
