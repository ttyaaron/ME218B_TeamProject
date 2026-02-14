/****************************************************************************
 Header file for BeaconDetectService
 
 Description
     This service handles input capture from a phototransistor circuit to
     measure beacon frequency. Uses interrupt-driven input capture to record
     signal edge timing and detect 1427Hz beacon signal.
     
 Notes
     The phototransistor input is captured on rising edges and the time
     between edges is used to calculate frequency. When frequency is close
     to 1427Hz, an ES_BEACON_DETECTED event is posted.

 ****************************************************************************/

#ifndef BeaconDetectService_H
#define BeaconDetectService_H

#include "ES_Types.h"
#include <xc.h>

// Public Function Prototypes

bool InitBeaconDetectService(uint8_t Priority);
bool PostBeaconDetectService(ES_Event_t ThisEvent);
ES_Event_t RunBeaconDetectService(ES_Event_t ThisEvent);


#endif /* BeaconDetectService_H */
