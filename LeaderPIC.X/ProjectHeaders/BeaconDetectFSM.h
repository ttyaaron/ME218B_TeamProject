/****************************************************************************

  Header file for BeaconDetect Flat State Machine
  based on the Gen2 Events and Services Framework

  Description
      This FSM handles input capture from a phototransistor circuit to
      measure beacon frequency. Uses interrupt-driven input capture to record
      signal edge timing and detect a 1427 Hz beacon signal.

  States
      NoSignal       - No valid signal edges are being received.
                       Reported frequency is 0 Hz.
      SignalDetected - Valid signal edges are being received. Smoothed
                       frequency is computed and ES_BEACON_DETECTED is
                       posted to MainLogicFSM when within tolerance.

  Transitions
      NoSignal       --[ES_NEW_SIGNAL_EDGE]--> SignalDetected
      SignalDetected --[ES_TIMEOUT / SIGNAL_WATCHDOG_TIMER]--> NoSignal

 ****************************************************************************/

#ifndef BeaconDetectFSM_H
#define BeaconDetectFSM_H

#include "ES_Configure.h"
#include "ES_Types.h"
#include <xc.h>

// State definitions for use with the query function
typedef enum
{
  InitPState,     // Initial pseudo-state; transitions to NoSignal on ES_INIT
  NoSignal,       // No signal present; frequency reported as 0 Hz
  SignalDetected, // Signal present; frequency actively computed
  BeaconLocked,   // Beacon of certain frequency is detected
} BeaconState_t;

// Public Function Prototypes
bool InitBeaconDetectFSM(uint8_t Priority);
bool PostBeaconDetectFSM(ES_Event_t ThisEvent);
ES_Event_t RunBeaconDetectFSM(ES_Event_t ThisEvent);
BeaconState_t QueryBeaconDetectFSM(void);
char QueryLockedBeaconId(void);

#endif /* BeaconDetectFSM_H */