/****************************************************************************
 Module
     Ports.h

 Revision
     0.1

 Description
     Centralized port initialization and read helpers.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 port management
*****************************************************************************/

#ifndef Ports_H
#define Ports_H

#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

void InitBeaconInputPin(void);
bool ReadBeaconInputPin(void);

void InitTapeSensorPin(void);
bool ReadTapeSensorPin(void);

void InitCommandSPIPins(void);

void InitDebugOutputPin(void);

#endif /* Ports_H */
