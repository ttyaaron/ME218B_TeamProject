/****************************************************************************
 Module
   Ports.c

 Revision
   0.1

 Description
   Centralized port initialization and read helpers.

 Notes
   Follow the idea of a centralized port module. Actual pin assignments
   should be updated for the target hardware.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 port management
****************************************************************************/

#include "Ports.h"
#include "CommonDefinitions.h"

/****************************************************************************
 Function
     InitBeaconInputPin

 Parameters
     None

 Returns
     None

 Description
     Initializes the beacon input pin as a digital input.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void InitBeaconInputPin(void)
{
  // Configure the beacon input pin (TRIS/ANSEL)
  TRISBbits.TRISB2 = 1;   // Set as input
  ANSELBbits.ANSB2 = 0;   // Set as digital
}

/****************************************************************************
 Function
     ReadBeaconInputPin

 Parameters
     None

 Returns
     bool: current beacon input state

 Description
     Reads the beacon input pin.

 Author
     Tianyu, 02/03/26
****************************************************************************/
bool ReadBeaconInputPin(void)
{
  // Return the beacon input pin state
  if (PORTBbits.RB2 == 1) {
    return true;
  }
  return false;
}

/****************************************************************************
 Function
     InitTapeSensorPin

 Parameters
     None

 Returns
     None

 Description
     Initializes the tape sensor input pin.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void InitTapeSensorPin(void)
{
  // Configure the beacon input pin (TRIS/ANSEL)
  TRISBbits.TRISB3 = 1;   // Set as input
  ANSELBbits.ANSB3 = 0;   // Set as digital
}

/****************************************************************************
 Function
     ReadTapeSensorPin

 Parameters
     None

 Returns
     bool: current tape sensor input state

 Description
     Reads the tape sensor input pin.

 Author
     Tianyu, 02/03/26
****************************************************************************/
bool ReadTapeSensorPin(void)
{
  // Return the tape sensor input pin state
  if (PORTBbits.RB3 == 1) {
    return true;
  }
  return false;
}

/****************************************************************************
 Function
     InitCommandSPIPins

 Parameters
     None

 Returns
     None

 Description
     Initializes the SPI pins used for the CommandGenerator link.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void InitCommandSPIPins(void)
{
  // TODO: Configure SPI pins for leader mode (SCK, MOSI, MISO, CS)
}

/****************************************************************************
 Function
     InitDebugOutputPin

 Parameters
     None

 Returns
     None

 Description
     Initializes the pin used for debugging

 Author
     Tianyu, 02/04/26
****************************************************************************/
void InitDebugOutputPin(void)
{
  // 
  DEBUG_OUTPUT_PIN_TRIS = 0; // Output
  DEBUG_OUTPUT_PIN_LAT = 0;  // Initialize low
  DEBUG_OUTPUT_PIN_ANSEL = 0; // Disable analog functionw
}
