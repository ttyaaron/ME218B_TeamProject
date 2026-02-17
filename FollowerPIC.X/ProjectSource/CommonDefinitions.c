/****************************************************************************
 Module
   CommonDefinitions.c

 Revision
   1.0.0

 Description
   This module contains common utility functions shared across multiple
   services in the motor control system.

 Notes
   These functions provide consistent conversion between ADC values,
   encoder periods, and RPM measurements across all services.

   LookUpTable for Commands
   Command Byte Description of action
   0x00 Stop, hold Position, do not move or rotate
   0x02 Rotate Clockwise by 90 degrees (allows 6 sec. to complete)
   0x03 Rotate Clockwise by 45 degrees (allows 3 sec. to complete)
   0x04 Rotate Counter-clockwise by 90 degrees (allows 6 sec. to complete)
   0x05 Rotate Counter-clockwise by 45 degrees (allows 3 sec. to complete)
   0x08 Drive forward half speed
   0x09 Drive forward full speed
   0x10 Drive in reverse half speed
   0x11 Drive in reverse full speed
   0x20 Align with beacon (allows 5 sec. to complete)
   0x40 Drive forward until tape detected.

   Directions
   FORWARD = 0
   REVERSE = 1

   Motor Indexes
   LEFT_MOTOR = 0
   RIGHT_MOTOR = 1

   Speed Levels (duty ticks or normalized units)
   HALF_SPEED
   FULL_SPEED

   Timer Durations (ms)
   SIMPLE_MOVE_90_MS = 6000
   SIMPLE_MOVE_45_MS = 3000
   BEACON_ALIGN_MS = 5000
   TAPE_SEARCH_MS = 5000

 History
 When           Who     What/Why
 -------------- ---     --------
 01/28/26       Tianyu  Initial creation for Lab 7
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "CommonDefinitions.h"

/*---------------------------- Module Variables ---------------------------*/

// Lookup table for prescale settings based on desired prescale
const uint8_t PrescaleLookup[] = {
  0b000, // 1:1 prescale
  0b001, // 1:2 prescale
  0b010, // 1:4 prescale
  0b011, // 1:8 prescale
  0b100, // 1:16 prescale
  0b101, // 1:32 prescale
  0b110, // 1:64 prescale
  0b111  // 1:256 prescale
};

// Lookup table for Commands
const uint8_t validCommandBytes[11] = {
  0x00, // Stop
  0x02, // Rotate Clockwise 90 degrees
  0x03, // Rotate Clockwise 45 degrees
  0x04, // Rotate Counter-clockwise 90 degrees
  0x05, // Rotate Counter-clockwise 45 degrees
  0x08, // Drive forward half speed
  0x09, // Drive forward full speed
  0x10, // Drive in reverse half speed
  0x11, // Drive in reverse full speed
  0x20, // Align with beacon
  0x40  // Drive forward until tape detected
};

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     PeriodToRPM

 Parameters
     uint32_t period - time between encoder edges in timer ticks

 Returns
     float - measured RPM

 Description
     Converts encoder period measurement to RPM. Uses the encoder timer
     prescale and edges per revolution to calculate actual motor speed.

 Author
     Tianyu, 01/28/26
****************************************************************************/
float PeriodToRPM(uint32_t period)
{
  // Prevent division by zero
  if (period == 0)
  {
    return 0.0f;
  }
  
  // Calculate timer clock frequency
  uint32_t timerClock = PBCLK_FREQ / ENCODER_TIMER_PRESCALE;
  
//  DB_printf("timerClock: %d\r\n", timerClock);
  
//  DB_printf("period: %d\r\n", period);
  
  // DEBUG: Print all values used in calculation
  
//  DB_printf("SECONDS_PER_MINUTE: %d\r\n", SECONDS_PER_MINUTE);
//  DB_printf("IC_ENCODER_EDGES_PER_REV: %d\r\n", IC_ENCODER_EDGES_PER_REV);
  
  // Calculate RPM from period
  // RPM = (timerClock * 60) / (period * edges_per_rev)
  float rpm = ((float)timerClock * SECONDS_PER_MINUTE) / 
              ((float)period * IC_ENCODER_EDGES_PER_REV);
  
//  DB_printf("RPM: %d\r\n", rpm);
  
  return rpm;
}

/****************************************************************************
 Function
     ADToRPM

 Parameters
     uint16_t adcValue - ADC reading (0-1023)

 Returns
     float - desired RPM

 Description
     Converts ADC value to desired RPM setpoint. Maps the full ADC range
     linearly to the motor's RPM range.

 Author
     Tianyu, 01/28/26
****************************************************************************/
float ADToRPM(uint16_t adcValue)
{
  // Map ADC range [0, 1023] to RPM range [0, MAX_RPM]
  return ((float)adcValue * MAX_RPM) / ADC_MAX_VALUE;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
