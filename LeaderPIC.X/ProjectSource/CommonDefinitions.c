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

// Shared Timer3 rollover counter used by BeaconDetectFSM (IC1),
// DCMotorService IC3 (left encoder) and IC2 (right encoder).
volatile uint16_t SharedTimer3RolloverCounter = 0;

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
const uint8_t validCommandBytes[21] = {
  0x00, // Stop
  0x01, // Initialize servos
  0x02, // Rotate Clockwise 90 degrees
  0x03, // Rotate Clockwise 45 degrees
  0x04, // Rotate Counter-clockwise 90 degrees
  0x05, // Rotate Counter-clockwise 45 degrees
  0x08, // Drive forward half speed
  0x09, // Drive forward full speed
  0x10, // Drive in reverse half speed
  0x11, // Drive in reverse full speed
  0x20, // Align with beacon
  0x40, // Drive forward until tape detected
  0x50, // Sweep servo action
  0x51, // Scoop servo action
  0x52, // Release servo action
  0x53, // Shoot servo action
  0x54, // Retract sweep servo
  0x55,  // Retract release servo
  0x60, // Side servo to blue field position
  0x61, // Side servo to green field position
  0x62  // Side servo to middle position
};

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     PeriodToSpeed_mm_s

 Parameters
     uint32_t period_ticks - Timer3 ticks between consecutive IC captures
                             for one wheel (from DCMotor_GetEncoderPeriod)

 Returns
     uint32_t - wheel surface speed in mm/s, 0 if period_ticks is 0

 Description
     Converts raw IC period ticks to mm/s using integer arithmetic only.
     speed_mm_s = SPEED_CONV_NUM / (SPEED_CONV_DEN * period_ticks)

     At 300 mm/s: period ~ 78125*314 / (150*300) = 544 ticks
     At  50 mm/s: period ~ 78125*314 / (150*50)  = 3271 ticks

 Author
     Tianyu, 03/01/26
****************************************************************************/
uint32_t PeriodToSpeed_mm_s(uint32_t period_ticks)
{
  if (period_ticks == 0u)
  {
    return 0u;
  }
  return (uint32_t)(SPEED_CONV_NUM / ((uint32_t)SPEED_CONV_DEN * period_ticks));
}

/****************************************************************************
 Function
     ICCountToDistance_mm

 Parameters
     uint32_t ic_event_count - cumulative IC capture event count for one wheel

 Returns
     uint32_t - distance traveled in mm using integer arithmetic only

 Description
     Converts a count of IC events to mm traveled.
     distance_mm = (count * WHEEL_CIRCUMFERENCE_MM) / IC_EVENTS_PER_REV
                 = (count * DIST_CONV_NUM) / DIST_CONV_DEN

     Resolution: 314/150 = 2.09 mm per IC event.

 Author
     Tianyu, 03/01/26
****************************************************************************/
uint32_t ICCountToDistance_mm(uint32_t ic_event_count)
{
  return (ic_event_count * (uint32_t)DIST_CONV_NUM) / (uint32_t)DIST_CONV_DEN;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
