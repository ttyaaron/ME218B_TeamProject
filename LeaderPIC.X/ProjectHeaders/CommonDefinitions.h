/****************************************************************************
 Header file for Common Definitions
 
 Description
     This file contains common definitions, constants, and utility functions
     shared across multiple services in the motor control system.
     
 Notes
     All services that need motor specifications, encoder parameters, or
     conversion functions should include this header to ensure consistency.

 History
 When           Who     What/Why
 -------------- ---     --------
 01/28/26       Tianyu  Initial creation for Lab 7
****************************************************************************/

#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

#include <stdint.h>

/*----------------------------- Module Defines ----------------------------*/

// Timer Prescale Configuration
// Enum type for prescale settings
typedef enum {
  PRESCALE_1 = 0,
  PRESCALE_2,
  PRESCALE_4,
  PRESCALE_8,
  PRESCALE_16,
  PRESCALE_32,
  PRESCALE_64,
  PRESCALE_256
} Prescale_t;

typedef enum
{
    CMD_STOP = 0x00,
    CMD_INIT_SERVOS = 0x01,       // Initialize all servos
    CMD_ROTATE_CW_90 = 0x02,
    CMD_ROTATE_CW_45 = 0x03,
    CMD_ROTATE_CCW_90 = 0x04,
    CMD_ROTATE_CCW_45 = 0x05,
    CMD_DRIVE_FWD_HALF = 0x08,
    CMD_DRIVE_FWD_FULL = 0x09,
    CMD_DRIVE_REV_HALF = 0x10,
    CMD_DRIVE_REV_FULL = 0x11,
    CMD_ALIGN_BEACON = 0x20,
    CMD_SEARCH_TAPE = 0x40,
    CMD_SWEEP = 0x50,             // Sweep servo action
    CMD_SCOOP = 0x51,             // Scoop servo action
    CMD_RELEASE = 0x52,           // Release servo action
    CMD_SHOOT = 0x53,             // Shoot servo action
    CMD_RETRACT_SWEEP = 0x54,     // Retract sweep servo
    CMD_RETRACT_SCOOP = 0x55,     // Retract scoop servo
    CMD_SIDE_BLUE = 0x60,         // Set side servo to blue field position
    CMD_SIDE_GREEN = 0x61,        // Set side servo to green field position
    CMD_SIDE_MIDDLE = 0x62        // Set side servo to middle/neutral position
} Command_t;

// Prescale lookup table (maps enum to hardware register bits)
extern const uint8_t PrescaleLookup[];

extern const uint8_t validCommandBytes[21];

// ----------------------------------------------------------------
// Layer 1: Raw measured hardware facts (update these if hardware changes)
// Verified on hardware 03/01/26
// ----------------------------------------------------------------

// Timer3 runs at PBCLK / ENCODER_TIMER_PRESCALE = 20000000 / 256 = 78125 Hz
#define PBCLK_FREQ              20000000u   // 20 MHz peripheral bus clock
#define ENCODER_TIMER_PRESCALE  256u        // Timer3 prescale
#define TIMER3_CLOCK_HZ         (PBCLK_FREQ / ENCODER_TIMER_PRESCALE)  // 78125 Hz

// IC fires on every 16th encoder edge
#define IC_PRESCALE             16u

// Measured: ~150 IC capture events per full wheel revolution
// (hand count ~146/rev, motor test 195 Hz / 1.295 rev/s = 150.6/rev)
// Raw encoder CPR = IC_EVENTS_PER_REV * IC_PRESCALE = 150 * 16 = 2400
#define IC_EVENTS_PER_REV       150u
#define ENCODER_CPR             (IC_EVENTS_PER_REV * IC_PRESCALE)  // 2400

// Wheel geometry: diameter measured with calipers
#define WHEEL_DIAMETER_MM       100u
#define WHEEL_CIRCUMFERENCE_MM  314u        // diameter * pi, rounded to nearest mm

// ----------------------------------------------------------------
// Layer 2: Derived conversion constants (do not edit — change Layer 1)
//
// Speed:
//   rev/s  = TIMER3_CLOCK_HZ / (IC_EVENTS_PER_REV * period_ticks)
//   mm/s   = rev/s * WHEEL_CIRCUMFERENCE_MM
//          = (TIMER3_CLOCK_HZ * WHEEL_CIRCUMFERENCE_MM * IC_PRESCALE)
//            / (IC_EVENTS_PER_REV * period_ticks)
//
//   speed_mm_s = SPEED_CONV_NUM / (SPEED_CONV_DEN * period_ticks)
// ----------------------------------------------------------------
#define SPEED_CONV_NUM  (TIMER3_CLOCK_HZ * WHEEL_CIRCUMFERENCE_MM * IC_PRESCALE)  // 24531250
#define SPEED_CONV_DEN  (IC_EVENTS_PER_REV)                         // 150

// Distance:
//   Each IC event advances the wheel by WHEEL_CIRCUMFERENCE_MM / IC_EVENTS_PER_REV mm
//   distance_mm = (ic_event_count * DIST_CONV_NUM * IC_PRESCALE) / DIST_CONV_DEN
#define DIST_CONV_NUM   WHEEL_CIRCUMFERENCE_MM * IC_PRESCALE   // 314
#define DIST_CONV_DEN   IC_EVENTS_PER_REV        // 150

// ----------------------------------------------------------------
// Layer 3: Named speed levels in mm/s for strategy layer use
// Initial values estimated; tune by running motor and measuring.
// ----------------------------------------------------------------
#define SPEED_STOP_MM_S         0u
#define SPEED_QUARTER_MM_S      80u
#define SPEED_HALF_MM_S         160u
#define SPEED_FULL_MM_S         300u

// PWM configuration (shared between DCMotorService and SpeedControlService)
#define DUTY_MAX_TICKS 2000        // Maximum duty cycle ticks (100%)
#define PWM_PERIOD_TICKS 1999        // PWM period in timer ticks (for 20kHz at 20MHz PBCLK with 1:1 prescale)
#define DUTY_MIN_TICKS 0            // Minimum duty cycle ticks (0%)

// Motor Indexes
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

// Directions
#define FORWARD 0
#define REVERSE 1

// Speed Levels (duty cycle ticks)
//#define QUARTER_SPEED 750 // 25% duty cycle
//#define HALF_SPEED 1500   // 50% duty cycle
//#define FULL_SPEED 2000   // 100% duty cycle
 
#define QUARTER_SPEED 500 // 25% duty cycle
#define HALF_SPEED 1000   // 50% duty cycle
#define FULL_SPEED 1500   // 100% duty cycle

// Timer Durations (ms)
#define SIMPLE_MOVE_90_MS 1500
#define SIMPLE_MOVE_45_MS 750
#define BEACON_ALIGN_MS 5000
#define TAPE_SEARCH_MS 10000
#define DRIVE_TO_BEACON_MS 3000

// Debug Pin
#define DEBUG_OUTPUT_PIN_LAT LATBbits.LATB15
#define DEBUG_OUTPUT_PIN_TRIS TRISBbits.TRISB15
#define DEBUG_OUTPUT_PIN_ANSEL ANSELBbits.ANSB15

/*---------------------------- Module Variables ---------------------------*/
// Shared Timer3 rollover counter used by BeaconDetectFSM (IC1),
// DCMotorService IC3 (left encoder) and IC2 (right encoder).
extern volatile uint16_t SharedTimer3RolloverCounter;

/*---------------------------- Public Functions ---------------------------*/

/****************************************************************************
 Function
     PeriodToRPM

 Parameters
     uint32_t period - time between encoder edges in timer ticks

 Returns
     float - measured RPM

 Description
     Converts encoder period measurement to RPM. Shared by EncoderService
     and SpeedControlService.
****************************************************************************/
//float PeriodToRPM(uint32_t period);

/****************************************************************************
 Function
     ADToRPM

 Parameters
     uint16_t adcValue - ADC reading (0-1023)

 Returns
     float - desired RPM

 Description
     Converts ADC value to desired RPM setpoint. Shared by ADService
     and SpeedControlService.
****************************************************************************/
float ADToRPM(uint16_t adcValue);

#endif /* COMMON_DEFINITIONS_H */
