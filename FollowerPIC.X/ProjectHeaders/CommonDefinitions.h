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
    CMD_ROTATE_CW_90 = 0x02,
    CMD_ROTATE_CW_45 = 0x03,
    CMD_ROTATE_CCW_90 = 0x04,
    CMD_ROTATE_CCW_45 = 0x05,
    CMD_DRIVE_FWD_HALF = 0x08,
    CMD_DRIVE_FWD_FULL = 0x09,
    CMD_DRIVE_REV_HALF = 0x10,
    CMD_DRIVE_REV_FULL = 0x11,
    CMD_ALIGN_BEACON = 0x20,
    CMD_SEARCH_TAPE = 0x40
    
} Command_t;

extern const uint8_t validCommandBytes[11];

// Prescale lookup table (maps enum to hardware register bits)
extern const uint8_t PrescaleLookup[];

// System clock configuration
#define PBCLK_FREQ 20000000         // 20 MHz peripheral bus clock

// Motor specifications
#define MAX_RPM 32                 // Maximum motor RPM

// ADC configuration
#define ADC_MAX_VALUE 1023          // 10-bit ADC maximum value

// Encoder configuration
#define IC_PRESCALE 16              // Input Capture prescale (captures every 16th edge)
#define IC_ENCODER_EDGES_PER_REV (3048 / IC_PRESCALE) // Encoder edges per revolution after prescale
#define ENCODER_TIMER_PRESCALE 256  // Timer3 prescale for encoder timing

// Time constants
#define SECONDS_PER_MINUTE 60       // Conversion factor for RPM calculations

// PWM configuration (shared between DCMotorService and SpeedControlService)
#define DUTY_MAX_TICKS 2000        // Maximum duty cycle ticks (100%)
#define PWM_PERIOD_TICKS 1999        // PWM period in timer ticks (for 5kHz at 20MHz PBCLK with 1:2 prescale)
#define DUTY_MIN_TICKS 0            // Minimum duty cycle ticks (0%)

#define TIMING_PIN_LAT LATBbits.LATB15

// Motor Indexes
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

// Directions
#define FORWARD 0
#define REVERSE 1

// Speed Levels (duty cycle ticks)
#define HALF_SPEED 1500   // 50% duty cycle
#define FULL_SPEED 2000   // 100% duty cycle

// Timer Durations (ms)
#define SIMPLE_MOVE_90_MS 1500
#define SIMPLE_MOVE_45_MS 750
#define BEACON_ALIGN_MS 5000
#define TAPE_SEARCH_MS 10000

// Debug Pin
#define DEBUG_OUTPUT_PIN_LAT LATBbits.LATB15
#define DEBUG_OUTPUT_PIN_TRIS TRISBbits.TRISB15
#define DEBUG_OUTPUT_PIN_ANSEL ANSELBbits.ANSB15


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
float PeriodToRPM(uint32_t period);

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
