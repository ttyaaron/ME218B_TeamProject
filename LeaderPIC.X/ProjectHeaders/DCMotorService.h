/****************************************************************************
 Module
     DCMotorService.h

 Revision
     2.0.0

 Description
     Header file for integrated DCMotorService.
     Combines motor control, encoder feedback, and PI speed control
     for two independent motors (left and right).

 Notes
     This integrated service provides:
     - PWM motor control
     - Dual encoder input capture feedback
     - PI speed control loops

 History
 When           Who     What/Why
 -------------- ---     --------
 02/25/26       Tianyu  Integrated encoder and speed control
 02/03/26       Tianyu  Updated for Lab 8 two-motor control
*****************************************************************************/

#ifndef DCMotorService_H
#define DCMotorService_H

#include "ES_Types.h"
#include <stdint.h>

// Public Function Prototypes

bool InitDCMotorService(uint8_t Priority);
bool PostDCMotorService(ES_Event_t ThisEvent);
ES_Event_t RunDCMotorService(ES_Event_t ThisEvent);

// Motor control wrapper
void MotorCommandWrapper(uint16_t speedLeft, uint16_t speedRight,
                         uint8_t dirLeft, uint8_t dirRight);
void DCMotor_SetSpeed_mm_s(uint16_t speedLeft_mm_s,
                           uint16_t speedRight_mm_s,
                           uint8_t  dirLeft,
                           uint8_t  dirRight);

// Encoder query function
uint32_t Encoder_GetLatestPeriod(uint8_t motorIndex);
uint32_t DCMotor_GetEncoderPeriod(uint8_t motorIndex);
uint32_t DCMotor_GetICEventCount(uint8_t motorIndex);
void     DCMotor_ResetICEventCount(uint8_t motorIndex);

// Tape sensor public functions
void TapeSensor_Read(void);           // Read all tape sensors
void TapeSensor_UpdateLineFollow(void); // Update line following control
void TapeSensor_CheckIntersect(void);   // Check for intersections

// Tape sensor value getters
uint32_t TapeSensor_GetLeftAnalog(void);
uint32_t TapeSensor_GetRightAnalog(void);
uint32_t TapeSensor_GetCenterAnalog(void);
bool TapeSensor_GetLeftDigital(void);
bool TapeSensor_GetRightDigital(void);

#endif /* DCMotorService_H */
