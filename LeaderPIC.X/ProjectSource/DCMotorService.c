/****************************************************************************
 Module
   DCMotorService.c

 Revision
   2.0.0

 Description
   Integrated DC motor control service with encoder feedback and PI speed control.
   This service combines motor PWM control, dual encoder input capture, and
   PI control loops for two independent motors (left/right).

 Notes
    This integrated service handles:
    - PWM motor control for left and right motors
    - Encoder input capture (IC1 for left, IC2 for right) using shared Timer3
    - PI speed control loops running at 2ms intervals (Timer4)
    - Motor direction control

 History
 When           Who     What/Why
 -------------- ---     --------
 02/25/26       Tianyu  Integrated encoder and speed control into DCMotorService
 01/21/26       Tianyu  Updated for Lab 6 motor speed control
 01/15/26       Tianyu  Fixed position wrapping logic for unsigned type
 01/14/26       Tianyu  Initial creation for Lab 5
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "DCMotorService.h"
#include "CommonDefinitions.h"
#include "Ports.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"
#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/

// Motor Port definitions
#define MOTOR_FORWARD_PIN_L   LATBbits.LATB4 // PWM pin (OC1)
#define MOTOR_REVERSE_PIN_L   LATBbits.LATB15

#define MOTOR_FORWARD_PIN_R   LATBbits.LATB5 // PWM pin (OC2)
#define MOTOR_REVERSE_PIN_R   LATAbits.LATA4

// PWM configuration (period defined in CommonDefinitions.h)
#define INITIAL_DUTY_TICKS 0  // Initial duty cycle in ticks

// Encoder Input Capture pin configuration
// IC3 (Left encoder) on RB11/pin22
#define IC_PIN_L_TRIS TRISBbits.TRISB11

// IC2 (Right encoder) on RA3/pin 10
#define IC_PIN_R_TRIS TRISAbits.TRISA3

// Timing pin for performance measurement
#define TIMING_PIN_TRIS TRISBbits.TRISB15
#define TIMING_PIN_ANSEL ANSELBbits.ANSB15
#define TIMING_PIN_LAT LATBbits.LATB15

// Encoder Timer configuration (Timer3, shared between both encoders)
#define ENCODER_TIMER_PRESCALE_VAL 256
#define ENCODER_PRESCALE_CHOSEN PRESCALE_256

// Control timer configuration (Timer4, runs PI controllers)
#define CONTROL_PERIOD_MS 2         // Control loop period in milliseconds
#define CONTROL_TIMER_PRESCALE 8
#define CONTROL_PRESCALE_CHOSEN PRESCALE_8
#define CONTROL_TIMER_PERIOD ((PBCLK_FREQ / CONTROL_TIMER_PRESCALE / 500) - 1)

// Control mode selection
#define USE_OPEN_LOOP_CONTROL false  // Set to true for open-loop, false for closed-loop

// PI Controller parameters
// PI gains for mm/s closed-loop control.
// Tuning procedure: increase KP until output tracks setpoint without
// oscillation, then increase KI to eliminate steady-state error.
// Use EncoderTestService serial output to observe measured vs target mm/s.
#define KP 2.25f                      // Proportional gain - start conservative
#define KI 3.5f                     // Integral gain - start small
#define TS 0.002f                    // Sampling time in seconds (2 ms)
#define INTEGRAL_CLAMP_TICKS  (DUTY_MAX_TICKS * 2 / 3)

// Smoothing factor: 0.0 = no update (frozen), 1.0 = no filtering (raw)
// 0.3 means each new reading contributes 30% of the new value
#define MEAS_SPEED_ALPHA  0.4f

// RPM calculation constants
#define INVALID_TIME 0xFFFFFFFF      // Marker for invalid/uninitialized time

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service */
// Motor control functions
static void ConfigureTimeBase(uint8_t prescale);
static void ConfigurePWM(void);
static void ConfigureDCMotorPins(void);
static uint16_t MapSpeedToDutyCycle(uint16_t desiredSpeed);

// Encoder functions
static void ConfigureEncoderTimer(void);
static void ConfigureInputCapture(void);
static float PeriodToRPM(uint32_t period);  // Local float version for PI controller only
uint32_t GetElapsedTicksSinceLastEdge(uint8_t motorIndex);

// Speed control functions
static void ConfigureControlTimer(void);
static int16_t ClampDutyCycle(float value);

/*---------------------------- Module Variables ---------------------------*/
// Module level Priority variable
static uint8_t MyPriority;

// Motor control variables
static float TargetSpeed_mm_s[2];     // Target speed in mm/s (set by DCMotor_SetSpeed_mm_s, read by PI controller)
static uint16_t DesiredSpeed[2];      // Duty cycle ticks (output from PI controller)
static uint8_t DesiredDirection[2];

// Encoder variables (for left and right wheels)
static volatile uint32_t CapturedTime[2] = {0, 0};          // Latest captured time for each wheel
static uint32_t LastCapturedTime[2] = {INVALID_TIME, INVALID_TIME}; // Previous capture for period calculation
// Note: SharedTimer3RolloverCounter is defined in CommonDefinitions.c
// and shared by BeaconDetectFSM, DCMotorService IC3 and IC2
static uint32_t EdgeTimeDifference[2] = {0, 0};             // Time between edges for each wheel

// Cumulative IC capture event counters (incremented in ISR)
static volatile uint32_t ICEventCount[2] = {0u, 0u};

// PI Controller variables (for left and right wheels)
static float AccumulatedError[2] = {0.0f, 0.0f};
static int16_t LastDutyCycleTicks[2] = {0, 0};

// Control monitoring variables (updated by ISR)
static volatile float CurrentDesiredSpeed[2] = {0.0f, 0.0f};
static volatile float CurrentMeasuredSpeed[2] = {0.0f, 0.0f};
static float FilteredSpeed[2] = {0.0f, 0.0f};
static volatile int16_t CurrentDutyCycleTicks[2] = {0, 0};

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDCMotorService

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the integrated DC Motor Service including PWM, encoders,
     and speed control

 Author
     Tianyu, 02/25/26 (integrated version)
****************************************************************************/
bool InitDCMotorService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  // Initialize motor control variables
  TargetSpeed_mm_s[LEFT_MOTOR] = 0.0f;
  TargetSpeed_mm_s[RIGHT_MOTOR] = 0.0f;
  DesiredSpeed[LEFT_MOTOR] = 0;
  DesiredSpeed[RIGHT_MOTOR] = 0;
  DesiredDirection[LEFT_MOTOR] = FORWARD;
  DesiredDirection[RIGHT_MOTOR] = FORWARD;
  
  // Initialize encoder variables
  CapturedTime[LEFT_MOTOR] = 0;
  CapturedTime[RIGHT_MOTOR] = 0;
  LastCapturedTime[LEFT_MOTOR] = INVALID_TIME;
  LastCapturedTime[RIGHT_MOTOR] = INVALID_TIME;
  SharedTimer3RolloverCounter = 0;
  EdgeTimeDifference[LEFT_MOTOR] = 0;
  EdgeTimeDifference[RIGHT_MOTOR] = 0;
  
  // Initialize PI controller variables
  AccumulatedError[LEFT_MOTOR] = 0.0f;
  AccumulatedError[RIGHT_MOTOR] = 0.0f;
  LastDutyCycleTicks[LEFT_MOTOR] = 0;
  LastDutyCycleTicks[RIGHT_MOTOR] = 0;
  CurrentDesiredSpeed[LEFT_MOTOR] = 0.0f;
  CurrentDesiredSpeed[RIGHT_MOTOR] = 0.0f;
  CurrentMeasuredSpeed[LEFT_MOTOR] = 0.0f;
  CurrentMeasuredSpeed[RIGHT_MOTOR] = 0.0f;
  CurrentDutyCycleTicks[LEFT_MOTOR] = 0;
  CurrentDutyCycleTicks[RIGHT_MOTOR] = 0;
  
  /********************************************
   Hardware Initialization
   *******************************************/
  
  // Configure motor control pins and PWM
  ConfigureDCMotorPins();
  ConfigurePWM();
  
  // Configure encoder Input Capture pins as digital inputs
  IC_PIN_L_TRIS = 1;   // Set as input
  IC3R = 0b0011;       // Map IC3 to RB11 (left encoder)
  
  IC_PIN_R_TRIS = 1;   // Set as input
  IC2R = 0b0000;       // Map IC2 to RA3 (right encoder)
    
  // Configure the encoders (Timer3 and Input Capture modules)
  ConfigureEncoderTimer();
  ConfigureInputCapture();
  
  // Configure the speed control timer (Timer4)
  ConfigureControlTimer();
  
  DB_printf("Integrated DC Motor Service Initialized\r\n");
  
  // Post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostDCMotorService

 Parameters
     ES_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Tianyu, 01/14/26
****************************************************************************/
bool PostDCMotorService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunDCMotorService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles events to control the DC motor

 Author
   Tianyu, 01/14/26
****************************************************************************/
ES_Event_t RunDCMotorService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // Assume no errors
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
      // Initialization already done in Init function
      break;
      
    case ES_MOTOR_ACTION_CHANGE:
    {
      // Pseudocode:
      // FOR each motor in Motors[]
      //   Map desired speed to duty ticks
      //   Clamp duty ticks to safe range
      //   IF motor.direction == FORWARD
      //     Set forward pin HIGH, reverse pin LOW
      //     Write duty ticks to OCxRS
      //   ELSE (REVERSE)
      //     Set forward pin LOW, reverse pin HIGH
      //     Write duty ticks to OCxRS (or inverted if hardware requires)
      // END FOR

      // This is driving the motor in drive-brake mode

      // Calculate duty cycle for LEFT motor
      uint16_t dutyCycleLeft = MapSpeedToDutyCycle(DesiredSpeed[LEFT_MOTOR]);
      // Calculate duty cycle for RIGHT motor
      uint16_t dutyCycleRight = MapSpeedToDutyCycle(DesiredSpeed[RIGHT_MOTOR]);

      // LEFT MOTOR
      if (DesiredDirection[LEFT_MOTOR] == 0)
      {
        MOTOR_REVERSE_PIN_L = 0;
        OC1RS = dutyCycleLeft;
//        DB_printf("dutyCycle left 0:%u\r\n", dutyCycleLeft);
      }
      else
      {
        MOTOR_REVERSE_PIN_L = 1;
        OC1RS = PWM_PERIOD_TICKS - dutyCycleLeft + 1;
        // DB_printf("dutyCycle left 1:%u\r\n", OC1RS);
      }

      // RIGHT MOTOR
      // Hardware motor already inversed for right motor, when forward means same current go through left and right motor
      if (DesiredDirection[RIGHT_MOTOR] == 0)
      {
        MOTOR_REVERSE_PIN_R = 0;
        OC2RS = dutyCycleRight;
        // DB_printf("dutyCycle right 0:%u\r\n", dutyCycleRight);
      }
      else
      {
        MOTOR_REVERSE_PIN_R = 1;
        OC2RS = PWM_PERIOD_TICKS - dutyCycleRight + 1;
        // DB_printf("dutyCycle right 1:%u\r\n", OC2RS);
      }

      break;
    }
      
    default:
      break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     MotorCommandWrapper

 Parameters
     uint16_t speedLeft, speedRight
     uint8_t dirLeft, dirRight

 Returns
     None

 Description
     Legacy wrapper for open-loop duty cycle commands.
     WARNING: Does NOT set TargetSpeed_mm_s. For closed-loop control,
     use DCMotor_SetSpeed_mm_s() instead.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void MotorCommandWrapper(uint16_t speedLeft, uint16_t speedRight,
                         uint8_t dirLeft, uint8_t dirRight)
{
  ES_Event_t ThisEvent;

  // Set desired directions
  DesiredDirection[LEFT_MOTOR] = dirLeft;
  DesiredDirection[RIGHT_MOTOR] = dirRight;

  #if USE_OPEN_LOOP_CONTROL
    // In open-loop mode, set duty cycles directly
    DesiredSpeed[LEFT_MOTOR] = speedLeft;
    DesiredSpeed[RIGHT_MOTOR] = speedRight;
        
    ThisEvent.EventType = ES_MOTOR_ACTION_CHANGE;
    ThisEvent.EventParam = 0;
    PostDCMotorService(ThisEvent);
  #endif
  // In closed-loop mode, PI controller will update DesiredSpeed (duty cycles)
  // and post the motor action change event

  DB_printf("TargetSpeed:%u %u, DesiredDirection: %u %u\r\n", speedLeft, speedRight
          , DesiredDirection[0], DesiredDirection[1]);
}

/****************************************************************************
 Function
     DCMotor_SetSpeed_mm_s

 Parameters
     uint16_t speedLeft_mm_s  - desired left wheel surface speed in mm/s
     uint16_t speedRight_mm_s - desired right wheel surface speed in mm/s
     uint8_t  dirLeft         - FORWARD or REVERSE
     uint8_t  dirRight        - FORWARD or REVERSE

 Returns
     None

 Description
     Sets target speeds in mm/s for closed-loop PI control.
     In closed-loop mode (USE_OPEN_LOOP_CONTROL false), the PI controller
     handles duty cycle conversion. In open-loop mode, performs linear
     scaling: duty = speed * DUTY_MAX_TICKS / SPEED_FULL_MM_S

 Author
     Tianyu, 03/01/26
****************************************************************************/
void DCMotor_SetSpeed_mm_s(uint16_t speedLeft_mm_s,
                           uint16_t speedRight_mm_s,
                           uint8_t  dirLeft,
                           uint8_t  dirRight)
{
  // Store mm/s targets for the PI controller
  TargetSpeed_mm_s[LEFT_MOTOR]  = (float)speedLeft_mm_s;
  TargetSpeed_mm_s[RIGHT_MOTOR] = (float)speedRight_mm_s;
  DesiredDirection[LEFT_MOTOR]  = dirLeft;
  DesiredDirection[RIGHT_MOTOR] = dirRight;
  // Reset the filter when a new speed command is issued to avoid old speed affecting the new command
  FilteredSpeed[LEFT_MOTOR] = 0.0f;
  FilteredSpeed[RIGHT_MOTOR] = 0.0f;

  // In closed-loop mode, PI controller handles duty cycle — nothing else to do here.
  // In open-loop mode (USE_OPEN_LOOP_CONTROL true), fall through to MotorCommandWrapper.
#if USE_OPEN_LOOP_CONTROL
  uint32_t dutyLeft  = ((uint32_t)speedLeft_mm_s  * (uint32_t)DUTY_MAX_TICKS)
                       / (uint32_t)SPEED_FULL_MM_S;
  uint32_t dutyRight = ((uint32_t)speedRight_mm_s * (uint32_t)DUTY_MAX_TICKS)
                       / (uint32_t)SPEED_FULL_MM_S;
  if (dutyLeft  > DUTY_MAX_TICKS) { dutyLeft  = DUTY_MAX_TICKS; }
  if (dutyRight > DUTY_MAX_TICKS) { dutyRight = DUTY_MAX_TICKS; }
  MotorCommandWrapper((uint16_t)dutyLeft, (uint16_t)dutyRight, dirLeft, dirRight);
#endif
}

/****************************************************************************
 Function
     DCMotor_GetEncoderPeriod

 Parameters
     uint8_t motorIndex - LEFT_MOTOR (0) or RIGHT_MOTOR (1)

 Returns
     uint32_t - most recent EdgeTimeDifference in Timer3 ticks for that motor,
                or 0 if motorIndex is out of range

 Description
     Public query function for EncoderTestService and future SpeedControlService
     to read raw encoder period without accessing module internals directly.

 Author
     Tianyu, 03/01/26
****************************************************************************/
uint32_t DCMotor_GetEncoderPeriod(uint8_t motorIndex)
{
  if (motorIndex >= 2) return 0;

  uint32_t edgePeriod   = EdgeTimeDifference[motorIndex];
  uint32_t elapsedSince = GetElapsedTicksSinceLastEdge(motorIndex);
  return (elapsedSince > edgePeriod) ? elapsedSince : edgePeriod;
}

/****************************************************************************
 Function
     DCMotor_GetICEventCount

 Parameters
     uint8_t motorIndex - LEFT_MOTOR (0) or RIGHT_MOTOR (1)

 Returns
     uint32_t - cumulative IC event count for the specified motor since last reset

 Description
     Returns cumulative IC event count for the specified motor since last reset.
     Used for distance tracking.

 Author
     Tianyu, 03/01/26
****************************************************************************/
uint32_t DCMotor_GetICEventCount(uint8_t motorIndex)
{
  if (motorIndex < 2u)
  {
    return ICEventCount[motorIndex];
  }
  return 0u;
}

/****************************************************************************
 Function
     DCMotor_ResetICEventCount

 Parameters
     uint8_t motorIndex - LEFT_MOTOR (0) or RIGHT_MOTOR (1)

 Returns
     None

 Description
     Resets the IC event counter for the specified motor to zero.
     Call before a distance-controlled move begins.

 Author
     Tianyu, 03/01/26
****************************************************************************/
void DCMotor_ResetICEventCount(uint8_t motorIndex)
{
  if (motorIndex < 2u)
  {
    ICEventCount[motorIndex] = 0u;
  }
}

/***************************************************************************
 Interrupt Service Routines
 ****************************************************************************/

/****************************************************************************
 Function
     InputCaptureISR_IC3

 Parameters
     None

 Returns
     None

 Description
     Input Capture 3 interrupt for LEFT encoder. Reads captured time,
     calculates period, and updates module variables.

 Author
     Tianyu, 02/25/26
****************************************************************************/
void __ISR(_INPUT_CAPTURE_3_VECTOR, IPL7SOFT) InputCaptureISR_IC3(void)
{
  // Increment IC event counter for distance tracking
  ICEventCount[LEFT_MOTOR]++;
  
  // Read the captured timer value from IC3 buffer
  uint16_t capturedTimer16 = IC3BUF;
      
  // Clear the input capture interrupt flag
  IFS0CLR = _IFS0_IC3IF_MASK;

  // If T3IF is pending and captured value is after rollover (in lower half of timer range)
  if (IFS0bits.T3IF && (capturedTimer16 < 0x8000))
  {
    // Increment the roll-over counter
    SharedTimer3RolloverCounter++;
    // Clear the roll-over interrupt flag
    IFS0CLR = _IFS0_T3IF_MASK;
  }

  // Combine roll-over counter with captured timer value to create a full 32-bit time
  CapturedTime[LEFT_MOTOR] = ((uint32_t)SharedTimer3RolloverCounter << 16) | capturedTimer16;
  
  // Calculate period if we have a valid previous capture
  if (LastCapturedTime[LEFT_MOTOR] != INVALID_TIME)
  {
    if (CapturedTime[LEFT_MOTOR] >= LastCapturedTime[LEFT_MOTOR])
    {
      EdgeTimeDifference[LEFT_MOTOR] = CapturedTime[LEFT_MOTOR] - LastCapturedTime[LEFT_MOTOR];
    }
    else
    {
      // Handle wraparound
      EdgeTimeDifference[LEFT_MOTOR] = (0xFFFFFFFF - LastCapturedTime[LEFT_MOTOR]) + CapturedTime[LEFT_MOTOR] + 1;
    }
  }
  
  // Store current capture as last capture for next calculation
  LastCapturedTime[LEFT_MOTOR] = CapturedTime[LEFT_MOTOR];
}

/****************************************************************************
 Function
     InputCaptureISR_IC2

 Parameters
     None

 Returns
     None

 Description
     Input Capture 2 interrupt for RIGHT encoder. Reads captured time,
     calculates period, and updates module variables.

 Author
     Tianyu, 02/25/26
****************************************************************************/
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL7SOFT) InputCaptureISR_IC2(void)
{
  // Increment IC event counter for distance tracking
  ICEventCount[RIGHT_MOTOR]++;
  
  // Read the captured timer value from IC2 buffer
  uint16_t capturedTimer16 = IC2BUF;
      
  // Clear the input capture interrupt flag
  IFS0CLR = _IFS0_IC2IF_MASK;

  // If T3IF is pending and captured value is after rollover (in lower half of timer range)
  if (IFS0bits.T3IF && (capturedTimer16 < 0x8000))
  {
    // Increment the roll-over counter
    SharedTimer3RolloverCounter++;
    // Clear the roll-over interrupt flag
    IFS0CLR = _IFS0_T3IF_MASK;
  }

  // Combine roll-over counter with captured timer value to create a full 32-bit time
  CapturedTime[RIGHT_MOTOR] = ((uint32_t)SharedTimer3RolloverCounter << 16) | capturedTimer16;
  
  // Calculate period if we have a valid previous capture
  if (LastCapturedTime[RIGHT_MOTOR] != INVALID_TIME)
  {
    if (CapturedTime[RIGHT_MOTOR] >= LastCapturedTime[RIGHT_MOTOR])
    {
      EdgeTimeDifference[RIGHT_MOTOR] = CapturedTime[RIGHT_MOTOR] - LastCapturedTime[RIGHT_MOTOR];
    }
    else
    {
      // Handle wraparound
      EdgeTimeDifference[RIGHT_MOTOR] = (0xFFFFFFFF - LastCapturedTime[RIGHT_MOTOR]) + CapturedTime[RIGHT_MOTOR] + 1;
    }
  }
  
  // Store current capture as last capture for next calculation
  LastCapturedTime[RIGHT_MOTOR] = CapturedTime[RIGHT_MOTOR];
}

/****************************************************************************
 Function
     Timer3ISR

 Parameters
     None

 Returns
     None

 Description
     Timer3 interrupt for encoder rollover tracking. Increments the shared
     rollover counter to extend timing range beyond 16-bit timer.
     This ISR is shared by BeaconDetectFSM (IC1), left encoder (IC3),
     and right encoder (IC2).

 Author
     Tianyu, 02/25/26
****************************************************************************/
void __ISR(_TIMER_3_VECTOR, IPL6SOFT) Timer3ISR(void)
{
  // Disable interrupts globally to prevent race condition with IC ISR
  __builtin_disable_interrupts();
  
  // If T3IF is pending (timer has rolled over)
  if (IFS0bits.T3IF)
  {
    // Increment the roll-over counter to track timer wraparounds
    SharedTimer3RolloverCounter++;
    // Clear the roll-over interrupt flag
    IFS0CLR = _IFS0_T3IF_MASK;
  }

  // Re-enable interrupts globally
  __builtin_enable_interrupts();
}

/****************************************************************************
 Function
     GetElapsedTicksSinceLastEdge

 Parameters
     uint8_t motorIndex

 Returns
     uint32_t - ticks elapsed since the last IC capture for that motor.
                If this exceeds EdgeTimeDifference significantly, the 
                motor has slowed or stalled since the last capture.

 Description
     Reads the current Timer3 32-bit virtual timestamp and subtracts
     LastCapturedTime. Called from the PI ISR to detect stale periods.
****************************************************************************/
uint32_t GetElapsedTicksSinceLastEdge(uint8_t motorIndex)
{
  // Snapshot current timer with rollover
  uint16_t tmr = TMR3;
  uint32_t currentTime = ((uint32_t)SharedTimer3RolloverCounter << 16) | tmr;
  
  if (LastCapturedTime[motorIndex] == INVALID_TIME)
  {
    return 0xFFFFFFFF; // never had a capture
  }
  
  if (currentTime >= LastCapturedTime[motorIndex])
  {
    return currentTime - LastCapturedTime[motorIndex];
  }
  else
  {
    return (0xFFFFFFFF - LastCapturedTime[motorIndex]) + currentTime + 1;
  }
}

/****************************************************************************
 Function
     ControlTimerISR

 Parameters
     None

 Returns
     None

 Description
     Control Timer (Timer4) interrupt. Executes PI control algorithms
     for both motors every 2ms to maintain desired speeds.

 Author
     Tianyu, 02/25/26
****************************************************************************/
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) ControlTimerISR(void)
{  
  // Clear control timer interrupt flag
  IFS0CLR = _IFS0_T4IF_MASK;
  
  // If using open-loop control, skip the PI control logic
#if USE_OPEN_LOOP_CONTROL
  return;
#endif
  
  // Process control for LEFT motor
  {
    // Read target speed for left motor
    float targetSpeed = TargetSpeed_mm_s[LEFT_MOTOR];

    // This is avoiding stale period measurements when the motor has slowed or stalled since the last encoder edge.
    uint32_t edgePeriod    = EdgeTimeDifference[LEFT_MOTOR];
    uint32_t elapsedSince  = GetElapsedTicksSinceLastEdge(LEFT_MOTOR);

    // If elapsed time since last capture is more than 4x the last measured period,
    // the motor has slowed below that speed or stalled.
    // Use elapsed time as the effective period instead — this makes measured
    // speed decay toward zero naturally rather than freezing at the last value.
    uint32_t effectivePeriod;
    
    if (edgePeriod == 0u)
      {
        // First IC capture only — no valid period yet, report as stopped
        effectivePeriod = elapsedSince;
      }
      else
      {
        effectivePeriod = (elapsedSince > edgePeriod * 4u) ? elapsedSince : edgePeriod;
      }

    float rawSpeed = PeriodToSpeed_mm_s(effectivePeriod);

    // Clamp to physical maximum — any reading above this is a measurement artifact
    if (rawSpeed > SPEED_FULL_MM_S) { rawSpeed = SPEED_FULL_MM_S; }

    // Exponential moving average — smooths spikes without adding much lag
    FilteredSpeed[LEFT_MOTOR] = MEAS_SPEED_ALPHA * rawSpeed
                              + (1.0f - MEAS_SPEED_ALPHA) * FilteredSpeed[LEFT_MOTOR];

    float measuredSpeed = FilteredSpeed[LEFT_MOTOR];

    // Update monitoring variables
    CurrentDesiredSpeed[LEFT_MOTOR] = targetSpeed;
    CurrentMeasuredSpeed[LEFT_MOTOR] = measuredSpeed;
    
    // Compute control error
    float currentError = targetSpeed - measuredSpeed;
    
    // PI control law
    float proportional = KP * currentError;

    AccumulatedError[LEFT_MOTOR] += currentError * TS;
    
    float integral = KI * AccumulatedError[LEFT_MOTOR];

    if (integral > INTEGRAL_CLAMP_TICKS)  { integral = INTEGRAL_CLAMP_TICKS; }
    if (integral < -INTEGRAL_CLAMP_TICKS) { integral = -INTEGRAL_CLAMP_TICKS; }

    float u_unsat = proportional + integral;
    
    // Clamp output to duty cycle limits
    int16_t u_sat = ClampDutyCycle(u_unsat);
    
    // Anti-windup: check if saturated and error drives further into saturation
    if (u_unsat != (float)u_sat)
    {
      bool drivingIntoSaturation = false;
      
      if ((u_unsat > DUTY_MAX_TICKS) && (currentError > 0))
      {
        drivingIntoSaturation = true;
      }
      else if ((u_unsat < DUTY_MIN_TICKS) && (currentError < 0))
      {
        drivingIntoSaturation = true;
      }
      
      // If driving into saturation, undo the last integration step
      if (drivingIntoSaturation)
      {
        AccumulatedError[LEFT_MOTOR] -= currentError * TS;
      }
    }
    
    // Store controlled duty cycle
    CurrentDutyCycleTicks[LEFT_MOTOR] = u_sat;
    LastDutyCycleTicks[LEFT_MOTOR] = u_sat;
    DesiredSpeed[LEFT_MOTOR] = u_sat;
  }
  
  // Process control for RIGHT motor
  {
    // Read target speed for right motor
    float targetSpeed = TargetSpeed_mm_s[RIGHT_MOTOR];
    
    // This is avoiding stale period measurements when the motor has slowed or stalled since the last encoder edge.
    uint32_t edgePeriod    = EdgeTimeDifference[RIGHT_MOTOR];
    uint32_t elapsedSince  = GetElapsedTicksSinceLastEdge(RIGHT_MOTOR);

    // If elapsed time since last capture is more than 4x the last measured period,
    // the motor has slowed below that speed or stalled.
    // Use elapsed time as the effective period instead — this makes measured
    // speed decay toward zero naturally rather than freezing at the last value.
    uint32_t effectivePeriod;
    
    if (edgePeriod == 0u)
      {
        // First IC capture only — no valid period yet, report as stopped
        effectivePeriod = elapsedSince;
      }
      else
      {
        effectivePeriod = (elapsedSince > edgePeriod * 4u) ? elapsedSince : edgePeriod;
      }

    float rawSpeed = PeriodToSpeed_mm_s(effectivePeriod);

    // Clamp to physical maximum — any reading above this is a measurement artifact
    if (rawSpeed > SPEED_FULL_MM_S) { rawSpeed = SPEED_FULL_MM_S; }

    // Exponential moving average — smooths spikes without adding much lag
    FilteredSpeed[RIGHT_MOTOR] = MEAS_SPEED_ALPHA * rawSpeed
                              + (1.0f - MEAS_SPEED_ALPHA) * FilteredSpeed[RIGHT_MOTOR];

    float measuredSpeed = FilteredSpeed[RIGHT_MOTOR];

    // Update monitoring variables
    CurrentDesiredSpeed[RIGHT_MOTOR] = targetSpeed;
    CurrentMeasuredSpeed[RIGHT_MOTOR] = measuredSpeed;
    
    // Compute control error
    float currentError = targetSpeed - measuredSpeed;
    
    // PI control law
    float proportional = KP * currentError;

    AccumulatedError[RIGHT_MOTOR] += currentError * TS;
    
    float integral = KI * AccumulatedError[RIGHT_MOTOR];

    if (integral > INTEGRAL_CLAMP_TICKS)  { integral = INTEGRAL_CLAMP_TICKS; }
    if (integral < -INTEGRAL_CLAMP_TICKS) { integral = -INTEGRAL_CLAMP_TICKS; }

    float u_unsat = proportional + integral;
    
    // Clamp output to duty cycle limits
    int16_t u_sat = ClampDutyCycle(u_unsat);
    
    // Anti-windup: check if saturated and error drives further into saturation
    if (u_unsat != (float)u_sat)
    {
      bool drivingIntoSaturation = false;
      
      if ((u_unsat > DUTY_MAX_TICKS) && (currentError > 0))
      {
        drivingIntoSaturation = true;
      }
      else if ((u_unsat < DUTY_MIN_TICKS) && (currentError < 0))
      {
        drivingIntoSaturation = true;
      }
      
      // If driving into saturation, undo the last integration step
      if (drivingIntoSaturation)
      {
        AccumulatedError[RIGHT_MOTOR] -= currentError * TS;
      }
    }
    
    // Store controlled duty cycle
    CurrentDutyCycleTicks[RIGHT_MOTOR] = u_sat;
    LastDutyCycleTicks[RIGHT_MOTOR] = u_sat;
    DesiredSpeed[RIGHT_MOTOR] = u_sat;
  }
  
  // Post motor action change event to update PWM outputs
  ES_Event_t ControlEvent;
  ControlEvent.EventType = ES_MOTOR_ACTION_CHANGE;
  ControlEvent.EventParam = 0;
  PostDCMotorService(ControlEvent);

//  // Print monitoring info every 500ms (250 calls * 2ms period)
//  static uint16_t printCount = 0;
//  if (++printCount >= 25) {   // print every 250 calls = every 50ms
//    uint32_t lFilteredSpeed_hundredths = (uint32_t)(FilteredSpeed[LEFT_MOTOR] * 100);
//    uint32_t rFilteredSpeed_hundredths = (uint32_t)(FilteredSpeed[RIGHT_MOTOR] * 100);
//    DB_printf("L tgt=%u meas=%u.%u mm/s duty=%d | R tgt=%u meas=%u.%u mm/s duty=%d\r\n",
//              (unsigned int)CurrentDesiredSpeed[LEFT_MOTOR],
//              lFilteredSpeed_hundredths / 100,
//              lFilteredSpeed_hundredths % 100,
//              (int)CurrentDutyCycleTicks[LEFT_MOTOR],
//              (unsigned int)CurrentDesiredSpeed[RIGHT_MOTOR],
//              rFilteredSpeed_hundredths / 100,
//              rFilteredSpeed_hundredths % 100,
//              (int)CurrentDutyCycleTicks[RIGHT_MOTOR]);
//    printCount = 0;
//  }
  
}

/***************************************************************************
 Private Functions
 ***************************************************************************/

/****************************************************************************
 Function
     PeriodToRPM

 Parameters
     uint32_t period - time between encoder edges in timer ticks

 Returns
     float - measured RPM

 Description
     LOCAL version for PI controller use only. Converts encoder period
     measurement to RPM. Uses hardware-verified constants from CommonDefinitions.h.

 Author
     Tianyu, 03/01/26
****************************************************************************/
static float PeriodToRPM(uint32_t period)
{
  // Prevent division by zero
  if (period == 0u)
  {
    return 0.0f;
  }
  
  // Timer3 clock is TIMER3_CLOCK_HZ = 78125 Hz
  // Each IC capture represents IC_EVENTS_PER_REV per revolution
  // Convert to rev/s: (TIMER3_CLOCK_HZ / IC_EVENTS_PER_REV) / period
  // Convert to RPM: rev/s * 60
  
  float rev_per_sec = (float)TIMER3_CLOCK_HZ / ((float)IC_EVENTS_PER_REV * (float)period);
  float rpm = rev_per_sec * 60.0f;
  
  return rpm;
}

/****************************************************************************
 Function
     ConfigureTimeBase

 Parameters
     uint8_t prescale

 Returns
     None

 Description
     Configures Timer2 as the time base for PWM operation

 Author
     Tianyu, 01/21/26
****************************************************************************/
static void ConfigureTimeBase(uint8_t prescale)
{
  // Clear the ON control bit to disable the timer
  T2CONbits.ON = 0;
  // Clear the TCS control bit to select the internal PBCLK source
  T2CONbits.TCS = 0;
  // Select the desired timer input clock prescale
  T2CONbits.TCKPS = PrescaleLookup[prescale];
  // Clear the timer register TMRx
  TMR2 = 0;
  // Enable the timer by setting the ON control bit
  T2CONbits.ON = 1;
}

/****************************************************************************
 Function
     ConfigurePWM

 Parameters
     None

 Returns
     None

 Description
     Configures the PWM Output Compare modules for both motors
 
 Notes
     This function configures both the timer base and the Output Compare modules.
     The timer configuration is done first to ensure proper initialization order.

 Author
     Tianyu, 01/21/26
****************************************************************************/
static void ConfigurePWM(void)
{
  // Step 1: Configure the timer base (must be done before OC config)
  ConfigureTimeBase(PRESCALE_2);
  // Keep timer off during configuration to avoid unintended pulses
  T2CONbits.ON = 0;
  // Disable the PWM Output Compare modules before configuration
  OC1CONbits.ON = 0;
  OC2CONbits.ON = 0;

  // Set the PWM period by writing to the timer period register
  PR2 = PWM_PERIOD_TICKS;
  
  // Configure OC1 for left motor
  OC1RS = INITIAL_DUTY_TICKS;
  OC1CONbits.OCM = 0b110; // PWM mode on OCx; Fault pin disabled
  OC1CONbits.ON = 1;

  // Configure OC2 for right motor
  OC2CONbits.OCM = 0b110;  // PWM Mode
  OC2CONbits.OCTSEL = 0;   // Also use Timer 2
  OC2RS = INITIAL_DUTY_TICKS;
  OC2CONbits.ON = 1;

  // Start the timer after PWM configuration is complete
  TMR2 = 0;       // Clear timer register for clean start
  T2CONbits.ON = 1;
}

/****************************************************************************
 Function
     ConfigureDCMotorPins

 Parameters
     None

 Returns
     None

 Description
     Configures the I/O pins for DC motor control as outputs

 Author
     Tianyu, 01/21/26
****************************************************************************/
static void ConfigureDCMotorPins(void)
{
  // Configure pins as digital outputs
  TRISBbits.TRISB4 = 0;  // MOTOR_FORWARD_L as output
  TRISBbits.TRISB15 = 0;  // MOTOR_REVERSE_L as output
  TRISBbits.TRISB5 = 0;  // MOTOR_FORWARD_R as output
  TRISAbits.TRISA4 = 0;  // MOTOR_REVERSE_R as output

  // Initialize all pins to low
  MOTOR_FORWARD_PIN_L = 0;
  MOTOR_REVERSE_PIN_L = 0;
  MOTOR_FORWARD_PIN_R = 0;
  MOTOR_REVERSE_PIN_R = 0;

  // Map OC1 output to RB4 (left motor PWM)
  RPB4R = 0b0101;
  // Map OC2 output to RB5 (right motor PWM)
  RPB5R = 0b0101;
}

/****************************************************************************
 Function
     MapSpeedToDutyCycle

 Parameters
     uint16_t desiredSpeed: desired speed value

 Returns
     uint16_t: duty cycle value, clamped to safe range

 Description
     Maps the desired speed to PWM duty cycle and clamps to valid bounds.

 Author
     Tianyu, 01/21/26
****************************************************************************/
static uint16_t MapSpeedToDutyCycle(uint16_t desiredSpeed)
{
  uint16_t dutyCycle = desiredSpeed;

  // Clamp duty cycle to safe range
  if (dutyCycle > DUTY_MAX_TICKS)
  {
    dutyCycle = DUTY_MAX_TICKS;
  }
  else if (dutyCycle < DUTY_MIN_TICKS)
  {
    dutyCycle = DUTY_MIN_TICKS;
  }
  
  return (uint16_t)dutyCycle;
}

/****************************************************************************
 Function
     ConfigureEncoderTimer

 Parameters
     None

 Returns
     None

 Description
     Configures Timer3 as the shared time base for both encoder input captures

 Author
     Tianyu, 02/25/26
****************************************************************************/
static void ConfigureEncoderTimer(void)
{
  // Disable the timer during configuration
  T3CONbits.ON = 0;
  
  // Select internal PBCLK as clock source
  T3CONbits.TCS = 0;
  
  // Set timer prescaler using lookup table
  T3CONbits.TCKPS = PrescaleLookup[ENCODER_PRESCALE_CHOSEN];
  
  // Clear timer register
  TMR3 = 0;
  
  // Load period register with maximum value for maximum range
  PR3 = 0xFFFF;
  
  // Clear timer interrupt flag
  IFS0bits.T3IF = 0;

  // Configure the interrupt priority
  IPC3bits.T3IP = 6; // Priority 6
  IPC3bits.T3IS = 0; // Subpriority 0

  // Enable timer interrupt
  IEC0bits.T3IE = 1;
  
  // Enable the timer
  T3CONbits.ON = 1;
}

/****************************************************************************
 Function
     ConfigureInputCapture

 Parameters
     None

 Returns
     None

 Description
     Configures Input Capture modules 1 and 2 for both encoders using
     Timer3 as the shared time base

 Author
     Tianyu, 02/25/26
****************************************************************************/
static void ConfigureInputCapture(void)
{
  // Disable base timer during configuration
  T3CONbits.ON = 0;

  // Configure Input Capture 3 (LEFT encoder)
  IC3CONbits.ON = 0;
  IC3CONbits.ICTMR = 0;          // Use Timer3
  IC3CONbits.ICM = 0b101;        // Capture every 16th rising edge
  IFS0CLR = _IFS0_IC3IF_MASK;    // Clear interrupt flag
  
  // Clear IC3 buffer
  volatile uint32_t dummy;
  while (IC3CONbits.ICBNE) {
    dummy = IC3BUF;
  }
  
  IPC3bits.IC3IP = 7;            // Priority 7
  IPC3bits.IC3IS = 0;            // Subpriority 0
  IEC0bits.IC3IE = 1;            // Enable interrupt
  IC3CONbits.ON = 1;             // Enable module
  
  // Configure Input Capture 2 (RIGHT encoder)
  IC2CONbits.ON = 0;
  IC2CONbits.ICTMR = 0;          // Use Timer3
  IC2CONbits.ICM = 0b101;        // Capture every 16th rising edge
  IFS0CLR = _IFS0_IC2IF_MASK;    // Clear interrupt flag
  
  // Clear IC2 buffer
  while (IC2CONbits.ICBNE) {
    dummy = IC2BUF;
  }
  
  IPC2bits.IC2IP = 7;            // Priority 7
  IPC2bits.IC2IS = 0;            // Subpriority 0
  IEC0bits.IC2IE = 1;            // Enable interrupt
  IC2CONbits.ON = 1;             // Enable module

  // Enable the timer after IC configuration is complete
  T3CONbits.ON = 1;
}

/****************************************************************************
 Function
     ConfigureControlTimer

 Parameters
     None

 Returns
     None

 Description
     Configures Timer4 as the control loop timer with 2ms period

 Author
     Tianyu, 02/25/26
****************************************************************************/
static void ConfigureControlTimer(void)
{
  // Disable timer during configuration
  T4CONbits.ON = 0;
  
  // Select internal PBCLK as clock source
  T4CONbits.TCS = 0;
  
  // Set timer prescaler using lookup table
  T4CONbits.TCKPS = PrescaleLookup[CONTROL_PRESCALE_CHOSEN];
  
  // Clear timer register
  TMR4 = 0;
  
  // Set period for 2ms interrupt (500 Hz)
  PR4 = CONTROL_TIMER_PERIOD;
  
  // Clear control timer interrupt flag
  IFS0CLR = _IFS0_T4IF_MASK;
  
  // Set interrupt priority lower than Input Capture
  IPC4bits.T4IP = 5;
  IPC4bits.T4IS = 0;
  
  // Enable control timer interrupt
  IEC0bits.T4IE = 1;
  
  // Enable timer
  T4CONbits.ON = 1;
}

/****************************************************************************
 Function
     ClampDutyCycle

 Parameters
     float value - unclamped duty cycle value

 Returns
     int16_t - clamped duty cycle value

 Description
     Clamps the duty cycle value to valid range

 Author
     Tianyu, 02/25/26
****************************************************************************/
static int16_t ClampDutyCycle(float value)
{
  if (value > DUTY_MAX_TICKS)
  {
    return DUTY_MAX_TICKS;
  }
  else if (value < DUTY_MIN_TICKS)
  {
    return DUTY_MIN_TICKS;
  }
  else
  {
    return (int16_t)value;
  }
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
