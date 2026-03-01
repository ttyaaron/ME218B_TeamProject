/****************************************************************************
 Module
   SpeedControlService.c

 Revision
   1.0.0

 Description
   This service implements a PI (Proportional-Integral) controller for
   DC motor speed control. The controller runs on a timer interrupt at
   2ms intervals to maintain consistent control loop timing.

 Notes
    When initializing the SpeedControl service:
    Configure a dedicated control timer not used by PWM or Input Capture time base:
        Disable timer
        Select internal PBCLK as clock source
        Choose prescaler and PRx so that timer interrupt period = 2 ms
        Clear TMRx
        Clear control timer interrupt flag
        Set control timer interrupt priority lower than Input Capture / Encoder ISR
        Enable control timer interrupt
        Enable timer
    Initialize controller state:
        accumulatedError = 0
        lastDutyCycleTicks = 0

    Control Timer Interrupt response routine (fires every 2 ms):
        Read inputs with minimal work:
            desiredSpeed = ADToRPM(ADCService_GetDesiredSpeed())
            measuredPeriod = EncoderService_GetLatestPeriod()
            measuredSpeed = PeriodToRPM(measuredPeriod)

        Compute control error:
            currentError = desiredSpeed – measuredSpeed

        PI control law:
            proportional = Kp * currentError
            accumulatedError += currentError * Ts (Ts = 0.002 s)
            integral = Ki * accumulatedError
            u_unsat = proportional + integral
            u_sat = clamp(u_unsat, dutyMinTicks, dutyMaxTicks)
            IF (u_unsat != u_sat) AND (currentError drives further into saturation):
                undo the last integration step: accumulatedError -= currentError * Ts

        controlledDutyCycleTicks = u_sat
        Post ES_DUTY_CYCLE_CHANGE(controlledDutyCycleTicks)
        Clear control timer interrupt flag
        Return from interrupt

 History
 When           Who     What/Why
 -------------- ---     --------
 01/28/26       Tianyu  Initial creation for Lab 7
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "SpeedControlService.h"
#include "ADService.h"
#include "EncoderService.h"
#include "DCMotorService.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"
#include <xc.h>
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
// Control timer configuration (using Timer4, not used by PWM or Input Capture)
#define CONTROL_PERIOD_MS 2         // Control loop period in milliseconds
#define TIMER_PRESCALE 8
#define PRESCALE_CHOSEN PRESCALE_8

// Calculate PR value for 2ms period: (PBCLK / prescale / frequency) - 1
// frequency = 1000/2 = 500 Hz
#define CONTROL_TIMER_PERIOD ((PBCLK_FREQ / TIMER_PRESCALE / 500) - 1)

// PI Controller parameters
//#define KP 65.0f                     // Proportional gain
//#define KI 100.0f                     // Integral gain
#define KP 65.0f                     // Proportional gain
#define KI 1000.0f                     // Integral gain
#define TS 0.002f                   // Sampling time in seconds (2 ms)

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service */
static void ConfigureControlTimer(void);
static int16_t ClampDutyCycle(float value);

/*---------------------------- Module Variables ---------------------------*/
// Module level Priority variable
static uint8_t MyPriority;

// Controller state variables
static float AccumulatedError = 0.0f;
static int16_t LastDutyCycleTicks = 0;

// Control inputs (updated by ISR, read by main loop if needed)
static volatile float CurrentDesiredSpeed = 0.0f;
static volatile float CurrentMeasuredSpeed = 0.0f;
static volatile int16_t CurrentDutyCycleTicks = 0;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSpeedControlService

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the Speed Control Service and configures the control timer

 Author
     Tianyu, 01/28/26
****************************************************************************/
bool InitSpeedControlService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  /********************************************
   Initialization code for Speed Control Service
   *******************************************/
  
  // Initialize controller state
  AccumulatedError = 0.0f;
  LastDutyCycleTicks = 0;
  CurrentDesiredSpeed = 0.0f;
  CurrentMeasuredSpeed = 0.0f;
  CurrentDutyCycleTicks = 0;
  
  // Configure the dedicated control timer
  ConfigureControlTimer();
  
  DB_printf("Speed Control Service Initialized\r\n");
  
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
     PostSpeedControlService

 Parameters
     ES_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue

 Author
     Tianyu, 01/28/26
****************************************************************************/
bool PostSpeedControlService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSpeedControlService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles events for the speed control service. The actual control
   computation is done in the timer ISR for precise timing.

 Author
   Tianyu, 01/28/26
****************************************************************************/
ES_Event_t RunSpeedControlService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // Assume no errors
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
      // Initialization complete, nothing additional to do
      DB_printf("Speed Control Service Running\r\n");
      break;
      
    default:
      break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     ControlTimerISR

 Parameters
     None

 Returns
     None

 Description
     Control Timer interrupt response routine. Executes the PI control
     algorithm every 2ms to maintain motor speed.

 Author
     Tianyu, 01/28/26
****************************************************************************/
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) ControlTimerISR(void)
{
  // Get target speed from DCMotorService (from 0 to 1023, representing 0 to max RPM)
  uint16_t speedValue = GetDesiredSpeed(); // TODO: need to write for both motors
  float desiredSpeed = ADToRPM(speedValue);
  // Debug print
//  uint32_t desiredSpeed_hundreds = (uint32_t)(desiredSpeed * 100);
//  DB_printf("         /%d.%d\r", desiredSpeed_hundreds / 100, desiredSpeed_hundreds % 100);
  
  uint32_t measuredPeriod = Encoder_GetLatestPeriod();
  float measuredSpeed = PeriodToRPM(measuredPeriod);
  //Debug print
//  uint32_t measuredSpeed_hundreds = (uint32_t)(measuredSpeed * 100);
//  DB_printf("%d.%d /%d.%d RPM \r", measuredSpeed_hundreds / 100, measuredSpeed_hundreds % 100, desiredSpeed_hundreds / 100, desiredSpeed_hundreds % 100);
  
  // Update module-level variables for monitoring
  CurrentDesiredSpeed = desiredSpeed;
  CurrentMeasuredSpeed = measuredSpeed;
  
  // Compute control error
  float currentError = desiredSpeed - measuredSpeed;
  
  // PI control law
  float proportional = KP * currentError;
  
  // Tentatively accumulate error
  AccumulatedError += currentError * TS;
  
//  DB_printf("%d PP \r\n", (uint32_t)(proportional));
  
//  DB_printf("%d AC \r\n", (uint32_t)(AccumulatedError));
  
  float integral = KI * AccumulatedError;
  float u_unsat = proportional + integral;
  
  // Clamp output to duty cycle limits
  int16_t u_sat = ClampDutyCycle(u_unsat);
  
  // Anti-windup: check if saturated and error drives further into saturation
  if (u_unsat != (float)u_sat)
  {
    // Check if current error drives further into saturation
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
      AccumulatedError -= currentError * TS;
    }
  }
  
  // Store controlled duty cycle
  CurrentDutyCycleTicks[LEFT_MOTOR] = u_sat;
  CurrentDutyCycleTicks[RIGHT_MOTOR] = u_sat;
  LastDutyCycleTicks[LEFT_MOTOR] = u_sat;
  LastDutyCycleTicks[RIGHT_MOTOR] = u_sat;
  
  // Post ES_DUTY_CYCLE_CHANGE event
  ES_Event_t ControlEvent;
  ControlEvent.EventType = ES_MOTOR_ACTION_CHANGE;
  ControlEvent.EventParam = (uint16_t)u_sat;
  PostDCMotorService(ControlEvent);
  
  // Clear control timer interrupt flag
  IFS0CLR = _IFS0_T4IF_MASK;
}

/***************************************************************************
 Private Functions
 ***************************************************************************/


/****************************************************************************
 Function
     Encoder_GetLatestPeriod

 Parameters
     uint8_t motorIndex - LEFT_MOTOR or RIGHT_MOTOR

 Returns
     uint32_t - the latest measured period in timer ticks

 Description
     Query function that returns the time period between encoder edges for
     the specified motor. Used by speed control or monitoring.

 Author
     Tianyu, 02/25/26
****************************************************************************/
uint32_t Encoder_GetLatestPeriod(uint8_t motorIndex)
{
  if (motorIndex < 2)
  {
    return EdgeTimeDifference[motorIndex];
  }
  return 0;
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
     Tianyu, 01/28/26
****************************************************************************/
static void ConfigureControlTimer(void)
{
  // Disable timer during configuration
  T4CONbits.ON = 0;
  
  // Select internal PBCLK as clock source
  T4CONbits.TCS = 0;
  
  // Set timer prescaler using lookup table
  T4CONbits.TCKPS = PrescaleLookup[PRESCALE_CHOSEN];
  
  // Clear timer register
  TMR4 = 0;
  
  // Set period for 2ms interrupt (500 Hz)
  PR4 = CONTROL_TIMER_PERIOD;
  
  // Clear control timer interrupt flag
  IFS0CLR = _IFS0_T4IF_MASK;
  
  // Set interrupt priority lower than Input Capture (IC is 7, Encoder timer is 6, so use 5)
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
     Tianyu, 01/28/26
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
