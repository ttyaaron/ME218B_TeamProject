/****************************************************************************
 Module
   DCMotorService.c

 Revision
   1.0.1

 Description
   This service controls two DC motors (left/right) using PWM outputs.

 Notes
    When initializing the DC Motor Service:
    Configure & initialize I/O pins connected with DC motor for PWM
    Map OC output to I/O pin, connected to one of the motor control pins
        Disable the PWM Output Compare module
    Disable the PWM timer
    Set the TMRy prescale value and enable the time base by setting TON (TxCON<15>) =1
    Set the PWM period by writing to the selected timer period register (PRy).
    IF (read from direction I/O pin, and it is LOW)
        Set the PWM duty cycle ticks by writing to the OCxRS register
        Set the other motor control pin to LOW
    ELSE
        Set the PWM duty cycle ticks to (PWM period – duty cycle ticks + 1) by writing to the OCxRS register
        Set the other motor control pin to HIGH
    Write the OCxR register with the initial duty cycle ticks.
    Configure the Output Compare module for one of two PWM Operation modes by writing to the Output Compare mode bits, OCM<2:0> (OCxCON<2:0>). Turn ON the Output Compare module
    Turn the timer to the PWM system on.

    On ES_MOTOR_ACTION_CHANGE event:
        Get desiredSpeed from event parameter
    Map the desired speed to duty cycle ticks
    IF (read from direction I/O pin, and it is LOW)
        Set the other motor control pin to LOW
    ELSE
        Set the new PWM duty cycle ticks to (PWM period – duty cycle ticks + 1)
        Set the other motor control pin to HIGH
    Clamp duty cycle ticks to safe range
    Write new duty cycle ticks to OCxRS



 History
 When           Who     What/Why
 -------------- ---     --------
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
#include "dbprintf.h"
#include <xc.h>

/*----------------------------- Module Defines ----------------------------*/

// Port definitions
#define MOTOR_FORWARD_PIN_L   LATBbits.LATB4
#define MOTOR_REVERSE_PIN_L   LATBbits.LATB5
//#define DIRECTION_PIN       PORTBbits.RB8  // Direction input pin

//Right Wheel Port definitions
#define MOTOR_FORWARD_PIN_R   LATBbits.LATB11
#define MOTOR_REVERSE_PIN_R   LATBbits.LATB13

// PWM configuration (period defined in CommonDefinitions.h)
#define INITIAL_DUTY_TICKS 0  // Initial duty cycle in ticks
#define ENABLE_POT_AD

/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service */
static void ConfigureTimeBase(uint8_t prescale);
static void ConfigurePWM(void);
static void ConfigureDCMotorPins(void);
static uint16_t MapSpeedToDutyCycle(uint16_t desiredSpeed);

/*---------------------------- Module Variables ---------------------------*/
// Module level Priority variable
static uint8_t MyPriority;
static uint16_t DesiredSpeed[2];
static uint8_t DesiredDirection[2];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDCMotorService

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes the DC Motor Service
     

 Author
     Tianyu, 01/14/26
****************************************************************************/
bool InitDCMotorService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  DesiredSpeed[LEFT_MOTOR] = 0;
  DesiredSpeed[RIGHT_MOTOR] = 0;
  DesiredDirection[LEFT_MOTOR] = FORWARD;
  DesiredDirection[RIGHT_MOTOR] = FORWARD;
  
  /********************************************
   Initialization code for DC Motor Control system
   *******************************************/
  // Initialize Output Compare Pins used
  // TODO: Expand ConfigureDCMotorPins for both motors
  ConfigureDCMotorPins();

  // Configure PWM module (includes timer configuration)
  ConfigurePWM();

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

      uint16_t dutyCycle = MapSpeedToDutyCycle(DesiredSpeed[LEFT_MOTOR]);

      if (DesiredDirection[0] == 0)
      {
        MOTOR_REVERSE_PIN_L = 0;
        OC1RS = dutyCycle;
        DB_printf("dutyCycle left 0:%u\r\n", dutyCycle);
      }
      else
      {
        MOTOR_REVERSE_PIN_L = 1;
        OC1RS = PWM_PERIOD_TICKS - dutyCycle + 1;
        DB_printf("dutyCycle left 1:%u\r\n", OC1RS);
      }

      // Hardware motor already inversed for right motor, when forward means same current go through left and right motor
      if (DesiredDirection[1] == 0)
      {
        MOTOR_REVERSE_PIN_R = 0;
        OC2RS = dutyCycle;
        DB_printf("dutyCycle right 0:%u\r\n", dutyCycle);
      }
      else
      {
        MOTOR_REVERSE_PIN_R = 1;
        OC2RS = PWM_PERIOD_TICKS - dutyCycle + 1;
        DB_printf("dutyCycle right 1:%u\r\n", OC2RS);
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
     Writes desired speeds/directions into module variables and posts
     ES_MOTOR_ACTION_CHANGE events to DCMotorService.

 Author
     Tianyu, 02/03/26
****************************************************************************/
void MotorCommandWrapper(uint16_t speedLeft, uint16_t speedRight,
                         uint8_t dirLeft, uint8_t dirRight)
{
  ES_Event_t ThisEvent;

  DesiredSpeed[LEFT_MOTOR] = speedLeft;
  DesiredSpeed[RIGHT_MOTOR] = speedRight;
  DesiredDirection[LEFT_MOTOR] = dirLeft;
  DesiredDirection[RIGHT_MOTOR] = dirRight;

  ThisEvent.EventType = ES_MOTOR_ACTION_CHANGE;
  ThisEvent.EventParam = 0;
  PostDCMotorService(ThisEvent);

  DB_printf("DesiredSpeed:%u %u, DesiredDirection: %u %u\r\n", DesiredSpeed[0], DesiredSpeed[1], DesiredDirection[0], DesiredDirection[1]);
}

/***************************************************************************
 Private Functions
 ***************************************************************************/

/****************************************************************************
 Function
     ConfigureTimeBase

 Parameters
     None

 Returns
     None

 Description
     Configures the timer used as the time base for PWM operation

 Author
     Tianyu, 01/21/26
****************************************************************************/
static void ConfigureTimeBase(uint8_t prescale)
{
  // Clear the ON control bit to disable the timer
  T2CONbits.ON = 0;
  // Clear teh TCS control bit to select the internal PBCLK source
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
     Configures the PWM Output Compare module for PWM operation
 
 Notes
     This function configures both the timer base and the Output Compare module.
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
  // Disable the PWM Output Compare module before configuration
  OC1CONbits.ON = 0;
  OC2CONbits.ON = 0;

  // Set the PWM period by writing to the timer period register
  PR2 = PWM_PERIOD_TICKS;
  // Write the OCxR register with the initial duty cycle
  OC1RS = INITIAL_DUTY_TICKS;
  // Configure the Output Compare module for PWM mode
  OC1CONbits.OCM = 0b110; // PWM mode on OCx; Fault pin disabled
  // Turn ON the Output Compare module
  OC1CONbits.ON = 1;

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
  // Configure pins as digital outputs (all pins here don't have analog functions)
  TRISBbits.TRISB4 = 0;  // MOTOR_FORWARD as output
  TRISBbits.TRISB5 = 0;  // MOTOR_REVERSE as output
  TRISBbits.TRISB11 = 0;  // MOTOR_FORWARD_R as output
  TRISBbits.TRISB13 = 0;  // MOTOR_REVERSE_R as output

  // Initialize all pins to low
  MOTOR_FORWARD_PIN_L = 0;
  MOTOR_REVERSE_PIN_L = 0;
  MOTOR_FORWARD_PIN_R = 0;
  MOTOR_REVERSE_PIN_R = 0;

  // Map OC1 output to RB4
  RPB4R = 0b0101;
  // Map OC2 output to RB11
  RPB11R = 0b0101;
}

/****************************************************************************
 Function
     MapSpeedToDutyCycle

 Parameters
     uint16_t desiredSpeed: desired speed value (0 to ADC_MAX_VALUE)

 Returns
     uint16_t: duty cycle value (0 to PWM_PERIOD_TICKS), clamped to safe range

 Description
     Maps the desired speed from the ADC range to PWM duty cycle range
     and clamps the result to ensure it stays within valid bounds.

 Notes
     Uses ADC_MAX_VALUE from CommonDefinitions.h as the source of truth for the
     ADC range to ensure consistency across services.

 Author
     Tianyu, 01/21/26
****************************************************************************/
static uint16_t MapSpeedToDutyCycle(uint16_t desiredSpeed)
{
  // Map the desired speed to duty cycle
  // desiredSpeed range: [0, ADC_MAX_VALUE] → dutyCycle range: [0, PWM_PERIOD_TICKS]
  // uint32_t dutyCycle = ((uint32_t)desiredSpeed * PWM_PERIOD_TICKS) / ADC_MAX_VALUE;
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

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
