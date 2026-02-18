/****************************************************************************
 Module
   EventCheckers.c

 Revision
   0.1

 Description
   Event checkers for beacon, tape, and keyboard input.

 Notes
   Use static variables to detect transitions.

 History
 When           Who     What/Why
 -------------- ---     --------
 02/03/26       Tianyu  Initial creation for Lab 8 event checkers
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_PostList.h"
#include "ES_ServiceHeaders.h"
#include "ES_Port.h"
#include "EventCheckers.h"
#include "CommonDefinitions.h"
#include "dbprintf.h"
#include "Ports.h"

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
   Check4Keystroke

 Parameters
   None

 Returns
   bool: true if a new key was detected & posted

 Description
   checks to see if a new key from the keyboard is detected and, if so,
   retrieves the key and posts an ES_NEW_KEY event to TestHarnessService0

 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c

 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if (IsNewKeyReady())   // new key waiting?
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType   = ES_NEW_KEY;
    ThisEvent.EventParam  = GetNewKey();
    ES_PostAll(ThisEvent);
    return true;
  }
  return false;
}

/****************************************************************************
 Function
   Check4CommandAvailable

 Parameters
   None

 Returns
   bool: true if a new command event was detected & posted

 Description
   Placeholder event checker. Command retrieval is handled by the
   CommandRetrieveService (SPI polling/interrupts).

 Author
   Tianyu, 02/03/26
****************************************************************************/
bool Check4CommandAvailable(void)
{
  // TODO: If moving command retrieval into an event checker, implement here.
  return false;
}
