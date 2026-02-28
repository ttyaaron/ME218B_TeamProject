/****************************************************************************
 Template header file for defining Events

 ****************************************************************************/
#ifndef SMEvents_H
#define SMEvents_H

// Universal events take up 0,1 & 2. User Events start at 3
typedef enum
{
  EV_NO_EVENT,
  EV_ENTRY,
  EV_ENTRY_HISTORY,
  EV_EXIT,
  EV_SET,
  EV_START,
  EV_POPCORN,
  EV_EXPIRED,
  EV_DEF_LIGHT,
  EV_DOOR_OPEN,
  EV_CLEAR
}Event_t;

#endif /*SMEvents_H */

