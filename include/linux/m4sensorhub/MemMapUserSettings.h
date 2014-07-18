/*********************************************************************
*
*   Copyright (C) 2012 Motorola, Inc.
*
**********************************************************************

File    : MemMapUserSettings.h
Purpose :
**********************************************************************/
#ifndef __MEMMAP_USERSETTING_H__
#define __MEMMAP_USERSETTING_H__
/****************************** Defines *******************************/
typedef struct memMapUserSettings
{
  u8 version;
  u8 userAge;
  u8 userGender;
  u8 userHeight;
  u16 userWeight;
  u8 screenStatus;
  u8 rtcReset;
}sUserData;

typedef enum
{
  FEMALE = 0,
  MALE = 1
}eGender;

typedef enum
{
  DISPLAY_OFF_INTERACTIVE_OFF = 0, /* was SCREEN_OFF */
  DISPLAY_OFF_INTERACTIVE_ON  = 1, /* was SCREEN_ON */
  DISPLAY_ON_INTERACTIVE_OFF  = 2,
  DISPLAY_ON_INTERACTIVE_ON   = 3
}eScreenStatus;

#define SCREEN_STATUS_INTERACTIVE_MASK 1
#define SCREEN_STATUS_DISPLAY_MASK 2

#define INTERACTIVE_ON  (SCREEN_STATUS_INTERACTIVE_MASK & DISPLAY_OFF_INTERACTIVE_ON)
#define INTERACTIVE_OFF (SCREEN_STATUS_INTERACTIVE_MASK & DISPLAY_OFF_INTERACTIVE_OFF)
#define DISPLAY_ON      (SCREEN_STATUS_DISPLAY_MASK     & DISPLAY_ON_INTERACTIVE_OFF)
#define DISPLAY_OFF     (SCREEN_STATUS_DISPLAY_MASK     & DISPLAY_OFF_INTERACTIVE_OFF)

typedef enum
{
  USERSETTINGS_VERSION = 0,
  USERSETTINGS_USERAGE = 1,
  USERSETTINGS_USERGENDER = 2,
  USERSETTINGS_USERHEIGHT = 3,
  USERSETTINGS_USERWEIGHT = 4,
  USERSETTINGS_SCREENSTATUS = 6,
  USERSETTINGS_RTCRESET = 7,
}eUserSettingsOffset;

#endif // __MEMMAP_USERSETTING_H__