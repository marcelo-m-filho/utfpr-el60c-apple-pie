/**
  ******************************************************************************
  * @file    USB_Device/AUDIO_EXT_Advanced_Player_Recorder/Inc/usbd_desc.h
  * @author  MCD Application Team 
  * @brief   Header for usbd_desc.c module
  * @version horoscope
  ******************************************************************************
  */

// define to prevent recursive inclusion
#ifndef __USBD_DESC_H
#define __USBD_DESC_H

// includes 
#include "usbd_def.h"

// exported types 

// exported constants 
#define DEVICE_ID1            (0x1FFF7A10)
#define DEVICE_ID2            (0x1FFF7A14)
#define DEVICE_ID3            (0x1FFF7A18)

#define USB_SIZ_STRING_SERIAL 0x1A
// exported macros

// exported functions ------------------------------------------------------- 
extern USBD_DescriptorsTypeDef AUDIO_Desc;

#endif // __USBD_DESC_H 
 