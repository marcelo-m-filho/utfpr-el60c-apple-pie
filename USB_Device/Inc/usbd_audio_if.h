/**
  ******************************************************************************
  * @file    usbd_audio_if.h
  * @author  MCD Application Team 
  * @brief   Header for usbd_audio_if.c file. (USB Device Audio Interface)
  * @version horoscope 0.1
  ******************************************************************************
  */ 

// Define to prevent recursive inclusion -------------------------------------
#ifndef __USBD_AUDIO_IF_H
#define __USBD_AUDIO_IF_H

// Includes ------------------------------------------------------------------
#include "usbd_audio.h"
#include "audio_sessions_usb.h"

// Exported constants --------------------------------------------------------
 extern USBD_AUDIO_InterfaceCallbacksf_t audio_class_interface;

// Exported types ------------------------------------------------------------
#if USE_AUDIO_USB_INTERRUPT
  typedef enum 
  {
  USBD_AUDIO_PLAYBACK  = 0x01,
  USBD_AUDIO_RECORD    = 0x02
  }
  USBD_AUDIO_FunctionTypedef;
#endif // USE_AUDIO_USB_INTERRUPT

// Exported functions ------------------------------------------------------- 
#if USE_AUDIO_USB_INTERRUPT
  int8_t USBD_AUDIO_ExecuteControl(uint8_t func, AUDIO_ControlCommand_t control , uint32_t val , uint32_t private_data);
#endif // USE_AUDIO_USB_INTERRUPT

#endif // __USBD_AUDIO_IF_H 