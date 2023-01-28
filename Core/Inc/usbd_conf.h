/**
  ******************************************************************************
  * @file    USB_Device/AUDIO_EXT_Advanced_Player_Recorder/Inc/usbd_conf.h
  * @author  MCD Application Team 
  * @brief   General low level driver configuration
  * @version horoscope
  ******************************************************************************
  */

// define to prevent recursive inclusion 
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

// includes 
#include "stm32f7xx_hal.h"
#include "hal_usb_ex.h"
#include "usb_audio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// exported constants 

// --- common config ---
#define USBD_MAX_NUM_INTERFACES               2
#define USBD_MAX_NUM_CONFIGURATION            1
#define USBD_MAX_STR_DESC_SIZ                 0x100
#define USBD_SUPPORT_USER_STRING              0 
#define USBD_SELF_POWERED                     1
#define USBD_DEBUG_LEVEL                      0

#if USE_AUDIO_PLAYBACK_USB_FEEDBACK
#define USBD_SUPPORT_AUDIO_OUT_FEEDBACK       1
#endif

#if USE_USB_AUDIO_CLASS_10
  #if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCIES)
    #define USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES 1
  #endif
#endif

// --- AUDIO Class Config ---

// exported types 

// exported macros

// memory management macros
#define USBD_malloc               malloc
#define USBD_free                 free
#define USBD_memset               memset
#define USBD_memcpy               memcpy
    
// debug macros
#if (USBD_DEBUG_LEVEL > 0)
#define  USBD_UsrLog(...)   printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)   
#endif                            
                            
#if (USBD_DEBUG_LEVEL > 1)

#define  USBD_ErrLog(...)   printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)   
#endif 
                                                        
#if (USBD_DEBUG_LEVEL > 2)                         
#define  USBD_DbgLog(...)   printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)                         
#endif

// exported functions
void USBD_error_handler(void);

#endif // __USBD_CONF_H
