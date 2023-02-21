/**
  ******************************************************************************
  * @file    USB_Device/AUDIO_EXT_Advanced_Player_Recorder/Inc/usbd_conf.h
  * @author  MCD Application Team 
  * @brief   General low level driver configuration
  * @version horoscope 0.1
  ******************************************************************************
  */

// Define to prevent recursive inclusion ---------------------------------------
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

// Includes --------------------------------------------------------------------
#include "stm32f7xx_hal.h"
#include "hal_usb_ex.h"
 #include "usb_audio_app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Exported constants ----------------------------------------------------------

// Common Config 
#define USBD_MAX_NUM_INTERFACES               2
#define USBD_MAX_NUM_CONFIGURATION            1
#define USBD_MAX_STR_DESC_SIZ                 0x100
#define USBD_SUPPORT_USER_STRING              0 
#define USBD_SELF_POWERED                     1
#define USBD_DEBUG_LEVEL                      0

#if USE_AUDIO_PLAYBACK_USB_FEEDBACK
  #define USBD_SUPPORT_AUDIO_OUT_FEEDBACK 1
#endif  // USE_AUDIO_PLAYBACK_USB_FEEDBACK 

#if USE_USB_AUDIO_CLASS_10
  #if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCIES)
    #define USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES 1
  #endif //(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCIES) 
#endif // USE_USB_AUDIO_CLASS_10 

// AUDIO Class Config 
// Memory management macros   
#define USBD_malloc   malloc
#define USBD_free     free
#define USBD_memset   memset
#define USBD_memcpy   memcpy
    
// DEBUG macros   
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

// Exported functions --------------------------------------------------------- 
void USBD_error_handler(void);

#endif // __USBD_CONF_H 

