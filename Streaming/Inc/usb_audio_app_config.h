/**
  ******************************************************************************
  * @file    usb_audio_user.h
  * @author  MCD Application Team 
  * @brief   USB audio application configuration.
  * @version horoscope 0.1
  * @details formerly known as usb_audio.h
  ******************************************************************************
  */

// define to prevent recursive inclusion
#ifndef __USB_AUDIO_APP_CONFIG_H
#define __USB_AUDIO_APP_CONFIG_H

// includes
#include <stdint.h>
#include "usb_audio_constants.h"
#include "audio_node.h"
#include "usb_audio_user_config.h"

// exported constants & macros

// definition of the USB IRQ priority and the USB FIFO size in word 
#define USB_IRQ_PREPRIO                               3U

#ifdef USE_USB_FS
  #define USB_FIFO_WORD_SIZE                          320U
#else  // USE_USB_FS
  #define USB_FIFO_WORD_SIZE                          1024U
#endif // USE_USB_FS

// play session : list of terminal and unit id for audio function
// must be greater than the highest interface number (to avoid request destination confusion)
#define USB_AUDIO_CONFIG_PLAY_TERMINAL_INPUT_ID       0x12
#define USB_AUDIO_CONFIG_PLAY_UNIT_FEATURE_ID         0x16
#define USB_AUDIO_CONFIG_PLAY_TERMINAL_OUTPUT_ID      0x14

// playback computing the max and the min frequency */  
#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MAX              USB_AUDIO_CONFIG_FREQ_192_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MAX              USB_AUDIO_CONFIG_FREQ_96_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MAX              USB_AUDIO_CONFIG_FREQ_48_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MAX              USB_AUDIO_CONFIG_FREQ_44_1_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_32_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MAX              USB_AUDIO_CONFIG_FREQ_32_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MAX              USB_AUDIO_CONFIG_FREQ_16_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MAX              USB_AUDIO_CONFIG_FREQ_8_K
#else
  #error "Playback frequency is missing"
#endif 

#if USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MIN              USB_AUDIO_CONFIG_FREQ_8_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MIN              USB_AUDIO_CONFIG_FREQ_16_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_32_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MIN              USB_AUDIO_CONFIG_FREQ_32_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MIN              USB_AUDIO_CONFIG_FREQ_44_1_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MIN              USB_AUDIO_CONFIG_FREQ_48_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MIN              USB_AUDIO_CONFIG_FREQ_96_K
#elif USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K
  #define USB_AUDIO_CONFIG_PLAY_FREQ_MIN              USB_AUDIO_CONFIG_FREQ_192_K
#endif 

// macro to compute the count of supported frequency
#define USB_AUDIO_CONFIG_PLAY_FREQ_COUNT  (USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K +\
                                          USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K   +\
                                          USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K   +\
                                          USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K +\
                                          USB_AUDIO_CONFIG_PLAY_USE_FREQ_32_K   +\
                                          USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K   +\
                                          USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K)

#define USB_AUDIO_CONFIG_PLAY_DEF_FREQ    USB_AUDIO_CONFIG_PLAY_FREQ_MAX

#if ((USB_AUDIO_CONFIG_PLAY_FREQ_COUNT) > 1)
  #define USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES 1
#endif 

  #define USBD_AUDIO_CONFIG_PLAY_SA_INTERFACE         0x01 // AUDIO STREAMING INTERFACE NUMBER FOR PLAY SESSION 
  #define USBD_AUDIO_CONFIG_PLAY_EP_OUT               0x01
    #if USE_AUDIO_PLAYBACK_USB_FEEDBACK   
    #define USB_AUDIO_CONFIG_PLAY_EP_SYNC             0x81
    #if USE_USB_AUDIO_CLASS_10
      #define USB_AUDIO_CONFIG_PLAY_FEEDBACK_REFRESH  0x07// refresh every 32(2^5) ms 
    #endif // USE_USB_AUDIO_CLASS_10 
    #endif // USE_AUDIO_PLAYBACK_USB_FEEDBACK  
  #else // USE_USB_AUDIO_PLAYBACK  
  #define USBD_AUDIO_CONFIG_RECORD_SA_INTERFACE       0x01 // AUDIO STREAMING INTERFACE NUMBER FOR RECORD SESSION  
  #define USB_AUDIO_CONFIG_RECORD_EP_IN               0x81

/* defining the max packet length*/
#if USE_AUDIO_PLAYBACK_USB_FEEDBACK
#define USBD_AUDIO_CONFIG_PLAY_MAX_PACKET_SIZE ((uint16_t)(AUDIO_USB_MAX_PACKET_SIZE((USB_AUDIO_CONFIG_PLAY_FREQ_MAX+ 1),\
        USB_AUDIO_CONFIG_PLAY_CHANNEL_COUNT,\
        USB_AUDIO_CONFIG_PLAY_RES_BYTE)))
  #else /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */
  #define USBD_AUDIO_CONFIG_PLAY_MAX_PACKET_SIZE ((uint16_t)(AUDIO_USB_MAX_PACKET_SIZE((USB_AUDIO_CONFIG_PLAY_FREQ_MAX),\
        USB_AUDIO_CONFIG_PLAY_CHANNEL_COUNT,\
        USB_AUDIO_CONFIG_PLAY_RES_BYTE)))
#endif /* USE_AUDIO_PLAYBACK_USB_FEEDBACK */

#endif // __USB_AUDIO_APP_CONFIG_H