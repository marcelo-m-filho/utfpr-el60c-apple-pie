/**
  ******************************************************************************
  * @file    usb_audio_user_cfg.h
  * @author  MCD Application Team 
  * @brief   USB audio application configuration.
  * @version horoscope
  ******************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef __USB_AUDIO_USER_CFG_H
#define __USB_AUDIO_USER_CFG_H

#ifdef __cplusplus
 extern "C" {
#endif

// includes 
#include <stdint.h>
#include "usb_audio_constants.h"

// exported constants 

// --- project configuration --- 
// define which class is used USE_USB_AUDIO_CLASS_10 : must be defined, future release will supports USE_USB_AUDIO_CLASS_20 
#define  USE_USB_AUDIO_CLASS_10                      1
/* for playback project define USE_USB_AUDIO_RECORDING,  for recording project define USE_USB_AUDIO_RECORDING and for si
  * for simultaneous playback and recording define both flags  USE_USB_AUDIO_RECORDING and USE_USB_AUDIO_RECORDING */

// define synchronization method 
#define USE_AUDIO_PLAYBACK_USB_FEEDBACK              1

// definition of channel count and  space mapping of channels (only two channels are currently supported)
#define USB_AUDIO_CONFIG_PLAY_CHANNEL_COUNT          0x02 // stereo audio  
#define USB_AUDIO_CONFIG_PLAY_CHANNEL_MAP            0x03 // channels Left and right 

// supported resolution definition (currently, expansion supports only 16 and 24 bit resolutions)
#define USB_AUDIO_CONFIG_PLAY_RES_BIT                16 
#define USB_AUDIO_CONFIG_PLAY_RES_BYTE               2  

// definition of the list of frequencies (1 to use, 0 to not support)
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K         0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K          0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K          1
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K        0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_32_K          0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K          0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K           0

#define USE_AUDIO_TIMER_VOLUME_CTRL                  0   
#define USB_AUDIO_CONFIG_PLAY_BUFFER_SIZE            (1024 * 10)   
 
// exported types 

// exported macros 

// exported functions

#ifdef __cplusplus
}
#endif

#endif // __USB_AUDIO_USER_CFG_H 
 