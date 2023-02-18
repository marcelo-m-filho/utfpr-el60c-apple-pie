/**
  ******************************************************************************
  * @file    usb_audio_user_cfg.h
  * @author  MCD Application Team 
  * @brief   USB audio application user configuration.
  * @version horoscope 0.1
  ******************************************************************************
  */

// define to prevent recursive inclusion
#ifndef __USB_AUDIO_USER_CFG_H
#define __USB_AUDIO_USER_CFG_H

#include <stdint.h>
#include "usb_audio_constants.h"

#define  USE_USB_AUDIO_CLASS_10 1

// define synchronization method 
#define USE_AUDIO_PLAYBACK_USB_FEEDBACK 1

// definition of channel count and space mapping of channels 
#define USB_AUDIO_CONFIG_PLAY_CHANNEL_COUNT   0x02 // stereo audio  
#define USB_AUDIO_CONFIG_PLAY_CHANNEL_MAP     0x03 // channels Left and right 

#define USB_AUDIO_CONFIG_PLAY_RES_BIT         16   // bits per sample
#define USB_AUDIO_CONFIG_PLAY_RES_BYTE        2    // bytes per sample

#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K  0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K   0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K   1
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K 0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_32_K   0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K   0
#define USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K    0

#define USE_AUDIO_TIMER_VOLUME_CTRL           0   
#define USB_AUDIO_CONFIG_PLAY_BUFFER_SIZE     (1024 * 10)   

#endif // __USB_AUDIO_USER_CFG_H
