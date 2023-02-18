/**
  ******************************************************************************
  * @file    audio_user_devices.h
  * @author  MCD Application Team 
  * @brief   Abstraction of board specific devices.
  * @version horoscope 0.1
  ******************************************************************************
  */
 
// define to prevent recursive inclusion
#ifndef __AUDIO_USER_DEVICES_H
#define __AUDIO_USER_DEVICES_H

#ifdef __cplusplus
  extern "C" {
#endif

// includes
#include "stm32f769i_discovery_audio.h"
#include "usb_audio_app_config.h"
   
// exported constants
#if USE_AUDIO_DFSDM_MEMS_MIC
  #define AUDIO_MAX_SAMPLE_COUNT_LENGTH(frq) (((frq) + 999)/1000)
  #define AUDIO_SAMPLE_COUNT_LENGTH(frq) ((uint32_t)(((uint32_t)(frq))/1000))
  #define AUDIO_PACKET_SAMPLES_COUNT(frq) ((frq)/1000)
#endif // USE_AUDIO_DFSDM_MEMS_MIC

// exported types 
typedef struct AUDIO_SpeakerSpecificParams
{
  uint16_t      injection_size;         // the nominal size of the unit packet sent using DMA to SAI
  uint8_t*      data;                   // a pointer to data, which is going to transmit using DMA to SAI 
  uint16_t      data_size;              // a size of data, which is going to transmit using DMA to SAI 
  uint8_t*      alt_buffer;             // an alternative buffer used  when underrun is produced (no enough data to inject) or when padding should be added
  uint16_t      alt_buf_half_size;      // the half size of the alternative buffer
  uint8_t       double_buff;            // when the padding is needed the double buffering are required. It means that the alt_buff will contain two packet 
  uint8_t       offset;                 // a binary flag. used to indicate if next packet is in the first half of alternate buffer or in the second half
  __IO uint8_t  cmd;                    // this field contains commands to execute within next transfer complete call(or in next Volume change interrupt) 
  uint16_t      dma_remaining;          // used for synchronization, it helps to provide the counter of played samples 
} 
AUDIO_SpeakerSpecificParams_t;


#ifdef __cplusplus
}
#endif

#endif  // __AUDIO_USER_DEVICES_H
