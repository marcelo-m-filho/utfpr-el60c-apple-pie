/**
  ******************************************************************************
  * @file    audio_speaker_node.h
  * @author  MCD Application Team 
  * @brief   header file for the audio_speaker_node.c file.
  * @version horoscope 0.1
  ******************************************************************************
  */

#ifndef __AUDIO_SPEAKER_NODE_H
#define __AUDIO_SPEAKER_NODE_H


#ifdef __cplusplus
 extern "C" {
#endif

// includes 
#include "audio_user_devices.h"
#include "audio_node.h"
#include "usb_audio_app_config.h"

// exported constants
// resolution of volume change; see the UAC specification for more details */
#define VOLUME_SPEAKER_RES_DB_256       128     // 0.5 db (0.5 * 256 = 128)
#define VOLUME_SPEAKER_DEFAULT_DB_256   0       // 0   db
#define VOLUME_SPEAKER_MAX_DB_256       1536    // 6   db (6256 = 1536)
#define VOLUME_SPEAKER_MIN_DB_256       -6400   // -25 db (-25*256 = -6400)

#if USE_AUDIO_TIMER_VOLUME_CTRL 
  // signals to the Timer's interrupt which responsible of volume change that a volume value change is required
  #define SPEAKER_CMD_CHANGE_VOLUME  0x10
  // signals to the Timer's interrupt, which responsible of volume change, that a mute state change is required
  #define SPEAKER_CMD_MUTE_UNMUTE    0x20
  // signals to the Timer's interrupt, which responsible of volume change, that it has to apply mute change then volume change
  #define SPEAKER_CMD_MUTE_FIRST     0x40
#endif // USE_AUDIO_TIMER_VOLUME_CTRL

#define  AUDIO_SpeakerInit AUDIO_SPEAKER_USER_Init

// speaker node main structure
typedef struct Audio_SpeakerNode
{
  AUDIO_Node_t                  node;            // generic node structure
  AUDIO_CircularBuffer_t*       buf;             // audio data buffer
  uint16_t                      packet_length;   // maximum packet length
  #if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
    uint16_t                    packet_length_max_44_1; // packet maximal length for frequency 44_1
    uint8_t                     injection_44_count;     // used as count to inject 9 packets (44 samples) then 1 packet(45 samples)
    uint8_t                     injection_45_pos;       // used as position of  45 samples packet
  #endif // USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
  int8_t                        (*SpeakerDeInit)          (uint32_t /*node_handle*/);
  int8_t                        (*SpeakerStart)           (AUDIO_CircularBuffer_t* /*buffer*/, uint32_t /*node handle*/);
  int8_t                        (*SpeakerStop)            (uint32_t /*node handle*/);
  int8_t                        (*SpeakerChangeFrequency) (uint32_t /*node handle*/);
  int8_t                        (*SpeakerMute)            (uint16_t /*channel_number*/, uint8_t /*mute */,uint32_t /*node handle*/);
  int8_t                        (*SpeakerSetVolume)       (uint16_t /*channel_number*/, int /*volume_db_256 */, uint32_t /*node handle*/);
  int8_t                        (*SpeakerStartReadCount)  (uint32_t /*node handle*/);
  uint16_t                      (*SpeakerGetReadCount)    (uint32_t /*node handle*/);
  AUDIO_SpeakerSpecificParams_t  specific; // should be defined by user for user speaker
}
AUDIO_SpeakerNode_t;

// exported functions
 int8_t  AUDIO_SpeakerInit(AUDIO_Description_t* audio_description, AUDIO_Session_t* session_handle, uint32_t node_handle);

// !!! MIGRAR O .C
#ifdef __cplusplus
}
#endif


#endif // __AUDIO_SPEAKER_NODE_H