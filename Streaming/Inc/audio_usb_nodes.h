/**
  ******************************************************************************
  * @file    audio_usb_nodes.h
  * @author  MCD Application Team 
  * @brief   header of audio_usb_nodes.c
  * @version horoscope 0.1
  ******************************************************************************
  */

// define to prevent recursive inclusion
#ifndef __AUDIO_USB_NODES_H
#define __AUDIO_USB_NODES_H

// includes
#include "usbd_audio.h"
#include "audio_node.h"

// exported constants
#define AUDIO_MAX_SUPPORTED_CHANNEL_COUNT 2    // stereo channels
#define AUDIO_IO_BEGIN_OF_STREAM          0x01 // beginning of stream sent to session when first packet is received 
#define AUDIO_IO_RESTART_REQUIRED         0x40 // restart of USB node is required , after frequency changes for examples 
#define AUDIO_IO_THRESHOLD_REACHED        0x08 // this flag is set when main circular audio buffer fill threshold is reached. Then, consumer node starts reading from the buffer. This is to avoid overrun and underrun in the beginning of streaming 

// exported types

typedef struct AUDIO_USBInputSpecificParams
{
    uint16_t threshold; // after starting playback, usb input node starts receiving packets and writing them in the audio circular buffer. When written data size reaches this threshold, it raises an event to playback session
}
AUDIO_USBInputSpecificParams_t;

typedef struct AUDIO_USBOutputSpecificParams {
  uint8_t* alt_buff; // zero filled buffer, to send to the host when required data not ready
  #if USB_AUDIO_CONFIG_RECORD_USE_FREQ_44_1_K
    uint8_t packet_44_counter;  /* counter to send 9 packets of 44 samples and tenth packet with 45 samples*/
  #endif // USB_AUDIO_CONFIG_RECORD_FREQ_44_1_K
}
AUDIO_USBOutputSpecificParams_t;

typedef struct AUDIO_USBInputOutputNode
{
  AUDIO_Node_t            node;               // generic node structure , must be the first field 
  uint8_t                 flags;              // flags for USB input/output status 
  AUDIO_CircularBuffer_t* buf;                // Audio circular buffer
  uint16_t                max_packet_length;  // the packet to read each time from buffer 
  uint16_t                packet_length;      // the packet normal length 
  int8_t                  (*IODeInit)   (uint32_t /*node_handle*/);
  int8_t                  (*IOStart)    (AUDIO_CircularBuffer_t* buffer, uint16_t threshold, uint32_t /*node handle*/);
  int8_t                  (*IORestart)  (uint32_t /*node handle*/);
  int8_t                  (*IOStop)     (uint32_t /*node handle*/);
  union
  {
    AUDIO_USBInputSpecificParams_t  input;
    AUDIO_USBOutputSpecificParams_t output;    
  } specific; 
}
AUDIO_USBInputOutputNode_t;

typedef struct AUDIO_USBFeatureUnitCommands
{
  int8_t    (*SetMute)            (uint16_t /*channel_number*/, uint8_t /*mute*/, uint32_t /*private_data*/);
  int8_t    (*SetCurrentVolume)   (uint16_t /*channel_number*/, int /*volume_db_256 */, uint32_t /*private_data*/);
  uint32_t  private_data;
} 
AUDIO_USBFeatureUnitCommands_t;

typedef struct AUDIO_USBFeatureUnitDefaults
{
  int                       max_volume;         // volume max on USB format 
  int                       min_volume;         // volume min on USB format 
  int                       res_volume;         // volume resolution on USB format 
  AUDIO_Description_t       *audio_description; // audio description (frequency , resolution , ...) 
} 
AUDIO_USBFeatureUnitDefaults_t;

// USB feature unit node structure
typedef struct
{
  AUDIO_Node_t                              node;                   // generic node structure , must be first field 
  uint8_t                                   unit_id;                // UNIT ID for usb audio function description and control
  USBD_AUDIO_FeatureControlCallbacks_t      usb_control_callbacks;  // list of callbacks 
  AUDIO_USBFeatureUnitCommands_t            control_cbks;
  int8_t                                    (*CFInit)     (USBD_AUDIO_Control_t* /*control*/, AUDIO_USBFeatureUnitDefaults_t* /*audio_defaults*/, uint8_t /*unit_id*/, uint32_t /*node_handle*/);
  int8_t                                    (*CFDeInit)   (uint32_t /*node_handle*/);
  int8_t                                    (*CFStart)    (AUDIO_USBFeatureUnitCommands_t* /* commands */, uint32_t /*node handle*/);
  int8_t                                    (*CFStop)     (uint32_t /*node handle*/);
  int8_t                                    (*CFSetMute)  (uint16_t /*channel*/,uint8_t /*mute*/, uint32_t /* node handle*/);
}
AUDIO_USB_CF_NodeTypeDef;

// exported macros
#define VOLUME_USB_TO_DB_256(v_db, v_usb) (v_db)  = (v_usb <= 0x7FFF) ? v_usb : -(((int)0xFFFF - v_usb)+1)
#define VOLUME_DB_256_TO_USB(v_usb, v_db) (v_usb) = (v_db >= 0)       ? v_db  : ((int)0xFFFF+v_db) +1   
#define AUDIO_MAX_PACKET_WITH_FEEDBACK_LENGTH(audio_desc) AUDIO_USB_MAX_PACKET_SIZE((audio_desc)->frequency + 1, (audio_desc)->channels_count, (audio_desc)->resolution)

int8_t  USB_AudioStreamingInputInit(USBD_AUDIO_EP_Data_t* data_ep, AUDIO_Description_t* audio_desc, AUDIO_Session_t* session_handle,  uint32_t node_handle);
int8_t  USB_AudioStreamingFeatureUnitInit(USBD_AUDIO_Control_t* usb_control_feature, AUDIO_USBFeatureUnitDefaults_t* audio_defaults, uint8_t unit_id, uint32_t node_handle);
void    USB_AudioStreamingInitializeDataBuffer(AUDIO_CircularBuffer_t* buf, uint32_t buffer_size, uint16_t packet_size, uint16_t margin);

#endif // __AUDIO_USB_NODES_H