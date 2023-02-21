/**
  ******************************************************************************
  * @file    audio_sessions_usb.h
  * @author  MCD Application Team 
  * @brief   header file for the audio_session_usb.c file. (?)
  * @version horoscope 0.1
  ******************************************************************************
  */


// define to prevent recursive inclusion
#ifndef __AUDIO_SESSIONS_USB_H
#define __AUDIO_SESSIONS_USB_H

// includes
#include "audio_node.h"
#include "audio_usb_nodes.h"

// exported types

#if USE_AUDIO_USB_INTERRUPT
    
    // list of commands that may be applied locally
    typedef enum AUDIO_ControlCommand
    {
      USBD_AUDIO_MUTE_UNMUTE,
      USBD_AUDIO_VOLUME                                                     
    }
    AUDIO_ControlCommand_t;

#endif // USE_AUDIO_USB_INTERRUPT

// may be instantiated as recording session or playback session
typedef struct AUDIO_USBSession
{
  AUDIO_Session_t         session; // the session structure
  int8_t                  (*SessionDeInit) (uint32_t /*session handle*/);
  #if USE_AUDIO_USB_INTERRUPT
    int8_t               (*ExternalControl)(AUDIO_ControlCommand_t /*control*/, uint32_t /*val*/, uint32_t/*  session_handle*/);/* function that may be called locally to execute some commands like set volume */
  #endif //USE_AUDIO_USB_INTERRUPT
  uint8_t                 interface_num;  // USB interface number
  uint8_t                 alternate;      // current alternate setting
  AUDIO_CircularBuffer_t  buffer;         // audio circular buffer 
}
AUDIO_USBSession_t;

 int8_t  AUDIO_PlaybackSessionInit(USBD_AUDIO_Streaming_interface_t* as_desc, USBD_AUDIO_Control_t* controls_desc, uint8_t* control_count, uint32_t session_handle);


#endif // __AUDIO_SESSIONS_USB_H