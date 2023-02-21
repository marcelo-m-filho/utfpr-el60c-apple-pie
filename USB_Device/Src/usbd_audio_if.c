/**
  ******************************************************************************
  * @file    usbd_audio_if.c
  * @author  MCD Application Team 
  * @brief   USB Device Audio interface file.
  * @version horoscope 0.1
  ******************************************************************************
  */

// Includes --------------------------------------------------------------------
#include "usbd_audio.h"
#include "usb_audio_app_config.h"
#include "audio_sessions_usb.h"
#include "usbd_audio_if.h"

// Private function prototypes -------------------------------------------------
static int8_t  AUDIO_USB_Init(USBD_AUDIO_FunctionDescriptionf_t* usb_audio_class_function, uint32_t private_data);
static int8_t  AUDIO_USB_DeInit(USBD_AUDIO_FunctionDescriptionf_t* audio_function, uint32_t private_data);
static int8_t  AUDIO_USB_GetState(uint32_t private_data);
static int8_t  AUDIO_USB_GetConfigDesc (uint8_t ** pdata, uint16_t * psize, uint32_t private_data);

// Exported variables ----------------------------------------------------------

 USBD_AUDIO_InterfaceCallbacksf_t audio_class_interface =
 {
   .Init          = AUDIO_USB_Init,
   .DeInit        = AUDIO_USB_DeInit,
   .GetConfigDesc = AUDIO_USB_GetConfigDesc,
   .GetState      = AUDIO_USB_GetState,
   .private_data  = 0 
 };

 // list of used sessions 
  AUDIO_USBSession_t USB_AudioPlaybackSession;
 
 // private  functions ---------------------------------------------------------
 
/**
  * @brief  Initializes the USB Audio streaming application. It provides information(endpoint number, supported controls, ...) and callbacks to the USB Audio Class
  * @param  usb_audio_class_function(OUT): description and callback about the audio function , like list of endpoints, see structure definition for more information
  * @param  private_data:  for future usage
  * @retval status
  */
static int8_t AUDIO_USB_Init(USBD_AUDIO_FunctionDescriptionf_t* usb_audio_class_function , uint32_t private_data)
{
  int interface_offset    = 0;
  int total_control_count = 0;
  uint8_t control_count   = 0;


  // initializes the USB play session 
  AUDIO_PlaybackSessionInit(&usb_audio_class_function->as_interfaces[interface_offset], &(usb_audio_class_function->controls[interface_offset]), &control_count, (uint32_t)&USB_AudioPlaybackSession);
  interface_offset++;
  total_control_count += control_count;

  usb_audio_class_function->as_interfaces_count = interface_offset;
  usb_audio_class_function->control_count       = total_control_count;

  #if USE_AUDIO_USB_INTERRUPT
    usb_audio_class_function->interrupt_ep_num = USB_AUDIO_CONFIG_INTERRUPT_EP_IN;
  #endif // USE_AUDIO_USB_INTERRUPT 

  return 0;
}

/**
  * @brief  De-Initializes the interface
  * @param  audio_function: The audio function description
  * @param  private_data:  for future usage
  * @retval status 0 if no error
  */

static int8_t AUDIO_USB_DeInit(USBD_AUDIO_FunctionDescriptionf_t* audio_function, uint32_t private_data)
{
  USB_AudioPlaybackSession.SessionDeInit((uint32_t)&USB_AudioPlaybackSession);
  audio_function->as_interfaces[0].alternate = 0;
  
  return 0;
}

/**
  * @brief  Returns the USB Audio state
  * @param  private_data:  for future usage
  * @retval status
  */
static int8_t AUDIO_USB_GetState(uint32_t private_data)
{
  return 0;
}

/**
  * @brief  Initializes the interface (?)
  * @param  pdata: the returned configuration descriptor 
  * @param  psize: configuration descriptor length
  * @param  private_data: for future usage
  * @retval status
  */
static int8_t AUDIO_USB_GetConfigDesc(uint8_t** pdata, uint16_t* psize, uint32_t private_data)
{
  *psize = USB_AUDIO_GetConfigDescriptor(pdata);
  return 0;
}

#if USE_AUDIO_USB_INTERRUPT
/**
  * @brief  USBD_AUDIO_ExecuteControl
  *         execute a control which isn't triggered by USB request. for example a control triggered by a user action(like user button)
  * @param  func(IN): bitwise field of USBD_AUDIO_PLAYBACK and  USBD_AUDIO_RECORD
  * @param  control(IN):  control type
  * @param  val(IN):  new value
  * @param  private_data(IN):  for future usage
  * @retval status : 0 if no error
  */
int8_t USBD_AUDIO_ExecuteControl( uint8_t func, AUDIO_ControlCommand_t control , uint32_t val , uint32_t private_data)
{
#if USE_USB_AUDIO_PLAYBACK
  if((func&USBD_AUDIO_PLAYBACK)&&((USB_AudioPlaybackSession.session.state != AUDIO_SESSION_OFF)&&
                                  (USB_AudioPlaybackSession.session.state != AUDIO_SESSION_ERROR)))
  {
   USB_AudioPlaybackSession.ExternalControl(control, val, (uint32_t) &USB_AudioPlaybackSession);
  }
#endif //  USE_USB_AUDIO_PLAYBACK 
#if  USE_USB_AUDIO_RECORDING
  if((func&USBD_AUDIO_RECORD)&&((USB_AudioRecordingSession.session.state != AUDIO_SESSION_OFF)&&
                                  (USB_AudioRecordingSession.session.state != AUDIO_SESSION_ERROR)))
  {
   USB_AudioRecordingSession.ExternalControl(control, val, (uint32_t) &USB_AudioRecordingSession);
  }
#endif //  USE_USB_AUDIO_RECORDING 
  return 0;
}
#endif // USE_AUDIO_USB_INTERRUPT
//*********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
