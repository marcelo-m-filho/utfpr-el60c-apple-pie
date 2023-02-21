/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube / horoscope 0.1
  * @brief          : USB Device implementation
  ******************************************************************************
  */

// includes --------------------------------------------------------------------
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_audio.h"
#include "usbd_audio_if.h"

// USB Device Core handle declaration. 
USBD_Handles_t usb_device_handle;

extern void Error_Handler(void);

/**
  * @brief Inits USB device Library; adds supported classes and starts the library
  * @retval None
  */
void USB_Device_Init(void)
{
  // inits device library
  if (USBD_Init(&usb_device_handle, &AUDIO_Desc, 0) != USBD_OK)
  {
    Error_Handler();
  }

  // adds supported classes
  if (USBD_RegisterClass(&usb_device_handle, &USBD_AUDIO) != USBD_OK)
  {
    Error_Handler();
  }

  if (USBD_AUDIO_RegisterInterface(&usb_device_handle, &audio_class_interface) != USBD_OK)
  {
    Error_Handler();
  }

  // starts the library
  if (USBD_Start(&usb_device_handle) != USBD_OK)
  {
    Error_Handler();
  }

}
