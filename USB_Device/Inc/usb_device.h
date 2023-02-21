/**
  ******************************************************************************
  * @file           : usb_device.h
  * @version        : v1.0_Cube / horoscope 0.1
  * @brief          : Header for usb_device.c file.
  ******************************************************************************
  */

// define to prevent recursive inclusion
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
 extern "C" {
#endif

// includes
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "usbd_def.h"

/** USB Device initialization function. */
void USB_Device_Init(void);

#ifdef __cplusplus
}
#endif

#endif // __USB_DEVICE__H__