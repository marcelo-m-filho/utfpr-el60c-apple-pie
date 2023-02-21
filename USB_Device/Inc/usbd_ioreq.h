/**
  ******************************************************************************
  * @file    usbd_ioreq.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_ioreq.c file
  * @version horoscope 0.1
  ******************************************************************************
  */

// Define to prevent recursive inclusion -------------------------------------
#ifndef __USBD_IOREQ_H
#define __USBD_IOREQ_H

#ifdef __cplusplus
 extern "C" {
#endif

// Includes ------------------------------------------------------------------
#include  "usbd_def.h"
#include  "usbd_core.h"

USBD_Status_t USBD_CtlSendData         (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len);

USBD_Status_t USBD_CtlContinueSendData (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len);

USBD_Status_t USBD_CtlPrepareRx        (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len);

USBD_Status_t USBD_CtlContinueRx       (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len);

USBD_Status_t USBD_CtlSendStatus       (USBD_Handles_t *pdev);

USBD_Status_t USBD_CtlReceiveStatus    (USBD_Handles_t *pdev);

uint32_t      USBD_GetRxCount          (USBD_Handles_t *pdev, uint8_t ep_addr);


#ifdef __cplusplus
}
#endif

#endif // __USBD_IOREQ_H 
