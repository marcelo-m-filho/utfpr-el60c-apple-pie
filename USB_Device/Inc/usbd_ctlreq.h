/**
  ******************************************************************************
  * @file    usbd_req.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_req.c file
  ******************************************************************************
  */

// Define to prevent recursive inclusion -------------------------------------
#ifndef __USB_REQUEST_H
#define __USB_REQUEST_H

#ifdef __cplusplus
 extern "C" {
#endif

// Includes ------------------------------------------------------------------
#include  "usbd_def.h"

USBD_Status_t USBD_StdDevReq          (USBD_Handles_t *pdev, USBD_SetupReq_t *req);
USBD_Status_t USBD_StdItfReq          (USBD_Handles_t *pdev, USBD_SetupReq_t *req);
USBD_Status_t USBD_StdEPReq           (USBD_Handles_t *pdev, USBD_SetupReq_t *req);
void          USBD_CtlError           (USBD_Handles_t *pdev, USBD_SetupReq_t *req);
void          USBD_ParseSetupRequest  (USBD_SetupReq_t *req, uint8_t *pdata);
void          USBD_GetString          (uint8_t *desc, uint8_t *unicode, uint16_t *len);

#ifdef __cplusplus
}
#endif

#endif // __USB_REQUEST_H 
