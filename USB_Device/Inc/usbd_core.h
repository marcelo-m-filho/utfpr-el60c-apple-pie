/**
  ******************************************************************************
  * @file    usbd_core.h
  * @author  MCD Application Team
  * @brief   Header file for usbd_core.c file
  * @version horoscope 0.1
  ******************************************************************************
  */

// define to prevent recursive inclusion ---------------------------------------
#ifndef __USBD_CORE_H
#define __USBD_CORE_H

#ifdef __cplusplus
 extern "C" {
#endif

// Includes --------------------------------------------------------------------
#include "usbd_conf.h"
#include "usbd_def.h"
#include "usbd_ioreq.h"
#include "usbd_ctlreq.h"

// exported defines ------------------------------------------------------------
#ifndef USBD_DEBUG_LEVEL
  #define USBD_DEBUG_LEVEL 0U
#endif // USBD_DEBUG_LEVEL 

#define USBD_SOF USBD_LL_SOF

// exported function prototypes ------------------------------------------------
USBD_Status_t USBD_Init                (USBD_Handles_t *pdev, USBD_Descriptors_t *pdesc, uint8_t id);
USBD_Status_t USBD_DeInit              (USBD_Handles_t *pdev);
USBD_Status_t USBD_Start               (USBD_Handles_t *pdev);
USBD_Status_t USBD_Stop                (USBD_Handles_t *pdev);
USBD_Status_t USBD_RegisterClass       (USBD_Handles_t *pdev, USBD_Class_t *pclass);

USBD_Status_t USBD_RunTestMode         (USBD_Handles_t *pdev);
USBD_Status_t USBD_SetClassConfig      (USBD_Handles_t *pdev, uint8_t cfgidx);
USBD_Status_t USBD_ClrClassConfig      (USBD_Handles_t *pdev, uint8_t cfgidx);

USBD_Status_t USBD_LL_SetupStage       (USBD_Handles_t *pdev, uint8_t *psetup);
USBD_Status_t USBD_LL_DataOutStage     (USBD_Handles_t *pdev , uint8_t epnum, uint8_t *pdata);
USBD_Status_t USBD_LL_DataInStage      (USBD_Handles_t *pdev , uint8_t epnum, uint8_t *pdata);

USBD_Status_t USBD_LL_Reset            (USBD_Handles_t *pdev);
USBD_Status_t USBD_LL_SetSpeed         (USBD_Handles_t *pdev, USBD_Speed_t speed);
USBD_Status_t USBD_LL_Suspend          (USBD_Handles_t *pdev);
USBD_Status_t USBD_LL_Resume           (USBD_Handles_t *pdev);

USBD_Status_t USBD_LL_SOF              (USBD_Handles_t *pdev);
USBD_Status_t USBD_LL_IsoINIncomplete  (USBD_Handles_t *pdev, uint8_t epnum);
USBD_Status_t USBD_LL_IsoOUTIncomplete (USBD_Handles_t *pdev, uint8_t epnum);

USBD_Status_t USBD_LL_DevConnected     (USBD_Handles_t  *pdev);
USBD_Status_t USBD_LL_DevDisconnected  (USBD_Handles_t  *pdev);

// USBD Low Level Driver 
USBD_Status_t USBD_LL_Init            (USBD_Handles_t *pdev);
USBD_Status_t USBD_LL_DeInit          (USBD_Handles_t *pdev);
USBD_Status_t USBD_LL_Start           (USBD_Handles_t *pdev);
USBD_Status_t USBD_LL_Stop            (USBD_Handles_t *pdev);
USBD_Status_t USBD_LL_OpenEP          (USBD_Handles_t *pdev, uint8_t  ep_addr, uint8_t  ep_type, uint16_t ep_mps);

USBD_Status_t USBD_LL_CloseEP         (USBD_Handles_t *pdev, uint8_t ep_addr);
USBD_Status_t USBD_LL_FlushEP         (USBD_Handles_t *pdev, uint8_t ep_addr);
USBD_Status_t USBD_LL_StallEP         (USBD_Handles_t *pdev, uint8_t ep_addr);
USBD_Status_t USBD_LL_ClearStallEP    (USBD_Handles_t *pdev, uint8_t ep_addr);
uint8_t       USBD_LL_IsStallEP       (USBD_Handles_t *pdev, uint8_t ep_addr);
USBD_Status_t USBD_LL_SetUSBAddress   (USBD_Handles_t *pdev, uint8_t dev_addr);
USBD_Status_t USBD_LL_Transmit        (USBD_Handles_t *pdev, uint8_t  ep_addr, uint8_t  *pbuf, uint16_t size);

USBD_Status_t USBD_LL_PrepareReceive  (USBD_Handles_t *pdev, uint8_t  ep_addr, uint8_t  *pbuf, uint16_t size);

uint32_t      USBD_LL_GetRxDataSize   (USBD_Handles_t *pdev, uint8_t  ep_addr);
void          USBD_LL_Delay (uint32_t Delay);


#ifdef __cplusplus
}
#endif

#endif // __USBD_CORE_H 



