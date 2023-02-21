/**
  ******************************************************************************
  * @file    usbd_ioreq.c
  * @author  MCD Application Team
  * @brief   This file provides the IO requests APIs for control endpoints.
  ******************************************************************************
  */

// Includes ------------------------------------------------------------------
#include "usbd_ioreq.h"


// private functions -----------------------------------------------------------

/**
* @brief  send data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be sent
* @retval status
*/
USBD_Status_t USBD_CtlSendData (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len)
{
  // set EP0 State 
  pdev->ep0_state             = USBD_EP0_DATA_IN;
  pdev->ep_in[0].total_length = len;
  pdev->ep_in[0].rem_length   = len;

  // start the transfer 
  USBD_LL_Transmit (pdev, 0x00U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  continue sending data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be sent
* @retval status
*/
USBD_Status_t USBD_CtlContinueSendData (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len)
{
 // Start the next transfer 
  USBD_LL_Transmit (pdev, 0x00U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  receive data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be received
* @retval status
*/
USBD_Status_t USBD_CtlPrepareRx (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len)
{
  // Set EP0 State 
  pdev->ep0_state               = USBD_EP0_DATA_OUT;
  pdev->ep_out[0].total_length  = len;
  pdev->ep_out[0].rem_length    = len;

  // Start the transfer 
  USBD_LL_PrepareReceive (pdev, 0U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  continue receive data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be received
* @retval status
*/
USBD_Status_t USBD_CtlContinueRx (USBD_Handles_t *pdev, uint8_t *pbuf, uint16_t len)
{
  USBD_LL_PrepareReceive(pdev, 0U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  send zero length packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
USBD_Status_t USBD_CtlSendStatus (USBD_Handles_t *pdev)
{
  // Set EP0 State 
  pdev->ep0_state = USBD_EP0_STATUS_IN;

  // Start the transfer 
  USBD_LL_Transmit(pdev, 0x00U, NULL, 0U);

  return USBD_OK;
}

/**
* @brief  receive zero lzngth packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
USBD_Status_t USBD_CtlReceiveStatus (USBD_Handles_t *pdev)
{
  // Set EP0 State 
  pdev->ep0_state = USBD_EP0_STATUS_OUT;

 // Start the transfer 
  USBD_LL_PrepareReceive (pdev, 0U, NULL, 0U);

  return USBD_OK;
}

/**
* @brief  returns the received data length
* @param  pdev: device instance
* @param  ep_addr: endpoint address
* @retval Rx Data blength
*/
uint32_t USBD_GetRxCount (USBD_Handles_t *pdev, uint8_t ep_addr)
{
  return USBD_LL_GetRxDataSize(pdev, ep_addr);
}
