/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team 
  * @brief   This file provides the Audio core functions.
  * @version horoscope 0.1
  * 
  * @verbatim
  *      
  *          ===================================================================      
  *                                AUDIO Class  Description
  *          ===================================================================
  *           This driver manages the Audio Class 1.0 following the 
  *           "USB Device Class Definition for Audio Devices V1.0 Mar 18, 98".
  *           It is a new implementation of the USB audio class which supports more features.
  *           This driver implements the following aspects of the specification:
  *             - Standard AC Interface Descriptor management
  *             - 2 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *           
  *
  ******************************************************************************
  */ 

// Includes ------------------------------------------------------------------
#include "usbd_audio.h"
#include "usbd_ctlreq.h"

// private type definitions ----------------------------------------------------
typedef enum USBD_AUDIO_EpUsage
{
  USBD_AUDIO_DATA_EP,
  USBD_AUDIO_FEEDBACK_EP,
  USBD_AUDIO_INTERRUPT_EP
}
USBD_AUDIO_EpUsage_t;

// Structure define ep:  description and state 
typedef struct USBD_AUDIO_EP
{
  union
  {
    USBD_AUDIO_EP_Data_t* data_ep;
    #if USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
      USBD_AUDIO_EP_SynchTypeDef* sync_ep;
    #endif // USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
  } ep_description;
  USBD_AUDIO_EpUsage_t    ep_type;
  uint8_t                 open;               // 0: closed, 1: open 
  uint16_t                max_packet_length;  // the max packet length 
  uint16_t                tx_rx_soffn;
}
USBD_AUDIO_EP_t;

// audio class data
typedef struct USBD_AUDIO_Handle
{
  USBD_AUDIO_FunctionDescriptionf_t aud_function; // description of audio function 
  USBD_AUDIO_EP_t ep_in[USBD_AUDIO_MAX_IN_EP]; //  list of IN EP 
  USBD_AUDIO_EP_t ep_out[USBD_AUDIO_MAX_OUT_EP]; //  list of OUT EP  

  // handle control
  struct USBD_AUDIO_HandleControl
  {
    union USBD_AUDIO_Entity
    {
      USBD_AUDIO_Control_t  *controller;            // related control unit 
      #if USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
        USBD_AUDIO_EP_DataTypeDef* data_ep;         // related Data End point 
      #endif // USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES 
    } entity;
    uint8_t                 request_target;
    uint8_t                 data[USB_MAX_EP0_SIZE]; // buffer to receive request value or send response 
    uint32_t                len;                    // used length of data buffer 
    uint16_t                wValue;                 // wValue of request which is specific for each control
    uint8_t                 req;                    // the request type specific for each unit
  } last_control;
}
USBD_AUDIO_Handle_t;

// private defines ------------------------------------------------------------
#define AUDIO_UNIT_CONTROL_REQUEST            0x01
#define AUDIO_EP_REQUEST                      0x02
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
  #define USBD_AUDIO_SOF_COUNT_FEEDBACK_BITS  7
  #define USBD_AUDIO_SOF_COUNT_FEEDBACK       (1 << USBD_AUDIO_SOF_COUNT_FEEDBACK_BITS)
#endif // USBD_SUPPORT_AUDIO_OUT_FEEDBACK 

// private function prototypes
static uint8_t USBD_AUDIO_Init                    (USBD_Handles_t *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit                  (USBD_Handles_t *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_Setup                   (USBD_Handles_t *pdev, USBD_SetupReq_t *req);
static uint8_t *USBD_AUDIO_GetCfgDesc             (uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length);
static uint8_t USBD_AUDIO_DataIn                  (USBD_Handles_t *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut                 (USBD_Handles_t *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady             (USBD_Handles_t *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady             (USBD_Handles_t *pdev);
static uint8_t USBD_AUDIO_SOF                     (USBD_Handles_t *pdev);
static uint8_t USBD_AUDIO_IsoINIncomplete         (USBD_Handles_t *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete        (USBD_Handles_t *pdev, uint8_t epnum);
static uint8_t AUDIO_REQ                          (USBD_Handles_t *pdev, USBD_SetupReq_t *req);

#if USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
  static uint8_t AUDIO_EP_REQ(USBD_Handles_t *pdev, USBD_SetupReqTypedef *req);
#endif // USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES

#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK
  static  unsigned get_usb_full_speed_rate(unsigned int rate, unsigned char * buf);
#endif // USBD_SUPPORT_AUDIO_OUT_FEEDBACK 

static uint8_t USBD_AUDIO_SetInterfaceAlternate(USBD_Handles_t *pdev, uint8_t as_interface_num, uint8_t new_alt);


// private variables ----------------------------------------------------------

USBD_Class_t USBD_AUDIO = 
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,  
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,      
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc, 
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

// USB Standard Device Descriptor 
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

static uint8_t *USBD_AUDIO_CfgDesc      = 0;
static uint16_t USBD_AUDIO_CfgDescSize  = 0;


// private functions ----------------------------------------------------------

/**
  * @brief  Initializes the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index , not used
  * @retval status
  */
static uint8_t USBD_AUDIO_Init (USBD_Handles_t *pdev, uint8_t cfgidx)
{
  // Allocate Audio structure 
  USBD_AUDIO_Handle_t*              audio_handle;
  USBD_AUDIO_InterfaceCallbacksf_t* audio_interface_callbacks;
  
  audio_handle = USBD_malloc(sizeof (USBD_AUDIO_Handle_t));
  if(audio_handle == NULL)
  {
    return USBD_FAIL; 
  }
  else
  {
    memset(audio_handle, 0, sizeof(USBD_AUDIO_Handle_t));
    audio_interface_callbacks = (USBD_AUDIO_InterfaceCallbacksf_t *)pdev->pUserData;

    // initializes the audio output hardware layer 
    if (audio_interface_callbacks->Init(&audio_handle->aud_function, audio_interface_callbacks->private_data) != USBD_OK)
    {
      USBD_free(pdev->pClassData);
      pdev->pClassData = 0;
      return USBD_FAIL;
    }
  }
  pdev->pClassData = audio_handle;
  return USBD_OK;
}

/**
  * @brief  Deinitializes the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index, not used 
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit (USBD_Handles_t *pdev, uint8_t cfgidx)
{
    USBD_AUDIO_Handle_t*              audio_handle;
    USBD_AUDIO_InterfaceCallbacksf_t* audio_interface_callbacks;
    
    audio_handle              = (USBD_AUDIO_Handle_t*)              pdev->pClassData;
    audio_interface_callbacks = (USBD_AUDIO_InterfaceCallbacksf_t*) pdev->pUserData;
    
    // Close open EP 
    for(int i = 1; i < USBD_AUDIO_MAX_IN_EP; i++)
    {
      if(audio_handle->ep_in[i].open)
      {
        USBD_LL_CloseEP(pdev, i | 0x80);
        audio_handle->ep_in[i].open = 0;
      }
    }

    for(int i = 1; i < USBD_AUDIO_MAX_OUT_EP; i++)
    {
      if(audio_handle->ep_out[i].open)
      {
        USBD_LL_CloseEP(pdev, i);
        audio_handle->ep_out[i].open = 0;
      }
    }

  // deinitialize physical interface components 
  if(audio_handle != NULL)
  {
    audio_interface_callbacks->DeInit(&audio_handle->aud_function, audio_interface_callbacks->private_data);
    USBD_free(audio_handle);
    pdev->pClassData = NULL;
  }
  
  return USBD_OK;
}

/**
  * @brief  Sets the Alternate interface of a streaming interface
  * @param  pdev: device instance
  * @param  as_interface_num: audio streaming interface number
  * @param  new_alt: new alternate number
  * @retval status
  */
static uint8_t USBD_AUDIO_SetInterfaceAlternate(USBD_Handles_t *pdev, uint8_t as_interface_num, uint8_t new_alt)
{
  USBD_AUDIO_Handle_t*        audio_handle;
  USBD_AUDIO_Streaming_interface_t*  pas_interface;
  USBD_AUDIO_EP_t*            ep;
  
  audio_handle  = (USBD_AUDIO_Handle_t*) pdev->pClassData;
  pas_interface = &audio_handle->aud_function.as_interfaces[as_interface_num];
  ep            = (pas_interface->data_ep.ep_num & 0x80) ? &audio_handle->ep_in[pas_interface->data_ep.ep_num & 0x0F] : &audio_handle->ep_out[pas_interface->data_ep.ep_num];
  
  
  // close old alternate interface 
  if(new_alt == 0)
  {
    // close all opened ep 
    if (pas_interface->alternate != 0)
    {
      // @TODO : Close related End Points 
      if(ep->open)
      {
        USBD_LL_CloseEP(pdev, ep->ep_description.data_ep->ep_num);
        ep->open = 0;
      }
      #if USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
        if(pas_interface->synch_enabled)
        {
          // close synch ep 
            ep=&audio_handle->ep_in[pas_interface->synch_ep.ep_num&0x0F];
            if(ep->open)
            {
              USBD_LL_CloseEP(pdev, ep->ep_description.sync_ep->ep_num);
              ep->open = 0;
            }
        }
        #endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
    }
    pas_interface->SetAS_Alternate(new_alt, pas_interface->private_data);
    pas_interface->alternate=0;
  }
  // start new  alternate interface 
  else
  {
    // prepare EP 
    ep->ep_description.data_ep = &pas_interface->data_ep;
    
    // open the data ep 
    pas_interface->SetAS_Alternate(new_alt, pas_interface->private_data);
    pas_interface->alternate  = new_alt;
    ep->max_packet_length     = ep->ep_description.data_ep->GetMaxPacketLength(ep->ep_description.data_ep->private_data);
    
    // open data end point 
    USBD_LL_OpenEP(pdev, ep->ep_description.data_ep->ep_num, USBD_EP_TYPE_ISOC, ep->max_packet_length);       
    
    ep->open = 1;
     
    // get usb working buffer  
    ep->ep_description.data_ep->buf = ep->ep_description.data_ep->GetBuffer(ep->ep_description.data_ep->private_data, &ep->ep_description.data_ep->length);        
    
    if(ep->ep_description.data_ep->ep_num & 0x80)  // IN EP 
    {
      USBD_LL_FlushEP(pdev, ep->ep_description.data_ep->ep_num);
      ep->tx_rx_soffn = USB_SOF_NUMBER();
      USBD_LL_Transmit(pdev,  ep->ep_description.data_ep->ep_num, ep->ep_description.data_ep->buf, ep->ep_description.data_ep->length);
    }
    else// OUT EP 
    {
      #if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
        uint32_t rate;
      #endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK   

    // Prepare Out endpoint to receive 1st packet  
    USBD_LL_PrepareReceive(pdev, ep->ep_description.data_ep->ep_num, ep->ep_description.data_ep->buf, ep->max_packet_length); 
        
    #if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
      if(pas_interface->synch_enabled)
        {
          USBD_AUDIO_EP_SynchTypeDef* sync_ep; // synchro ep description 
          ep = &audio_handle->ep_in[pas_interface->synch_ep.ep_num&0x0F];
          sync_ep = &pas_interface->synch_ep;
          ep->ep_description.sync_ep = sync_ep;
          ep->max_packet_length = AUDIO_FEEDBACK_EP_PACKET_SIZE;
          ep->ep_type = USBD_AUDIO_FEEDBACK_EP;
          // open synchro ep 
          USBD_LL_OpenEP(pdev, sync_ep->ep_num, USBD_EP_TYPE_ISOC, ep->max_packet_length);             
            ep->open = 1;
            rate = sync_ep->GetFeedback(sync_ep->private_data);
            get_usb_full_speed_rate(rate,sync_ep->feedback_data);
            ep->tx_rx_soffn = USB_SOF_NUMBER();
            USBD_LL_Transmit(pdev, sync_ep->ep_num, sync_ep->feedback_data, ep->max_packet_length);
        }
    #endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK    
    }
  }
  return USBD_OK;
}

#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK
  /**
    * @brief   get_usb_full_speed_rate
    *         Set feedback value from rate 
    * @param  rate: 
    * @param  buf: 
    * @retval 
    */
  static  unsigned get_usb_full_speed_rate(unsigned int rate, unsigned char * buf)
  {
          uint32_t freq =  ((rate << 13) + 62) / 125;
          buf[0] =    freq>> 2;
          buf[1] =    freq>> 10;
          buf[2] =    freq>> 18;
  return 0;
  }
#endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK  

/**
  * @brief  Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_AUDIO_Setup (USBD_Handles_t *pdev, USBD_SetupReq_t *usbd_setup_request)
{
  USBD_AUDIO_Handle_t*  audio_handle;
  uint16_t              length;
  uint8_t               *buffer_p;
  uint8_t               return_value = USBD_OK;

  audio_handle = (USBD_AUDIO_Handle_t*) pdev->pClassData;
  
  switch (usbd_setup_request->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:  
    if((usbd_setup_request->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE)
    {
      switch (usbd_setup_request->bRequest)
      {
        case USBD_AUDIO_REQ_GET_CUR:
        case USBD_AUDIO_REQ_GET_MIN:
        case USBD_AUDIO_REQ_GET_MAX:
        case USBD_AUDIO_REQ_GET_RES:
        case USBD_AUDIO_REQ_SET_CUR:
          AUDIO_REQ(pdev, usbd_setup_request);
          break;
        default:
          USBD_CtlError(pdev, usbd_setup_request);
          return_value = USBD_FAIL; 
      }
    }
    else
      #if USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
        {

          switch (req->bRequest)
          {
          case USBD_AUDIO_REQ_SET_CUR:
              AUDIO_EP_REQ(pdev, req);
            break;
            
          default:
            USBD_CtlError (pdev, req);
            ret = USBD_FAIL; 
          }
        }
      #else // USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
        {
          USBD_CtlError (pdev, usbd_setup_request);
          return_value = USBD_FAIL;
        }
      #endif //USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (usbd_setup_request->bRequest)
    {
      case USB_REQ_GET_DESCRIPTOR:      
        if( (usbd_setup_request->wValue >> 8) == USBD_AUDIO_DESC_TYPE_CS_DEVICE)
        {
          buffer_p = USBD_AUDIO_CfgDesc + 18;
          length = MIN(USBD_AUDIO_DESC_SIZ, usbd_setup_request->wLength);
          
          
          USBD_CtlSendData (pdev, buffer_p, length);
        }
        break;
        
    case USB_REQ_GET_INTERFACE :
      {
        for(int i = 0;i < audio_handle->aud_function.as_interfaces_count; i++)
        {
            if((uint8_t)(usbd_setup_request->wIndex) == audio_handle->aud_function.as_interfaces[i].interface_num)
            {
              USBD_CtlSendData (pdev, (uint8_t *)&(audio_handle->aud_function.as_interfaces[i].alternate), 1);
              return USBD_OK;
            }
        }
        USBD_CtlError (pdev, usbd_setup_request);
        return_value = USBD_FAIL; 
      }
      break;
      
    case USB_REQ_SET_INTERFACE :
      {
        for(int i=0;i<audio_handle->aud_function.as_interfaces_count;i++)
        {
          if((uint8_t)(usbd_setup_request->wIndex)==audio_handle->aud_function.as_interfaces[i].interface_num)
          {
            if((uint8_t)(usbd_setup_request->wValue)==audio_handle->aud_function.as_interfaces[i].alternate)
            {
              // Nothing to do
              return USBD_OK;
            }
            else
            {               
              //Alternate is changed
              return USBD_AUDIO_SetInterfaceAlternate(pdev,i,(uint8_t)(usbd_setup_request->wValue));
            }
          }
        } 

        
        if(((uint8_t)(usbd_setup_request->wIndex) == 0) && ((uint8_t)(usbd_setup_request->wValue)) == 0)
        {
          // Audio Control Control interface, only alternate zero is accepted       
          return USBD_OK;
        }
          // Call the error management function (command will be nacked 
          USBD_CtlError (pdev, usbd_setup_request);
          return_value = USBD_FAIL; 
      } 
      break;      
      
    default:
      USBD_CtlError (pdev, usbd_setup_request);
      return_value = USBD_FAIL;     
    }
  }
  return return_value;
}


/**
  * @brief  returns configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length)
{
  *length = USBD_AUDIO_CfgDescSize;
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  handles data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DataIn (USBD_Handles_t *pdev, uint8_t endpoint_index)
{
  USBD_AUDIO_EP_t* endpoint;

   endpoint = &((USBD_AUDIO_Handle_t*) pdev->pClassData)->ep_in[endpoint_index & 0x7F];
   if(endpoint->open)
   {
      #if USBD_SUPPORT_AUDIO_OUT_FEEDBACK    
        if(endpoint->ep_type==USBD_AUDIO_DATA_EP)
        {
      #endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
      endpoint->ep_description.data_ep->buf = endpoint->ep_description.data_ep->GetBuffer(endpoint->ep_description.data_ep->private_data, &endpoint->ep_description.data_ep->length);
      endpoint->tx_rx_soffn = USB_SOF_NUMBER();
      USBD_LL_Transmit(pdev, endpoint_index|0x80, endpoint->ep_description.data_ep->buf, endpoint->ep_description.data_ep->length);     
      #if USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
        }
        else
        if(endpoint->ep_type==USBD_AUDIO_FEEDBACK_EP)
        {
          
          uint32_t rate; 
          USBD_AUDIO_EP_SynchTypeDef* sync_ep=endpoint->ep_description.sync_ep;
          rate = sync_ep->GetFeedback(sync_ep->private_data);
          get_usb_full_speed_rate(rate, sync_ep->feedback_data);
          endpoint->tx_rx_soffn = USB_SOF_NUMBER();
          USBD_LL_Transmit(pdev, endpoint_index|0x80, sync_ep->feedback_data, AUDIO_FEEDBACK_EP_PACKET_SIZE);
        }
      #endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK 

    }
   else
   {
     // Should not be reproduced 
     USBD_error_handler();
   }
  
  return USBD_OK;
}

/**
  * @brief  handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_RxReady(USBD_Handles_t *pdev)
{
  USBD_AUDIO_Handle_t*  audio_handle;
  uint16_t*             tmpdata;
  
  audio_handle = (USBD_AUDIO_Handle_t*)pdev->pClassData; 

  if(audio_handle->last_control.req == 0x00)
  {
    // @TODO Manage this error 
    return USBD_OK;
  }

  if(audio_handle->last_control.request_target == AUDIO_UNIT_CONTROL_REQUEST)
  {
    USBD_AUDIO_Control_t *audio_control;
    audio_control = audio_handle->last_control.entity.controller;

    switch(audio_control->type)
    {
      case USBD_AUDIO_CS_AC_SUBTYPE_FEATURE_UNIT:
      {
        uint16_t                              selector        = HIBYTE(audio_handle->last_control.wValue);
        USBD_AUDIO_FeatureControlCallbacks_t* feature_control = audio_control->Callbacks.feature_control;

        switch(selector)
        {
          case USBD_AUDIO_CONTROL_FEATURE_UNIT_MUTE:
          {
            // @TODO treat multi channel case and error when req! of GetCur  
            if(feature_control->SetMute)
            {
              feature_control->SetMute(LOBYTE(audio_handle->last_control.wValue), audio_handle->last_control.data[0], audio_control->private_data);
            }
            break;
          }
          case USBD_AUDIO_CONTROL_FEATURE_UNIT_VOLUME:
          {
            // @TODO check the len uses cases and control req->wLength
            switch(audio_handle->last_control.req)
            {
              case USBD_AUDIO_REQ_SET_CUR:
                if(feature_control->SetCurVolume)
                {
                  tmpdata = (uint16_t*) &(audio_handle->last_control.data);
                  feature_control->SetCurVolume(LOBYTE(audio_handle->last_control.wValue), *tmpdata, audio_control->private_data);
                }
                break;
              default:                              
                USBD_error_handler();
            }
            break;
          }
          default :
            USBD_error_handler();
        }
        break;
      }           
      default: // switch(ctl->type)
        USBD_error_handler();
    }
  }
#if USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
  else
  {
    USBD_AUDIO_EP_DataTypeDef* data_ep = haudio->last_control.entity.data_ep;
    uint16_t selector = HIBYTE(haudio->last_control.wValue);
    if(selector == USBD_AUDIO_CONTROL_EP_SAMPL_FREQ)
    {
      // @TODO check the len uses cases and control req->wLength
        switch(haudio->last_control.req)
        {
          case USBD_AUDIO_REQ_SET_CUR:
            if(data_ep->control_cbk.SetCurFrequency)
            {
              uint8_t restart_interface = 0;
                data_ep->control_cbk.SetCurFrequency(AUDIO_FREQ_FROM_DATA(haudio->last_control.data),
                                                     &restart_interface, data_ep->private_data);
        for(int i=0; i<haudio->aud_function.as_interfaces_count; i++)
        {
            if(data_ep == &haudio->aud_function.as_interfaces[i].data_ep)
            {
 // update sampling rate for syenchronization EP 
#if USBD_SUPPORT_AUDIO_OUT_FEEDBACK
              if(haudio->aud_function.as_interfaces[i].synch_enabled)
              {
                uint32_t rate; 
                rate = haudio->aud_function.as_interfaces[i].synch_ep.GetFeedback(
                                 haudio->aud_function.as_interfaces[i].synch_ep.private_data);
                get_usb_full_speed_rate(rate,haudio->aud_function.as_interfaces[i].synch_ep.feedback_data);
              }
#endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
              if(restart_interface)
              {
                if(haudio->aud_function.as_interfaces[i].alternate != 0)
                {
                  int alt = haudio->aud_function.as_interfaces[i].alternate;
                  USBD_AUDIO_SetInterfaceAlternate(pdev, i, 0);
                  USBD_AUDIO_SetInterfaceAlternate(pdev, i, alt);
                }
              }
              break;
            }
        }

            }
            break;
          default :
            
              USBD_error_handler();
         }
     }
    else
    {
       USBD_error_handler();
    }
  }
#endif // USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES 
  return USBD_OK;
}
/**
  * @brief  handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_Handles_t *pdev)
{
  // Only OUT control data are processed 
  return USBD_OK;
}

/**
  * @brief  handles SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_SOF (USBD_Handles_t *pdev)
{
  USBD_AUDIO_Handle_t *audio_handle;
  audio_handle = (USBD_AUDIO_Handle_t*)pdev->pClassData; 
  
  for(int i = 0; i < audio_handle->aud_function.as_interfaces_count; i++)
  {
      if(audio_handle->aud_function.as_interfaces[i].alternate != 0)
      {
        if(audio_handle->aud_function.as_interfaces[i].SofReceived)
        {
          audio_handle->aud_function.as_interfaces[i].SofReceived(audio_handle->aud_function.as_interfaces[i].private_data);
        }
      }
  }
  return USBD_OK;
}

/**
  * @brief  handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_Handles_t *pdev, uint8_t epnum)
{
  USBD_AUDIO_EP_t*     endpoint;
  USBD_AUDIO_Handle_t* audio_handle;
  uint16_t             current_sof;

  audio_handle = (USBD_AUDIO_Handle_t*)pdev->pClassData;

  // @TODO check if the feedback is responsible of event 
  for(int i = 1; i < USBD_AUDIO_MAX_IN_EP; i++)
  {
    endpoint    = &audio_handle->ep_in[i];
    current_sof = USB_SOF_NUMBER();

    if((endpoint->open) && IS_ISO_IN_INCOMPLETE_EP(i, current_sof, endpoint->tx_rx_soffn))
    {
      epnum = i | 0x80;
      USB_CLEAR_INCOMPLETE_IN_EP(epnum);
      USBD_LL_FlushEP(pdev, epnum);
      endpoint->tx_rx_soffn = USB_SOF_NUMBER();
      #if USBD_SUPPORT_AUDIO_OUT_FEEDBACK  
        if(endpoint->ep_type==USBD_AUDIO_FEEDBACK_EP)
        {
          USBD_LL_Transmit(pdev, epnum, endpoint->ep_description.sync_ep->feedback_data, endpoint->max_packet_length);
          continue;
        }
        else
      #endif //USBD_SUPPORT_AUDIO_OUT_FEEDBACK 
      if(endpoint->ep_type == USBD_AUDIO_DATA_EP)
      {
        USBD_LL_Transmit(pdev, epnum, endpoint->ep_description.data_ep->buf, endpoint->ep_description.data_ep->length);
      }
      else
      {
        USBD_error_handler();
      }
    
    }
  }
  return 0;
}
/**
  * @brief  handles data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_Handles_t *pdev, uint8_t endpoint_index)
{
  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t  USBD_AUDIO_DataOut (USBD_Handles_t *pdev, uint8_t endpoint_index)
{
  
  USBD_AUDIO_EP_t*  endpoint;
  uint8_t*          buffer_p;
  uint16_t          packet_length;


  endpoint = &((USBD_AUDIO_Handle_t*) pdev->pClassData)->ep_out[endpoint_index];

  if(endpoint->open)
  {
    // get received length 
    packet_length = USBD_LL_GetRxDataSize(pdev, endpoint_index);

    // inform user about data reception  
    endpoint->ep_description.data_ep->DataReceived(packet_length, endpoint->ep_description.data_ep->private_data);
     
    // get buffer to receive new packet   
    buffer_p = endpoint->ep_description.data_ep->GetBuffer(endpoint->ep_description.data_ep->private_data, &packet_length);

    // Prepare Out endpoint to receive next audio packet 
    USBD_LL_PrepareReceive(pdev, endpoint_index, buffer_p, packet_length);
  }
  else
  {
    USBD_error_handler();
  }

  return USBD_OK;
}

/**
  * @brief  Handles the Control requests.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static uint8_t AUDIO_REQ(USBD_Handles_t *pdev, USBD_SetupReq_t *control_request)
{  
  USBD_AUDIO_Handle_t*  audio_handle;
  USBD_AUDIO_Control_t* audio_control = 0;
  uint8_t               unit_id,control_selector;
  uint16_t*             tmpdata = NULL;
 
  audio_handle = (USBD_AUDIO_Handle_t*) pdev->pClassData;
  
  // reset last command 
  audio_handle->last_control.req = 0x00;
  
  // get the Unit Id 
  unit_id = HIBYTE(control_request->wIndex);
  
  for (int i = 0; i < audio_handle->aud_function.control_count; i++)
  {
    if(unit_id == audio_handle->aud_function.controls[i].id)
    {
      audio_control = &audio_handle->aud_function.controls[i];
      break;
    }     
  }
  
  if(!audio_control)
  {
    // control not supported 
    USBD_CtlError (pdev, control_request);
    return  USBD_FAIL; 
  }
  
  control_selector = HIBYTE(control_request->wValue);
  
  if((audio_control->control_selector_map & control_selector) == 0)
  {
    // control not supported 
    USBD_CtlError (pdev, control_request);
      return  USBD_FAIL; 
  }
  
  if(!(control_request->bRequest & 0x80))
  {
    // set request 
    // @TODO check the length 
    audio_handle->last_control.wValue            = control_request->wValue;
    audio_handle->last_control.entity.controller = audio_control;
    audio_handle->last_control.request_target    = AUDIO_UNIT_CONTROL_REQUEST;
    audio_handle->last_control.len               = control_request->wLength;
    audio_handle->last_control.req               = control_request->bRequest;

    USBD_CtlPrepareRx(pdev, audio_handle->last_control.data, control_request->wLength);
    return USBD_OK;   
  }
  
  switch(audio_control->type)
  {
    case USBD_AUDIO_CS_AC_SUBTYPE_FEATURE_UNIT:
    {
      USBD_AUDIO_FeatureControlCallbacks_t* feature_control = audio_control->Callbacks.feature_control;
      switch(control_selector)
      {
        case USBD_AUDIO_CONTROL_FEATURE_UNIT_MUTE:
        {
          // @TODO treat multi channel case and error when req! of GetCur
          audio_handle->last_control.data[0] = 0;
          if(feature_control->GetMute)
          {
            feature_control->GetMute(LOBYTE(control_request->wValue), &audio_handle->last_control.data[0], audio_control->private_data);
          }
          // Send the current mute state 
          USBD_CtlSendData (pdev, audio_handle->last_control.data,1);
          break;
        }
        case USBD_AUDIO_CONTROL_FEATURE_UNIT_VOLUME:
        {           
          // set request 
          // @TODO check the len uses cases and control req->wLength             
          tmpdata =  (uint16_t*) &(audio_handle->last_control.data);
          switch(control_request->bRequest)
          {
            case USBD_AUDIO_REQ_GET_CUR:
              tmpdata = 0;
              if(feature_control->GetCurVolume)
              {
                feature_control->GetCurVolume(LOBYTE(control_request->wValue), (uint16_t*)audio_handle->last_control.data, audio_control->private_data);
              }
              break;            
            case USBD_AUDIO_REQ_GET_MIN:
              tmpdata = (uint16_t*) &(feature_control->MinVolume);
              break;
            case USBD_AUDIO_REQ_GET_MAX:
              tmpdata = (uint16_t*) &(feature_control->MaxVolume);
              break;           
            case USBD_AUDIO_REQ_GET_RES:
              tmpdata = (uint16_t*) &(feature_control->ResVolume);
              break;
            default :
              USBD_error_handler();
          }
          // Send the current mute state 
          USBD_CtlSendData (pdev, (uint8_t*) tmpdata,2);
          break;
        }
        default:
          USBD_error_handler();
      }
      break;
    }
                 
  default : // switch(ctl->type)
            USBD_error_handler();
    }
  return USBD_OK;
}
#if USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
/**
  * @brief  AUDIO_EP_REQ
  *         Handles the EP requests.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static uint8_t AUDIO_EP_REQ(USBD_Handles_t *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_Handle_t   *haudio;
  USBD_AUDIO_EP_DataTypeDef* data_ep = 0;
  uint8_t ep_num, control_selector;

  // get the main structure handle 
  haudio = (USBD_AUDIO_Handle_t*) pdev->pClassData;
  // get the EP number 
  ep_num = LOBYTE(req->wIndex);
  
  // look for registered data EP 
  for (int i = 0;i < haudio->aud_function.as_interfaces_count; i++)
  {
    if(ep_num == haudio->aud_function.as_interfaces[i].data_ep.ep_num)
    {
        data_ep = &haudio->aud_function.as_interfaces[i].data_ep;
        break;
    }     
  }
  
  if(!data_ep)
  {
    // The EP not found 
    USBD_CtlError (pdev, req);
    return  USBD_FAIL; 
  }
  
  // get the CS field
  control_selector = HIBYTE(req->wValue);
  
  // check if control is supported 
  if((data_ep->control_selector_map & control_selector) == 0)
  {
    // control not supported 
    USBD_CtlError (pdev, req);
    return  USBD_FAIL; 
  }
  
  // for set request we prepare EP for data Stage 
  if(!(req->bRequest&0x80))
  {
    // set request 
    // @TODO check the length 
     haudio->last_control.wValue  = req->wValue;
     haudio->last_control.entity.data_ep = data_ep;
     haudio->last_control.request_target = AUDIO_EP_REQUEST;
     haudio->last_control.len = req->wLength;
     haudio->last_control.req = req->bRequest;
     USBD_CtlPrepareRx (pdev,
                        haudio->last_control.data,                                  
                       req->wLength);
      return USBD_OK;   
  }
  
  // current implementation supports only FREQUENCY control , to support other ones change next code 
  if(control_selector == USBD_AUDIO_CONTROL_EP_SAMPL_FREQ)
  {
    switch(req->bRequest)
    {
      case USBD_AUDIO_REQ_GET_CUR:
      {
        uint32_t freq=0;
        if(data_ep->control_cbk.GetCurFrequency)
        {
            data_ep->control_cbk.GetCurFrequency(&freq, data_ep->private_data);
        }
        AUDIO_FREQ_TO_DATA(freq , haudio->last_control.data)
        break;
      }
      case USBD_AUDIO_REQ_GET_MIN:
            AUDIO_FREQ_TO_DATA(data_ep->control_cbk.MinFrequency , haudio->last_control.data)
			break;
      case USBD_AUDIO_REQ_GET_MAX:
            AUDIO_FREQ_TO_DATA(data_ep->control_cbk.MaxFrequency , haudio->last_control.data)
          break;
          
      // case USBD_AUDIO_REQ_GET_RES:
      default :
             USBD_CtlError (pdev, req);
              return  USBD_FAIL; 
      }
  }
  else
  {
    USBD_error_handler();
  }
               // Send the current mute state 
   USBD_CtlSendData (pdev, haudio->last_control.data,3);
  return 0;
}
#endif //USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES

/**
* @brief  returns Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t USBD_AUDIO_RegisterInterface (USBD_Handles_t *pdev, USBD_AUDIO_InterfaceCallbacksf_t *audio_interface_callbacks)
{
  if(audio_interface_callbacks != NULL)
  {
    pdev->pUserData = audio_interface_callbacks;
    audio_interface_callbacks->GetConfigDesc(&USBD_AUDIO_CfgDesc, &USBD_AUDIO_CfgDescSize, audio_interface_callbacks->private_data);
    
  }
  return 0;
}