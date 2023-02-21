/**
  ******************************************************************************
  * @file    hal_usb_interface_extension
  * @author  MCD Application Team 
  * @brief   Extension of the usb hal interface.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_USB_INTERFACE_EXTENSION
#define __HAL_USB_INTERFACE_EXTENSION

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_ll_usb.h"
#ifdef USE_USB_FS_INTO_HS
  #define USB_OTG_BASE_ADDRESS  USB_OTG_HS  
#else
#ifdef USE_USB_FS
  #define USB_OTG_BASE_ADDRESS  USB_OTG_FS   
#endif 
#ifdef USE_USB_HS
  #define USB_OTG_BASE_ADDRESS  USB_OTG_HS   
#endif
#endif

/* MACRO ------------------------------------------------------------------*/  
#define USB_DIEPCTL(ep_addr) ((USB_OTG_INEndpointTypeDef *)((uint32_t)USB_OTG_BASE_ADDRESS + USB_OTG_IN_ENDPOINT_BASE + (ep_addr&0x7FU)*USB_OTG_EP_REG_SIZE))->DIEPCTL
#define USB_DOEPCTL(ep_addr) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_BASE_ADDRESS + USB_OTG_OUT_ENDPOINT_BASE + (ep_addr)*USB_OTG_EP_REG_SIZE))->DOEPCTL

#define USB_CLEAR_INCOMPLETE_IN_EP(ep_addr)     if((((ep_addr) & 0x80U) == 0x80U)){  \
            USB_DIEPCTL(ep_addr) |= (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);  \
                                         };

#define USB_DISABLE_EP_BEFORE_CLOSE(ep_addr)\
if((((ep_addr) & 0x80U) == 0x80U))\
  {\
    if (USB_DIEPCTL(ep_addr)&USB_OTG_DIEPCTL_EPENA_Msk)\
      {\
       USB_DIEPCTL(ep_addr)|= USB_OTG_DIEPCTL_EPDIS;   \
      }\
  } ;
                                         
#define USB_SOF_NUMBER() ((((USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_BASE_ADDRESS + USB_OTG_DEVICE_BASE))->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos)



#define IS_ISO_IN_INCOMPLETE_EP(ep_addr,current_sof, transmit_soffn) ((USB_DIEPCTL(ep_addr)&USB_OTG_DIEPCTL_EPENA_Msk)&&\
                                                          (((current_sof&0x01) == ((USB_DIEPCTL(ep_addr) & USB_OTG_DIEPCTL_EONUM_DPID_Msk) >> USB_OTG_DIEPCTL_EONUM_DPID_Pos))\
                                                            ||(current_sof== ((transmit_soffn+2)&0x7FF))))

#ifdef __cplusplus
}
#endif

#endif  /* __HAL_USB_INTERFACE_EXTENSION*/  
