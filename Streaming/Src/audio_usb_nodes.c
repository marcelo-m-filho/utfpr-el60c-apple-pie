/**
  ******************************************************************************
  * @file    audio_usb_nodes.c
  * @author  MCD Application Team 
  * @brief   Usb input output and Feature unit implementation.
  * @version horoscope 0.1
  ******************************************************************************
  */ 

// includes
#include "usb_audio_app_config.h"
#include "audio_usb_nodes.h"

// private defines
#if USE_USB_AUDIO_CLASS_10
  #if (defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCIES)
    #if USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES
      #define USE_AUDIO_USB_MULTI_FREQUENCIES 1
    #else /* USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES */
      #error "USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES must be defined to support multi-frequencies"
    #endif /* USBD_SUPPORT_AUDIO_MULTI_FREQUENCIES */
  #endif /*(defined USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES)||(defined USE_AUDIO_USB_RECORD_MULTI_FREQUENCIES) */
#endif /* USE_USB_AUDIO_CLASS_10 */

#define DEBUG_USB_NODES  0 // set to 1  to debug USB input for playback
#if DEBUG_USB_NODES
#define USB_INPUT_NODE_DEBUG_BUFFER_SIZE 1000

// private typedefs

typedef struct AUDIO_USBInputBufferDebugStats
{
  uint32_t time;
  uint16_t write;
  uint16_t read;
  uint16_t error;
} 
AUDIO_USBInputBufferDebugStats_t;
#endif // DEBUG_USB_NODES

// private function prototypes
static int8_t     USB_AudioStreamingInputOutputDeInit(uint32_t node_handle);
static int8_t     USB_AudioStreamingInputOutputStart( AUDIO_CircularBuffer_t* buffer, uint16_t threshold ,uint32_t node_handle);
static int8_t     USB_AudioStreamingInputOutputStop( uint32_t node_handle);
static uint16_t   USB_AudioStreamingInputOutputGetMaxPacketLength(uint32_t node_handle);
#if USE_USB_AUDIO_CLASS_10
  static int8_t   USB_AudioStreamingInputOutputGetState(uint32_t node_handle);
#endif /*USE_USB_AUDIO_CLASS_10*/
static int8_t     USB_AudioStreamingInputOutputRestart( uint32_t node_handle);
static int8_t     USB_AudioStreamingInputDataReceived( uint16_t data_len,uint32_t node_handle);
static uint8_t*   USB_AudioStreamingInputGetBuffer(uint32_t node_handle, uint16_t* max_packet_length);
static int8_t     USB_AudioStreamingFeatureUnitDInit(uint32_t node_handle);
static int8_t     USB_AudioStreamingFeatureUnitStart(AUDIO_USBFeatureUnitCommands_t* commands, uint32_t node_handle);
static int8_t     USB_AudioStreamingFeatureUnitStop( uint32_t node_handle);
static int8_t     USB_AudioStreamingFeatureUnitGetMute(uint16_t channel,uint8_t* mute, uint32_t node_handle);
static int8_t     USB_AudioStreamingFeatureUnitSetMute(uint16_t channel,uint8_t mute, uint32_t node_handle);
static int8_t     USB_AudioStreamingFeatureUnitSetCurVolume(uint16_t channel, uint16_t volume, uint32_t node_handle);
static int8_t     USB_AudioStreamingFeatureUnitGetCurVolume(uint16_t channel, uint16_t* volume, uint32_t node_handle);

#if USE_USB_AUDIO_CLASS_10
  static int8_t USB_AudioStreamingFeatureUnitGetStatus(uint32_t node_handle);

  #ifdef USE_AUDIO_USB_MULTI_FREQUENCIES  
    static int8_t  USB_AudioStreamingInputOutputGetCurFrequency(uint32_t* freq, uint32_t node_handle);
    static int8_t  USB_AudioStreamingInputOutputSetCurFrequency(uint32_t freq,uint8_t*  usb_ep_restart_is_required , uint32_t node_handle);
    static uint32_t  USB_AudioStreamingGetNearestFrequency(uint32_t freq,  uint32_t* freq_table, int freq_count);
  #endif /*USE_AUDIO_USB_MULTI_FREQUENCIES*/

  // private variables
  #ifdef USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES
    // declares table of all supported frequencies, to select frequency when set frequency control is received */
    uint32_t USB_AUDIO_CONFIG_PLAY_FREQENCIES[USB_AUDIO_CONFIG_PLAY_FREQ_COUNT]=
    {
      #if USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K
      USB_AUDIO_CONFIG_FREQ_192_K,
      #endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_192_K */
      #if USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K
      USB_AUDIO_CONFIG_FREQ_96_K,
      #endif /* USB_AUDIO_CONFIG_PLAY_USE_FREQ_96_K */
      #if USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K
      USB_AUDIO_CONFIG_FREQ_48_K,
      #endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_48_K*/
      #if USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K
      USB_AUDIO_CONFIG_FREQ_44_1_K,
      #endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_44_1_K*/
      #if USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K
      USB_AUDIO_CONFIG_FREQ_16_K,
      #endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_16_K*/
      #if USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K
      USB_AUDIO_CONFIG_FREQ_8_K,
      #endif /*USB_AUDIO_CONFIG_PLAY_USE_FREQ_8_K*/
    };
  #endif /* USE_AUDIO_USB_PLAY_MULTI_FREQUENCIES*/
#endif /* USE_USB_AUDIO_CLASS_10 */

#if DEBUG_USB_NODES
  static AUDIO_USBInputBufferDebugStats_t stats_buffer [USB_INPUT_NODE_DEBUG_BUFFER_SIZE];
  static int                              stats_count=0;
  extern __IO uint32_t                    uwTick;
#endif /* DEBUG_USB_NODES */

/**
  * @brief  De-Initializes the AUDIO usb input node
  * @param  node_handle(IN): the node handle, node must be allocated
  * @retval  0 for no error
  */
 static int8_t USB_AudioStreamingInputOutputDeInit(uint32_t node_handle)
{
  ((AUDIO_USBInputOutputNode_t*)node_handle)->node.state = AUDIO_NODE_OFF;
  return 0;
}

/**
  * @brief  Stops Usb input or output node
  * @param  node_handle: the node handle, node must be initialized
  * @retval 0 for no error
  */
static int8_t USB_AudioStreamingInputOutputStop(uint32_t node_handle)
{
  AUDIO_USBInputOutputNode_t * io_node;

  io_node             = (AUDIO_USBInputOutputNode_t *)node_handle;
  io_node->node.state = AUDIO_NODE_STOPPED;
  return 0;
}
/**
  * @brief  Called when the node restart is required, for example after frequency change
  * @param  node_handle(IN): 
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingInputOutputRestart( uint32_t node_handle)
{
  AUDIO_USBInputOutputNode_t * io_node;
  io_node = (AUDIO_USBInputOutputNode_t *)node_handle;
  if(io_node->node.state == AUDIO_NODE_STARTED)
  {
    io_node->flags = AUDIO_IO_RESTART_REQUIRED;   /* this flag to stop node when next time USB Audio class calls the node via callback*/
    return 0;
  }
  return 0;
}

/**
  * @brief  Callback called by USB class when new packet is received
  * @param  data_len(IN):           packet length
  * @param  node_handle(IN):        the input node handle, node must be initialized  and started
  * @retval 0 if no error
  */
static int8_t USB_AudioStreamingInputDataReceived(uint16_t data_len, uint32_t node_handle)
 {
   AUDIO_USBInputOutputNode_t *input_node;
   AUDIO_CircularBuffer_t     *buf;
   uint16_t                    buffer_data_count;
   

   
   input_node = (AUDIO_USBInputOutputNode_t*)node_handle;

   if(input_node->node.state == AUDIO_NODE_STARTED)
   {
      /* @TODO add overrun detection */
      if(input_node->flags & AUDIO_IO_RESTART_REQUIRED)
      {
        // when restart is required, ignore the packet and reset buffer
        input_node->flags = 0;
        input_node->buf->rd_ptr = input_node->buf->wr_ptr = 0;
        return 0;
      }

      buf = input_node->buf;

      uint8_t* newDataPointer = buf->data + buf->wr_ptr;

      buf->wr_ptr += data_len; // increments buffer

     if((input_node->flags & AUDIO_IO_BEGIN_OF_STREAM) == 0)
     { /* this is the first packet */
       input_node->node.session_handle->SessionCallback(AUDIO_BEGIN_OF_STREAM, (AUDIO_Node_t*)input_node, input_node->node.session_handle);   /* send event to mother session */
       input_node->flags |= AUDIO_IO_BEGIN_OF_STREAM;
     }
     else
     {   /* if some sample are in the margin area , then copy them to regular area */
        if(buf->wr_ptr > buf->size)
        {
          buf->wr_ptr -= buf->size;
          memcpy(buf->data, buf->data + buf->size, buf->wr_ptr);
        }

      /* count pending audio samples in the buffer */
      buffer_data_count = AUDIO_BUFFER_FILLED_SIZE(buf); 
      if(buf->wr_ptr == buf->size)
      {
        buf->wr_ptr = 0;
      }

      if(((input_node->flags & AUDIO_IO_THRESHOLD_REACHED) == 0) && (buffer_data_count >= input_node->specific.input.threshold))
      {  
          // informs session that the buffer threshold is reached 
          input_node->node.session_handle->SessionCallback(AUDIO_THRESHOLD_REACHED, (AUDIO_Node_t*)input_node, input_node->node.session_handle);   
          input_node->flags |= AUDIO_IO_THRESHOLD_REACHED ;
       }
       else
       {
        // informs session that a packet is received
        input_node->node.session_handle->SessionCallback(AUDIO_PACKET_RECEIVED, (AUDIO_Node_t*)input_node, input_node->node.session_handle);
       }
     }
    }
    else
    {
      Error_Handler();
    }

    return 0;
 }

/**
  * @brief  Callback called by USB Audio class to get working buffer in order to receive next packet           
  * @param  node_handle(IN):        the input node handle, node must be initialized and started
  * @param  max_packet_length(OUT):  max packet length to be received
  * @retval  0 for no error (?)                     
  */
static uint8_t* USB_AudioStreamingInputGetBuffer(uint32_t node_handle, uint16_t* max_packet_length)
{
  AUDIO_USBInputOutputNode_t* input_node;
  uint16_t                    buffer_free_size;
  
  input_node = (AUDIO_USBInputOutputNode_t *)node_handle;
  #if DEBUG_USB_NODES
    stats_buffer[stats_count].read  = input_node->buf->rd_ptr;
    stats_buffer[stats_count].write = input_node->buf->wr_ptr;
    stats_buffer[stats_count].time  = uwTick;
    
    stats_count++;
    if(stats_count == USB_INPUT_NODE_DEBUG_BUFFER_SIZE)
    {
      stats_count=0;
    }
  #endif /*DEBUG_USB_NODES*/
  *max_packet_length = input_node->max_packet_length;
  if( input_node->node.state == AUDIO_NODE_STARTED)
  {
    /* control of possible overflow */
    buffer_free_size  = AUDIO_BUFFER_FREE_SIZE(input_node->buf);
    
    if(buffer_free_size < input_node->max_packet_length)
    {
      input_node->node.session_handle->SessionCallback(AUDIO_OVERRUN, (AUDIO_Node_t*)input_node, input_node->node.session_handle);
    }
    
    if(input_node->flags&AUDIO_IO_RESTART_REQUIRED)
    {
     input_node->flags = 0;
     input_node->buf->rd_ptr = input_node->buf->wr_ptr = 0;
    }
    return input_node->buf->data+input_node->buf->wr_ptr;
  }
  else
  {
    Error_Handler();
  }
}


/**
  * @brief  Returns max packet length; it is called by the USB Audio Class
  * @param  node_handle [in]: the input node handle, node must be initialized
  * @retval max packet length
*/
static uint16_t  USB_AudioStreamingInputOutputGetMaxPacketLength(uint32_t node_handle)
{
  return ((AUDIO_USBInputOutputNode_t *)node_handle)->max_packet_length;
}


#if USE_USB_AUDIO_CLASS_10
  /**
    * @brief  returns data ep state   
    * @param  node_handle: the input node handle, node must be initialized
    * @retval 0
  */
  static int8_t  USB_AudioStreamingInputOutputGetState(uint32_t node_handle)
  {
    return 0;
  }
#endif /* USE_USB_AUDIO_CLASS_10 */

/**
  * @brief  USB_AudioStreamingFeatureUnitInit
  *         Initializes control feature unit node
  * @param  usb_control_feature(OUT): structure to communicate with USB Audio Class, it contains information, controls
  *                                   and callbacks to handle controls that target the feature unit
  * @param  audio_defaults(IN):             audio defaults setting
  * @param  unit_id(IN):                    usb unit id
  * @param  node_handle(IN):                the node handle, node must be allocated
  * @retval  0 for no error
  */
 int8_t USB_AudioStreamingFeatureUnitInit(USBD_AUDIO_ControlTypeDef* usb_control_feature, AUDIO_USBFeatureUnitDefaults_t* audio_defaults, uint8_t unit_id, uint32_t node_handle)
{
  AUDIO_USB_CF_NodeTypeDef * cf;
 
  cf =              (AUDIO_USB_CF_NodeTypeDef*)node_handle;  
  memset(cf,0,sizeof(AUDIO_USB_CF_NodeTypeDef));

  cf->node.state  = AUDIO_NODE_INITIALIZED;
  cf->node.type   = AUDIO_CONTROL;
  cf->unit_id     = unit_id;
  
  cf->CFInit      = USB_AudioStreamingFeatureUnitInit;
  cf->CFDeInit    = USB_AudioStreamingFeatureUnitDInit;
  cf->CFStart     = USB_AudioStreamingFeatureUnitStart;
  cf->CFStop      = USB_AudioStreamingFeatureUnitStop;
  cf->CFSetMute   = USB_AudioStreamingFeatureUnitSetMute;

  #if USE_USB_AUDIO_CLASS_10
    cf->usb_control_callbacks.GetStatus = USB_AudioStreamingFeatureUnitGetStatus;
  #endif /*USE_USB_AUDIO_CLASS_10*/

  cf->usb_control_callbacks.GetMute       = USB_AudioStreamingFeatureUnitGetMute;
  cf->usb_control_callbacks.SetMute       = USB_AudioStreamingFeatureUnitSetMute;
  cf->usb_control_callbacks.GetCurVolume  = USB_AudioStreamingFeatureUnitGetCurVolume;
  cf->usb_control_callbacks.SetCurVolume  = USB_AudioStreamingFeatureUnitSetCurVolume;

  VOLUME_DB_256_TO_USB(cf->usb_control_callbacks.MaxVolume, audio_defaults->max_volume);
  VOLUME_DB_256_TO_USB(cf->usb_control_callbacks.MinVolume, audio_defaults->min_volume);

  cf->usb_control_callbacks.ResVolume     = audio_defaults->res_volume;
  cf->node.audio_description              = audio_defaults->audio_description;
  
  /* fill structure used by USB Audio Class module */
  usb_control_feature->id                         = unit_id;
  usb_control_feature->control_req_map            = 0;  
  usb_control_feature->control_selector_map       = USBD_AUDIO_FU_MUTE_CONTROL|USBD_AUDIO_FU_VOLUME_CONTROL;
  usb_control_feature->type                       = USBD_AUDIO_CS_AC_SUBTYPE_FEATURE_UNIT;
  usb_control_feature->Callbacks.feature_control  = &cf->usb_control_callbacks;
  usb_control_feature->private_data               = node_handle;
  return 0;
}

/**
  * @brief  De-Initializes control feature unit node
  * @param  node_handle: the node handle, node must be Initialized
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingFeatureUnitDInit(uint32_t node_handle)
{
  ((AUDIO_USB_CF_NodeTypeDef*)node_handle)->node.state = AUDIO_NODE_OFF;
  return 0;
}

/**
  * @brief  Starts control feature unit node. After start, calls the feature unit node executes controls like set volume and mute.
  *         If some commands are already received and pending, they will be executed in this function.
  * @param  commands(IN): list of callback to execute controls like setvolume and mute. this function depends on codec and microphone.
  * @param  node_handle(IN): the node handle, node must be allocated
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingFeatureUnitStart(AUDIO_USBFeatureUnitCommands_t* commands, uint32_t node_handle)
{
  AUDIO_USB_CF_NodeTypeDef *cf;
  
  cf = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  cf->control_cbks = *commands;
  cf->node.state = AUDIO_NODE_STARTED;
  
  if(cf->control_cbks.SetCurrentVolume)
  {
    cf->control_cbks.SetCurrentVolume(0, 
                                      cf->node.audio_description->audio_volume_db_256,
                                      cf->control_cbks.private_data);
  }
  return 0;
}

/**
  * @brief  Stops control feature node
  * @param  node_handle: the node handle, node must be started
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingFeatureUnitStop( uint32_t node_handle)
{
  /* @TODO develop feature */
  AUDIO_USB_CF_NodeTypeDef* control_feature;
  
  control_feature             = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  control_feature->node.state = AUDIO_NODE_STOPPED;
  return 0;
}

/**
  * @brief  Retrieves mute value
  * @param  channel [in]:     channel number, 0 for master channel (only this option is supported now)
  * @param  mute [out]:       returned mute value
  * @param  node_handle [in]: the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingFeatureUnitGetMute(uint16_t channel, uint8_t* mute, uint32_t node_handle)
{
  /**@TODO add channel management  */
  *mute = ((AUDIO_USB_CF_NodeTypeDef*)node_handle)->node.audio_description->audio_mute; 
  return 0; 
}

/**
  * @brief  Sets mute value
  * @param  channel [in]:     channel number , 0 for master channel(only this option is supported now)
  * @param  mute [out]:       mute value
  * @param  node_handle [in]: the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingFeatureUnitSetMute(uint16_t channel, uint8_t mute, uint32_t node_handle)
{
  /**@TODO add channel management  */
  AUDIO_USB_CF_NodeTypeDef* cf;
  
  cf                                     = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  cf->node.audio_description->audio_mute = mute;

  if((cf->node.state == AUDIO_NODE_STARTED) && cf->control_cbks.SetMute)
  {
      cf->control_cbks.SetMute(channel, mute, cf->control_cbks.private_data);
  }
  return 0;
}

/**
  * @brief  Retrieves the current volume value
  * @param  channel:      channel number , 0 for master channel(only this option is supported now)
  * @param  volume:       returned volume value
  * @param  node_handle:  the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingFeatureUnitGetCurVolume(uint16_t channel, uint16_t* volume, uint32_t node_handle)
{
  /**@TODO add channel management  */
  VOLUME_DB_256_TO_USB(*volume, ((AUDIO_Node_t*)node_handle)->audio_description->audio_volume_db_256);
  return 0; 
}

/**
  * @brief  Sets the current volume value
  * @param  channel:     channel number , 0 for master channel(only this option is supported now)
  * @param  volume:      volume value
  * @param  node_handle: the Feature node handle, node must be initialized
  * @retval  0 for no error
  */
static int8_t USB_AudioStreamingFeatureUnitSetCurVolume(uint16_t channel, uint16_t volume, uint32_t node_handle)
{
  AUDIO_USB_CF_NodeTypeDef* cf;
  
  cf = (AUDIO_USB_CF_NodeTypeDef*)node_handle;
  /**@TODO add channel management  */
  
  VOLUME_USB_TO_DB_256(cf->node.audio_description->audio_volume_db_256, volume);

  if((cf->node.state == AUDIO_NODE_STARTED) && cf->control_cbks.SetCurrentVolume)
  {
    cf->control_cbks.SetCurrentVolume(channel, cf->node.audio_description->audio_volume_db_256, cf->control_cbks.private_data);
  }
  return 0;
}

#if USE_USB_AUDIO_CLASS_10
/**
  * @brief  Retrieves feature unit status
  * @param  node_handle: the Feature node handle, node must be initialized      
  * @retval 0 for no error
  */
static int8_t  USB_AudioStreamingFeatureUnitGetStatus(uint32_t node_handle)
{
  return 0;
}
#endif /* USE_USB_AUDIO_CLASS_10 */

/**
  * @brief  The circular buffer has the total size of buffer_size. 
  *         This size is divided into two: the regular size and the margin.
  *         The margin is located at the tail of the circular buffer. It is used as some packets have regular size +-1 sample. 
  * @param  buf:          main circular buffer               
  * @param  buffer_size:  whole buffer size when allocated                
  * @param  packet_size:  USB Audio packet size 
  * @param  margin:       protection area size
  * @retval 0 if no error
  */
  void USB_AudioStreamingInitializeDataBuffer(AUDIO_CircularBuffer_t* buf, uint32_t buffer_size, uint16_t packet_size, uint16_t margin)
   {
    buf->size   = ((int)((buffer_size - margin) / packet_size)) * packet_size; 
    buf->rd_ptr = buf->wr_ptr = 0;
  }