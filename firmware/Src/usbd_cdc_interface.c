#include "main.h"

USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

extern USBD_HandleTypeDef  USBD_Device;

static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);

static void ComPort_Config(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

static int8_t CDC_Itf_Init(void)
{
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);

  return (USBD_OK);
}

static int8_t CDC_Itf_DeInit   (void){

}

static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length){

}

static int8_t CDC_Itf_Receive  (uint8_t* pbuf, uint32_t *Len){

}
