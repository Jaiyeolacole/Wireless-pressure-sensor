/* USER CODE BEGIN Header */
/**
  * @file    usbd_cdc_if.c
  * @brief   CDC (virtual COM port) interface + a non-blocking TX ring buffer.
  */
/* USER CODE END Header */

#include "usbd_cdc_if.h"
#include <string.h>

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN PRIVATE_VARIABLES */
#define USB_TX_RING_SIZE 1024U   /* must be a power of two */
#define USB_RX_RING_SIZE 64U     /* must be a power of two */

static uint8_t           tx_ring[USB_TX_RING_SIZE];
static volatile uint32_t tx_head, tx_tail;      /* head: written by main, tail: by usb_poll (main) */
static volatile uint32_t tx_dropped;

static uint8_t           rx_ring[USB_RX_RING_SIZE];
static volatile uint32_t rx_head, rx_tail;      /* head: written in USB ISR, tail: main */

/* The PC terminal can ask for these; we just store and echo them back */
static uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 }; /* 115200 8N1 */
static volatile uint8_t port_opened;   /* set when the PC terminal raises DTR */
/* USER CODE END PRIVATE_VARIABLES */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
    case CDC_SET_LINE_CODING:
      if (length >= 7U) { memcpy(line_coding, pbuf, 7); }
      break;
    case CDC_GET_LINE_CODING:
      if (length >= 7U) { memcpy(pbuf, line_coding, 7); }
      break;
    case CDC_SET_CONTROL_LINE_STATE:
    {
      /* No data stage: pbuf points to the setup request. wValue bit0 = DTR. */
      USBD_SetupReqTypedef *req = (USBD_SetupReqTypedef *)pbuf;
      if (req->wValue & 0x0001U) { port_opened = 1; }
      break;
    }
    default:
      break;
  }
  return (USBD_OK);
  /* USER CODE END 5 */
}

/* Called from the USB interrupt when the PC sends us bytes */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  for (uint32_t i = 0; i < *Len; i++)
  {
    uint32_t next = (rx_head + 1U) & (USB_RX_RING_SIZE - 1U);
    if (next != rx_tail)
    {
      rx_ring[rx_head] = Buf[i];
      rx_head = next;
    }
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL)
  {
    return USBD_FAIL;          /* not enumerated yet */
  }
  if (hcdc->TxState != 0U)
  {
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  int8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint32_t usb_write(const char *data, uint32_t len)
{
  uint32_t written = 0;
  while (written < len)
  {
    uint32_t next = (tx_head + 1U) & (USB_TX_RING_SIZE - 1U);
    if (next == tx_tail)
    {
      tx_dropped += (len - written);   /* PC not reading fast enough: drop, never block */
      break;
    }
    tx_ring[tx_head] = (uint8_t)data[written++];
    tx_head = next;
  }
  return written;
}

void usb_print(const char *s)
{
  usb_write(s, (uint32_t)strlen(s));
}

void usb_poll(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hcdc == NULL)
  {
    tx_tail = tx_head;     /* no PC attached: discard so we don't send stale data later */
    return;
  }
  if (hcdc->TxState != 0U || tx_tail == tx_head)
  {
    return;                /* previous packet still in flight, or nothing to send */
  }

  /* Copy a chunk out of the ring into the linear USB buffer */
  uint32_t n = 0;
  while (tx_tail != tx_head && n < APP_TX_DATA_SIZE)
  {
    UserTxBufferFS[n++] = tx_ring[tx_tail];
    tx_tail = (tx_tail + 1U) & (USB_TX_RING_SIZE - 1U);
  }
  CDC_Transmit_FS(UserTxBufferFS, (uint16_t)n);
}

int usb_getc(void)
{
  if (rx_tail == rx_head)
  {
    return -1;
  }
  int c = rx_ring[rx_tail];
  rx_tail = (rx_tail + 1U) & (USB_RX_RING_SIZE - 1U);
  return c;
}

uint32_t usb_dropped_bytes(void)
{
  return tx_dropped;
}

/* USB enumeration stage, for diagnosis on the LCD */
const char *usb_link_text(void)
{
  switch (hUsbDeviceFS.dev_state)
  {
    case USBD_STATE_DEFAULT:    return "--";   /* no PC, or PC sees nothing */
    case USBD_STATE_ADDRESSED:  return "ad";   /* PC talked to us but did not finish setup */
    case USBD_STATE_CONFIGURED: return "OK";   /* COM port should exist on the PC */
    case USBD_STATE_SUSPENDED:  return "sp";   /* PC put the bus to sleep */
    default:                    return "??";
  }
}

int usb_take_open_event(void)
{
  if (port_opened) { port_opened = 0; return 1; }
  return 0;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
