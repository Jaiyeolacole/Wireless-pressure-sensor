#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  256

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */
/*
 * Non-blocking serial API used by the application.
 *  usb_write(): copies bytes into a 4 KB ring buffer. If the PC is not reading,
 *               the buffer fills and extra bytes are DROPPED (we never block the radio).
 *  usb_poll():  call often from the main loop; pushes buffered bytes to the PC.
 *  usb_getc():  returns next received char, or -1 if none.
 */
uint32_t usb_write(const char *data, uint32_t len);
void     usb_print(const char *s);
void     usb_poll(void);
int      usb_getc(void);
uint32_t usb_dropped_bytes(void);
const char *usb_link_text(void);      /* "--", "ad", "OK", "sp": enumeration stage */
int      usb_take_open_event(void);   /* 1 once each time a terminal opens the port */
/* USER CODE END EXPORTED_FUNCTIONS */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
