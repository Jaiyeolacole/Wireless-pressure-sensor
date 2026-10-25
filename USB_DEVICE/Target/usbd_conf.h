/* USER CODE BEGIN Header */
/**
  * @file    usbd_conf.h
  * @brief   USB device low-level configuration (STM32F103, USB FS device, CDC class)
  */
/* USER CODE END Header */

#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       512U
#define USBD_DEBUG_LEVEL            0U
#define USBD_SELF_POWERED           1U
#define MAX_STATIC_ALLOC_SIZE       512U

#define DEVICE_FS                   0

/* Static memory allocation: no heap needed by the USB stack */
#define USBD_malloc         (void *)USBD_static_malloc
#define USBD_free           USBD_static_free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

#define USBD_UsrLog(...)    do {} while (0)
#define USBD_ErrLog(...)    do {} while (0)
#define USBD_DbgLog(...)    do {} while (0)

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF__H__ */
