/* usbd_conf.h — USB Device HAL configuration header */
#ifndef __USBD_CONF_H__
#define __USBD_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USB Device Core settings */
#define USBD_MAX_NUM_INTERFACES      1U
#define USBD_MAX_NUM_CONFIGURATION   1U
#define USBD_MAX_STR_DESC_SIZ        512U
#define USBD_DEBUG_LEVEL             0U
#define USBD_SELF_POWERED            1U
#define USBD_CDC_INTERVAL            2000U

/* Memory management (static — no heap needed) */
#define USBD_malloc         malloc
#define USBD_free           free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

/* Debug macros (disabled) */
#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)  printf(__VA_ARGS__)
#else
#define USBD_UsrLog(...)  do {} while(0)
#endif

#define USBD_ErrLog(...)  do {} while(0)
#define USBD_DbgLog(...)  do {} while(0)

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H__ */
