/* usbd_cdc_if.h — USB CDC application interface API */
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/**
 * @brief  Transmit data over USB CDC Full Speed.
 * @param  Buf  Pointer to data buffer (must stay valid until transfer completes)
 * @param  Len  Number of bytes to send
 * @retval USBD_OK / USBD_BUSY / USBD_FAIL
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
