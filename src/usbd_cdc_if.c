/* usbd_cdc_if.c — USB CDC application interface
 *
 * Bridges the STM32 USB_Device_Library CDC class driver to the application.
 * Implements the USBD_CDC_ItfTypeDef callbacks.
 *
 * CDC_Transmit_FS() is the only function the application calls directly.
 * It is thread-blocking (waits until the endpoint is free) with a 10ms timeout.
 *
 * printf() is retargeted here via __io_putchar() so that:
 *   printf("hello\r\n") → USB CDC → /dev/cu.usbmodem on macOS
 *
 * NOTE: The UART1 __io_putchar() in main.c is intentionally replaced here.
 *       Both this file and main.c define __io_putchar; only one will link.
 *       PlatformIO links src/usbd_cdc_if.c AFTER src/main.c, so this
 *       definition wins via the WEAK attribute on the main.c version.
 *       (We add __attribute__((weak)) to the one in main.c to allow override.)
 */

#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

/* -----------------------------------------------------------------------
 * RX / TX buffers
 * ----------------------------------------------------------------------- */
#define APP_RX_DATA_SIZE  512
#define APP_TX_DATA_SIZE  512

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* -----------------------------------------------------------------------
 * CDC class handle (set by USBD_CDC_SetTxBuffer / SetRxBuffer)
 * ----------------------------------------------------------------------- */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* -----------------------------------------------------------------------
 * CDC interface callbacks (forward declarations)
 * ----------------------------------------------------------------------- */
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* -----------------------------------------------------------------------
 * CDC line coding (host sets baud, stop bits, etc. — we ignore on STM32)
 * ----------------------------------------------------------------------- */
static USBD_CDC_LineCodingTypeDef LineCoding = {
  115200, // dwDTERate   (baud — host sees 115200)
  0x00,   // bCharFormat (1 stop bit)
  0x00,   // bParityType (none)
  0x08    // bDataBits   (8)
};

/* =======================================================================
 * CDC interface implementation
 * ======================================================================= */

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return USBD_OK;
}

static int8_t CDC_DeInit_FS(void) { return USBD_OK; }

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  (void)length;
  switch (cmd) {
    case CDC_SET_LINE_CODING:
      LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1]<<8) |
                                          (pbuf[2]<<16) | (pbuf[3]<<24));
      LineCoding.format     = pbuf[4];
      LineCoding.paritytype = pbuf[5];
      LineCoding.datatype   = pbuf[6];
      break;
    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(LineCoding.bitrate);
      pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
      pbuf[4] = LineCoding.format;
      pbuf[5] = LineCoding.paritytype;
      pbuf[6] = LineCoding.datatype;
      break;
    default:
      break;
  }
  return USBD_OK;
}

extern void Jetson_Heartbeat_Callback(void);

extern void Process_Serial_Data(char *data, uint32_t len);

/* Called by middleware when host sends data to device */
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len)
{
  static char rx_str[64];
  uint32_t length = (*Len < 63) ? *Len : 63;
  
  memcpy(rx_str, pbuf, length);
  rx_str[length] = '\0';
  
  Process_Serial_Data(rx_str, length);
  
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return USBD_OK;
}

/* Called when TX transfer is complete */
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum)
{
  (void)pbuf; (void)Len; (void)epnum;
  return USBD_OK;
}

/* =======================================================================
 * Public API: CDC_Transmit_FS
 * ======================================================================= */

/**
 * @brief  Send a buffer over USB CDC (Full Speed).
 *         Waits up to 10ms for the endpoint to become free.
 * @param  Buf  Pointer to data buffer
 * @param  Len  Number of bytes to send
 * @retval USBD_OK on success, USBD_BUSY if endpoint busy, USBD_FAIL on error
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  USBD_CDC_HandleTypeDef *hcdc =
    (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  if (hcdc == NULL) return USBD_FAIL;

  /* Wait for endpoint to become free (max 10ms) */
  uint32_t t0 = HAL_GetTick();
  while (hcdc->TxState != 0) {
    if ((HAL_GetTick() - t0) > 10) return USBD_BUSY;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

/* =======================================================================
 * printf retarget: __io_putchar → USB CDC
 *
 * This overrides the UART version declared weak in main.c.
 * Characters are buffered until '\n' then flushed.
 * This avoids hammering CDC_Transmit_FS one byte at a time.
 * ======================================================================= */
static uint8_t cdc_tx_buf[128];
static uint16_t cdc_tx_len = 0;

int __io_putchar(int ch)
{
  if (cdc_tx_len < sizeof(cdc_tx_buf))
    cdc_tx_buf[cdc_tx_len++] = (uint8_t)ch;

  if (ch == '\n' || cdc_tx_len >= sizeof(cdc_tx_buf)) {
    CDC_Transmit_FS(cdc_tx_buf, cdc_tx_len);
    cdc_tx_len = 0;
  }
  return ch;
}
