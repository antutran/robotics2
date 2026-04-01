/* usbd_desc.c — USB CDC device descriptor for STM32F411CEU6
 *
 * Provides VID/PID, string descriptors, and device descriptor data that
 * the USB Device stack sends to the host during enumeration.
 *
 * VID 0x0483 = STMicroelectronics
 * PID 0x5740 = Virtual COM Port (standard ST CDC PID)
 * macOS will enumerate this as /dev/cu.usbmodem...
 */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* -----------------------------------------------------------------------
 * Device descriptor constants
 * ----------------------------------------------------------------------- */
#define USBD_VID              0x0483  // STMicroelectronics
#define USBD_PID_FS           0x5740  // Virtual COM Port
#define USBD_LANGID_STRING    0x0409  // English (US)
#define USBD_MANUFACTURER_STRING  "STMicroelectronics"
#define USBD_PRODUCT_STRING_FS    "STM32 Virtual ComPort"
#define USBD_SERIALNUMBER_STRING_FS "00000000001A"
#define USBD_CONFIGURATION_STRING_FS "CDC Config"
#define USBD_INTERFACE_STRING_FS    "CDC Interface"
#define USB_SIZ_BOS_DESC    0x0C

/* -----------------------------------------------------------------------
 * Descriptor callbacks (function pointers for the middleware)
 * ----------------------------------------------------------------------- */
uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor,
};

/* -----------------------------------------------------------------------
 * Device descriptor (18 bytes, USB 2.0 CDC device)
 * ----------------------------------------------------------------------- */
#define USB_SIZ_DEVICE_DESC  18
static uint8_t USBD_FS_DeviceDesc[USB_SIZ_DEVICE_DESC] = {
  0x12,              // bLength
  USB_DESC_TYPE_DEVICE, // bDescriptorType
  0x00, 0x02,        // bcdUSB  (USB 2.0)
  0x02,              // bDeviceClass    (CDC)
  0x02,              // bDeviceSubClass
  0x00,              // bDeviceProtocol
  USB_MAX_EP0_SIZE,  // bMaxPacketSize0 (64)
  LOBYTE(USBD_VID),
  HIBYTE(USBD_VID),
  LOBYTE(USBD_PID_FS),
  HIBYTE(USBD_PID_FS),
  0x00, 0x02,        // bcdDevice (2.00)
  USBD_IDX_MFC_STR,  // iManufacturer
  USBD_IDX_PRODUCT_STR, // iProduct
  USBD_IDX_SERIAL_STR,  // iSerialNumber
  USBD_MAX_NUM_CONFIGURATION // bNumConfigurations
};

uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

/* -----------------------------------------------------------------------
 * String descriptors — returned from a shared work buffer
 * ----------------------------------------------------------------------- */
#define USBD_STR_BUF_SIZE  64
static uint8_t USBD_StrDesc[USBD_STR_BUF_SIZE];

/* Helper: ASCII → UTF-16LE USB string descriptor in USBD_StrDesc */
static void ASCII_To_Unicode(const char *str, uint8_t *desc, uint16_t *length)
{
  uint8_t len = (uint8_t)strlen(str);
  *length = (uint16_t)(len * 2 + 2);
  desc[0] = *length;
  desc[1] = USB_DESC_TYPE_STRING;
  for (uint8_t i = 0; i < len; i++) {
    desc[2 + i * 2]     = (uint8_t)str[i];
    desc[2 + i * 2 + 1] = 0x00;
  }
}

uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  static uint8_t lang[4] = { 4, USB_DESC_TYPE_STRING,
                              LOBYTE(USBD_LANGID_STRING),
                              HIBYTE(USBD_LANGID_STRING) };
  *length = sizeof(lang);
  return lang;
}

uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ASCII_To_Unicode(USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ASCII_To_Unicode(USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ASCII_To_Unicode(USBD_SERIALNUMBER_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ASCII_To_Unicode(USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  ASCII_To_Unicode(USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}
