/* usbd_conf.c — USB Device HAL-layer glue for STM32F411CEU6 (USB FS)
 *
 * Implements the low-level callbacks and hardware init that the
 * STM32_USB_Device_Library middleware requires.
 *
 * USB FS pins:  PA11 = D-   PA12 = D+   (no external PHY needed)
 * USB clock:    derived from 48 MHz PLL48CK (PLLQ=4 from 96 MHz VCO)
 */

#include "stm32f4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"

/* -----------------------------------------------------------------------
 * PCD handle (USB device low-level driver)
 * ----------------------------------------------------------------------- */
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* =======================================================================
 * HAL PCD callbacks — called from USB OTG FS IRQ handler
 * ======================================================================= */

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Instance == USB_OTG_FS)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA11 = USB_OTG_FS_DM,  PA12 = USB_OTG_FS_DP */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpio);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Instance == USB_OTG_FS)
  {
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  }
}

/* -----------------------------------------------------------------------
 * USB Device event callbacks → forwarded to middleware
 * ----------------------------------------------------------------------- */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                        hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                       hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_SpeedTypeDef speed;
  switch (hpcd->Init.speed) {
    case PCD_SPEED_FULL:  speed = USBD_SPEED_FULL; break;
    default:              speed = USBD_SPEED_FULL; break;
  }
  USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, speed);
  USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/* =======================================================================
 * USB Device Library LL interface — called by middleware
 * ======================================================================= */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  hpcd_USB_OTG_FS.pData               = pdev;
  pdev->pData                         = &hpcd_USB_OTG_FS;

  hpcd_USB_OTG_FS.Instance            = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints  = 4;
  hpcd_USB_OTG_FS.Init.speed          = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable     = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface     = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable     = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable   = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable         = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE; // no VBUS pin on black-pill
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1  = DISABLE;

  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
    return USBD_FAIL;

  /* FIFO sizes (words): RX=0x80, EP0 TX=0x40, EP1 TX=0x80 */
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);

  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit((PCD_HandleTypeDef *)pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start((PCD_HandleTypeDef *)pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop((PCD_HandleTypeDef *)pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                   uint8_t ep_type, uint16_t ep_mps)
{
  HAL_PCD_EP_Open((PCD_HandleTypeDef *)pdev->pData, ep_addr, ep_mps, ep_type);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close((PCD_HandleTypeDef *)pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Flush((PCD_HandleTypeDef *)pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)pdev->pData, ep_addr);
  return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  if ((ep_addr >> 7) & 0x01)
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  else
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_PCD_SetAddress((PCD_HandleTypeDef *)pdev->pData, dev_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                     uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                           uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay) { HAL_Delay(Delay); }

/* Weak stubs required by some middleware versions */
void *USBD_static_malloc(uint32_t size) { (void)size; return NULL; }
void  USBD_static_free(void *p)          { (void)p; }

/* USB OTG FS IRQ handler — routes to HAL PCD handler */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
