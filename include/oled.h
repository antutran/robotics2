/* oled.h — SH1106 128x64 OLED driver for STM32 HAL
 * Framebuffer-based, page-addressing mode.
 * SH1106 has 132-column GDDRAM; visible pixels start at hardware column 2.
 * All drawing goes into a RAM buffer; call SSD1306_UpdateScreen() to flush.
 */
#ifndef OLED_H
#define OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * Display geometry and I2C address
 * Buffer = 128x8 pages = 1024 bytes (framebuffer matches visible area).
 * SH1106 GDDRAM has 132 columns; column offset +2 applied at flush time.
 * ----------------------------------------------------------------------- */
#define SSD1306_WIDTH    128
#define SSD1306_HEIGHT   64
#define SSD1306_PAGES    8              // 64px / 8 = 8 pages
#define SSD1306_ADDR     (0x3C << 1)   // HAL 8-bit shifted address = 0x78
#define SSD1306_TIMEOUT  50            // I2C timeout ms (129-byte frame at 400kHz ≈3ms; 50ms gives ample margin)

/* -----------------------------------------------------------------------
 * Module initialisation
 * Pass the I2C handle that the display is connected to.
 * Sets internal oled_ok flag; safe to call even if display absent.
 * ----------------------------------------------------------------------- */
void OLED_Module_Init(I2C_HandleTypeDef *hi2c);

/* -----------------------------------------------------------------------
 * Low-level driver API (SSD1306_ prefix kept for API compatibility;
 * the actual hardware target is SH1106)
 * ----------------------------------------------------------------------- */
void SSD1306_WriteCommand(uint8_t cmd);
void SSD1306_WriteData(uint8_t *pData, uint16_t size);
void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_SetCursor(uint8_t col, uint8_t page);
void SSD1306_WriteChar(char c);
void SSD1306_WriteString(uint8_t col, uint8_t page, const char *str);

/* -----------------------------------------------------------------------
 * Menu / highlight drawing helpers
 * ----------------------------------------------------------------------- */
/* Write string with inverted pixels (dark text on white bar) */
void SSD1306_WriteStringInverted(uint8_t col, uint8_t page, const char *str);
/* Fill an entire page row: colour=0xFF fills all pixels, 0x00 clears */
void SSD1306_FillRow(uint8_t page, uint8_t colour);

/* -----------------------------------------------------------------------
 * Legacy wrapper API (used by MPU display and button handlers)
 * ----------------------------------------------------------------------- */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t page, const char *str);
void OLED_ShowStatus(const char *status);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
