/* oled.c — SH1106 / SSD1306 128x64 OLED driver for STM32 HAL
 *
 * Extracted from main.c. Fixed for SH1106 hardware.
 *
 * SH1106 has 132-column GDDRAM (vs 128 on SSD1306).
 * Visible 128 pixels start at hardware column 2, so every page flush
 * must set column address to 2, not 0.
 *
 * SH1106 also does NOT support the 0x20 "Set Memory Addressing Mode"
 * command — it is always in page-addressing mode.
 *
 * I2C control bytes: 0x00=command, 0x40=data
 */

#include "oled.h"

/* -----------------------------------------------------------------------
 * Module-private state
 * ----------------------------------------------------------------------- */
static I2C_HandleTypeDef *oled_i2c   = NULL;
static uint8_t oled_ok               = 0;
static uint8_t ssd1306_buf[SSD1306_PAGES][SSD1306_WIDTH];
static uint8_t oled_cursor_page      = 0;
static uint8_t oled_cursor_col       = 0;

/* -----------------------------------------------------------------------
 * Minimal 5x7 ASCII font, characters 0x20..0x7E, 5 bytes per char
 * ----------------------------------------------------------------------- */
static const uint8_t font5x7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // ' '
  {0x00,0x00,0x5F,0x00,0x00}, // '!'
  {0x00,0x07,0x00,0x07,0x00}, // '"'
  {0x14,0x7F,0x14,0x7F,0x14}, // '#'
  {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
  {0x23,0x13,0x08,0x64,0x62}, // '%'
  {0x36,0x49,0x55,0x22,0x50}, // '&'
  {0x00,0x05,0x03,0x00,0x00}, // '\''
  {0x00,0x1C,0x22,0x41,0x00}, // '('
  {0x00,0x41,0x22,0x1C,0x00}, // ')'
  {0x14,0x08,0x3E,0x08,0x14}, // '*'
  {0x08,0x08,0x3E,0x08,0x08}, // '+'
  {0x00,0x50,0x30,0x00,0x00}, // ','
  {0x08,0x08,0x08,0x08,0x08}, // '-'
  {0x00,0x60,0x60,0x00,0x00}, // '.'
  {0x20,0x10,0x08,0x04,0x02}, // '/'
  {0x3E,0x51,0x49,0x45,0x3E}, // '0'
  {0x00,0x42,0x7F,0x40,0x00}, // '1'
  {0x42,0x61,0x51,0x49,0x46}, // '2'
  {0x21,0x41,0x45,0x4B,0x31}, // '3'
  {0x18,0x14,0x12,0x7F,0x10}, // '4'
  {0x27,0x45,0x45,0x45,0x39}, // '5'
  {0x3C,0x4A,0x49,0x49,0x30}, // '6'
  {0x01,0x71,0x09,0x05,0x03}, // '7'
  {0x36,0x49,0x49,0x49,0x36}, // '8'
  {0x06,0x49,0x49,0x29,0x1E}, // '9'
  {0x00,0x36,0x36,0x00,0x00}, // ':'
  {0x00,0x56,0x36,0x00,0x00}, // ';'
  {0x08,0x14,0x22,0x41,0x00}, // '<'
  {0x14,0x14,0x14,0x14,0x14}, // '='
  {0x00,0x41,0x22,0x14,0x08}, // '>'
  {0x02,0x01,0x51,0x09,0x06}, // '?'
  {0x32,0x49,0x79,0x41,0x3E}, // '@'
  {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
  {0x7F,0x49,0x49,0x49,0x36}, // 'B'
  {0x3E,0x41,0x41,0x41,0x22}, // 'C'
  {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
  {0x7F,0x49,0x49,0x49,0x41}, // 'E'
  {0x7F,0x09,0x09,0x09,0x01}, // 'F'
  {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
  {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
  {0x00,0x41,0x7F,0x41,0x00}, // 'I'
  {0x20,0x40,0x41,0x3F,0x01}, // 'J'
  {0x7F,0x08,0x14,0x22,0x41}, // 'K'
  {0x7F,0x40,0x40,0x40,0x40}, // 'L'
  {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
  {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
  {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
  {0x7F,0x09,0x09,0x09,0x06}, // 'P'
  {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
  {0x7F,0x09,0x19,0x29,0x46}, // 'R'
  {0x46,0x49,0x49,0x49,0x31}, // 'S'
  {0x01,0x01,0x7F,0x01,0x01}, // 'T'
  {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
  {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
  {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
  {0x63,0x14,0x08,0x14,0x63}, // 'X'
  {0x07,0x08,0x70,0x08,0x07}, // 'Y'
  {0x61,0x51,0x49,0x45,0x43}, // 'Z'
  {0x00,0x7F,0x41,0x41,0x00}, // '['
  {0x02,0x04,0x08,0x10,0x20}, // '\\'
  {0x00,0x41,0x41,0x7F,0x00}, // ']'
  {0x04,0x02,0x01,0x02,0x04}, // '^'
  {0x40,0x40,0x40,0x40,0x40}, // '_'
  {0x00,0x01,0x02,0x04,0x00}, // '`'
  {0x20,0x54,0x54,0x54,0x78}, // 'a'
  {0x7F,0x48,0x44,0x44,0x38}, // 'b'
  {0x38,0x44,0x44,0x44,0x20}, // 'c'
  {0x38,0x44,0x44,0x48,0x7F}, // 'd'
  {0x38,0x54,0x54,0x54,0x18}, // 'e'
  {0x08,0x7E,0x09,0x01,0x02}, // 'f'
  {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
  {0x7F,0x08,0x04,0x04,0x78}, // 'h'
  {0x00,0x44,0x7D,0x40,0x00}, // 'i'
  {0x20,0x40,0x44,0x3D,0x00}, // 'j'
  {0x7F,0x10,0x28,0x44,0x00}, // 'k'
  {0x00,0x41,0x7F,0x40,0x00}, // 'l'
  {0x7C,0x04,0x18,0x04,0x78}, // 'm'
  {0x7C,0x08,0x04,0x04,0x78}, // 'n'
  {0x38,0x44,0x44,0x44,0x38}, // 'o'
  {0x7C,0x14,0x14,0x14,0x08}, // 'p'
  {0x08,0x14,0x14,0x18,0x7C}, // 'q'
  {0x7C,0x08,0x04,0x04,0x08}, // 'r'
  {0x48,0x54,0x54,0x54,0x20}, // 's'
  {0x04,0x3F,0x44,0x40,0x20}, // 't'
  {0x3C,0x40,0x40,0x40,0x7C}, // 'u'
  {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
  {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
  {0x44,0x28,0x10,0x28,0x44}, // 'x'
  {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
  {0x44,0x64,0x54,0x4C,0x44}, // 'z'
  {0x00,0x08,0x36,0x41,0x00}, // '{'
  {0x00,0x00,0x7F,0x00,0x00}, // '|'
  {0x00,0x41,0x36,0x08,0x00}, // '}'
  {0x10,0x08,0x08,0x10,0x08}, // '~'
};

/* =======================================================================
 * Module init
 * ======================================================================= */
void OLED_Module_Init(I2C_HandleTypeDef *hi2c)
{
  oled_i2c = hi2c;
}

/* =======================================================================
 * Low-level I2C helpers
 * ======================================================================= */

void SSD1306_WriteCommand(uint8_t cmd)
{
  uint8_t buf[2] = {0x00, cmd}; // 0x00 = Co=0, D/C#=0 (command)
  HAL_I2C_Master_Transmit(oled_i2c, SSD1306_ADDR, buf, 2, SSD1306_TIMEOUT);
}

void SSD1306_WriteData(uint8_t *pData, uint16_t size)
{
  uint8_t tmp[129];              // 1 control byte + up to 128 data bytes
  if (size > 128) size = 128;
  tmp[0] = 0x40;                 // Co=0, D/C#=1 (data)
  for (uint16_t i = 0; i < size; i++) tmp[i + 1] = pData[i];
  HAL_I2C_Master_Transmit(oled_i2c, SSD1306_ADDR, tmp, size + 1, SSD1306_TIMEOUT);
}

/* =======================================================================
 * Hardware init — SH1106-compatible init sequence
 *
 * Key differences from SSD1306:
 *   - 0xAD, 0x8B : SH1106 internal DC-DC on  (NOT 0x8D,0x14 which is SSD1306)
 *   - 0xD9, 0x22 : pre-charge period per SH1106 datasheet
 *   - No 0x2E    : SH1106 has no hardware scroll (command unsupported)
 *   - No 0x20    : SH1106 is always page-addressing mode
 * ======================================================================= */
void SSD1306_Init(void)
{
  if (oled_i2c == NULL) return;

  if (HAL_I2C_IsDeviceReady(oled_i2c, SSD1306_ADDR, 3, 50) != HAL_OK)
  {
    oled_ok = 0;
    return;
  }

  HAL_Delay(10);

  SSD1306_WriteCommand(0xAE); // Display OFF

  SSD1306_WriteCommand(0xD5); // Set Display Clock Divide Ratio / Oscillator
  SSD1306_WriteCommand(0x80); //   Fosc=8 (reset), Divide ratio=1

  SSD1306_WriteCommand(0xA8); // Set Multiplex Ratio
  SSD1306_WriteCommand(0x3F); //   64MUX (1/64 duty, 64-row panel)

  SSD1306_WriteCommand(0xD3); // Set Display Offset
  SSD1306_WriteCommand(0x00); //   No vertical offset

  SSD1306_WriteCommand(0x40); // Set Display Start Line = 0

  // --- Charge Pump / DC-DC Converter (The most common reason for black screen) ---
  // We send both SSD1306 and SH1106 style commands to be safe
  SSD1306_WriteCommand(0x8D); 
  SSD1306_WriteCommand(0x14); // SSD1306 Enable Charge Pump
  
  SSD1306_WriteCommand(0xAD);
  SSD1306_WriteCommand(0x8B); // SH1106 Enable DC-DC

  SSD1306_WriteCommand(0xA1); // Set Segment Re-map: col 127 → SEG0 (Normal for 128x64)
  SSD1306_WriteCommand(0xC8); // Set COM Scan Direction: remapped (top→bottom)

  SSD1306_WriteCommand(0xDA); // Set COM Pins Hardware Configuration
  SSD1306_WriteCommand(0x12); //   Alternative COM pin, no left/right remap

  SSD1306_WriteCommand(0x81); // Set Contrast Control
  SSD1306_WriteCommand(0x80); //   0x80 = 128/255 (mid-range, suits SH1106)

  SSD1306_WriteCommand(0xD9); // Set Pre-charge Period
  SSD1306_WriteCommand(0x22); //   Phase1=2, Phase2=2 (SH1106 datasheet default)

  SSD1306_WriteCommand(0xDB); // Set VCOMH Deselect Level
  SSD1306_WriteCommand(0x40); //   ~0.77 x Vcc

  // NOTE: 0x2E "Deactivate Scroll" is SSD1306-only; SH1106 has no scroll.
  // Sending it to SH1106 has undefined effect — omit it.

  SSD1306_WriteCommand(0xA4); // Entire Display ON: output follows GDDRAM
  SSD1306_WriteCommand(0xA6); // Normal Display (not inverted)

  SSD1306_WriteCommand(0xAF); // Display ON

  oled_ok = 1;
  SSD1306_Clear();
  SSD1306_UpdateScreen();
}

/* =======================================================================
 * Framebuffer operations
 * ======================================================================= */

void SSD1306_Clear(void)
{
  memset(ssd1306_buf, 0x00, sizeof(ssd1306_buf));
  oled_cursor_page = 0;
  oled_cursor_col  = 0;
}

void SSD1306_UpdateScreen(void)
{
  if (!oled_ok) return;

  // SH1106 column addressing:
  //   Hardware has 132-column GDDRAM. Visible 128 pixels start at col 2.
  //   cmd 0x10 = Set Column Address HIGH nibble = 0  (col 0..15)
  //   cmd 0x02 = Set Column Address LOW  nibble = 2  (col start = 2)
  //   cmd 0xB0 | page = Set Page Address
  //
  // CRITICAL: All 3 address commands MUST be sent in ONE I2C transaction.
  // If each command gets its own START/STOP (as SSD1306_WriteCommand does),
  // the SH1106 treats each frame as complete and resets its address latch
  // mid-sequence — the column-low nibble never gets applied → missing pixels.
  //
  // This exactly matches U8g2 SH1106 I2C protocol:
  //   Frame 1: [0x00, 0x10, 0x02, 0xB0|page]   ← 3 commands, 1 I2C transfer
  //   Frame 2: [0x40, d0, d1, ..., d127]        ← 128 data bytes

  for (uint8_t page = 0; page < SSD1306_PAGES; page++)
  {
    // ---- Frame 1: address setup (3 commands in one I2C transaction) ----
    uint8_t addr[4];
    addr[0] = 0x00;         // Control byte: Co=0, D/C=0 → command stream
    addr[1] = 0x10;         // Set column HIGH nibble = 0
    addr[2] = 0x02;         // Set column LOW  nibble = 2  (SH1106 +2 offset)
    addr[3] = 0xB0 | page;  // Set page address
    HAL_I2C_Master_Transmit(oled_i2c, SSD1306_ADDR, addr, 4, SSD1306_TIMEOUT);

    // ---- Frame 2: pixel data (128 bytes in one I2C transaction) ----
    SSD1306_WriteData(ssd1306_buf[page], SSD1306_WIDTH);
  }
}

void SSD1306_SetCursor(uint8_t col, uint8_t page)
{
  oled_cursor_col  = (col  < SSD1306_WIDTH) ? col  : 0;
  oled_cursor_page = (page < SSD1306_PAGES) ? page : 0;
}

void SSD1306_WriteChar(char c)
{
  if (!oled_ok) return;
  if (oled_cursor_col + 6 > SSD1306_WIDTH) return;
  if (oled_cursor_page >= SSD1306_PAGES) return;

  uint8_t ch = (uint8_t)c;
  if (ch < 0x20 || ch > 0x7E) ch = '?';

  const uint8_t *glyph = font5x7[ch - 0x20];
  for (int i = 0; i < 5; i++)
    ssd1306_buf[oled_cursor_page][oled_cursor_col + i] = glyph[i];
  ssd1306_buf[oled_cursor_page][oled_cursor_col + 5] = 0x00;
  oled_cursor_col += 6;
}

void SSD1306_WriteString(uint8_t col, uint8_t page, const char *str)
{
  if (!oled_ok || !str) return;
  SSD1306_SetCursor(col, page);
  while (*str) {
    if (oled_cursor_col + 6 > SSD1306_WIDTH) break;
    SSD1306_WriteChar(*str++);
  }
}

/* =======================================================================
 * Menu drawing helpers
 * ======================================================================= */

/* Fill an entire page row (8 rows of pixels) with a solid colour.
 * colour = 0xFF → all pixels ON (white bar)
 * colour = 0x00 → all pixels OFF (clear) */
void SSD1306_FillRow(uint8_t page, uint8_t colour)
{
  if (page >= SSD1306_PAGES) return;
  memset(ssd1306_buf[page], colour, SSD1306_WIDTH);
}

/* Write a string at (col, page) with inverted pixels.
 * Used to render the currently selected menu item as white text on a
 * black background by XOR-ing each glyph byte with 0xFF. */
void SSD1306_WriteStringInverted(uint8_t col, uint8_t page, const char *str)
{
  if (!oled_ok || !str) return;
  if (page >= SSD1306_PAGES) return;

  /* Fill the whole row white first so characters appear against a solid bar */
  SSD1306_FillRow(page, 0xFF);

  uint8_t cur_col = (col < SSD1306_WIDTH) ? col : 0;

  while (*str) {
    if (cur_col + 6 > SSD1306_WIDTH) break;

    uint8_t ch = (uint8_t)(*str);
    if (ch < 0x20 || ch > 0x7E) ch = '?';

    const uint8_t *glyph = font5x7[ch - 0x20];
    for (int i = 0; i < 5; i++)
      ssd1306_buf[page][cur_col + i] = glyph[i] ^ 0xFF; /* invert */
    ssd1306_buf[page][cur_col + 5] = 0xFF;               /* gap — stays white */
    cur_col += 6;
    str++;
  }
}

/* =======================================================================
 * Legacy wrappers — keeps all existing call sites in mpu6050.c and
 * main.c unchanged; no functional difference.
 * ======================================================================= */

void OLED_Init(void)  { SSD1306_Init(); }

void OLED_Clear(void)
{
  SSD1306_Clear();
  SSD1306_UpdateScreen();
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *str)
{
  if (!oled_ok) return;
  SSD1306_WriteString(x, page, str);
  SSD1306_UpdateScreen();
}

/* Status on page 7 (bottom row) — leaves MPU data on pages 0-6 intact */
void OLED_ShowStatus(const char *status)
{
  if (!oled_ok) return;
  memset(ssd1306_buf[7], 0x00, SSD1306_WIDTH);
  SSD1306_WriteString(0, 7, status);
  SSD1306_UpdateScreen();
}
