#include "lcd_i2c.h"

/*
 * Typical PCF8574 backpack wiring:
 *   P0 = RS, P1 = RW, P2 = EN, P3 = backlight, P4..P7 = LCD D4..D7
 * The LCD runs in 4-bit mode: every byte is sent as two nibbles.
 */
#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08

static I2C_HandleTypeDef *s_hi2c;
static uint16_t s_addr8;   /* HAL wants the 8-bit (shifted) address */

static void pcf_write(uint8_t data)
{
    HAL_I2C_Master_Transmit(s_hi2c, s_addr8, &data, 1, 10);
}

/* LCD latches data on the falling edge of EN. One I2C byte at 100 kHz
 * takes ~90 us, far longer than the 450 ns minimum EN pulse. */
static void write_nibble(uint8_t nibble_hi, uint8_t flags)
{
    uint8_t d = (nibble_hi & 0xF0) | flags | LCD_BL;
    pcf_write(d | LCD_EN);
    pcf_write(d & (uint8_t)~LCD_EN);
}

static void lcd_send(uint8_t byte, uint8_t flags)
{
    write_nibble(byte & 0xF0, flags);
    write_nibble((uint8_t)(byte << 4), flags);
}

static void lcd_cmd(uint8_t c)
{
    lcd_send(c, 0);
    if (c == 0x01 || c == 0x02) {
        HAL_Delay(2);   /* clear / home are slow (~1.5 ms) */
    }
}

HAL_StatusTypeDef lcd_init(I2C_HandleTypeDef *hi2c)
{
    static const uint8_t candidates[] = { 0x27, 0x3F };
    s_hi2c = hi2c;
    s_addr8 = 0;

    for (uint32_t i = 0; i < sizeof(candidates); i++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(candidates[i] << 1), 3, 10) == HAL_OK) {
            s_addr8 = (uint16_t)(candidates[i] << 1);
            break;
        }
    }
    if (s_addr8 == 0) {
        return HAL_ERROR;
    }

    /* HD44780 "initialization by instruction" into 4-bit mode */
    HAL_Delay(50);
    write_nibble(0x30, 0); HAL_Delay(5);
    write_nibble(0x30, 0); HAL_Delay(1);
    write_nibble(0x30, 0); HAL_Delay(1);
    write_nibble(0x20, 0); HAL_Delay(1);   /* now in 4-bit mode */

    lcd_cmd(0x28);   /* 4-bit, 2-line mode (20x4 uses this), 5x8 font */
    lcd_cmd(0x08);   /* display off */
    lcd_cmd(0x01);   /* clear */
    lcd_cmd(0x06);   /* cursor moves right, no display shift */
    lcd_cmd(0x0C);   /* display on, cursor off */
    return HAL_OK;
}

void lcd_clear(void)
{
    lcd_cmd(0x01);
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    /* 20x4 DDRAM row start addresses: rows 2 and 3 continue rows 0 and 1 */
    static const uint8_t row_offset[4] = { 0x00, 0x40, 0x14, 0x54 };
    if (row > 3) row = 3;
    if (col > 19) col = 19;
    lcd_cmd((uint8_t)(0x80 | (row_offset[row] + col)));
}

void lcd_print(const char *s)
{
    while (*s) {
        lcd_send((uint8_t)*s++, LCD_RS);
    }
}

void lcd_print_at(uint8_t col, uint8_t row, const char *s)
{
    lcd_set_cursor(col, row);
    lcd_print(s);
}

uint8_t lcd_get_address(void)
{
    return (uint8_t)(s_addr8 >> 1);
}
