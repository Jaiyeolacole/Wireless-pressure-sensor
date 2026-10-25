#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* HD44780 20x4 LCD behind a PCF8574 I2C backpack (address 0x27 or 0x3F, auto-detected) */

HAL_StatusTypeDef lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *s);
void lcd_print_at(uint8_t col, uint8_t row, const char *s);
uint8_t lcd_get_address(void);   /* 7-bit address found, 0 if none */

#endif /* LCD_I2C_H */
