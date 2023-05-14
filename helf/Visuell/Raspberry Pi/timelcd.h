//
// Created by pauld on 30.03.2023.
//

#ifndef rightLCD_TIMELCD_H
#define rightLCD_TIMELCD_H

#include "lcd_i2c.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

char* getDate();
char* getTime();
void setupLcd(lcd_i2c_t *lcd);
void setOpts(lcd_i2c_t *lcd, int opt_clear, int opt_backlight, int rows, int cols);
void writeLcd(lcd_i2c_t *lcd, int x, int y, char* output);

#endif //rightLCD_TIMELCD_H
