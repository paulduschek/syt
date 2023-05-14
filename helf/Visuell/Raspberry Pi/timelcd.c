//
// Created by pauld on 30.03.2023.
//

#include "timelcd.h"

time_t T;
struct tm tm;
char timeStampDate[24];
char timeStampTime[24];

char* getDate(){
    T = time(NULL);
    // translate epoche into date
    tm = *localtime(&T);
    // build text from time stamp values
    sprintf(timeStampDate,"%04d-%02d-%02d ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
    return timeStampDate;
}

char* getTime(){
    T = time(NULL);
    // translate epoche into date
    tm = *localtime(&T);
    sprintf(timeStampTime, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    return timeStampTime;
}

void setupLcd(lcd_i2c_t *lcd){
    if( lcd_i2c_setup(lcd,LCD_I2C_PCF8574_ADDRESS_DEFAULT) == -1  ){
        printf("Error intialising PCF8574 at address i2c 0x%02x: %s\n",LCD_I2C_PCF8574_ADDRESS_DEFAULT);
    }
}

void setOpts(lcd_i2c_t *lcd, int opt_clear, int opt_backlight, int rows, int cols){
    if(opt_clear == 1){
        lcd_i2c_clear(lcd);
    }
    lcd_i2c_init(lcd);

    if(opt_backlight==0){
        LCD_I2C_BACKLIGHT_OFF(lcd);
    }else if(opt_backlight==1){
        LCD_I2C_BACKLIGHT_ON(lcd);
    }

    lcd->rows=rows;
    lcd->cols=cols;
}

void writeLcd(lcd_i2c_t *lcd, int x, int y, char* output){
    lcd->x = x;
    lcd->y = y;
    lcd_i2c_gotoxy(lcd, lcd->x,lcd->y);
    lcd_i2c_puts(lcd, output);
}