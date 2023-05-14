#include "timelcd.h"

int8_t opt_backlight = 1;
int8_t opt_clear = 1;
int8_t opt_cols = 16;
int8_t opt_init;
int8_t opt_rows = 2;
int8_t opt_x=0;
int8_t opt_y=1;

lcd_i2c_t lcd={0};

#define PATH "/sys/bus/w1/devices/28-01145011ffaa/w1_slave"
#define MAXLENGTH 255
#define SEARCH_CHAR 't'

int main() {
    setupLcd(&lcd);
    setOpts(&lcd, opt_clear, opt_backlight, opt_rows, opt_cols);
    FILE *file = fopen(PATH, "r");
    char *currentLine = malloc(MAXLENGTH+1);
    bool isRunning = true;
    char *result = malloc(2048);

    if(file == NULL){
        printf("Error opening \n");
    }
    else{
        do{
            fgets(currentLine, MAXLENGTH+1, file);
            for(int i = 0; i < strlen(currentLine); i++){
                char currentChar = currentLine[i];
                if(currentChar == SEARCH_CHAR){
                    for(int j = (i+2); j < strlen(currentLine); j++){
                        if(currentLine[j] == "\n"){
                            isRunning = false;
                            i = strlen(currentLine);
                            j = i;
                        }
                        else{
                            strncat(result, &currentLine[j], 1);
                        }
                    }
                }
            }
        }
        while(isRunning && currentLine != NULL);
    }
    //fclose(file);

    double temp;
    char out[2048];
    sscanf(result, "%lf", &temp);
    temp = temp/1000;
    sprintf(out, "Temp: &f", temp);
    writeLcd(&lcd, 0, 0, out);
    //while (1){
        //writeLcd(&lcd, 0, 0, getDate());
        //writeLcd(&lcd, 0, 1, getTime());
        //writeLcd(&lcd, 0,0, out);





        // timestamp of current values
        /*time_t T; // epoche (= seconds since 1.1.1970 00:00)
        struct tm tm; // year -1900, month - 1, day of month, ...
        // time stamp as text
        char timeStampDate[24];
        char timeStampTime[24];

        // get current time
        T = time(NULL);
        // translate epoche into date
        tm = *localtime(&T);
        // build text from time stamp values
        sprintf(timeStampDate,"%04d-%02d-%02d ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);

        sprintf(timeStampTime, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

        // init lcd
        lcd_i2c_t lcd={0};
        if( lcd_i2c_setup(&lcd,LCD_I2C_PCF8574_ADDRESS_DEFAULT) == -1  ){
            printf("Error intialising PCF8574 at address i2c 0x%02x: %s\n",LCD_I2C_PCF8574_ADDRESS_DEFAULT);
        }

        if(opt_clear == 1){
            lcd_i2c_clear(&lcd);
            opt_clear = 0;
        }

        if(opt_init)
            lcd_i2c_init(&lcd);

        // backlight
        if(opt_backlight==0){
            LCD_I2C_BACKLIGHT_OFF(&lcd);
        }else if(opt_backlight==1){
            LCD_I2C_BACKLIGHT_ON(&lcd);
        }

        // geometry
        lcd.rows=opt_rows;
        lcd.cols=opt_cols;

        // x and y
        // note: if only one is specified, the other is set to 0
        lcd.x=0;
        lcd.y=0;
        if(opt_x!=-1){
            lcd_i2c_gotoxy(&lcd, lcd.x,lcd.y);
            lcd_i2c_puts(&lcd, timeStampDate);
        }

        lcd.x = 0;
        lcd.y = 1;
        if(opt_x!=-1){
            lcd_i2c_gotoxy(&lcd, lcd.x,lcd.y);
            lcd_i2c_puts(&lcd, timeStampTime);
        }*/

    //}
    return 0;
}