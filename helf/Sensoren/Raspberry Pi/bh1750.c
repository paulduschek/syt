#include <stdio.h>
#include <stdlib.h>
#include <zconf.h>
#include <stdbool.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define BH1750_ADDRESS 0x5c
#define BH1750_CODE 0x10

int readBH1750(int address, int code);

int main(){
	while(true){
		int val = readBH1750(BH1750_ADDRESS, BH1750_CODE);
		printf("BH1750: &d\n", val);
	}
	return 0;
}

int readBH1750(int address, int code){
	int handle = wiringPiI2CSetup(address);
	
	wiringPiI2CWrite(handle, code);
	
	sleep(1);
	
	int word = wiringPiI2CReadReg16(handle, 0x00);		// read value
	int lux = ((word & 0xff00) >> 8) | ((word & 0x00ff) << 8);		// convert value to lux
	return lux;
}