#include <logutils.h>

File *file;
#define PATH "/media/sd/LOG.csv"

#define BUFFERLEN 6
char *buffer[6];

#define PIN_DHT11 A1
float *temp;
int *hum;
local_time_t *timestamp;

void main(){
	file = fopen(PATH, "a");
	while(1){
		for(int i = 0; i < BUFFERLEN; i++){
			getDateTime(&timestamp);
			if(getDHT11(PIN_DHT11, &temp, &hum) == 0){
				sprintf(buffer[i], "%0.4d-%0.2d-%0.2d;%0.2d:%0.2d:%0.2d;%f;%d\n", timestamp.year+1900, timestamp.month, timestamp.day, timestamp.hour, timestamp.minute, 
																			 timestamp.second, temp, hum);
			}
			delay(10000);	
		}
		writeToSD(buffer);
	}
}

void writeToSD(int values[BUFFERLEN]){
	if(file == null){
		printf("Error: file not found \n");
	}
	else{
		for(int i = 0; i < BUFFERLEN; i++){
			fprintf(*file, "%s", values[i]);
			fflush(file);
		}
	}
}