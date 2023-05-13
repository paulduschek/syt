#define PATH /sys/bus/w1/devices/28-01144fb9e5aa/w1_slave
#define MAXLENGTH 255
// t for temperature localted anywhere in the file but not directly in the beginning
#define SEARCH_CHAR 't'

void main(){
	File *file = fopen(PATH, "r");
	char *currentLine = malloc(MAXLENGTH+1);
	bool isRunning = true;
	char *result = malloc(2048);
	
	if(file == null){
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
		while(isRunning && currentLine != null);
	}
	fclose(file);
	
	double temp;
	sscanf(result, "%lf", &temp);
	temp = temp/1000;
	printf(temp);	
	return 0;
}