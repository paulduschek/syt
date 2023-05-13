#define LED_RED 3
#define LDR 5

void setup(){
	Serial.begin(115200);
	pinMode(LED_RED, OUTPUT);
	pinMode(LDR, INPUT);
}

void loop(){
	int ldrval = analogRead(NTC);
	int ledval = map(ldrval, 0, 1023, 0, 255);
	
	analogWrite(LED_RED, ledval);
	Serial.printf("LED: &d\n", ledval);
}