#define LED_RED 3
#define NTC 5

void setup(){
	Serial.begin(115200);
	pinMode(LED_RED, OUTPUT);
	pinMode(NTC, INPUT);
}

void loop(){
	int ntcval = analogRead(NTC);
	int ledval = map(ntcval, 0, 1023, 0, 255);
	
	analogWrite(LED_RED, ledval);
	Serial.printf("LED: &d\n", ledval);
}