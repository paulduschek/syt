#include <SoftwareSerial.h>

#define PIN_RX 7
#define PIN_TX 8

#define PIN_LED A1
#define PIN_LDR A2
#define PIN_SWITCH A3

#define LED_ON 0x01
#define LED_OFF 0x02
#define READ_LED 0x03
#define READ_LDR 0x04
#define SEND_JOY 0x05
#define NO_ACTION 0xFF

byte received;
boolean prevLed = false;

SoftwareSerial softSerial(PIN_RX, PIN_TX);

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  softSerial.begin(9600);
}

void loop(){
  if(softSerial.available() > 0){
    received = softSerial.read();
    doCommand();
  }
}

void doCommand(){
  switch(received){
    case LED_ON:
      if(!prevLed){
        digitalWrite(PIN_LED, HIGH);
        prevLed = true;
      }
      break;
    case LED_OFF:
      if(prevLed){
        digitalWrite(PIN_LED, LOW);
        prevLed = false;
      }
      break;  
    case READ_SWITCH:
      writeSerialByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;  
    case READ_LDR:
      int ldr = analogRead(PIN_LDR);
      writeSerialInt(ldr);
      break;  
    case SEND_JOY:
      int joy;
      if(readSerialInt(&joy)){
        int map = map(joy, 0, 1023, 0, 255);
        analogWrite(PIN_LED, map);
      }
      break;  
    case NO_ACTION:
      break;
    default:
      break;  
  }
}

void writeSerialByte(byte value){
  softSerial.write(value);
}

void writeSerialInt(int value){
  softSerial.write((value / 256) & 0xFF);
  softSerial.write(value & 0xFF);
}

boolean readSerialInt(int *value){
  boolean success = false;
  byte received[2];

  success = softSerial.available >= 2;
  if(success){
    received[0] = softSerial.read();
    received[1] = softSerial.read();
    *value = received[0] * 256 + received[1];
  }
  return success;
}