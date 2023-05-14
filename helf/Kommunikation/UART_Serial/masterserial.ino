#include <SoftwareSerial.h>

#define PIN_LED A1
#define PIN_JOYSTICK A2
#define PIN_SWITCH A3

#define LED_ON 0x01
#define LED_OFF 0x02
#define READ_LED 0x03
#define READ_LDR 0x04
#define SEND_JOY 0x05
#define NO_ACTION 0xFF

#define READ_SWITCH_DELAY 2000
#define READ_LDR_DELAY 2200

#define PIN_RX 7
#define PIN_TX 8

long previousReadSwitch = 2000L;
long previousReadLDR = 2000L;

boolean prevSwitch = false;
int prevJoy;

SoftwareSerial softSerial(PIN_RX, PIN_TX);

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  softSerial.begin(9600);
  prevJoy = analogRead(PIN_JOYSTICK);
}

void loop(){
  doSwitch();
  readSlaveSwitch();
  readSlaveLDR();
  doJoystick();
  delay(500);
}

void doSwitch(){
  boolean currSwitch = digitalRead(PIN_SWITCH);
  if(currSwitch != prevSwitch){
    prevSwitch = currSwitch;
    writeSerialByte(currSwitch ? LED_ON : LED_OFF);
  }
}

void readSlaveSwitch(){
  byte received = 0;
  long now = millis();
  if(now - READ_SWITCH_DELAY > previousReadSwitch){
    previousReadSwitch = now;
    if(requestSerialByte(READ_SWITCH, &received)){
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
        case NO_ACTION:
          break;
        default:
          break;  
      }
    }
  }
}

void readSlaveLDR(){
  int ldr;
  long now = millis();
  if(now - READ_LDR_DELAY > previousReadLDR){
    previousReadLDR = now;
    if(requestSerialInt(READ_LDR, &received)){
      Serial.write(ldr);
    }
  }
}

void doJoystick(){
  int currJoy = analogRead(PIN_JOYSTICK);
  if(currJoy != prevJoy){
    writeSerialByte(SEND_JOY);
    delay(1);
    writeSerialInt(currJoy);
    prevJoy = currJoy;
  }
}

void writeSerialByte(byte value){
  softSerial.write(value);
}

void writeSerialInt(int value){
  softSerial.write((value / 256) & 0xFF);
  softSerial.write(value & 0xFF);
}

boolean requestSerialByte(byte command, byte *value){
  boolean success = false;
  softSerial.write(command);
  delay(1);

  success = softSerial.available() >= 1;
  if(success){
    *value = softSerial.read();
  }
  return success;
}

boolean requestSerialInt(byte command, int *value){
  boolean success = false;
  byte received[2];

  softSerial.write(command);
  delay(1);

  success = softSerial.available() >= 2;
  if(success){
    received[0] = softSerial.read();
    received[1] = softSerial.read();
    *value = received[0] * 256 + received[1];
  }
  return success;
}