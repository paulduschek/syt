//Die Standardlibrary inkludieren
#include <stdlib.h>

//Library für SPI einfügen. CANBUS verwendet zur Kommunikation mit dem CPU intern SPI
#include <SPI.h>

//Library für CANBUS inkludieren. MCP steht dabei für den Herstelle Microchip
#include <mcp2515.h>

//Pin für LED
#define PIN_LED 4

//Pin für Taster
#define PIN_SWITCH  2

//Pin für Joystick
#define PIN_JOYSTICK  A1

/*
Commands definieren -> diese werden später an den Slave geschickt
Theoretisch könnte man für die Befehle jeden x-beliebigen Wert nehmen, wichtig ist nur, dass jeder Befeh lam Master und am Slave den
selben Wert hat. Command LED_OFF kann also nicht am Master 0x02 und am Slave 0xAA sein
*/
#define LED_OFF         0x01  //LED am Slave ausschalten
#define LED_ON          0x02  //LED am Slave einschalten
#define READ_SWITCH     0x03  //Taster am Slave einlesen
#define READ_LDR        0x04  //LDR am Slave auslesen
#define WRITE_JOYSTICK  0x05  //Joystick-Wert am Master einlesen und an den Slave schicken
#define NO_ACTION       0xFF  //Befehl, nichts zu tun. Vor allem hier bei SPI wichtig, da dieser Befehl vom Slave zurückkommt, wenn kein Wert zurückgeschickt werden soll

#define READ_SWITCH_DELAY   1000  //Delay zwischen dem Einlesen von 2 Switch-Befehlen
#define READ_LDR_DELAY      2500  //Delay zwischen dem Einlesen von 2 LDR-Werten
#define READ_JOY_DELAY      2500
#define READ_JOYSTICK_DELAY 500   //Delay zwischen dem Einlesen von 2 Joystick-Werten 
#define WRITE_READ_DELAY    1     //Delay zwischen dem Senden eines Wertes und dem Ergebnis das zurückkommt

long previousReadSwitch = 2000L;
long previousReadLDR = 1500L;
long previousReadJoy = 2000L;

boolean prevSwitch = false;
boolean prevLed = false;

Mcp2515 mcp2515(10)
struct can_frame canMsg[4];

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  setupCAN();
  buildCanMsg();
}

void setupCAN(){
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void buildCanMsg(){
  // LDR + Switch
  canMsg[0].can_id = 0x01;
  canMsg[0].can_dlc = 3;

   // LED
  canMsg[1].can_id = 0x02;
  canMsg[1].can_dlc = 2;

  // Joystick
  canMsg[2].can_id = 0x03;
  canMsg[2].can_dlc = 3;
}

void loop(){
  doSwitch();
  readSlaveSwitch();
  readSlaveLDR();
  doJoystick();
}

void doSwitch(){
  boolean currSwitch = digitalRead(PIN_SWITCH);
  if(currSwitch != prevSwitch){
    prevSwitch = currSwitch;
    writeCANByte(currSwitch ? LED_ON : LED_OFF);
  }
}

void readSlaveSwitch(){
  byte received = 0;
  long now = millis();
  if(now - READ_SWITCH_DELAY > previousReadSwitch){
    previousReadSwitch = now;
    if(requestCANByte(READ_SWITCH, &received)){
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
    if(requestCANInt(READ_LDR, &ldr)){
      Serial.write(ldr);
    }
  }
}

void doJoystick(){
  long now = millis();
  if(now - READ_JOY_DELAY > previousReadJoy){
    int joyval = analogRead(PIN_JOYSTICK);
    previousReadJoy = now;
    writeCANInt(WRITE_JOYSTICK, joyval);
  }
}

void writeCANByte(byte value){
  canMsg[1].data[0] = value;
  mcp2515.sendMessage(&canMsg[1]);
}

void writeCANInt(byte command, int value){
  canMsg[2].data[0] = command;
  canMsg[2].data[1] = (value / 256) & 0xFF;
  canMsg[2].data[2] = value & 0xFF;
  mcp2515.sendMessage(&canMsg[2]);
}

boolean requestCANByte(byte command, byte *value){
  boolean sucess = false;
  canMsg[0].data[0] = command;
  mcp2515.sendMessage(&canMsg[0]);
  delay(1);

  success = mcp2515.readMessage(&canMsg[3]) == MCP2515::ERROK_OK;
  if(success){
    *value = canMsg[3].data[0];
  }
  return success;
}

boolean requestCANInt(byte command, int *value){
  boolean success = false;
  canMsg[1].data[0] = command;
  mcp2515.sendMessage(&canMsg[1]);
  delay(1);

  success = mcp2515.readMessage(&canMsg[3]) == MCP2515::ERROR_OK;
  if(success){
    *value = canMsg[3].data[0] * 256 + canMsg[3].data[1];
  }
  return success;
}