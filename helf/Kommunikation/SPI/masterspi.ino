#include <SPI.h>

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

#define READ_SWITCH_DELAY 1000  //Delay zwischen dem einlesen von 2 Switch-Befehlen
#define READ_LDR_DELAY    2500  //Delay zwischen dem einlesen von 2 LDR-Werten
#define WRITE_READ_DELAY  1     //Delay zwischen dem Senden eines Wertes und dem Ergebnis das zurückkommt

long prevReadSwitch = 2000L;
long prevReadLDR = 1800L;

boolean prevLed = false;
boolean prevSwitch = false;
int prevJoy;

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  setupSPI();
  prevJoy = analogRead(PIN_JOYSTICK);
}

void setupSPI(){
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  digitalWrite(SS, HIGH);
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
    writeSPIByte(currSwitch ? LED_ON : LED_OFF);
  }
}

void readSlaveSwitch(){
  byte received = 0;
  long now = millis();
  if(now - READ_SWITCH_DELAY > previousReadSwitch){
    previousReadSwitch = now;
    if(requestSPIByte(READ_SWITCH, &received)){
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
    if(requestSPIInt(READ_LDR, &received)){
      Serial.write(ldr);
    }
  }
}

void doJoystick(){
  int currJoy = analogRead(PIN_JOYSTICK);
  if(currJoy != prevJoy){
    writeSPIByte(SEND_JOY);
    delay(1);
    writeSPIInt(SEND_JOY, currJoy);
    prevJoy = currJoy;
  }
}

void writeSPIByte(byte value){
  digitalWrite(SS, LOW);
  SPI.transfer(value);
  digitalWrite(SS, HIGH);
}

void writeSPIInt(int value){
  digitalWrite(SS, LOW);
  SPI.transfer((value / 256) & 0xFF);
  SPI.transfer(value & 0xFF);
  digitalWrite(SS, HIGH);
}

void requestSPIByte(byte command, byte *value){
  digitalWrite(SS, LOW);
  SPI.transfer(command);
  digitalWrite(SS, HIGH);
  delay(1);

  digitalWrite(SS, LOW);
  *value = SPI.transfer(NO_ACTION);
  digitalWrite(SS, HIGH);
}

void requestSPIInt(byte command, int *value){
  byte received[2];
  digitalWrite(SS, LOW);
  SPI.transfer(command);
  digitalWrite(SS, HIGH);
  delay(1);

  digitalWrite(SS, LOW);
  received[0] = SPI.transfer(NO_ACTION);
  delay(1);
  received[1] = SPI.transfer(NO_ACTION);
  digitalWrite(SS, HIGH);
  *value = received[0] * 256 + received[1];
}