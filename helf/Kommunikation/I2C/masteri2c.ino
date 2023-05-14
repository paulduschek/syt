#include <Wire.h>

//Im Gegensatz zu Serial werden bei I²C keine Leitungen (RX. TX) angesprochen sondern Adressen
#define SLAVE_I2C_ADDRESS 4 //I²C-Adresse vom Slave

//Pin für LED
#define PIN_LED 3

//Pin für Taster
#define PIN_SWITCH  2

//Pin für Joystick
#define PIN_JOYSTICK  A1

/* Commands definieren -> diese werden später an den Slave geschickt
  Theoretisch könnte man für die Befehle jeden x-beliebigen Wert nehmen, wichtig ist nur, das jeder Befehl am Master und am Slave den 
  selben Wert hat. Command LED_OFF kann also nicht am Master 0x01 und am Slave 0xFF sein
*/
#define LED_OFF         0x01  //LED am Slave ausschalten
#define LED_ON          0x02  //LED am Slave einschalten
#define READ_SWITCH     0x03  //Den Taster am Slave einlesen
#define READ_LDR        0x04  //Den Wert des LDRs am Slave auslesen
#define WRITE_JOYSTICK  0x05  //Den Wert des Joysticks am Master an den Slave senden
#define NO_ACTION       0xFF  // dummy command for 'do nothing'

#define READ_SWITCH_DELAY 1000  //Delay zwischen dem Einlesen von zwei Switch-Befehlen
#define READ_LDR_DELAY    2500  //Delay zwischen dem Einlesen von zwei LDR-Werten
#define WRITE_READ_DELAY  1     //Delay zwischen dem senden eines Befehls und dem lesen des zurückgesendeten Ergebnis

long prevReadSwitch = 2000L;
long prevReadLDR = 2000L;

boolean prevSwitch = false;

int prevJoy;

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  Wire.begin();
  prevJoy = analogRead(PIN_JOYSTICK);
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
    writeI2CByte(currSwitch ? LED_ON : LED_OFF);
  }
}

void readSlaveSwitch(){
  byte received = 0;
  long now = millis();
  if(now - READ_SWITCH_DELAY > previousReadSwitch){
    previousReadSwitch = now;
    if(requestI2CByte(READ_SWITCH, &received)){
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
    if(requestI2CInt(READ_LDR, &ldr)){
      Serial.write(ldr);
    }
  }
}

void doJoystick(){
  int currJoy = analogRead(PIN_JOYSTICK);
  if(currJoy != prevJoy){
    writeI2ClByte(SEND_JOY);
    delay(1);
    writeI2CInt(currJoy);
    prevJoy = currJoy;
  }
}

void writeI2CByte(byte value){
  Wire.beginTransmission(SLAVE_I2C_ADDRESS);
  Wire.write(value);
  Wire.endTransmission();
}

void writeI2CInt(int value){
  Wire.beginTransmission(SLAVE_I2C_ADDRESS);
  Wire.write((value / 256) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

boolean requestI2CByte(byte command, byte *value){
  boolean success = false;
  Wire.beginTransmission(SLAVE_I2C_ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(SLAVE_I2C_ADDRESS, 1);
  success = Wire.available() >= 1;
  if(success){
    *value = Wire.read();
  } 
  return success;
}

boolean requestI2CInt(byte command, int *value){
  boolean success = false;
  byte received[2];
  Wire.beginTransmission(SLAVE_I2C_ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(SLAVE_I2C_ADDRESS, 2);
  success = Wire.available() >= 2;
  if(success){
    received[0] = Wire.read();
    received[1] = Wire.read();
    *value = received[0] * 256 + received[1];
  }
  return success;
}