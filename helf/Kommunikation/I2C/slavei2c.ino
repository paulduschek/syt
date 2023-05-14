//Library für I²C hinzufügen
#include <Wire.h>

//Im Gegensatz zu Serial werden bei I²C keine Leitungen (RX. TX) angesprochen sondern Adressen
#define SLAVE_I2C_ADDRESS 4 //I²C-Adresse vom Master

//Pin für LED
#define PIN_LED 4

//Pin für Taster
#define PIN_SWITCH  2

//Pin für LDR
#define PIN_LDR  A1

/* Commands definieren -> diese werden später vom Master empfangen bzw. an den Master geschickt
  Theoretisch könnte man für die Befehle jeden x-beliebigen Wert nehmen, wichtig ist nur, das jeder Befehl am Master und am Slave den 
  selben Wert hat. Command LED_OFF kann also nicht am Master 0x01 und am Slave 0xFF sein
*/
#define LED_OFF         0x01  //LED am Slave ausschalten
#define LED_ON          0x02  //LED am Slave einschalten
#define READ_SWITCH     0x03  //Den Taster am Slave einlesen
#define READ_LDR        0x04  //Den Wert des LDRs am Slave auslesen
#define READ_JOYSTICK   0x05  //Den Wert des Joysticks vom Master empfangen (ACHTUNG: dieser Command wurde im Master WRITE_JOYSTICK genannt. Die Benennung ist allerdings egal solange der Wert der gleiche ist)
#define NO_ACTION       0xFF  // dummy command for 'do nothing'

boolean jobAvailable;
byte received;
int joyRec;
int joyVal;
boolean prevLed = false;

int unsentCount = 0;
byte unsentData[2];

int unreadCount = 0;
byte readData[2];

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  setupI2C();
}

void setupI2C(){
  Wire.begin(SLAVE_I2C_ADDRESS);

  jobAvailable = false;

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void receiveEvent(int howMany){
  if(Wire.available() > 0){
    if(unreadCount > 0){
      while(unreadCount > 0 && Wire.available() > 0){
        readData[--unreadCount] = Wire.read();
      }

      if(unreadCount == 0){
        jobAvailable = true;
        joyRec = readData[0] + readData[1] * 256;
      }
    }
    else{
      received = Wire.read();

      if(received == READ_JOYSTICK){
        unreadCount = 2;
        jobAvailable = false;
      }
      else{
        jobAvailable = true;
      }
    }
  }
}
  

void requestEvent(){
  while(unsentCount > 0){
    Wire.write(unsentData[--unsentCount]);
  }
}

void loop(){
  if(jobAvailable){
    noInterrupts();
    jobAvailable = false;
    joyVal = joyRec;
    interrupts();
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
      writeI2CByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;   
    case READ_LDR:
      writeI2CInt(analogRead(PIN_LDR));
      break;  
    case READ_JOYSTICK:
      int mapped = map(joyVal, 0, 1023, 0, 255);
      analogWrite(PIN_LED, mapped)
      break;   
    case NO_ACTION:
      break;
    default:
      break;  
  }
}

void writeI2CByte(byte value){
  noInterrupts();
  unsentCount = 1;
  unsentData[0] = value;
  interrupts();
}

void writeI2CInt(int value){
  noInterrupts();
  unsentCount = 2;
  unsentData[1] = (value / 256) & 0xFF;
  unsentData[0] = value & 0xFF;
  interrupts();
}