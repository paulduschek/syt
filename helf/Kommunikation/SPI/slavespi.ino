#include <SPI.h>

//Pin für LED (zum Steuern mit Taster)
#define PIN_LED 4

//Pin für LED (zum Steuern der Helligkeit mit Joystick)
#define PIN_LED_JOYSTICK  6

//Pin für Taster
#define PIN_SWITCH  2

//Pin für LDR
#define PIN_LDR A1

/* Commands definieren -> diese werden später vom Master empfangen bzw. an den Master geschickt
  Theoretisch könnte man für die Befehle jeden x-beliebigen Wert nehmen, wichtig ist nur, das jeder Befehl am Master und am Slave den 
  selben Wert hat. Command LED_OFF kann also nicht am Master 0x01 und am Slave 0xFF sein
*/
#define LED_OFF       0x01
#define LED_ON        0x02
#define READ_SWITCH   0x03
#define READ_LDR      0x04
#define READ_JOYSTICK 0x05
#define NO_ACTION     0xFF

boolean jobAvailable;
byte received;
boolean prevLed = false;

int unsentCount = 0;
byte unsentData[2];

int unreadCount = 0;
byte readData[2];

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  setupSPI();
}

void setupSPI(){
  jobAvailable = false;
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.atachInterrupt();
}

void loop(){
  if(jobAvailable){
    noInterrupts();
    jobAvailable = false;
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
      writeSPIByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;   
    case READ_LDR:
      int ldr = analogRead(PIN_LDR);
      writeSPIInt(ldr);
      break;    
    case SEND_JOY:
      int joy = readData[0] + readData[1] * 256;
      int map = map(joy, 0, 1023, 0, 255);
      analogWrite(PIN_LED, map);
      break;  
    case NO_ACTION:
      break;
    default:
      break;  
  }
}

ISR(SPI_STC_vect){
  if(unreadCount > 0){
    readData[--unreadData] = SPDR;
    SPDR = NO_ACTION;
    if(unreadData == 0){
      jobAvailable = true;
    }
  }
  else if(unsentCount > 0){
    SPDR = unsentData[--unsentCount];
  }
  else{
    received = SPDR;

    if(received == READ_JOYSTICK){
      unreadCount = 2;
      jobAvailable = false;
      SPDR = NO_ACTION;
    }
    else{
      jobAvailable = true;
    }
    SPDR = NO_ACTION;
  }
}

void writeSPIByte(byte value){
  noInterrupts();
  SPDR = value;
  interrupts();
}

void writeSPIInt(int value){
  noInterrupts();
  SPDR = (value / 256) & 0xFF;
  unsentCount = 1;
  unsentData[0] = value & 0xFF;
  interrupts();
}