//Library für SPI einfügen. CANBUS verwendet zur Kommunikation mit dem CPU intern SPI
#include <SPI.h>

//Library für CANBUS inkludieren. MCP steht dabei für den Hersteller Microchip
#include <mcp2515.h>

//Pin für die LED (zum Steuern mit Taster)
#define PIN_LED 4

//Pin für LED (zum Steuern der Helligkeit mit Joystick)
#define PIN_LED_JOYSTICK  6

//Pin für den Taster
#define PIN_SWITCH  2

//Pin für den LDR
#define PIN_LDR A1

/* Commands definieren -> diese werden später vom Master empfangen bzw. an den Master geschickt
  Theoretisch könnte man für die Befehle jeden x-beliebigen Wert nehmen, wichtig ist nur, das jeder Befehl am Master und am Slave den 
  selben Wert hat. Command LED_OFF kann also nicht am Master 0x01 und am Slave 0xFF sein
*/
#define LED_OFF       0x01  //LED am Slave ausschalten
#define LED_ON        0x02  //LED am Slave einschalten
#define READ_SWITCH   0x03  //Den Taster am Slave einlesen
#define READ_LDR      0x04  //Den Wert des LDRs am Slave auslesen
#define READ_JOYSTICK 0x05  //Den Wert des Joysticks am Master empfangen
#define NO_ACTION     0xFF

Mcp2515 mcp2515 (10)
struct can_frame canMsg;

byte received;
boolean previousLedState = false;

void setup(){
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  setupCAN();
}

void setupCAN(){
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void loop(){
  if(mcp2515.readMessage(&canMsg) == MCP2515::ERROK_OK){
    received = canMsg.data[0];
    doCommand();
  }
}

void doCommand(){
  switch(received){
    case LED_ON:
      //Überprüfen, ob der vorherige LED Zustand FALSE ist, die LED also ausgeschaltet ist
      if(!previousLedState)
      {
        //die LED am Slave einschalten
        digitalWrite(PIN_LED, HIGH);

        //den letzten Zustand auf den aktuellen Zustand (also eingeschaltet = TRUE) aktualisieren
        previousLedState = true;
      }
      break;
    case LED_OFF:
      //Überprüfen, ob der vorherige LED Zustand TRUE ist, die LED also eingeschalten ist
      if(previousLedState)
      {
        //die LED am Slave ausschalten
        digitalWrite(PIN_LED, LOW);

        //den letzten Zustand auf den aktuellen Zustand (also ausgeschaltet = FALSE) aktualisieren
        previousLedState = false;
      }
      break;
    case READ_SWITCH:
      writeCANByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;  
    case READ_LDR:
      int ldr = analogRead(PIN_LDR);
      writeCANInt(ldr);
      break;  
    case READ_JOYSTICK:
      int joy = canMsg.data[1] * 256 + canMsg.data[2];
      int map = map(joy, 0, 1023, 0, 255);
      analogWrite(PIN_LED, map);
      break;  
    case NO_ACTION:
      break;
    default:
      break;
  }
}

void writeCANByte(byte value){
  canMsg.can_id = 0x01;
  canMsg.dlc = 1;

  canMsg.data[0] = value;
  mcp2515.sendMessage(&canMsg);
}

void writeCANInt(int value){
  canMsg.can_id = 0x02;
  canMsg.can_dlc = 2;

  canMsg.data[0] = (value / 256) & 0xFF;
  canMsg.data[1] = value & 0xFF;
  mcp2515.sendMessage(&canMsg);
}