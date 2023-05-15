/* Musterlösung für CANBUS-Kommunikation - MASTER
  Author: Niklas Schachl
  Date: 07.01.2023
*/
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
#define READ_JOYSTICK_DELAY 500   //Delay zwischen dem Einlesen von 2 Joystick-Werten 
#define WRITE_READ_DELAY    1     //Delay zwischen dem Senden eines Wertes und dem Ergebnis das zurückkommt

//Den CANBUS initialisieren
MCP2515 mcp2515(10);

//Einen Array an CANBUS-Frames initialisieren: 3 zum Senden und eines zum Empfangen (deswegen 4). Es würde aber mit 1 auch gehen, der Code wäre dann aber nicht so übersichtlich
struct can_frame canMsg[4];

//Variable zum zwischenspeichern des letzten Zustands des Tasters
static boolean previousSwitchState = false;

//Variable zum zwischenspeichern wann der Taster vom Slave das letzte Mal eingelesen wurde
static unsigned long previousReadSwitch = 2000L;

//Variable zum zwischenspeichern des letzten Zustands der LED am Master
static boolean previousLedState = false;

//Variable zum zwischenspeichern wann der LDR vom Slave das letzte Mal eingelesen wurde
static unsigned long previousReadLDR = 2000L;

//Variable zum zwischenspeichern wann der Joystick das letzte Mal eingelesen wurde
static unsigned long previousReadJoystick = 2000L;
// --------------------------------------------------------------------------------------------------------------------------------

/* 
    setup-Methode
*/
void setup() 
{
  //Serial-Konsole starten mit Baud Rate 115200
  Serial.begin(115200);

  //den LED-Pin als OUTPUT definieren
  pinMode(PIN_LED, OUTPUT);

  //CANBUS-Kommunikation initialisieren
  setupCAN();

  //CANBUS-Pakete bauen
  buildCanMsgs();
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setupCAN():
  CANBUS-Kommunikation initialisieren
*/
void setupCAN()
{
  //SPI-Kommunikation (zwischen CANBUS und der CPU) initialisieren
  SPI.begin();

  //CANBUS zurücksetzten
  mcp2515.reset();

  //Bitrate auf 125kbps setzen
  mcp2515.setBitrate(CAN_125KBPS);

  //CANBUS Modus einstellen (?)
  mcp2515.setNormalMode();
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  buildCanMsgs():
  CANBUS-Pakete zusammenbauen
*/
void buildCanMsgs()
{
  /*
    canMsg[0]: Paket für Switch State / LDR-Wert
    canMsg[1]: Paket für LED ein/aus
    canMsg[2]: Paket für das Senden des Joystick-Wertes
  */
  //Jedes Paket bekommt eine ID. Mit dieser wird priorisiert, wobei 0x00 die höchste Priorität und 0xFF die niedrigste Priorität hat
  canMsg[0].can_id = 0x0F6;

  //Data Length Code (DLC) gibt die Länge eines Pakets an. Wir senden bei diesem Paket nur maximal 3 Bytes (1 für den Befehl und 2 für den Integer-Wert beim LDR. Beim Switch senden wir sogar nur 2 Bytes)
  canMsg[0].can_dlc = 3;


  //Selbes Prozedere für das zweite Paket
  canMsg[1].can_id  = 0x036;  //ID -> diese ist niedriger als die obere, hat dadurch also eine höhere Priorität
  canMsg[1].can_dlc = 2;      //Länge des Datenpakets (bei den LEDs nur maximal 2, einen für den Befehl und einen für den Rückgabewert)

  //Selbes Prozeder für das dritte Paket
  canMsg[2].can_id = 0x022;    //ID -> diese ist niedriger als die oberen beiden, hat dadurch also eine höhere Priorität
  canMsg[2].can_dlc = 3;      //Länge des Datenpakets (1 Byte für den Befehl und 2 Bytes für den Integer des Joysticks)
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  loop-Methode
*/  
void loop() 
{
  //Eingabe vom Taster am MASTER einlesen und damit die LED am SLAVE ein- bzw. ausschalten
  doSwitch();

  //Eingabe vom Taster am SLAVE einlesen und damit die LED am MASTER ein- bzw. ausschalten
  readSlaveSwitch();

  //Wert vom LDR am SLAVE auslesen
  readSlaveLDR();

  //Wert vom Joystick am MASTER einlesen und damit die Helligkeit einer LED am SLAVE steuern
  sendSlaveJoystick();
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  doSwitch():
  Eingabe vom Taster am MASTER einlesen und damit die LED am SLAVE ein- bzw. ausschalten
*/
void doSwitch()
{
  //einlesen des Tasters. Da ein TAster nur EIN und AUS sein kann, wird digitalRead() verwendet
  boolean switchState = digitalRead(PIN_SWITCH);

  //Überprüfen ob sich der Zustand des Tasters geändert hat (also ob er gedrückt wurde oder nicht)
  if(switchState != previousSwitchState)
  {
    //den letzten Zustand des Tasters auf den aktuellen Zustand aktualisieren
    previousSwitchState = switchState;

    //Wenn switchState TRUE ist (Der Taster also gedrückt wurde), schicke den Befehl LED_ON (1 Byte) an den Slave. Ist switchState FALSE schicke den Befehl LED_OFF
    writeCANByte(switchState ? LED_ON : LED_OFF);
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  readSlaveSwitch():
  Eingabe vom Taster am SLAVE einlesen und damit die LED am MASTER ein- bzw. ausschalten
*/
void readSlaveSwitch()
{
  //Variable in die später der vom Slave empfangene Befehl (1 Byte) gespeichert wird
  byte received = 0;

  //aktuelle "Zeit" (= Millisekunden seit das Programm gestartet wurde)
  unsigned long now = millis();
  
  //Überprüfen, ob seit dem letzten Einlesen (previousReadSwitch) die gewünschte Delay-Zeit (READ_SWITCH_DELAY) vergangen ist
  if(now - READ_SWITCH_DELAY > previousReadSwitch)
  {
    //Zeit des letzten Einlesens auf aktuelle Zeit aktualisieren
    previousReadSwitch = now;

    /*
      requestCANByte(READ_SWITCH, &received): den Befehl READ_SWITCH an den Slave schicken und ein Ergebnis zurückverlangen. Dieses wird in der Variable received gespeichert
      if(...): Überprüfen, ob der Slave auch wirklich genügend Bytes gesendet hat
    */
    if(requestCANByte(READ_SWITCH, &received))
    {
      //Überprüfen, welcher Befehl vom Slave empfangen wurde
      switch(received)
      {
        case LED_ON:
          //Überprüfen, ob der vorherige LED Zustand FALSE ist, die LED also ausgeschaltet ist
          if(!previousLedState)
          {
            //LED am Master einschalten
            digitalWrite(PIN_LED, HIGH);

            //den letzten Zustand der LED auf den aktuellen Zustand aktualisieren
            previousLedState = true;                        
          }
          break;
        case LED_OFF:
          //Überprüfen, ob der vorherige LED Zustand TRUE ist, die LED also eingeschaltet ist
          if(previousLedState)
          {
            //LED am Master ausschalten
            digitalWrite(PIN_LED, LOW);

            //den letzten Zustand der LED auf den aktuellen Zustand aktualisieren
            previousLedState = false;
          }
          break;
        case NO_ACTION:
          break;
        default:
          //der DEFAULT-Case titt ein, wenn ein nicht erwarteter / nicht definierter Command vom Slave zurückgeschickt wurde (z.B. 0xCC)
          Serial.print(F("[ MASTER ] Received unexpected command: 0x"));
          Serial.println(received, HEX);
      }
    }
  }  
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  readSlaveLDR():
  Wert vom LDR am SLAVE auslesen
*/
void readSlaveLDR()
{
  //Variable, in die später der vom Slave erhaltene Wert (Integer) gespeichert wird
  unsigned int ldrValue;

  //aktuelle "Zeit" (= Millisekunden seit das Programm gestartet wurde);
  unsigned long now = millis();

  //Überprüfen, ob seit dem letzten einlesen (previousReadLDR) die gewünschte Delay-Zeit (READ_LDR_DELAY) vergangen ist
  if(now - READ_LDR_DELAY > previousReadLDR)
  {
    //Zeit des letzten Einlesens auf die aktuelle Zeit aktualisieren
    previousReadLDR = now;

    /*
      requestCANInt(READ_LDR, &ldrValue): den Befehl READ_LDR an den Slave schicken und ein Ergebnis anfordern. Dieses wird in der Variable ldrValue gespeichert
      if(...): Überprüfen, ob der Slave auch genügend Bytes gesendet hat
    */
    if(requestCANInt(READ_LDR, &ldrValue))
    {
      //Ausgabe des gelesenen Wertes auf der Konsole
      Serial.print(F("[ MASTER ] Read ldrValue from Slave: "));
      Serial.println(ldrValue);
    }
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  sendSlaveJoystick():
  Wert vom Joystick am MASTER einlesen und damit die Helligkeit einer LED am SLAVE steuern
*/
void sendSlaveJoystick()
{
  //aktuelle "Zeit" (= Millisekunden seit das Programm gestartet wurde)
  unsigned long now = millis();

  //Überprüfen ob seit dem letzten Einlesen (previousReadJoystick) die gewünschte Delay-Zeit (READ_JOYSTICK_DELAY) vergangen ist
  if(now - READ_JOYSTICK_DELAY > previousReadJoystick)
  {
    //Zeit des letzten Einlesens auf die aktuelle Zeit aktualisieren
    previousReadJoystick = now;

    //aktuellen Wert vom Joystick einlesen. Dieser liefert einen Wert zwischen 0 und 1023 (die Position), daher wird analogRead() verwendet
    int joystickValue = analogRead(PIN_JOYSTICK);

    //Den Wert des Joysticks (Integer) an den Slave schicken
    writeCANInt(WRITE_JOYSTICK, joystickValue);
  }
}  
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeCANByte():
  Ein Byte mittels CANBUS an den Slave senden
*/
void writeCANByte(byte value)
{
  //An die erste Datenstelle (.data[0]) des Pakets für die LED-Befehle (canMsg[1]) den zu sendenden Wert schreiben
  canMsg[1].data[0] = value;

  //Das Paket für die LED-Befehle(canMsg[1]) an den Slave senden
  mcp2515.sendMessage(&canMsg[1]);
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  requestCANByte():
  byte command: das Byte welches an den Slave gesendet wird (in unserem Fall immer der Befehl)
  byte *value: Adresse (!) der Variable in die das Byte, welches vom Slave empfangen wird, geschrieben werden soll
  --
  return: TRUE wenn die Übertragung korrekt abgelaufen ist, ansonsten FALSE
*/
boolean requestCANByte(byte command, byte *value)
{
  //Variable zum speichern ob die Übertragung korrekt abgelaufen ist
  boolean success = false;

  //an die erste Datenstelle (.data[0]) des Pakets für die Switch- und LDR-Befehle (canMsg[0]) den zu sendenden Befehl schreiben
  canMsg[0].data[0] = command;

  //Das Paket für die Switch- und LDR-Befehle (canMsg[0]) an den Slave senden
  mcp2515.sendMessage(&canMsg[0]);

  //Delay, damit der Slave genug Zeit hat den Befehl zu verarbeiten
  delay(WRITE_READ_DELAY);

  /*
    mcp2515.readMessage(&canMsg[3]): Empfange Daten vom Slave und speichere diese im Paket canMsg[3]
    MCP2515::ERROR_OK: ist die Übertragung korrekt abgelaufen, bekommen wir den Statuscode ERROR_OK
    mcp2515.readMessage(&canMsg[3]) == MCP2515::ERROR_OK: Wenn der Statuscode der Übertragung ERROR_OK entspricht, soll success TRUE werden, ansonsten FALSE
  */
  success = mcp2515.readMessage(&canMsg[3]) == MCP2515::ERROR_OK;

  //Überprüfe, ob die Übertragung korrekt abgelaufen ist
  if(success)
  {
    //Hole die Daten aus dem empfangenen Paket und speichere sie in der Variable value
    *value = canMsg[3].data[0];
  }
  return success;
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  requestCANInt():
  byte command: das Byte welches an den Slave gesendet wird (in unserem Fall immer der Befehl)
  int *value: Adresse (!) der Variable in die das Integer-Ergebnis vom Slave geschrieben werden soll
  --
  return: TRUE wenn die Übertragung korrekt abgelaufen ist, ansonsten FALSE
*/
boolean requestCANInt(byte command, int *value)
{
  //Variable zum speichern ob die Übertragung korrekt abgelaufen ist
  boolean success = false;

  //an die erste Datenstelle (.data[0]) des Pakets für die Switch- und LDR-Befehle (canMsg[0]) den zu übertragenden Befehl schreiben
  canMsg[0].data[0] = command;  

  //Das Paket für die Switch- und LDR-Befehle (canMsg[0]) an den Slave schicken
  mcp2515.sendMessage(&canMsg[0]);

  //Delay, damit der Salve genügend Zeit hat den Befehl zu verarbeiten
  delay(WRITE_READ_DELAY);

  /*
    mcp2515.readMessage(&canMsg[3]): Empfange Daten vom Slave und speichere diese im Paket canMsg[3]
    MCP2515::ERROR_OK: ist die Übertragung korrekt abgelaufen, bekommen wir den Statuscode ERROR_OK
    mcp2515.readMessage(&canMsg[3]) == MCP2515::ERROR_OK: Wenn der Statuscode der Übertragung ERROR_OK entspricht, soll success TRUE werden, ansonsten FALSE
  */
  success = mcp2515.readMessage(&canMsg[3]) == MCP2515::ERROR_OK;

  //Überprüfen, ob die Übertragung korrekt abgelaufen ist
  if(success)
  {
    //Den Integer-Wert aus den beiden Bytes im empfangenen Paket zusammenbauen
    *value = canMsg[3].data[0] * 256 + canMsg[3].data[1];
  }
  return success;
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeCANInt():
  Einen Integer-Wert (2 Byte) an den Slave schicken
*/
void writeCANInt(byte command, int value)
{
  //an die erste Datenstelle (.data[0]) des Pakets für die Joystick-Werte (canMsg[2]) den zu sendenden Befehl schreiben
  canMsg[2].data[0] = command;

  //Erstes Byte des Integers (Most Significant Byte, MSB) an die zweite Datenstelle (.data[1]) des Pakets schreiben
  canMsg[2].data[1] = (value / 256) & 0xFF;

  //Zweites Byte (Least Significant Byte, LSB) an die dritte Datenstelle (.data[2]) des Pakets schreiben
  canMsg[2].data[2] = value & 0xFF;

  //das Paket für die Joystick-Werte (canMsg[2]) an den Slave schicken
  mcp2515.sendMessage(&canMsg[2]);
}