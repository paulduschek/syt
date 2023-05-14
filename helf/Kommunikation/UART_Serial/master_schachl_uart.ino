/* Musterlösung für Serial-Kommunikation - MASTER
  Author: Niklas Schachl
  Date: 19.11.2022
*/

//Library für Serial hinzufügen
#include <SoftwareSerial.h>

/* Pin-Layout definieren:
Master    Slave
RX   ->   TX
TX   ->   RX
*/
#define PIN_RX  7
#define PIN_TX  8

//Pin für LED
#define PIN_LED 3

//Pin für Taster
#define PIN_SWITCH  2

//Pin für Joystick
#define PIN_JOYSTICK A3

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
#define WRITE_READ_DELAY  2     //Delay zwischen dem senden eines Befehls und dem lesen des zurückgesendeten Ergebnis

//Variable zum zwischenspeichern des letzten Zustands des Tasters 
static boolean previousSwitchState = false;

//Variable zum speichern, wann der Taster vom Slave zuletzt gelesen wurde
static unsigned long previousReadSwitch = 2000L; 

//Variable die den letzten LED Zustand zwischenspeichert
static boolean previousLedState = false;

//Variable zum speichern, wann der LDR vom Slave zuletzt gelesen wurde
static unsigned long previousReadLDR = 2000L;

//Variable zum speichern der letzten Position des Joysticks
static int previousJoystickValue = 0;

//Serial Kommunikation mit den RX und TX Pins initialisieren
SoftwareSerial softSerial(PIN_RX, PIN_TX);
// ------------------------------------------------------------------------------------------------------------------------------------------

/* 
    setup-Methode
*/
void setup()
{
  Serial.begin(112500); //Serial-Konsole starten mit Baud Rate 112500

  pinMode(PIN_LED, OUTPUT);     //den LED-Pin als OUTPUT definieren
  pinMode(PIN_JOYSTICK, INPUT); //den Joystick-Pin als INPUT definieren

  softSerial.begin(9600);   //Software-Serial starten mit Baud Rate von 9600
}
// ------------------------------------------------------------------------------------------------------------------------------------------

/*
  loop-Methode
*/
void loop() 
{
  doSwitch();         //Eingabe vom Taster am MASTER auslesen und damit die LED am SLAVE (!) ein- bzw. ausschalten
  readSlaveSwitch();  //Eingabe vom Taster am SLAVE auslesen und damit die LED am MASTER ein- bzw. ausschalten
  readSlaveLDR();     //Wert vom LDR am Slave auslesen
  doJoystick();       //Eingabe vom Joystick auslesen und damit die Helligkeit einer LED am Slave steuern
  delay(1000);        //Delay zwischen Durchgängen der Loop
}
// ------------------------------------------------------------------------------------------------------------------------------------------

/*
  doSwitch():
  Eingabe vom Taster am MASTER auslesen und damit die LED am SLAVE (!) ein- bzw. ausschalten
*/
void doSwitch()
{
  //einlesen des Tasters. Da ein Taster nur EIN oder AUS sein kann, wird digitalRead() verwendet
  boolean switchState = digitalRead(PIN_SWITCH);

  //Überprüfen ob sich der Zustand des Tasters geändert hat (also ob er gedrückt wurde oder nicht)
  if(switchState != previousSwitchState)
  {
    //letzten Stand des Tasters auf aktuellen Stand aktualisieren (für nächsten Schleifendurchlauf wichtig)   
    previousSwitchState = switchState;
    
    //Wenn switchState TRUE ist (der Taster also gedrückt wurde) sende an den Slave den Befehl LED_ON, wenn switchState FALSE ist (der Taster also nicht gedrückt ist), sende LED_OFF
    writeSerialByte(switchState ? LED_ON : LED_OFF);
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------------

/*
  readSlaveSwitch():
  Eingabe vom Taster am SLAVE auslesen und damit die LED am MASTER ein- bzw. ausschalten  
*/
void readSlaveSwitch()
{
  //Variable, in die später der vom Slave erhaltene Befehl gespeichert wird
  byte received = 0;

  //aktuelle "Zeit" (Millisekunden seit das Programm gestartet wurde)
  unsigned long now = millis();  

  //überprüfung ob seit dem letzten auslesen (previousReadSwitch) die eingestellte delay-Zeit (READ_SWITCH_DELAY) vergangen ist
  if(now - READ_SWITCH_DELAY > previousReadSwitch)
  {
    //Zeit des letzten auslesen wird auf die aktuelle Zeit aktualisiert
    previousReadSwitch = now;

    /*
      requestSerialByte(READ_SWITCH, &received): sende den Befehl READ_SWITCH an den Slave und speichere das zurückgesendete ERgebnis in der VAriable received
      if(...): Überprüfung ob der Slave auch wirklich genügend Bytes zurückgesendet hat
    */
    if(requestSerialByte(READ_SWITCH, &received))
    {
      //Überprüfen, welcher Befehl vom Slave empfangen wurde
      switch(received)
      {
        case LED_ON:
          //Überprüfen ob der vorherige LED Zustand FALSE ist, die LED also ausgeschaltet ist
          if(!previousLedState)
          {
            //LED am Master einschalten
            digitalWrite(PIN_LED, HIGH);

            //den aktuellen Zustand der LED speichern (TRUE = LED ist EINGESCHALTET)
            previousLedState = true;
          }
          break;
        case LED_OFF:
          //Überprüfen ob der vorherige LED Zustand TRUE ist, die LED also eingeschaltet ist
          if(previousLedState)
          {
            //LED am Master ausschalten
            digitalWrite(PIN_LED, LOW);

            //den aktuellen Zustand der LED speichern (FALSE = LED ist AUSGESCHALTET)
            previousLedState = false;
          }                   
          break;
        case NO_ACTION:
          break;
        default:
          //Der DEFAULT-case tritt ein, wenn ein nicht erwarteter / nicht definierter Command vom Slave zurückgeschickt wurde (7.B. 0xA5)
          Serial.print(F("[ MASTER ] received unexpected command: 0x"));
          Serial.println(received, HEX);      
      }
    }
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------------

/*
  readSlaveLDR():
  Wert vom LDR am Slave auslesen
*/
void readSlaveLDR()
{
  //Variable, in die der Integer-Wert vom LDR später gespeichert wird
  unsigned int ldrValue;

  //aktuelle "Zeit" (Millisekunden seit das Programm gestartet wurde)
  unsigned long now = millis();

  //überprüfung ob seit dem letzten auslesen (previousReadLDR) die eingestellte delay-Zeit (READ_LDR_DELAY) vergangen ist
  if(now - READ_LDR_DELAY > previousReadLDR)
  {
    //Zeit des letzten auslesen wird auf die aktuelle Zeit aktualisiert
    previousReadLDR = now;

    /*
      requestSerialInt(READ_LDR, &ldrValue): sende den Befehl READ_LDR an den Slave und speichere das zurückgesendete ERgebnis in der Variable ldrValue
      if(...): Überprüfung ob der Slave auch wirklich genügend Bytes zurückgesendet hat
    */
    if(requestSerialInt(READ_LDR, &ldrValue))
    {
      //Ausgabe des gelesenen Wertes auf der Konsole
      Serial.print(F("[ MASTER ] Read ldrValue from Slave: "));
      Serial.println(ldrValue);
    }
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------------


/*
  doJoystick()
  Eingabe vom Joystick auslesen und damit die Helligkeit einer LED am Slave steuern
*/
void doJoystick()
{
  //einlesen des Wertes vom Joystick (dieser gibt uns einen Positionswert zwischen 0 und 1023 zurück, deswegen analogRead)
  int joystickValue = analogRead(PIN_JOYSTICK);

  //Überprüfen, ob sich die Position des Joysticks geändert hat
  if(joystickValue != previousJoystickValue)
  {
    //Den Befehl WRITE_JOYSTICK an den Slave senden
    writeSerialByte(WRITE_JOYSTICK);

    //Den Integer-Wert des Joysticks an den Slave senden
    writeSerialInt(joystickValue);

    //die letzte Position des Joysticks aktualisieren
    previousJoystickValue = joystickValue;     
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------------

/*
  writeSerialByte():
  Ein Byte an den Slave senden
*/
void writeSerialByte(byte value)
{
  //Übertragung eines Bytes via Serial
  softSerial.write(value);  
}
// ------------------------------------------------------------------------------------------------------------------------------------------

/*
  requestSerialByte():
  Ein Byte (den Befehl) an den Slave schicken und ein Byte als Rückgabe vom Slave empfangen
    byte command: das Byte welches an den Slave gesendet wird (der Befehl)
    byte *value:  Adresse (!) der Variable in die das Empfangene Byte geschrieben werden soll
    --      
    return: TRUE wenn 1 Byte erfolgreich empfangen wurde, ansonsten FALSE 
*/
boolean requestSerialByte(byte command, byte *value)
{
  //Variable zum speichern ob die Übertragung korrekt abgelaufen ist
  boolean success = false;

  //Ein Byte (den Befehl) an den Slave mittels Serial übertragen
  softSerial.write(command);

  //Delay, damit der Slave genug Zeit hat den Befehl zu empfangen und zurückzusenden
  delay(WRITE_READ_DELAY);

  //Überprüfen ob mindestens 1 Byte empfangen wird
  success = softSerial.available() >= 1;  //TRUE wenn >= 1 Byte empfangen wird, FALSE wenn weniger als 1 Byte empfangen wird

  //Falls mindestens 1 Byte empfangen wird, wird dieses Byte von der Leitung ausgelesen
  if(success)
  {
    //1 Byte vom slave mittels Serial lesen und speichern
    *value = softSerial.read();
  }

  return success;
}
// ------------------------------------------------------------------------------------------------------------------------------------------

/*
  requestSerialInt():
  Ein Byte (den Befehl) an den Slave schicken und 2 Byte als Rückgabe vom Slave empfangen. Die 2 Byte wieder zu einem Integer zusammenbauen
    byte command: das Byte welches an den Slave gesendet wird (der Befehl)
    int *value:  Adresse (!) der Variable in die der Empfangene Wert als Integer geschrieben werden soll
    --      
    return: TRUE wenn 2 Byte erfolgreich empfangen wurden, ansonsten FALSE  
*/
boolean requestSerialInt(byte command, int *value)
{
  //Variable zum speichern ob die Übertragung korrekt abgelaufen ist
  boolean success = false;

  //Variable, in die das vom Slave zurückgesendete Ergebnis geschrieben wird. Da ein Integer aus 2 Byte besteht, muss hier ein Array verwendet werden
  byte received[2];

  //Ein Byte (den Befehl) an den Slave mittels Serial übertragen
  softSerial.write(command);

  //Delay, damit der Slave genug Zeit hat den Befehl zu empfangen und zurückzusenden
  delay(WRITE_READ_DELAY);

  //Überprüfen ob mindestens 2 Byte empfangen wurden
  success = softSerial.available() >= 2;  //TRUE wenn >= 2 Byte empfangen wurden, FALSE wenn weniger als 2 Byte empfangen wurden

  //Falls mindestens 2 Bytes empfangen wurden, werden diese Bytes von der Leitung ausgelesen
  if(success)  
  {
    //Das erste Byte(Most Significant Byte, MSB) empfangen und speichern
    received[0] = softSerial.read();

    //Das zweite Byte(Least Significant Byte, LSB) empfangen und speichern
    received[1] = softSerial.read();

    //Den Integer-Wert aus beiden Bytes zusammenbauen
    *value = received[0] * 256 + received[1];    
  }

  return success;
}
// ------------------------------------------------------------------------------------------------------------------------------------------


/*
  writeSerialInt():
  Einen Integer-Wert (2 Byte) an den Slave schicken
*/
void writeSerialInt(int value)
{
  //Ein Integer ist 2 Byte lang, über die Leitung kann aber nur 1 Byte geschickt werden. Deswegen muss der 2-Byte-Integer aufgeteilt werden
  //Erstes Byte des Integers(Most Significant Byte, MSB) über Serial an den Slave schicken
  softSerial.write((value / 256) & 0xFF);

  //Zweites Byte(Least Significant Byte, LSB) über Serial an den Slave schicken
  softSerial.write(value & 0xFF);  
}
