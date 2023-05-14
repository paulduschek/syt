/* Musterlösung für Serial-Kommunikation - SLAVE
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
#define PIN_LED 4

//Pin für Taster
#define PIN_SWITCH  2

//Pin für LDR
#define PIN_LDR A1

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

//Delay zwischen 2 Einlesevorgängen
#define READ_READ_DELAY 2

//Variable, um 1 empfangenes Byte (meist der Befehl) zu speichern
volatile byte received;

//Variable die den letzten LED Zustand zwischenspeichert
static boolean previousLedState = false;

//Serial Kommunikation mit den RX und TX Pins initialisieren
SoftwareSerial softSerial(PIN_RX, PIN_TX);
// --------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(112500); //Serial-Konsole starten mit Baud Rate 112500

  pinMode(PIN_LED, OUTPUT);     //den LED-Pin als OUTPUT definieren

  softSerial.begin(9600); //Software-Serial starten mit Baud Rate von 9600

}
// --------------------------------------------------------------------------------------------------------------------------------

void loop() 
{
  //Überprüfen, ob Bytes vom Master geschickt wurden
  if(softSerial.available() > 0)
  {
    //1 Byte von der Leitung lesen und in der Variable speichern
    received = softSerial.read();

    //Den empfangenen Befehl vom Master verarbeiten
    doCommand();
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  doCommand():
  Verarbeitet den vom Master empfangenen Befehl und führt je nach Befehl die entsprechende Funktion durch
*/
void doCommand()
{
  //Den in received gespeicherten Befehl verarbeiten
  switch(received)
  {
    case LED_ON:
      //Überprüfen ob der vorherige LED Zustand FALSE ist, die LED also ausgeschaltet ist
      if(!previousLedState)
      {
        //LED am Slave einschalten 
        digitalWrite(PIN_LED, HIGH);

        //den aktuellen Zustand der LED speichern (TRUE = LED ist EINGESCHALTET)
        previousLedState = true;
      }          
      break;
    case LED_OFF:
      //Überprüfen ob der vorherige LED Zustand TRUE ist, die LED also eingeschaltet ist
      if(previousLedState)
      {
        //LED am Slave ausschalten
        digitalWrite(PIN_LED, LOW);

        //den aktuellen Zustand der LED speichern (FALSE = LED ist AUSGESCHALTET)
        previousLedState = false;
      } 
      break;
    case READ_SWITCH:
      //den Wert des Tasters einlesen und den entsprechenden Befehl an den Master senden
      writeSerialByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;
    case READ_LDR:
      //Einlesen des Wertes vom LDR am Slave
      int ldrValue = analogRead(PIN_LDR);

      //Den Wert des LDR an den Master senden
      writeSerialInt(ldrValue);
      break;
    case READ_JOYSTICK:
      //Variable in die der Integer-Wert vom Master gespeichert wird
      int joystickValue = 0;
      
      /*
        readSerialInt: Einen Integer Wert vom Master empfangen
        if(...) Überprüfen ob die Kommunikation ordnungsgemäß abgelaufen ist
      */
      if(readSerialInt(&joystickValue))
      {
        //Der Wert vom Joystick liegt zwischen 0 und 1023. Die LED hat aber eine Helligkeit von 0 bis 255. Deswegen muss der Joystick-Wert auf den Wertebereich der LED gemappt werden
        int ledValue = map(joystickValue, 0, 1023, 0, 255);

        //Der Helligkeits-Wert wird mit auf die LED hinausgeschrieben
        analogWrite(PIN_LED, ledValue);
      }
      break;
    case NO_ACTION:
      break;
    default:
    //Der DEFAULT-case tritt ein, wenn ein nicht erwarteter / nicht definierter Command vom Slave zurückgeschickt wurde (7.B. 0xA5)
    Serial.print(F("[ SLAVE ] received undefined command: 0x")); 
    Serial.println(received, HEX);
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeSerialByte:
  Ein Byte mittels Serial-Kommunikation an den Master senden
*/
void writeSerialByte(byte value)
{
  //Übertragung eines Bytes via Serial
  softSerial.write(value);
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeSerialInt()
  Einen Integer-Wert (2 Byte) an den Master schicken
*/
void writeSerialInt(int value)
{
  //Ein Integer ist 2 Byte lang, über die Leitung kann aber nur 1 Byte geschickt werden. Deswegen muss der 2-Byte-Integer aufgeteilt werden
  //Erstes Byte des Integers(Most Significant Byte, MSB) über Serial an den Master schicken
  softSerial.write((value / 256) & 0xFF);

  //Zweites Byte(Least Significant Byte, LSB) über Serial an den Slave schicken
  softSerial.write(value & 0xFF);
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  readSerialInt():
  Einen Integer-Wert (2 Byte) vom Master empfangen
    int *value: Adresse (!) der Variable in die der Empfangene Wert als Integer geschrieben werden soll
    --
    return: TRUE wenn 2 Byte erfolgreich gesendet wurden, ansonsten FALSE
*/
boolean readSerialInt(int *value)
{
  //Variable zum speichern ob die Übertragung korrekt abgelaufen ist
  boolean success = false;

  //Variable, in die der vom Master empfangen Integer-Wert geschrieben wird. Da ein Integer aus 2 Byte besteht, muss hier ein Array verwendet werden
  byte received[2];

  delay(READ_READ_DELAY);  

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
