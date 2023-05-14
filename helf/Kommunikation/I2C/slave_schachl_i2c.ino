/* Musterlösung für I²C-Kommunikation - SLAVE
  Author: Niklas Schachl
  Date: 19.11.2022
*/

//Library für I²C hinzufügen
#include <Wire.h>

//Im Gegensatz zu Serial werden bei I²C keine Leitungen (RX. TX) angesprochen sondern Adressen
#define MASTER_I2C_ADDRESS 4 //I²C-Adresse vom Master

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

//Variable, in die später der Wert des Joysticks geschrieben wird (im receiveEvent)
static int joystickValueReceive = 0;

//Variable, in die später der Wert des Joysticks geschrieben wird
static int joystickValue = 0;

//Variable die den letzten LED Zustand zwischenspeichert
static boolean previousLedState = false;

//Variable in die die empfangenen Daten gespeichert werden
volatile byte received;

//In diesem Array werden alle Daten abgelegt, die gelesen aber noch nicht verarbeitet wurden
volatile byte readData[2];

//Anzahl der Bytes die noch nicht verarbeitet wurden
volatile int unreadCount;

//Variable zum speichern ob ein Job verfügbar ist
volatile byte jobAvailable;

//In diesem Array werden alle Daten abgelegt, welche gesendet werden sollen
volatile byte unsentData[2];

//Anzahl der Bytes die noch gesendet werden müssen
volatile int unsentCount;
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setup-methode
*/
void setup() 
{
  //Serial-Konsole starten mit Baud Rate 112500
  Serial.begin(112500); 

  //den LED-Pin als OUTPUT definieren
  pinMode(PIN_LED, OUTPUT);    

  //I²C-Kommunikation initialisieren (siehe unten)
  setupI2C();

}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setupI2C:
  I²C-Kommunikation initialisieren
*/
void setupI2C()
{
  //Da noch nichts empfangen wurde, gibt es auch keine Jobs zu erledigen
  jobAvailable = false; 

  //Verbinde mit dem I²C-Bus auf Adresse 4                  
  Wire.begin(MASTER_I2C_ADDRESS); 

  //Registriere die Methode receiveEvent (siehe unten) als event handler für das Empfangen von Daten             
  Wire.onReceive(receiveEvent);           

  //Registriere die Methode requestEvent (siehe unten) als event handler zum Beantworten von Requests
  Wire.onRequest(requestEvent);           
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  receiveEvent:
  event handler für das Empfangen von Daten
*/
void receiveEvent(int howMany)
{
  //Überprüfen ob Daten gesendet wurden
  if(Wire.available() > 0)
  {
    //Überprüfe ob es Daten gibt die noch nicht gelesen wurden
    if(unreadCount > 0)
    {
      //Schleife wird ausgeführt so lange es noch ungelesene Daten gibt und Daten auf dem Bus ankommen
      while(unreadCount > 0 && Wire.available() > 0)
      {
        //die Daten vom Bus werden rückwärts in das Array geschrieben            
        readData[--unreadCount] = Wire.read();
      }

      //IF wird ausgeführt wenn alle Daten gelesen wurden
      if(unreadCount == 0)
      {
        //eingelesene Daten wieder zu einem Integer zusammenbauen. ACHTUNG: da die Daten rückwärts gelesen wurden, muss "* 256" auf die zweite Stelle ausgeführt werden
        joystickValueReceive = readData[0] + readData[1] * 256;

        //speichere das es etwas zu tun gibt
        jobAvailable = true;
      }
    }
    else  //ausgeführt, wenn alle Daten gelesen wurden
    {
      //1 Byte vom I²C-Bus auslesen
      received = Wire.read();

      //Überprüfe ob der empfangene Befehl READ_JOYSTICK, also ein Spezialfall ist
      if(received == READ_JOYSTICK)
      {
        //Wenn der empfangene Befehl READ_JOYSTICK ist, müssen noch 2 Bytes (der Joystick Integer-Wert) empfangen werden
        unreadCount = 2;
      }
      else
      {
        //speichere das es etwas zu tun gibt          
        jobAvailable = true;
      }
    }
  }  
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  requestEvent:
  event handler zum Beantworten von Requests
*/
void requestEvent()
{
  //Schleife wird ausgeführt, so lange es Daten gibt die noch nicht gesendet wurden
  while(unsentCount > 0)
  {
    //Der Array der ungesendeten Daten wird heruntergezählt und versendet
    Wire.write(unsentData[--unsentCount]);
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  loop-Methode
*/
void loop() 
{
  //Überprüfe, ob es etwas zu tun gibt
  if(jobAvailable)
  {
    //Deaktiviere Interrupts, damit kein Event die Variable gleichzeitig überschreibt
    noInterrupts();

    //Da der Job nun erledigt wird, kann jobAvailable wieder auf FALSE gesetzt werden, da es nun nichts mehr zu erledigen gibt
    jobAvailable = false;

    //Joystick-Wert vom receiveEvent abholen
    joystickValue = joystickValueReceive;

    //Interrupts wieder aktivieren
    interrupts();

    //Den Job ausführen
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
      writeI2CByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;
    case READ_LDR:
      //Einlesen des Wertes vom LDR am Slave
      int ldrValue = analogRead(PIN_LDR);

      //Den Wert des LDR an den Master senden
      writeI2CInt(ldrValue);
      break;
    case READ_JOYSTICK:
      //Der Wert vom Joystick liegt zwischen 0 und 1023. Die LED hat aber eine Helligkeit von 0 bis 255. Deswegen muss der Joystick-Wert auf den Wertebereich der LED gemappt werden
      int ledValue = map(joystickValue, 0, 1023, 0, 255);

      //Der Helligkeits-Wert wird mit auf die LED hinausgeschrieben
      analogWrite(PIN_LED, ledValue);
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
  writeI2CByte():
  Ein Byte mittels I²C an den Master senden
*/
void writeI2CByte(byte value)
{
  //Deaktiviere Interrupts, damit kein Event die Variable gleichzeitig überschreibt
  noInterrupts();

  //speichere das Ergebnis im Array unsentData, damit es bei der nächsten Übertragung an den Master gesendete wird
  unsentData[0] = value;

  //es muss nun 1 Byte übertragen werden. 
  unsentCount = 1;

  //aktiviere Interrupts
  interrupts();
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeI2CInt():
  Einen Integer-Wert (2 Byte) an den Master schicken
*/
void writeI2CInt(int value)
{
  //Deaktiviere Interrupts, damit kein Event die Variable gleichzeitig überschreibt
  noInterrupts();

  //Zweites Byte(Least Significant Byte, LSB) des Integers fürs Senden vormerken
  unsentData[0] = value & 0xFF;

  //Erstes Byte des Integers(Most Significant Byte, MSB) des Integers fürs Senden vormerken
  unsentData[1] = (value / 256) & 0xFF;

  //es müssen nun 2 Bytes übertragen werden
  unsentCount = 2;

  //aktiviere interrupts
  interrupts();
}
