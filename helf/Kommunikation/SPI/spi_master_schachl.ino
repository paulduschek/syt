/* Musterlösung für SPI-Kommunikation - MASTER
  Author: Niklas Schachl
  Date: 06.01.2023
*/
//Library für SPI einfügen
#include <SPI.h>

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

#define READ_SWITCH_DELAY 1000  //Delay zwischen dem einlesen von 2 Switch-Befehlen
#define READ_LDR_DELAY    2500  //Delay zwischen dem einlesen von 2 LDR-Werten
#define WRITE_READ_DELAY  1     //Delay zwischen dem Senden eines Wertes und dem Ergebnis das zurückkommt

//Variable zum zwischenspeichern des letzten Zustands des Tasters
static boolean previousSwitchState = false;

//Variable zum zwischenspeichern, wann der Taster vom Slave das letzte Mal eingelesen wurde
static unsigned long previousReadSwitch = 2000L;

//Variable zum zwischenspeichern des letzten Zustands der LED am Master
static boolean previousLedState = false;

//Variable zum zwischenspeichern wann der LDR vom Slave das letzte Mal eingelesen wurde
static unsigned long previousReadLDR = 2000L;

//Wert zum zwischenspeichern des vorherigen Wertes des Joysticks
int previousJoystickValue;

// --------------------------------------------------------------------------------------------------------------------------------

/* 
    setup-Methode
*/
void setup()
{
  //Serial-Konsole starten mit Baud-Rate von 115200
  Serial.begin(115200);

  //den LED-Pin als OUTPUT definieren
  pinMode(PIN_LED, OUTPUT);

  //SPI-Kommunikation initialisieren
  setupSPI();

  //den Joystick einlesen und den Startwert speichern
  previousJoystickValue = analogRead(PIN_JOYSTICK);  
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setupSPI:
  SPI-Kommunikation initialisieren
*/
void setupSPI()
{
  //SPI-Kommunikation starten
  SPI.begin();

  //Den SPI-Takt auf 8 setzen (=> 16 / 8 = 2MHz)
  SPI.setClockDivider(SPI_CLOCK_DIV8);  

  //SlaveSelect-Leitung auf HIGH setzen und damit den Slave deaktivieren
  digitalWrite(SS, HIGH);
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

  //Delay zwischen den Durchgängen der Loop, beliebig
  delay(500);
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  doSwitch():
  Eingabe vom Taster am MASTER einlesen und damit die LED am SLAVE ein- bzw. ausschalten
*/
void doSwitch()
{
  //einlesen des Tasters. Da ein Taster nur EIN und AUS sein kann, wird digitalRead() verwendet
  boolean switchState = digitalRead(PIN_SWITCH);  

  //Überprüfen ob sich der Zustand des Tasters geändert hat (also ob er gedrückt wurde oder nicht)
  if(switchState != previousSwitchState)
  {
    //den letzten Zustand des Tasters auf den aktuellen Zustand aktualisieren (für den nächsten Schleifendurchlauf wichtig)
    previousSwitchState = switchState;

    //Wenn switchState TRUE ist (der Taster also gedrückt wurde), schicke den Befehl LED_ON (ein Byte) an den Slave. Ist switchState FALSE (der Taster also nicht gedrückt), sende den Befehl LED_OFF
    writeSPIByte(switchState ? LED_ON : LED_OFF);    
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  readSlaveSwitch():
  Eingabe vom Taster am SLAVE einlesen und damit die LED am MASTER ein- bzw. ausschalten
*/
void readSlaveSwitch()
{
  //Variable, in die später dem vom Slave erhaltene Befehl gespeichert wird
  byte received = 0;

  //aktuelle "Zeit" (= Millisekunden seit das Programm gestartet wurde)
  unsigned long now = millis();

  //Überprüfen ob seit dem letzten einlesen (previousReadSwitch) die gewünschte Delay-Zeit (READ_SWITCH_DELAY) vergangen ist
  if(now - READ_SWITCH_DELAY > previousReadSwitch)
  {
    //Zeit des letzten Einlesens auf die aktuelle Zeit aktualisieren
    previousReadSwitch = now;    

    //den Befehl READ_SWITCH an den Slave schicken und ein Ergebnis zurückverlangen. Dieses wird in der Variable received gespeichert. 
    requestSPIByte(READ_SWITCH, &received);
    //Überprüfen, welcher Befehl vom Slave empfangen wurde
    switch(received)
    {
      case LED_ON:
        //Überprüfen, ob der vorherige LED Zustand FALSE ist, die LED also ausgeschaltet ist
        if(!previousLedState)
        {
          //LED am Master einschalten
          digitalWrite(PIN_LED, HIGH);

          //den aktuellen Zustand der LED (also eingeschaltet = TRUE) speichern
          previousLedState = true;
        }
        break;
      case LED_OFF:
        //Überprüfen, ob der vorherige LED Zustand FALSE ist, die LED also ausgeschaltet ist   
        if(previousLedState)
        {
          //LED am Master ausschalten
          digitalWrite(PIN_LED, LOW);

          //den aktuellen Zustand der LED (also ausgeschaltet = FALSE) speichern
          previousLedState = false;
        }        
        break;
      case NO_ACTION:
        break;
      default:
        //der DEFAULT-Case titt ein, wenn ein nicht erwarteter / nicht definierter Command vom Slave zurückgeschickt wurde (z.B. 0xBB)
        Serial.print(F("[ MASTER ] Received unexpected command: 0x"));
        Serial.println(received, HEX);
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
  //Variable in die der vom Slave erhaltene Wert gespeichert wird, wenn er wieder zu einem Integer zusammengebaut wurde
  unsigned int ldrValue;    

  //aktuelle "Zeit" (= Millisekunden seit das Programm gestartet wurde)
  unsigned long now = millis();

  //Überprüfen, ob seit dem letzten einlesen (previousReadLDR) die gewünschte Delay-Zeit (READ_LDR_DELAY) vergangen ist
  if(now - READ_LDR_DELAY > previousReadLDR)
  {
    //Zeit des letzten einlesens auf die aktuelle Zeit aktualisieren
    previousReadLDR = now;

    //Den Befehl READ_LDR an den Slave schicken und ein Ergebnis anfordern. Dieses wird in der Variable ldrValue gespeichert
    requestSPIInt(READ_LDR, &ldrValue);

    //Ausgabe des gelesenen Wertes auf der Konsole
    Serial.print(F("[ MASTER ] Read ldrValue from Slave: "));
    Serial.println(ldrValue);
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  sendSlaveJoystick():
  Wert vom Joystick am MASTER einlesen und damit die Helligkeit einer LED am SLAVE steuern
*/
void sendSlaveJoystick()
{
  //aktuellen Wert vom Joystick einlesen. Dieser liefert einen Wert zwischen 0 und 1023 (die Position), daher wird analogRead() verwendet
  int joystickValue = analogRead(PIN_JOYSTICK);

  //Überprüfen, ob sich die Position des Joystick verändert hat
  if(joystickValue != previousJoystickValue)
  {
    //Den Befehl WRITE_JOYSTICK an den Slave senden
    writeSPIByte(WRITE_JOYSTICK);   

    //Delay zwischen schreiben und lesen
    delay(WRITE_READ_DELAY);
    
    //Den Integer-Wert des Joysticks an den Slave senden
    writeSPIInt(joystickValue);

    //die letzte Position des Joysticks auf die aktuelle Position aktualisieren
    previousJoystickValue = joystickValue;
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeSPIByte():
  Ein Byte an den Slave senden
*/
void writeSPIByte(byte value)
{
  //den Slave auswählen und die SPI Kommunikation starten indem man die Select-Leitung auf LOW setzt
  digitalWrite(SS, LOW);

  //Übertragen eines Bytes an den Slave, der Rückgabewert wird dabei ignoriert
  SPI.transfer(value);    

  //die Kommunikation mit dem Slave beenden, indem die Select-Leitung auf HIGH gesetzt wird
  digitalWrite(SS, HIGH);
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  requestSPIByte():
  byte command: das Byte welches an den Slave gesendet wird (in unserem Fall immer der Befehl)
  byte *value: Adresse (!) der Variable in die das Byte, welches vom Slave als Rückgabewert erhalten wird, geschrieben werden soll
*/  
void requestSPIByte(byte command, byte *value)
{
  //den Slave auswählen und die SPI Kommunikation starten indem man die Select-Leitung auf LOW setzt
  digitalWrite(SS, LOW);

  //Übertragen eines Bytes (des Befehls) an den Slave, der Rückgabewert wird dabei ignoriert
  SPI.transfer(command);

  //die Kommunikation mit dem Slave beenden, in dem die Select-Leitung auf HIGH gesetzt wird
  digitalWrite(SS, HIGH);

  //Delay, damit der Slave genug Zeit hat den Befeh zu verarbeiten
  delay(WRITE_READ_DELAY);

  //den Slave auswählen und die SPI Kommunikation starten indem man die Select-Leitung auf LOW setzt
  digitalWrite(SS, LOW);

  //den Befehl NO_ACTION an den Slave senden und den Rückgabewert (= das Ergebnis des letzten Befehls) speichern
  *value = SPI.transfer(NO_ACTION);

  //die Kommunikation mit dem Slave beenden, indem die Select-Leitung auf HIGH gesetzt wird
  digitalWrite(SS, HIGH);
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  requestSPIInt():   
  byte command: das Byte welches an den Slave gesendet wird (in unserem Fall immer der Befehl)
  int *value: Adresse (!) der Variable in die das Integer-Ergebnis vom Slave geschrieben werden soll
*/
void requestSPIInt(byte command, int *value)
{
  //Variable, in die das vom Slave zurückgesendete Ergebnis geschrieben wird. Da ein Integer aus 2 Byte besteht muss hier ein byte-Array mit 2 Stellen verwendet werden
  byte received[2];

  //den Slave auswählen und die SPI Kommunikation starten indem man die Select-Leitung auf LOW setzt
  digitalWrite(SS, LOW);

  //Übertragen eines Bytes (des Befehls) an den Slave, der Rückgabewert wird dabei ignoriert
  SPI.transfer(command);

  //die SPI Kommunikation beenden, indem die Select-Leitung auf HIGH gesetzt wird
  digitalWrite(SS, HIGH);

  //Delay, damit der Slave genug Zeit hat den Befehl zu verarbeiten
  delay(WRITE_READ_DELAY);

  //den Slave auswählen und die SPI Kommunikation starten indem die Select-Leitung auf LOW gesetzt wird
  digitalWrite(SS, LOW);

  //den Befehl NO_ACTION an den Slave senden und den Rückgabewert (= das Ergebnis des letzten Befehls) an der Stelle 0 im Array speichern.
  //hier wird das erste Byte des Integers (Most Significant Byte, MSB) gespeichert
  received[0] = SPI.transfer(NO_ACTION);

  //Delay zwischen zwei Sende-Vorgängen
  delay(WRITE_READ_DELAY);

  //den Befehl NO_ACTION an den Slave senden und den Rückgabewert (= das Ergebnis des letzten Befehls) an der Stelle 1 im Array speichern   
  //hier wird das zweite Byte des Integers (Least Significant Byte, LSB) gespeichert
  received[1] = SPI.transfer(NO_ACTION);

  //die SPI Kommunikation beenden, indem die Select-Leitung auf HIGH gesetzt wird
  digitalWrite(SS, HIGH);

  //Den Integer-Wert aus den beiden Bytes zusammenbauen
  *value = received[0] * 256 + received[1];
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeSPIInt():
  Einen Integer-Wert (2 Byte) an den Slave schicken
*/
void writeSPIInt(int value)
{
  //den Slave auswählen und die SPI Kommunikation starten indem die Select-Leitung auf LOW gesetzt wird
  digitalWrite(SS, LOW);

  //Erstes Byte des Integers (Most Significant Byte, MSB) mittels SPI an den Slave übertragen, der Rückgabewert wird dabei ignoriert
  SPI.transfer((value / 256) & 0xFF);

  //Delay zwischen zwei Schreib-Vorgängen
  delay(WRITE_READ_DELAY);

  //Zweites Byte (Least Significant Byte, LSB) mittels SPI an den Slave übertragen, der Rückgabewert wird dabei ignoriert
  SPI.transfer(value & 0xFF);

  //die SPI Kommunikation beenden indem die Select-Leitung auf HIGH gesetzt wird
  digitalWrite(SS, HIGH);
}