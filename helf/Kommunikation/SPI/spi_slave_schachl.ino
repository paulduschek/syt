/* Musterlösung für SPI-Kommunikation - SLAVE
  Author: Niklas Schachl
  Date: 06.01.2023
*/

//Library für SPI einfügen
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

//Minimum Wert den der Joystick annehmen kann (später für "map" benötigt)
#define JOYSTICK_MIN  0

//Maximum Wert den der Joystick annehmen kann (später für "map" benötigt)
#define JOYSTICK_MAX  1023

//Minimum Wert der LED-Helligkeit (später für "map" benötigt)
#define LED_MIN 0

//Maximum Wert der LED-Helligkeit (später für "map" benötigt)
#define LED_MAX 255

//Variable in die 1-Byte-lange erhaltene Daten gespeichert werden
volatile byte received;

//Anzahl der Bytes die noch nicht empfangen wurden
volatile int unreceivedCount;

//Variable in die 2-Byte-lange erhaltene Daten gespeichert werden
volatile byte receivedData[2];

//Variable zum speichern ob ein Job verfügbar ist
volatile boolean jobAvailable;

//In diesem Array werden alle Daten abgelegt, welche gesendet werden sollen
volatile byte unsentData[2];

//Anzahl der Bytes die noch gesendet werden müssen
volatile int unsentCount;

//Variable zum speichern des letzten Zustands der LED
static boolean previousLedState = false;

//Variable, in die der vom Joystick empfangene Wert später gespeichert wird
int joystickValue;
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setup-methode
*/
void setup() 
{
  //Serial-Konsole starten mit Baud-Rate 115200
  Serial.begin(115200);

  //die beiden LED-Pins als OUTPUT definieren
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED_JOYSTICK, OUTPUT);

  //SPI Kommunikation initialisieren
  setupSPI();    
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setupSPI:
  SPI-Kommunikation initialisieren
*/
void setupSPI()
{
  //MISO-Anschluss als OUTPUT definieren (die Daten werden auf Master IN gesendet)
  pinMode(MISO, OUTPUT);

  //Da noch nichts empfangen wurde, gibt es auch keine Jobs zu erledigen
  jobAvailable = false;

  //SPI im Slave-Modus einschalten
  SPCR |= _BV(SPE); //WTF? 
  
  //Interrupts für die SPI-Kommunikation einschalten
  SPI.attachInterrupt();      
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  loop-Methode
*/
void loop() 
{
  //Überprüfen, ob es etwas zu tun gibt
  if(jobAvailable)
  {
    //Deaktiviere Interrupts, damit kein Interrupt-Event die Variable gleichzeitig überschreibt
    noInterrupts();

    //Da der Job nun erledigt wird, kann jobAvailable wieder auf FALSE gesetzt werden, da es nun nichts mehr zu erledigen gibt
    jobAvailable = false;

    //Interrupts wieder aktivieren
    interrupts();

    //Den Job ausführen
    doCommand();               
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  doCommand():
  Verarbeitet den vom Master empfangenen Befehl und führt je nach Befehl die enstsprechende Funktion durch
*/
void doCommand()
{
  //Überprüfen, welche Befehl empfangen wurde
  switch(received)
  {
    case LED_ON:    
      //Überprüfen ob der vorherige LED Zustand FALSE ist, die LED also ausgeschaltet ist
      if(!previousLedState)
      {
        //die LED am Slave einschalten
        digitalWrite(PIN_LED, HIGH);

        //den letzten Zustand auf den aktuellen Zustand (also eingeschaltet = TRUE) aktualisieren
        previousLedState = true;
      }
      break;
    case LED_OFF:
      //Überprüfen ob der vorherige LED Zustand TRUE ist, die LED also eingeschaltet ist
      if(previousLedState)
      {
        //die LED am Slave ausschalten
        digitalWrite(PIN_LED, LOW);

        //den letzten Zustand auf den aktuellen Zustand (also ausgeschaltet = FALSE) aktualisieren
        previousLedState = false;
      }
      break;
    case READ_SWITCH:
      //Wenn der Taster TRUE zurückgibt, sende den Befehl LED_ON an den Master, wenn der Taster FALSE  zurückgibt, sende den Befehl LED_OFF     
      writeSPIByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;
    case READ_LDR:
      //Den Wert des LDRs am Slave einlesen
      int ldrValue = analogRead(PIN_LDR);

      //Den Wert des LDRs an den Master senden
      writeSPIInt(ldrValue);
      break;
    case READ_JOYSTICK:
      joystickValue = receivedData[0] + recievedData[1] * 256; 
      //Der Wert vom Joystick liegt zwischen 0 und 1023. Die LED hat aber eine Helligkeit von 0 bis 255. Deswegen muss der Joystick-Wert auf den Wertebereich der LED gemappt werden
      int mappedValue = map(joystickValue, JOYSTICK_MIN, JOYSTICK_MAX, LED_MIN, LED_MAX);

      //den Helligkeitswert auf die LED hinausschreiben
      analogWrite(PIN_LED_JOYSTICK, mappedValue);  
      break;
    case NO_ACTION:
      break;
    default:
      //Der DEFAULT-Case tritt ein wenn ein nicht erwarteter/ nicht definierter Befehl vom Master geschickt wurde (z.B. 0xAB)
      Serial.print(F("[ SLAVE ] received undefined command: 0x"));
      Serial.println(received, HEX);      
  }
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeSPIByte():
  Ein Byte mittels SPI an den Master senden
*/
void writeSPIByte(byte value)
{
  //Deaktiviere Interrupts, damit kein Event die Variable gleichzeitig überschreibt
  noInterrupts();

  //Den zu sendenden Wert in das SPI Data Register (SPDR) schreiben. Der Wert wird bei der nächsten Übertragung an den Master gesendet
  SPDR = value;

  //Interrupts wieder aktivieren
  interrupts();
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeSPIInt():
  Einen Integer-Wert (2 Byte) an den Master schicken
*/
void writeSPIInt(int value)
{
  //Deaktiviere Interrupts, damit kein Event die Variable gleichzeitig überschreibt
  noInterrupts();

  //Zweites Byte des Integers (Least Significant Byte, LSB) fürs Senden vormerken
  unsentData[0] = value & 0xFF;

  //Die Zahl der noch zu sendenden Bytes auf 1 erhöhen, da das LSB Byte zum Senden vorgemerkt ist aber noch nicht gesendet wurde
  unsentCount = 1;

  //Erstes Byte des Integers (Most Significant Byte, MSB) in das SPI Data Register (SPDR) schreiben. Der WErt wird bei der nächsten Übertragung an den Master gesendet
  SPDR = (value / 256) & 0xFF;

  //Interrupts wieder aktivieren
  interrupts();  
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  ISR-Methode:
  ISR = Interrupt Service Routine
  Wird immer bei einem Interrupt ausgeführt (zumindest soweit ich weiß)
  SPI_STC_vect: Keine Ahnung wozu das gut ist, ist aber für uns nicht relevant
*/  
ISR(SPI_STC_vect)
{
  //Überprüfen, ob noch was empfangen werden muss
  if(unreceivedCount > 0)
  {
    //Die Daten vom SPI Data Register (SPDR) herausholen und in einem Array speichern
    receivedData[--unreceivedCount] = SPDR;

    //NO_ACTION in das SPI Data Register (SPDR) schreiben. Der Wert wird bei der nächsten Übertragung an den Master gesendet
    SPDR = NO_ACTION;

    //wenn alle Daten empfangen wurden, gibt es etwas zu tun => speicher dies in jobAvailable
    if(unreceivedCount == 0)
    {
      jobAvailable = true;
    }
  }
  else if(unsentCount > 0)  //Überprüfen, ob noch was gesendet werden muss
  {
    //noch ungesendete Daten in das SPI Data Register (SPDR) schreiben. Der Wert wird bei der nächsten Übertragung an den Master gesendet
    SPDR = unsentData[--unsentCount];
  }
  else  //Wenn keine Daten mehr empfangen werden müssen UND nichts mehr gesendet werden muss, kann ein neuer Befehl empfangen werden
  {
    //das empfangene Byte (den Befehl) aus dem SPI Data Regiser (SPDR) holen
    received = SPDR;

    //Überprüfen, ob der empfangene Befehl READ_JOYSTICK (ein Spezialfall) ist. 
    if(received == READ_JOYSTICK)
    {
      //Beim Befehl READ_JOYSTICK werden danach noch 2 weitere Bytes (der Integer-Wert des Joysticks) erwartet. Daher setzen wir die Anzahl der noch zu empfangenen Bytes auf 2
      unreceivedCount = 2;

      //Da noch zwei weitere Bytes empfangen werden müssen, gibt es für den Slave gerade nichts zu tun
      jobAvailable = false;

      //NO_ACTION in das SPI Data Register (SPDR) schreiben. Der Wert wird bei der nächsten Übertragung an den Master gesendet
      SPDR = NO_ACTION;      
    }
    else
    {
      //ist der Befehl kein Spezialfall, kann der Slave ihn entsprechend abarbeiten
      jobAvailable = true;
    }

    //NO_ACTION in das SPI Data Register (SPDR) schreiben. Der Wert wird bei der nächsten Übertragung an den Master gesendet
    SPDR = NO_ACTION; 
  }
}
