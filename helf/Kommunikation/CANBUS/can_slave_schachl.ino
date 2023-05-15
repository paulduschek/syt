/* Musterlösung für CANBUS-Kommunikation - SLAVE
  Author: Niklas Schachl
  Date: 07.01.2023
*/
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

//Variable in die 1-Byte-lange erhaltene Daten gespeichert werden
volatile byte received;

//den CANBUS initialisieren
MCP2515 mcp2515(10);

//einen CANBUS-Frame anlegen
struct can_frame canMsg;

//Variable zum zwischenspeichern des letzten Zustands der LED am Slave
static boolean previousLedState = false;
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setup-methode
*/
void setup() 
{
  //Serial-Konsole starten mit Baud Rate 115200 
  Serial.begin(115200);

  //die beiden LED-Pins als OUTPUT definieren
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED_JOYSTICK, OUTPUT);

  //CANBUS-Kommunikation initialisieren
  setupCAN();
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  setupCAN:
  CANBUS-Kommunikation initialisieren
*/
void setupCAN()
{
  //SPI-Kommunikation (zwischen CANBUS und der CPU) initialisieren
  SPI.begin();

  //CANBUS zurücksetzen
  mcp2515.reset();

  //Bitrate auf 125kbps setzen
  mcp2515.setBitrate(CAN_125KBPS);

  //CANBUS Modus einstellen (?)
  mcp2515.setNormalMode();
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  loop-Methode
*/
void loop() 
{
  /*
    mcp2515.readMessage(&canMsg): Empfange Daten vom Master und speichere diese im Paket canMsg
    MCP2515::ERROR_OK: ist die Übertragung korrekt abgelaufen, bekommen wir den Statuscode ERROR_OK
    if(...): Überpüfen, ob der Statuscode der Übertragung ERROR_OK ist, also alles korrekt abgelaufen ist
  */
  if(mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {
    //Den Wert an der ersten Datenstelle (.data[0]) des empfangenen Pakets (canMsg) in der Variable received speichern
    received = canMsg.data[0];

    //Den Befehl verarbeiten
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
  //Überprüfen, welcher Befehl empfangen wurde
  switch(received)
  {
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
      //Wenn der Taster TRUE zurückgibt, sende den Befehl LED_ON an den Master, wenn der Taster FALSE zurückgibt, sende den Befehl LED_OFF
      writeCANByte(digitalRead(PIN_SWITCH) ? LED_ON : LED_OFF);
      break;
    case READ_LDR:
      //den Wert des LDRs am Slave einlesen
      int ldrValue = analogRead(PIN_LDR);

      //den Wert des LDRs an den Master senden
      writeCANInt(ldrValue);
      break;
    case READ_JOYSTICK:
      //Integer-Wert aus den Werten der zweiten und dritten Datenstelle (.data[1] und .data[2]) des empfangenen Pakets(canMsg) zusammenbauen
      int joystickValue = canMsg.data[1] * 256 + canMsg.data[2];

      //Der Wert vom Joystick liegt zwischen 0 und 1023. Die LED hat aber eine Helligkeit von 0 bis 255. Deswegen muss der Joystick-Wert auf den Wertebereich der LED gemappt werden
      int mappedValue = map(joystickValue, 0, 1023, 0, 255);

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
  writeCANByte():
  Ein Byte mittels CANBUS an den Master senden
*/
void writeCANByte(byte value)
{
  //Dem CANBUS-Paket eine ID geben
  canMsg.can_id = 0x06F;

  //Die Länge des Pakets festlegen. Da wir nur 1 Byte übertragen wird die Länge auf 1 festgelegt
  canMsg.can_dlc = 1;

  //Den zu übertragenden Wert an die erste Datenstelle (.data[0]) des zu übertragenden Pakets (canMsg) schreiben
  canMsg.data[0] = value;

  //Das Paket (canMsg) an den Master senden
  mcp2515.sendMessage(&canMsg);
}
// --------------------------------------------------------------------------------------------------------------------------------

/*
  writeCANInt():
  Einen Integer-Wert (2 Byte) an den Master schicken
*/
void writeCANInt(int value)
{
  //Dem CANBUS-Paket eine ID geben
  canMsg.can_id = 0x06F;

  //Die Länge des Pakets festlegen. Da ein Integer 2 Byte lang ist, muss die Länge hier auf 2 eingestellt werden
  canMsg.can_dlc = 2;

  //Erstes Byte des Integers (Most Significant Byte, MSB) an die erste Datenstelle (.data[0]) des zu übertragenden Pakets (canMsg) schreiben
  canMsg.data[0] = (value / 256) & 0xFF;

  //Zweites Byte des Integers (Least Significant Byte, LSB) an die zweite Datenstelle (.data[1]) des zu übertragenden Pakets (canMsg) schreiben
  canMsg.data[1] = value & 0xFF;

  //Das Paket (canMsg) an den Master senden
  mcp2515.sendMessage(&canMsg); 
}
// --------------------------------------------------------------------------------------------------------------------------------