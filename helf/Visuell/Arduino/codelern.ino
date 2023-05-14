/**
	@author Paul Duschek
	@version 1, created 23.3.2022
*/
// Includes:
#include <NewPing.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "SD.h"
#include "SPI.h"
#include <Wire.h>
#include <DS3231.h>
#include <DHT.h>
#include <DHT_U.h>


// PIN der SD Karte
const int CHIP_SLCT = 10;

// alle 10 geschriebenen Datensätze wird geflusht
int flushIntervall = 10;
int writeTotal = 0;
int count = 0;

long INTERVALL = 10000;

// Dateiname
String FILENAME = "data.csv";

// Variable, welcher später angehängt wird, was in die CSV-Datei gehört
String data;

// Datei für Sensoren
File sensorData;
// Echtzeituhr
DS3231 clock;
RTCDateTime dt;

// Variablen für die zweite Seite mit Taster
const int BUTTON_PIN 2;
// bestimmt später, auf welcher Seite man sich befindet
int hits = 0;
// für die Stellungen des Tasters zuständig
int switchState = 0;
int prevSwitchState = 0;

// PINs der Sensoren definieren
#define LDR A2
#define NTC A1
#define DHTPIN A0 

// DHT11 definieren
#define DHTTYPE DHT11 
DHT_Unified dht(DHTPIN, DHTTYPE);

// Spalten des LCD Screens
#define COLS 20;

// DS18B20
// Pin definieren
#define ONEWIRE_PIN 8

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);

// HC-SR04
// Trigger und Echo PIN definieren (beide dasselbe in unserem Fall)
#define TRIG_PIN 7
#define ECHO_PIN 7
// maximale messbare Distanz
#define MAX_DIST 999

// Definition
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

// LCD Definition
// LCD PIN definieren
#define BACKLIGHT_PIN 3
// I2C Adresse wird zugewiesen
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, BACKLIGHT_PIN, POSITIVE);  

// setup wird bei Start des Programmes 1x ausgeführt
void setup() {
	// LCD initalisieren (20 Spalten, 4 Reihen)
	lcd.begin(20, 4);
	
	// setzt den LCD Cursor auf 0, 0
	lcd.home();
	
	// setCursor(Spalte, Reihe) Werte beginnen bei 0
	lcd.setCursor(0, 1);
	
	Serial.begin(115200);
	// ---------------------------
	// DHT11 wird gestartet
	dht.begin();

	// Sensor wird angelegt
	sensor_t sensor;
	// & holt sich die Adresse der Variable
	dht.temperature().getSensor(&sensor);
	
	// DS18B20 wird gestartet (DallasTemperature)
	sensors.begin();
	
	// --------------------------
	// Serialisierung 
	Serial.print(F("SD-Karte wird initialisiert\n"));
	// SD-Karte wird auf OUTPUT Mode gestellt
	pinMode(CHIP_SLCT, OUTPUT);
	// prüfen, ob Karte verfügbar
	if (!SD.begin(CHIP_SLCT)) {
     Serial.println(F("\nLesen der Karte fehlgeschlagen\n"));
     return;
   }
   
   Serial.println(F("Karte erfolgerich eingelesen"));
   // Datei auf der SD-Karte wird geöffnet, hineingeschrieben
   sensorData = SD.open(FILENAME, FILE_WRITE);
   // Datei erfolgreicht geöffnet
   if(sensorData) {
		sensorData.close();
   }
   // Datei öffnen nicht erfolgreich
   else {
		Serial.println(FILENAME + "konnte nicht geöffnet werden");
		Serial.println(F("Initalisiere DS3231"));
   }
}

// loop wird immer wieder ausgeführt
void loop() {
	// Taster für das Umschalten auf die zweite Seite wird eingelesen
	switchState = digitalRead(BUTTON_PIN);
	
	// es wird überprüft, ob die derzeitige Stellung des Tasters der lezten registrierten entspricht
	// ist dies nicht der Fall, bekommt sie einen neuen Wert und die hits steigen um 1
	if (switchState != prevSwitchState) {
		if (switchState == HIGH) {
			hits = hits + 1;
			delay(10);
		}
	}
	
	// -----------------------------
	// Definitionen und Einlesen aller Werte
	// Ausgaben erfolgen im if später
	// LDR und NTC werden eingelesen
	int ldrValue = analogRead(LDR);
	int ntcValue = analogRead(NTC);
	
	// ------------------------------------
	// Ausgabe am LCD von den LDR Werten
	// Array, in dem die Werte des LDRs gespeichert werden
	char ldrRead[COLS+1];
	
	// durch sprintf werden die Werte im Array ldrRead gespeichert, ldrValue ist ganzzahlig 4 Stellen 
	sprintf(ldrRead, "%04d", ldrValue);
	
	// -----------------------------------
	// Ausgabe am LCD von den NTC Werten
	// Array, in dem die Werte des NTCs gespeichert werden
	char ntcRead[COLS+1];
	
	// durch sprintf werden die Werte im Array ntcRead gespeichert, ntcValue ist ganzzahlig 4 Stellen 
	sprintf(ntcRead, "%04d", ntcValue);
	
	// ------------------------------------
	// Sensor für DHT11 wird definiert
	sensors_event_t event;
	// Event für den DHT11 wird eingerichtet, auf Adresse der Variable event gezeigt
	dht.temperature().getEvent()(&event);
	// Variable für die CSV-Ausgabe der Temperatur
	int writeTempDHT11 = event.temperature;
	// nun wird der Humidity Wert des DHT 11 eingelesen
	dht.humidity().getEvent(&event);
	// Variable für die CSV-Ausgabe der Humidity	
	int writeHumidity = event.relative_humidity;
	
	// -----------------------------------
	// Zeit und Datum 
	// akutelle Uhrzeit, akutelles Datum werden eingeholt
	dt = clock.getDateTime();
	
	// Arrays für die Ausgabe in der CSV-Datei
	char dateRead[COLS+1];
	char timeRead[COLS+1];
    
	// Speichern der Strings im jeweiligen Array
	// Ausgabe von Datum und Uhrzeit am LCD Screen
	sprintf(dateRead, "%04d-%02d-%02d", dt.year, dt.month, dt.day);
	sprintf(timeRead, "%02d:%02d:%02d", dt.hour, dt.minute, dt.second);
	
	// ----------------------------------
	// HC-SR04 Entfernung
	// Wert vom Entfernungsmesser wird eingelesen
	int distanceCM = sonar.ping_cm();
    
	// Array für die Ausgabe in der CSV-Datei
	char writehcsr04[COLS+1];
	// Speichern der Ausgabe im Array, 3 Stellen, ganzzahlig
	sprintf(writehcsr04, "%03d", distanceCM);
	
	// -----------------------------------
	// DS18B20
	//char ds18b20[COLS+1];
	// Temperatur vom DS18B20 wird eingelesen
	sensors.requestTemperatures();
	float tempOut = sensors.getTempCByIndex(0);
    
	//sprintf(ds18b20, "%02d", tempOut);
	// -----------------------------------
		
	// hits == 1 ruft Seite 1 auf
	if(hits == 1) {
		// alte Anzeige wird gelöscht
		lcd.clear();
		
		// -----------------------------------
		// LDR am LCD ausgeben
		lcd.setCursor(0,2);
		lcd.print(F("LDR: "));
		lcd.setCursor(0,3);
		lcd.print(ldrRead);

		// -----------------------------------
		// NTC am LCD ausgeben
		lcd.setCursor(6,2);
		lcd.print(F("NTC: "));
		lcd.setCursor(6,3);
		lcd.print(ntcRead);
	
		// -----------------------------------
		// DHT11 Ausgabe LCD Screen
		// überprüft ob Temperatur eine Nummer ist, wenn nicht -> Fehler
		if (isnan(event.temperature)){
			Serial.println(F("Fehler beim Ermitteln der Temperatur"));
		}
		// ist Temperatur eine Nummer -> Ausgabe am LCD Screen
		else {
			lcd.setCursor(12, 1);
			lcd.print(F("Tmp:"));
			lcd.setCursor(16, 1);
			lcd.print(event.temperature);
		}
		// auch hier wird auf eine Nummer überprüft
		if (isnan(event.relative_humidity)){
			Serial.println(F("Fehler beim Ermitteln der Menschlichkeit"));
		}
		// ist die Überprüfung erfolgreich -> Humidity am LCD Screen ausgegeben
		else {
			lcd.setCursor(12, 2);
			lcd.print(F("Hum:"));
			lcd.setCursor(16, 2);
			lcd.print(event.relative_humidity);
		}
	}
	// hits == 2 ruft Seite 2 auf
	else if(hits == 2) {
		// alte Anzeige wird gelöscht
		lcd.clear();
		// ----------------------------------
		// Zeit, Datum Ausgabe
		lcd.setCursor(0, 0);
		lcd.print(dateRead);
		lcd.setCursor(0, 1);
		lcd.print(timeRead);
		
		// ----------------------------------
		// HC-SR04 Ausgabe am LCD Screen
		lcd.setCursor(0,2);
		lcd.print(F("Dist:"));
		lcd.setCursor(0,3);
		lcd.print(distanceCM);
		
		// ---------------------------------
		// Ausgabe am LCD Screen
		lcd.setCursor(7,2);
		lcd.print(F("Temp2:"));
		lcd.setCursor(7,3);
		lcd.print(tempOut);
	}
	// ist hits weder 1 noch 2 wird es auf 0 gesetzt, der Taster ist sozusagen neutral
	else{
		Serial.println(F("Counter reset"));
		hits = 0;
	}
	
	// alle Arrays in einen String speichern, durch ; trennen
	data = (String)dateRead + ";" + (String)timeRead + ";" + (String)ldrRead + ";" + (String)ntcRead + ";" + 
	(String)writeTempDHT11 + ";" + (String)writeHumidity + ";" + (String)writehcsr04;
	
	// Daten aus dem String data werden in die CSV-Datei gespeichert
	saveData();
	// alle 20 Datensätze werden die Daten im Serial Monitor ausgegeben
	if(writeTotal % 20 == 0) {
		readData();
	}
}

// durch diese Methode werden die gewünschten Daten auf der SD-Karte gespeichert
void saveData() {
	// es wird überprüft, ob eine Datei mit dem gewünschten Namen existiert
	if (SD.exists(FILENAME)) { 
      // die gewünschte Datei wird geöffnet
      sensorData = SD.open(FILENAME, FILE_WRITE);
	  // es wird geprüft, ob das öffnen der Datei erfolgreich war
      if (sensorData) {
		// der String, welche alle Arrays mit den Werte beeinhaltet, wird auf die CSV-Datei geschrieben
        sensorData.println(data);
		// count, writeTotal wird erhöht, wenn das Schreiben erfolgreich war
        count++;
        writeTotal++;
        
		// es wird überprüft, ob schon 10 Datensätze geschrieben wurden
		// wenn ja wird geflusht, der count auf 0 gesetzt, wenn nein läuft das Programm wieder von vorne los
        if (count >= flushIntervall) {
          Serial.println(F("flushing..."));
          sensorData.flush();
          Serial.println(F("flushing erfolgreich"));
          count = 0;
        }
		// die geöffnete Datei wird geschlossen
        sensorData.close(); 
      }
    }
    else {
		Serial.println(F("Es gab einen Fehler beim Öffnen der Datei!"));
	}
}

// durch diese Methode werden die Datensätze auf der CSV-Datei ausgelesen
void readData() {
    // die Datei wird geöffnet, dieses Mal nur zum Lesen 
    sensorData = SD.open(FILENAME);
	// es wird überprüft, ob die Datei erfolgreich geöffnet werden konnte
    if (sensorData) {
		// der Name der Datei wird ausgegeben
		Serial.println("\n" + FILENAME + ":");
		
		// solange es in der Datei nicht gelesene Datensätze gibt, sollen diese im Serial Monitor ausgegeben werden
		while (sensorData.available()) {
			Serial.write(sensorData.read());
			Serial.println(F("\nLesen abgeschlossen\n"));
		}
		// die Datei wird wieder geschlossen
		sensorData.close();
    }
    else {
		Serial.println(F("Es gab einen Fehler beim Öffnen der Datei!"));
	}
}