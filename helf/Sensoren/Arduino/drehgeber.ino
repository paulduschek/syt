#include <Encoder.h>

// Pins für den Encoder
#define CLK 2
#define DT 3

// Erzeuge ein Encoder-Objekt
Encoder myEncoder(CLK, DT);

// Vorherige Position des Encoders
long previousPosition = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Lese die aktuelle Position des Encoders aus
  long currentPosition = myEncoder.read();

  // Überprüfe, ob sich die Position geändert hat
  if (currentPosition != previousPosition) {
    // Zeige die aktuelle Position im Seriellen Monitor an
    Serial.print("Encoder Position: ");
    Serial.println(currentPosition);
    
    // Aktualisiere die vorherige Position
    previousPosition = currentPosition;
  }
}