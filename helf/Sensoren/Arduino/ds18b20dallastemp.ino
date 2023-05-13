// OneWire and DallasTemperature Library required
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

void setup()
{
  // Start serial communication for debugging purposes
  Serial.begin(115200);
  
  // Start up the library
  sensors.begin();
}

void loop()
{
  // Call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  sensors.requestTemperatures();
  
  // Get the temperature in Celsius for device index 0
  float tempC = sensors.getTempCByIndex(0);

  // Print the temperature to the serial monitor
  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println(" °C");

  // Wait for a second before taking another reading
  delay(1000);
}
