#include <LiquidCrystal_I2C.h>

#define POTI_PIN A0

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

void setup() {
  pinMode(POTI_PIN, INPUT);
  lcd.begin(20,4);
  lcd.home();
  lcd.clear();
  Serial.begin(115200);
}

void loop() {
  int poti = analogRead(POTI_PIN);

  lcd.setCursor(6, 3);
  lcd.print(poti);
}