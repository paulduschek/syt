#include <LiquidCrystal_I2C.h>

#define POTI_PIN A0

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

void setup() {
  pinMode(POTI_PIN, INPUT);
  // set cols and rows of the lcd in use (in this case 20 columsn, 4 rows)
  lcd.begin(20,4);
  // set the cursor of the lcd to 0,0
  lcd.home();
  // clear the lcd screen
  lcd.clear();
  Serial.begin(115200);
}

void loop() {
  int poti = analogRead(POTI_PIN);

  // set the cursor to a specific position (cols, rows)
  // numbers beginning with 0, so if there are 4 rows -> 0,1,2,3
  lcd.setCursor(6, 3);
  // print something to the lcd display on the cursor pos currently set
  lcd.print(poti);
}