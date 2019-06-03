/***
 * Code to debug the different components of the ejection circuit board.
 */

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

// Pin definitions
#define main1 4
#define main2 5
#define drogue1 2
#define drogue2 3
#define buzzer 6
#define led 13
float freq = 375;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(main1, OUTPUT);
  pinMode(main2, OUTPUT);
  pinMode(drogue1, OUTPUT);
  pinMode(drogue2, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(main1, HIGH);
  digitalWrite(main2, HIGH);
  digitalWrite(drogue1, HIGH);
  digitalWrite(drogue2, HIGH);
  digitalWrite(buzzer, LOW);
  digitalWrite(led, HIGH);

  // buzzer test
  for (int i=0; i<3; i++) {
    tone(buzzer, freq);
    delay(100);
    noTone(buzzer);
    delay(100);
  }

}
