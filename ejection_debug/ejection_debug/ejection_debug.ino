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
float freq = 3750;

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
  digitalWrite(main2, LOW);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(led, HIGH);

}
