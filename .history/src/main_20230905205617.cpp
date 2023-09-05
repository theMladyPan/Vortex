#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <BMI160Gen.h>

TFT_eSPI tft = TFT_eSPI(135, 240);

#define PIN_MISO  27  // GPIO27
#define PIN_MOSI  26  // GPIO26
#define PIN_SCK   25  // GIPO25
#define PIN_CS    33  // GPIO33

void setup() {
  Serial.begin(1500000);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  SPI.setFrequency(1000000);
  BMI160.begin(BMI160GenClass::SPI_MODE, 33);

  // TFT setup
  tft.init();
  tft.setTextFont(1);
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Hello", 0, 50, 4);
  delay(1000);  
}

void loop() {
  // put your main code here, to run repeatedly:
}