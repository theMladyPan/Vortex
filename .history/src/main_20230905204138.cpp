#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <BMI160Gen.h>

TFT_eSPI tft = TFT_eSPI(135, 240);

void setup() {
  Serial.begin(1500000);

  BMI160.begin(BMI160GenClass::SPI_MODE, 10);

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