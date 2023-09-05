#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <BMI160Gen.h>

TFT_eSPI tft = TFT_eSPI(135, 240);

#define PIN_MISO  27  // GPIO27
#define PIN_MOSI  26  // GPIO26
#define PIN_SCK   25  // GIPO25
#define PIN_CS    33  // GPIO33


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

void setup() {
  Serial.begin(1500000);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  SPI.setFrequency(1000000);
  BMI160.begin(BMI160GenClass::SPI_MODE, 33);

  uint8_t dev_id = BMI160.getDeviceID();

  ESP_LOGI("BMI160", "Device ID: %d", dev_id);

   // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(250);

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