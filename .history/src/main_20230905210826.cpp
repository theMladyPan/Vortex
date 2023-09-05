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
  Serial.begin(115200);
  delay(100);
  ESP_LOGI("BMI160", "Starting BMI160");
  // SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  // SPI.setFrequency(1000000);

  // TFT setup
  /*
  tft.init();
  tft.setTextFont(1);
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Hello", 0, 50, 4);
  delay(1000);  
  */

  BMI160.begin(BMI160GenClass::SPI_MODE, 33);
  uint8_t dev_id = BMI160.getDeviceID();

  ESP_LOGI("BMI160", "Device ID: %d", dev_id);

   // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(250);
}

void loop() {
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  float gx, gy, gz;

  // read raw gyro measurements from device
  BMI160.readGyro(gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  // display tab-separated gyro x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();

  delay(500);
}
