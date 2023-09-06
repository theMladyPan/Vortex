#include <Arduino.h>
#include <TFT_eSPI.h>
#include <BMI160Gen.h>
#include <ESP32Servo.h>

Servo sx_pos;
Servo sy_pos;
Servo sx_neg;
Servo sy_neg;

TFT_eSPI tft = TFT_eSPI(135, 240);


float convertRawGyro(int gRaw) {
    float g = (gRaw * 1000.0) / 32768.0;

    return g;
}

float convertRawAccel(int aRaw) {
    float a = (aRaw * 2.0) / 32768.0;

    return a;
}

void setup() {
    Serial.begin(1000000);
    Wire.begin(21, 22, 1000000);
    tft.init();
    tft.setTextFont(1);
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Hello", 0, 50, 4);

	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
    sx_pos.setPeriodHertz(200);
    sx_pos.attach(25, 500, 2500);
    sy_pos.setPeriodHertz(200);
    sy_pos.attach(26, 500, 2500);
    sx_neg.setPeriodHertz(200);
    sx_neg.attach(27, 500, 2500);
    sy_neg.setPeriodHertz(200);
    sy_neg.attach(33, 500, 2500);

    while(!Serial) { 
        delay(1);
    }
    ESP_LOGI("BMI160", "Starting BMI160");

    ESP_LOGD("BMI160", "Initializing IMU");
    BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 0);
    ESP_LOGD("BMI160", "Getting device ID");
    uint8_t dev_id = BMI160.getDeviceID();

    ESP_LOGI("BMI160", "Device ID: %d", dev_id);

    // Set the accelerometer range to 250 degrees/second
    ESP_LOGD("BMI160", "Setting accelerometer range ...");
    BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_1000);
    ESP_LOGD("BMI160", "Setting accelerometer range done.");
    BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
    
}

void loop() {
    int gxRaw, gyRaw, gzRaw;         // raw gyro values
    int axRaw, ayRaw, azRaw;         // raw accelerometer values
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;

    while(1) {
        int64_t time_start = esp_timer_get_time();
        // read raw gyro measurements from device
        BMI160.readGyro(gxRaw, gyRaw, gzRaw);
        BMI160.readAccelerometer(axRaw, ayRaw, azRaw);

        // convert the raw accelerometer data to degrees/second
        ax = convertRawAccel(axRaw) * 0.01 + ax * 0.99;
        ay = convertRawAccel(ayRaw) * 0.01 + ay * 0.99;
        az = convertRawAccel(azRaw) * 0.01 + az * 0.99;

        // convert the raw gyro data to degrees/second
        gx = convertRawGyro(gxRaw) * 0.01 + gx * 0.99;
        gy = convertRawGyro(gyRaw) * 0.01 + gy * 0.99;
        gz = convertRawGyro(gzRaw) * 0.01 + gz * 0.99;
        int64_t dt = esp_timer_get_time() - time_start;
        sx_pos.writeMicroseconds(1500 + gz * 100 + ax * 1000);

        // display tab-separated gyro x/y/z values
        Serial.printf("\rax: %.2f, ay: %.2f, az: %.2f, gx: %d°, \tgy: %d°, \tgz: %d°, \tdt: %lldus", ax, ay, az, (int)gx, (int)gy, (int)gz, dt);
        //delay(10);
    }
}
