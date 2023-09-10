#include <Arduino.h>
#include <BMI160Gen.h>
#include <PID_v1.h>
#include <math.h>
#include <vector>
// #include <xlog.h>
#include <chrono>
#include "ESP32_ISR_Servo.h"
#include "esp_log.h"

#ifdef TFT_DISPLAY
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(135, 240);
#endif // TFT_DISPLAY

#undef NDEBUG

#define PIN_SERVO_XPOS 33
#define PIN_SERVO_XNEG 26
#define PIN_SERVO_YPOS 25
#define PIN_SERVO_YNEG 27
#define PIN_THROTTLE 32

#define MIN_MICROS 500
#define MAX_MICROS 2500

double Kp=1, Ki=10, Kd=0;

//Define Variables we'll be connecting to
double angle_x_set, angle_x, corr_x;
PID pid_sx_pos(&angle_x, &corr_x, &angle_x_set, Kp, Ki, Kd, DIRECT);

float convertRawGyro(int gRaw) {
    float g = (gRaw * 1000.0) / 32768.0;
    return g;
}

float convertRawAccel(int aRaw) {
    float a = (aRaw * 2.0) / 32768.0;
    return a;
}

class IServo {
public:
    IServo() { }
    IServo(uint8_t pin) {

	    _servoIndex = ESP32_ISR_Servos.setupServo(pin, MIN_MICROS, MAX_MICROS);
        
    }
    void setAngle(int angle) {
        ESP32_ISR_Servos.setPosition(_servoIndex, angle + 90);
    }

private:
    int8_t _servoIndex;
};

void axesToVector(std::array<float, 3> &vec, float ax, float ay, float az) {
    vec[0] = ax;
    vec[1] = ay;
    vec[2] = az;
}

float vectorLength(std::array<float, 3> &vec) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

void setup() {
    Serial.begin(1000000);
    Wire.begin(I2C_SDA, I2C_SCL, 1000000); // join i2c bus (address optional for master
    #ifdef TFT_DISPLAY
    tft.init();
    tft.setTextFont(1);
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Hello", 0, 50, 4);
    #endif // TFT_DISPLAY

	// ESP32PWM::allocateTimer(0); 

    //turn the PID on
    pid_sx_pos.SetMode(AUTOMATIC);
    pid_sx_pos.SetSampleTime(1);
    pid_sx_pos.SetOutputLimits(-90, 90);

    ESP_LOGI("main", "Starting BMI160");

    ESP_LOGD("main", "Initializing IMU");
    BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 0);
    ESP_LOGD("main", "Getting device ID");
    uint8_t dev_id = BMI160.getDeviceID();

    ESP_LOGI("main", "Device ID: %d", dev_id);

    // Set the accelerometer range to 250 degrees/second
    ESP_LOGD("main", "Setting accelerometer range ...");
    BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_1000);
    ESP_LOGD("main", "Setting accelerometer range done.");
    BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);


	ESP32_ISR_Servos.useTimer(0);
}

void loop() {
    int gxRaw, gyRaw, gzRaw;         // raw gyro values
    int axRaw, ayRaw, azRaw;         // raw accelerometer values
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;
    std::array<float, 3> vecAcc;

    uint64_t iter = 0;

    IServo servoXPos(PIN_SERVO_XPOS);
    IServo servoYPos(PIN_SERVO_YPOS);
    IServo servoXNeg(PIN_SERVO_XNEG);
    IServo servoYNeg(PIN_SERVO_YNEG);
    IServo throttle(PIN_THROTTLE);

    while(1) {
        auto start = std::chrono::high_resolution_clock::now();
        // read raw gyro measurements from device
        BMI160.readGyro(gxRaw, gyRaw, gzRaw);
        BMI160.readAccelerometer(axRaw, ayRaw, azRaw);

        // convert the raw accelerometer data to degrees/second
        ax = convertRawAccel(axRaw);
        ay = convertRawAccel(ayRaw);
        az = convertRawAccel(azRaw);

        axesToVector(vecAcc, ax, ay, az);
        float len = vectorLength(vecAcc);
        // convert to unit vector
        for (auto &v : vecAcc) {
            v /= len;
        }
        len = vectorLength(vecAcc);

        // ESP_LOGI("main", "BMI160", "Accel: %f, %f, %f", ax, ay, az);
        // convert the raw gyro data to degrees/second
        gx = convertRawGyro(gxRaw);
        gy = convertRawGyro(gyRaw);
        gz = convertRawGyro(gzRaw);
        angle_x_set = 0;
        if(ay >= 1.0) ay = 1.0;
        if(ay <= -1.0) ay = -1.0;
        float asiny = asin(ay);
        angle_x = asin(ay) * -180 / M_PI;
        pid_sx_pos.Compute();

        float rotationX = gx * 0.001;  // 1000˚/s -> 1˚/ms

        servoYPos.setAngle(-angle_x + rotationX*50);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        int dt = duration.count();
        if (dt < 1000) {
            delayMicroseconds(1000 - dt);
        }
        if (iter++ % 100 == 0) {
            // ESP_LOGI("main", "Angle: " << angle_x << ", " << corr_x << ", dt: " << dt << "s");
            ESP_LOGI("main", "Angle: %f, %f, dt: %d", angle_x, corr_x, dt);
            // print vector length
            ESP_LOGI("main", "Vector length: %f", len);
        }
    }
}
