#include <Arduino.h>
#include <BMI160Gen.h>
#include <PID_v1.h>
#include <math.h>
#include <vector>
#include <xlog.h>
#include <chrono>

#ifdef TFT_DISPLAY
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(135, 240);
#endif // TFT_DISPLAY

#undef NDEBUG

#ifdef RP2040
#include "RP2040_ISR_Servo.h"
#define PIN_SERVO_XPOS 18
#define PIN_SERVO_XNEG 19
#define PIN_SERVO_YPOS 20
#define PIN_SERVO_YNEG 21
#define PIN_THROTTLE 22

#define SERVO_PIN_1       10
#define SERVO_PIN_2       11
#define SERVO_PIN_3       12
#define SERVO_PIN_4       13
#define SERVO_PIN_5       14
#define SERVO_PIN_6       15
#define NUM_SERVOS        6

typedef struct
{
  int     servoIndex;
  uint8_t servoPin;
} ISR_servo_t;

ISR_servo_t ISR_servo[NUM_SERVOS] =
{
  { -1, SERVO_PIN_1 }, { -1, SERVO_PIN_2 }, { -1, SERVO_PIN_3 }, { -1, SERVO_PIN_4 }, { -1, SERVO_PIN_5 }, { -1, SERVO_PIN_6 }
};
#endif // RP2040

#ifdef CONFIG_IDF_TARGET_ESP32
#include <ESP32Servo.h>
#define PIN_SERVO_XPOS 33
#define PIN_SERVO_XNEG 26
#define PIN_SERVO_YPOS 25
#define PIN_SERVO_YNEG 27
#define PIN_THROTTLE 32


#endif // CONFIG_IDF_TARGET_ESP32

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
#ifdef CONFIG_IDF_TARGET_ESP32 
public:
    IServo() { }
    IServo(Servo &servo) {
        _servo = servo;
    }
    IServo(int pin) {
        _servo = Servo();
        _servo.attach(pin, 500, 2500);
        _servo.setPeriodHertz(200);
    }
    void setAngle(int angle) {
        // 500ms = -90°
        // 1500ms = 0°
        // 2500ms = 90°
        // 1° = 11.11111ms

        int ms = 1500 + (angle * 11.11111); 
        _servo.writeMicroseconds(ms);
    }

private:
    Servo _servo;

#else
public:
    IServo(int pin) {

    }
    void setAngle(int angle) {

    }
    
#endif // CONFIG_IDF_TARGET_ESP32
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
    Wire.setClock(1000000);
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

    xlogi("Starting BMI160");

    xlogd("Initializing IMU");
    BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 0);
    xlogd("Getting device ID");
    uint8_t dev_id = BMI160.getDeviceID();

    xlogi("Device ID: " << dev_id);

    // Set the accelerometer range to 250 degrees/second
    xlogd("Setting accelerometer range ...");
    BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_1000);
    xlogd("Setting accelerometer range done.");
    BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
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

        // xlogi("BMI160", "Accel: %f, %f, %f", ax, ay, az);
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
            xlogi("Angle: " << angle_x << ", " << corr_x << ", dt: " << dt << "s");
            // print vector length
            xlogi("Vector length: " << len);
        }
    }
}
