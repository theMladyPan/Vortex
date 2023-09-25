#include <Arduino.h>
#include "esp_log.h"
#include "Rocket.h"
#include "IMU/IMU.h"
#include "Regulator/PIDRegulator.h"
#include "Control/Remote.h"
#include <chrono>

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

#ifdef MIN_PULSE_WIDTH
#undef MIN_PULSE_WIDTH
#define MIN_PULSE_WIDTH 540
#endif // MIN_PULSE_WIDTH
#ifdef MAX_PULSE_WIDTH
#undef MAX_PULSE_WIDTH
#define MAX_PULSE_WIDTH 2400
#endif // MAX_PULSE_WIDTH

#ifndef NDEBUG
#define LOOP_FREQ_HZ 2.0
#else
#define LOOP_FREQ_HZ 200.0
#endif // NDEBUG
#define LOOP_PERIOD (1 / LOOP_FREQ_HZ)

#define SERVO_ANGLE_MIN    -45
#define SERVO_ANGLE_MAX     45

#ifdef I2C_SDA
#undef I2C_SDA
#define I2C_SDA 19
#endif // I2C_SDA


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
}

void loop() {
    pid_params_t pid_params = {
        .kp = 3,
        .ki = 3,
        .kd = 0,
        .sampling_period = LOOP_PERIOD  // seconds
    };

    // Setup the aircraft
    rocket_param_t rocket_params = {
        .loop_period = LOOP_PERIOD,  // seconds
        .angle_min = SERVO_ANGLE_MIN,
        .angle_max = SERVO_ANGLE_MAX,
        .max_thrust = 7.0,  // Newtons
        .mass = 1.0,  // kg
        .servo_pin_xpos = PIN_SERVO_XPOS,
        .servo_pin_ypos = PIN_SERVO_YPOS,
        .servo_pin_xneg = PIN_SERVO_XNEG,
        .servo_pin_yneg = PIN_SERVO_YNEG,
        .servo_pin_throttle = PIN_THROTTLE
    };
    

    ESP_LOGI("main", "Creating aircraft");
    Rocket<IMU, PIDRegulator, Remote> rocket(rocket_params);

    rocket.setup_regulator(&pid_params);

    ESP_LOGI("main", "Setting up rocket");
        
    #ifdef NDEBUG
    ESP_LOGW("main", "Pre-flight initiating pre-flight check");
    rocket.pre_flight_check();  // Turn on after testing
    #endif // NDEBUG


    uint64_t loopn = 0;
    ESP_LOGI("main", "Entering main loop");
    while(1) {
        auto start = std::chrono::high_resolution_clock::now();

        rocket.update();
        rocket.calculate_corrections();
        rocket.steer();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        int dt = duration_us.count();

        if(loopn++ % 100 == 0) {
            rocket.print_status();
            std::cout << "Loop duration: " << dt << " us" << std::endl << std::endl;
        }
            
        if (dt < LOOP_PERIOD * 1e6) {
            delayMicroseconds(LOOP_PERIOD * 1e6 - dt);
        }
    }
}
