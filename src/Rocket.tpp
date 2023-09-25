#include "Rocket.h"
#include <type_traits>
#include "IMU/BaseIMU.h"
#include "Regulator/RegulatorBase.h"
#include "Control/Control.h"
#include <iostream>


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::convert_acc_to_orientation() {
    // convert _acc_vals to roll, pitch
    double current_acceleration = _acc_vals.norm() - 1;  // g
    _throttle_corr = -current_acceleration;

    Eigen::Vector3d acc_norm = _acc_vals.normalized();

    _current_orientation[0] = asin(acc_norm[1]) * 180 / M_PI;
    _current_orientation[1] = -asin(acc_norm[0]) * 180 / M_PI;
    if (std::isnan(_current_orientation[0]) || std::isnan(_current_orientation[1])) {
        ESP_LOGE("Rocket", "Nan values in orientation");
        std::cout << "Acc vals: " << _acc_vals.transpose() << std::endl;
        std::cout << "Orientation: " << _current_orientation.transpose() << std::endl;
    }

    // true yaw can be only obtained from GPS/Magnetometer, approximate from gyro:
    _current_orientation[2] += _gyro_vals[2];
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::Rocket(rocket_param_t &params) {
    static_assert(std::is_base_of<BaseIMU, IMU_T>::value, "IMU_T must be derived from BaseIMU");
    static_assert(std::is_base_of<RegulatorBase, REGULATOR_T>::value, "REGULATOR_T must be derived from RegulatorBase");
    static_assert(std::is_base_of<Control, CONTROLLER_T>::value, "CONTROLLER_T must be derived from Control");

    _params = params;
    _imu = new IMU_T();
    _controller = new CONTROLLER_T();

    _servo_xpos = new IServo(params.servo_pin_xpos);
    _servo_ypos = new IServo(params.servo_pin_ypos);
    _servo_xneg = new IServo(params.servo_pin_xneg);
    _servo_yneg = new IServo(params.servo_pin_yneg);
    _throttle = new IServo(params.servo_pin_throttle);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::setup() {
    _imu.calibrate(100);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::setup_regulator(void* params) {
    ESP_LOGI("Rocket", "Setting up regulator");
    static_assert(std::is_base_of<RegulatorBase, REGULATOR_T>::value, "REGULATOR_T must be derived from RegulatorBase");

    _regulator = new REGULATOR_T(&_current_orientation, &_desired_orientation, &_corrections);
    _regulator->setup(&params, _params.angle_min, _params.angle_max);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::calculate_corrections() {
    ESP_LOGI("Rocket", "Calculating corrections");
    // calculate corrections

    _controller->update();
    _controller->get_desired_orientation(_desired_orientation);
    _controller->get_desired_throttle(_desired_throttle);

    _regulator->update();

    float roll = _corrections[0];
    float pitch = _corrections[1];
    float yaw = _corrections[2];

    _throttle_value = _desired_throttle;
    _xpos_value = roll + yaw;
    _ypos_value = pitch + yaw;
    _xneg_value = -roll + yaw;
    _yneg_value = -pitch + yaw;
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::steer() {
    ESP_LOGI("Rocket", "Steering");

    set_servo_xpos(_xpos_value);
    set_servo_ypos(_ypos_value);
    set_servo_xneg(_xneg_value);
    set_servo_yneg(_yneg_value);
    
    set_throttle();
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::set_throttle() {
    ESP_LOGI("Rocket", "Setting servo throttle");
    float percent = _throttle_value;
    if (percent > 100) {
        percent = 100;
    } else if (percent < 0) {
        percent = 0;
    }
    _throttle->set_percent(percent);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::update() {
    ESP_LOGI("Rocket", "Updating");
    // read raw gyro measurements from device
    _imu->get_rotations_dps(_gyro_vals);
    // read raw accelerometer measurements from device
    _imu->get_accelerations_g(_acc_vals);

    ESP_LOGD("Rocket", "Gyro: %f, %f, %f", _gyro_vals[0], _gyro_vals[1], _gyro_vals[2]);
    ESP_LOGD("Rocket", "Acc: %f, %f, %f", _acc_vals[0], _acc_vals[1], _acc_vals[2]);

    // convert _acc_vals to roll, pitch, yaw
    convert_acc_to_orientation();
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::set_servo_xpos(float angle) {
    ESP_LOGI("Rocket", "Setting servo xpos");
    if (angle > _params.angle_max) {
        angle = _params.angle_max;
    } else if (angle < _params.angle_min) {
        angle = _params.angle_min;
    }
    _servo_xpos->set_angle(angle);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::set_servo_ypos(float angle) {
    ESP_LOGI("Rocket", "Setting servo ypos");
    if (angle > _params.angle_max) {
        angle = _params.angle_max;
    } else if (angle < _params.angle_min) {
        angle = _params.angle_min;
    }
    _servo_ypos->set_angle(angle);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::set_servo_xneg(float angle) {
    ESP_LOGI("Rocket", "Setting servo xneg");
    if (angle > _params.angle_max) {
        angle = _params.angle_max;
    } else if (angle < _params.angle_min) {
        angle = _params.angle_min;
    }
    _servo_xneg->set_angle(angle);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::set_servo_yneg(float angle) {
    ESP_LOGI("Rocket", "Setting servo yneg");
    if (angle > _params.angle_max) {
        angle = _params.angle_max;
    } else if (angle < _params.angle_min) {
        angle = _params.angle_min;
    }
    _servo_yneg->set_angle(angle);
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::print_status() {
    ESP_LOGI("Rocket", "Printing status");
    std::cout << "Gyro: " << _gyro_vals.transpose() << std::endl;
    std::cout << "Acc: " << _acc_vals.transpose() << std::endl;
    std::cout << "Orientation: " << _current_orientation.transpose() << std::endl;
    std::cout << "Desired orientation: " << _desired_orientation.transpose() << std::endl;
    std::cout << "Corrections: " << _corrections.transpose() << std::endl;
    std::cout << "Throttle: " << _throttle_value << std::endl;
    std::cout << "Xpos: " << _xpos_value << std::endl;
    std::cout << "Ypos: " << _ypos_value << std::endl;
    std::cout << "Xneg: " << _xneg_value << std::endl;
    std::cout << "Yneg: " << _yneg_value << std::endl;
    std::cout << std::endl;
}


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
void Rocket<IMU_T, REGULATOR_T, CONTROLLER_T>::pre_flight_check() {
    ESP_LOGI("Rocket", "Pre-flight check");
    set_servo_xneg(-45);
    set_servo_xpos(-45);
    set_servo_yneg(-45);
    set_servo_ypos(-45);
    delay(500);
    set_servo_xneg(45);
    set_servo_xpos(45);
    set_servo_yneg(45);
    set_servo_ypos(45);
    delay(500);
    set_servo_xneg(0);
    set_servo_xpos(0);
    set_servo_yneg(0);
    set_servo_ypos(0);
    ESP_LOGI("Rocket", "Pre-flight check done");
}
