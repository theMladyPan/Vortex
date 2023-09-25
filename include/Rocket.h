#pragma once

#include <ArduinoEigenDense.h>
#include "IServo.h"
#include "IMU/BaseIMU.h"
#include "Control/Control.h"
#include "Regulator/RegulatorBase.h"

typedef struct
{    
    double loop_period;
    float angle_min;
    float angle_max;

    float max_thrust;  // Newtons
    float mass;  // kg

    uint servo_pin_xpos;
    uint servo_pin_ypos;
    uint servo_pin_xneg;
    uint servo_pin_yneg;
    uint servo_pin_throttle;
} rocket_param_t;


template <typename IMU_T, typename REGULATOR_T, typename CONTROLLER_T>
class Rocket {
private:
    Eigen::Vector3d _gyro_vals;
    Eigen::Vector3d _acc_vals;
    Eigen::Vector3d _desired_orientation;  // roll, pitch, yaw
    Eigen::Vector3d _current_orientation;  // roll, pitch, yaw
    float _desired_throttle;
    rocket_param_t _params;

    IMU_T *_imu;
    REGULATOR_T* _regulator;
    CONTROLLER_T* _controller;

    IServo* _servo_xpos;
    IServo* _servo_ypos;
    IServo* _servo_xneg;
    IServo* _servo_yneg;
    IServo* _throttle;
    
    // curent values for steering:
    float _throttle_value;  // 0-100%
    float _xpos_value;  // -45, 45
    float _ypos_value;  // -45, 45
    float _xneg_value;  // -45, 45
    float _yneg_value;  // -45, 45


    // calculated values for steering:
    Eigen::Vector3d _corrections;  // roll, pitch, yaw
    double _throttle_corr;

    void convert_acc_to_orientation();

public:
    Rocket(rocket_param_t &params) ;

    void setup();
    
    void setup_regulator(void* params);

    void calculate_corrections();

    void steer();

    void update();

    /**
     * @brief Set the x positive angle
     * 
     * @param angle in degrees in range -45, 45
     */
    void set_servo_xpos(float angle);

    /**
     * @brief Set the rudder angle
     * 
     * @param angle in degrees in range -45, 45
     */
    void set_servo_ypos(float angle);

    void set_servo_xneg(float angle);

    void set_servo_yneg(float angle);

    /**
     * @brief Set the throttle value
     */
    void set_throttle();

    /**
     * @brief Performs pre-flight checks for the aircraft's control surfaces and throttle.
     *
     * The pre_flight_check function conducts a series of control surface and throttle
     * tests to ensure the aircraft is ready for operation. These checks are sequenced
     * with delays to allow for visual and instrumental verification.
     *
     * - Tailerons are set to simulate roll maneuvers (both CW and CCW).
     * - Tailerons are also adjusted for pitch up and pitch down.
     * - Rudder is set for a left and right turn.
     * - Throttle is momentarily set to 20% before returning to 0%.
     *
     * A log entry is generated to indicate the completion of the pre-flight checks.
     *
     * @note Make sure that all sub-systems are initialized before calling this function.
     * @note The control settings and gains should be properly configured prior to running this check.
     *
     * Example usage:
     * @code
     * pre_flight_check();
     * @endcode
     */
    void pre_flight_check();

    void print_status();
}; 

#include "Rocket.tpp"