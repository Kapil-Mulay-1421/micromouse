#ifndef SIM_HPP
#define SIM_HPP

// #include "macros.hpp"
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <PID_v1.h>
#include "../IMUSensor.h"

// External references to PID variables defined in Mouse.cpp
extern double Setpoint, Input, Output;
extern PID myPID;
extern IMUSensor g_BNOSensor;

class Sim {
public:
    bool errCrct(double distance = 0.25, float velocity = 50); // x is number of blocks to move
    bool turnLeft(int leftPwm = 100, int rightPwm = 100);
    bool turnRight(int leftPwm = 100, int rightPwm = 100);

    static void setID();
    static void scanI2C(); // I2C scanner for debugging

    // Read all sensors and print their values
    static int readLeftSensor();
    static int readRightSensor();
    static int readFrontSensor();

    // Sensor objects and measurement data
    static Adafruit_VL53L0X lox1;
    static Adafruit_VL53L0X lox2;
    static Adafruit_VL53L0X lox3;

    static VL53L0X_RangingMeasurementData_t measure1;
    static VL53L0X_RangingMeasurementData_t measure2;
    static VL53L0X_RangingMeasurementData_t measure3;

};

#endif // SIM_HPP
