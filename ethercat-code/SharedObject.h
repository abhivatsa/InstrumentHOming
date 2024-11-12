#pragma once

#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

constexpr int NUM_JOINTS = 4; // Change this to the desired number of joints

enum class DriveState
{
    INITIALIZE,
    NOT_READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    ERROR,
};

enum class OperationModeState
{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
};

enum class SafetyStates
{
    INITIALIZE,
    READY_FOR_OPERATION,
    OPERATION,
    ERROR,
    RECOVERY,
};

struct JointData
{
    void setZero()
    {
        std::fill_n(joint_position, NUM_JOINTS, 0.0);
        std::fill_n(joint_velocity, NUM_JOINTS, 0.0);
        std::fill_n(joint_torque, NUM_JOINTS, 0.0);
        std::fill_n(target_position, NUM_JOINTS, 0.0);
        std::fill_n(target_velocity, NUM_JOINTS, 0.0);
        std::fill_n(target_torque, NUM_JOINTS, 0.0);
        sterile_detection_status = false;
        instrument_detection_status = false;
    }

    double joint_position[NUM_JOINTS];
    double joint_velocity[NUM_JOINTS];
    double joint_torque[NUM_JOINTS];
    double target_position[NUM_JOINTS];
    double target_velocity[NUM_JOINTS];
    double target_torque[NUM_JOINTS];
    bool sterile_detection_status;
    bool instrument_detection_status;
};

struct SystemStateData
{
    void setZero()
    {
        drive_state = DriveState::ERROR;
        drive_operation_mode = OperationModeState::POSITION_MODE;
        safety_state = SafetyStates::INITIALIZE;
        initialize_drives = false;
        switch_to_operation = false;

        status_switched_on = false;
        status_operation_enabled = false;
        safety_controller_enabled = false;
        trigger_error_mode = false;

        safety_check_done = false;
        start_safety_check = false;
        std::fill_n(drive_enable_for_operation, NUM_JOINTS, false);
    }

    DriveState drive_state;
    OperationModeState drive_operation_mode;
    SafetyStates safety_state;
    bool status_switched_on;
    bool status_operation_enabled;
    bool safety_controller_enabled;
    bool trigger_error_mode;
    bool start_safety_check;
    bool safety_check_done;
    bool initialize_drives;
    bool switch_to_operation;
    bool drive_enable_for_operation[NUM_JOINTS];
};
