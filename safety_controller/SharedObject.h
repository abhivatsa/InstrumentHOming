#pragma once

#include <cstring>
#include <iostream>
#include <unistd.h>
#include <cmath>

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

struct AppData
{
    void setZero()
    {
        // Initialize boolean flags
        switch_to_operation = false;
        initialize_drives = false;
        initialize_system = false;
        trigger_error = false;
        safety_process_status = false;
        drive_initialized = false;
        safety_check_done = false;
        reset_error = false;
        operation_enable_status = false;

        // Use std::fill_n for array initialization
        std::fill_n(actual_position, NUM_JOINTS, 0.0);
        std::fill_n(actual_velocity, NUM_JOINTS, 0.0);
        std::fill_n(actual_torque, NUM_JOINTS, 0.0);
        std::fill_n(cart_pos, NUM_JOINTS, 0.0);
        std::fill_n(target_position, NUM_JOINTS, 0.0);
        std::fill_n(target_velocity, NUM_JOINTS, 0.0);
        std::fill_n(target_torque, NUM_JOINTS, 0.0);

        // Initialize other members
        drive_operation_mode = OperationModeState::POSITION_MODE;
        switched_on = false;
        sterile_detection = false;
        instrument_detection = false;
        simulation_mode = false;
    }

    double actual_position[NUM_JOINTS];
    double actual_velocity[NUM_JOINTS];
    double actual_torque[NUM_JOINTS];
    double cart_pos[NUM_JOINTS];
    double target_position[NUM_JOINTS];
    double target_velocity[NUM_JOINTS];
    double target_torque[NUM_JOINTS];
    OperationModeState drive_operation_mode;
    bool switched_on;
    bool sterile_detection;
    bool instrument_detection;
    bool simulation_mode;

    bool trigger_error;
    bool safety_process_status;
    bool initialize_system;
    bool initialize_drives;
    bool drive_initialized;
    bool switch_to_operation;
    bool safety_check_done;
    bool operation_enable_status;
    bool reset_error;
};
