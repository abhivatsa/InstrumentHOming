#pragma once

#include "safety_controller.h"

void SafetyController::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    while (!exitFlag)
    {
        do_rt_task();
        wait_rest_of_period(&pinfo);
    }
}

void SafetyController::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void SafetyController::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void SafetyController::do_rt_task()
{
    // move initialize out of real
    switch (systemStateDataPtr->safety_state)
    {
    case SafetyStates::INITIALIZE:
        if (systemStateDataPtr->drive_state == DriveState::INITIALIZE)
        {
            // send signal to motion planner
            if (appDataPtr->initialize_system) // motion planner initialized?
            {
                systemStateDataPtr->initialize_drives = true;
            }
        }

        if (systemStateDataPtr->drive_state == DriveState::SWITCHED_ON)
        {
            appDataPtr->drive_initialized = true;
            if (systemStateDataPtr->start_safety_check)
            {
                read_data();
                if (check_limits())
                {
                    appDataPtr->trigger_error = false;
                    systemStateDataPtr->safety_state = SafetyStates::READY_FOR_OPERATION;
                }
                else
                {
                    appDataPtr->trigger_error = true;
                    systemStateDataPtr->safety_state = SafetyStates::RECOVERY;
                }
                systemStateDataPtr->safety_check_done = true;
            }
        }

        // if (systemStateDataPtr->drive_state == DriveState::ERROR)
        // {
        //     // Take a command to reset Error From User
        //     // appDataPtr->drive_initialized = true;
        //     // do someting
        // }
        break;
    case SafetyStates::READY_FOR_OPERATION:
        check_limits();
        read_data();

        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            appDataPtr->target_position[jnt_ctr] = appDataPtr->actual_position[jnt_ctr];
            appDataPtr->target_velocity[jnt_ctr] = appDataPtr->actual_velocity[jnt_ctr];
            appDataPtr->target_torque[jnt_ctr] = appDataPtr->actual_torque[jnt_ctr];
        }

        if (appDataPtr->switch_to_operation) // switch to operation? from motion planner
        {

            systemStateDataPtr->switch_to_operation = true;
            if (systemStateDataPtr->drive_state == DriveState::OPERATION_ENABLED)
            {
                systemStateDataPtr->safety_state = SafetyStates::OPERATION;
            }
        }
        break;
    case SafetyStates::OPERATION:
        if (systemStateDataPtr->drive_state == DriveState::OPERATION_ENABLED)
        {
            appDataPtr->operation_enable_status = true;
            // read write
            read_data();
            if (check_limits())
            {
                write_data();
            }
            else{
                systemStateDataPtr->safety_state = SafetyStates::ERROR; 
            }
        }
        else if (systemStateDataPtr->drive_state == DriveState::ERROR)
        {
            appDataPtr->trigger_error = true;
            systemStateDataPtr->safety_state = SafetyStates::ERROR;
            // send signal to motion planner
        }
        break;
    case SafetyStates::ERROR:
        appDataPtr->setZero();
        if (systemStateDataPtr->drive_state == DriveState::SWITCHED_ON)
        {
            systemStateDataPtr->safety_state = SafetyStates::READY_FOR_OPERATION;
        }
        break;
    case SafetyStates::RECOVERY:
        appDataPtr->setZero();
        if (systemStateDataPtr->drive_state == DriveState::SWITCHED_ON)
        {
            systemStateDataPtr->safety_state = SafetyStates::READY_FOR_OPERATION;
        }
        break;
    default:
        break;
    }
}

void SafetyController::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

bool SafetyController::check_limits()
{

    // joint_pos_limit_check();
    // joint_vel_limit_check();
    // joint_torq_limit_check();

    return !systemStateDataPtr->trigger_error_mode;
}

void SafetyController::joint_pos_limit_check()
{
    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        if ((conv_to_actual_pos(jointDataPtr->joint_position[jnt_ctr], jnt_ctr)) > pos_limit[jnt_ctr])
        {
            systemStateDataPtr->trigger_error_mode = true;
        }
    }
}

void SafetyController::joint_vel_limit_check()
{

    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        if ((conv_to_actual_velocity(jointDataPtr->joint_velocity[jnt_ctr], jnt_ctr)) > vel_limit[jnt_ctr])
        {
            systemStateDataPtr->trigger_error_mode = true;
        }
    }
}

void SafetyController::joint_torq_limit_check()
{
    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        if ((conv_to_actual_torque(jointDataPtr->joint_torque[jnt_ctr], jnt_ctr)) > torque_limit[jnt_ctr])
        {
            systemStateDataPtr->trigger_error_mode = true;
        }
    }
}