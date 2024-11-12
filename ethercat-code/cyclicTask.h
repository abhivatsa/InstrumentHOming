#pragma once

#include "transistionState.h"

using namespace std;

void EthercatMaster::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void EthercatMaster::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void EthercatMaster::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

void EthercatMaster::cyclicTask()
{
    struct period_info pinfo;

    periodic_task_init(&pinfo);

    while (!exitFlag && systemStateDataPtr->safety_controller_enabled)
    {
        ecrt_master_application_time(master, ((uint64_t)pinfo.next_period.tv_sec * 1000000000 + pinfo.next_period.tv_nsec));
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        checkDomainState();
        checkMasterState();
        do_rt_task();
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
        wait_rest_of_period(&pinfo);
    }

    return;
}

void EthercatMaster::do_rt_task()
{
    switch (systemStateDataPtr->drive_state)
    {
    case DriveState::INITIALIZE:
        initializeDrives();
        break;
    case DriveState::SWITCHED_ON:
        handleSwitchedOnState();
        break;
    case DriveState::OPERATION_ENABLED:
        handleOperationEnabledState();
        break;
    case DriveState::ERROR:
        handleErrorState();
        break;
    default:
        break;
    }
}

void EthercatMaster::initializeDrives()
{
    
    int all_drives_enabled = 0;

    for (size_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].max_current, 250);
    }

    if (systemStateDataPtr->initialize_drives == true) // Waiting for command to initialize the Drives
    {
        
        for (size_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            StatusWordValues drive_statusWord = readDriveState(jnt_ctr);

            switch (drive_statusWord)
            {
            case StatusWordValues::SW_FAULT_REACTION_ACTIVE:
                systemStateDataPtr->drive_state = DriveState::ERROR;
                return;
                break;
            case StatusWordValues::SW_FAULT:
                systemStateDataPtr->drive_state = DriveState::ERROR;
                return;
                break;
            case StatusWordValues::SW_SWITCH_ON_DISABLED:
                transitionToState(ControlWordValues::CW_SHUTDOWN, jnt_ctr);
                break;
            case StatusWordValues::SW_READY_TO_SWITCH_ON:
                transitionToState(ControlWordValues::CW_SWITCH_ON, jnt_ctr);
                break;
            case StatusWordValues::SW_SWITCHED_ON:
                all_drives_enabled++;
                break;
            }
        }
    }

    // std::cout<<"initialize drives, all_drives_enabled : "<<all_drives_enabled<<std::endl;
    // std::cout<<"joint max current : "<<1<<" : "<<(EC_READ_U16(domainPd + driveOffset[1].max_current))<<std::endl;


    if (all_drives_enabled == NUM_JOINTS && systemStateDataPtr->initialize_drives == true)
    {
        // std::cout<<"initialize drives "<<std::endl;
        systemStateDataPtr->drive_state = DriveState::SWITCHED_ON;
    }
}

void EthercatMaster::handleSwitchedOnState()
{
    int all_drives_switched_on = 0;

    // std::cout<<"all drives switched on "<<std::endl;

    for (size_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        StatusWordValues drive_state = readDriveState(jnt_ctr);

        if (drive_state == StatusWordValues::SW_SWITCHED_ON || drive_state == StatusWordValues::SW_OPERATION_ENABLED)
        {
            // EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].max_current, 400);
            all_drives_switched_on++;
        }
    }


    if (all_drives_switched_on == NUM_JOINTS)
    {
        read_data();
        systemStateDataPtr->start_safety_check = true;
        if (systemStateDataPtr->safety_check_done && systemStateDataPtr->switch_to_operation)
        {
            int all_drives_op_enable = 0;
            for (size_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
            {
                transitionToState(ControlWordValues::CW_ENABLE_OPERATION, jnt_ctr);
                StatusWordValues drive_state = readDriveState(jnt_ctr);

                if (drive_state == StatusWordValues::SW_OPERATION_ENABLED)
                {
                    // EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].max_current, 1000);
                    all_drives_op_enable++;
                }
            }

            if (all_drives_op_enable == NUM_JOINTS)
            {
                systemStateDataPtr->drive_state = DriveState::OPERATION_ENABLED;
            }
        }
    }
    else
    {
        systemStateDataPtr->drive_state = DriveState::ERROR;
        return;
    }
}

void EthercatMaster::handleOperationEnabledState()
{

    int all_drives_op_enable = 0;

    for (size_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        StatusWordValues drive_state = readDriveState(jnt_ctr);

        // std::cout<<"joint current : "<<jnt_ctr<<" : "<<(EC_READ_S16(domainPd + driveOffset[jnt_ctr].current_actual_value))<<std::endl;
        // jointDataPtr->joint_torque[jnt_ctr] = EC_READ_S16(domainPd + driveOffset[jnt_ctr].torque_actual_value);

        if (drive_state == StatusWordValues::SW_OPERATION_ENABLED)
        {
            all_drives_op_enable++;
        }
    }

    if (all_drives_op_enable == NUM_JOINTS)
    {
        read_data();

        systemStateDataPtr->status_operation_enabled = true;

        jointDataPtr->sterile_detection_status = true;
        jointDataPtr->instrument_detection_status = true;

        switch (systemStateDataPtr->drive_operation_mode)
        {
        case OperationModeState::POSITION_MODE:
            handlePositionMode();
            break;
        case OperationModeState::VELOCITY_MODE:
            handleVelocityMode();
            break;
        case OperationModeState::TORQUE_MODE:
            handleTorqueMode();
            break;
        }
    }
    else
    {
        systemStateDataPtr->drive_state = DriveState::ERROR;
        return;
    }
}

void EthercatMaster::handlePositionMode()
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].modes_of_operation, 8);
        EC_WRITE_S32(domainPd + driveOffset[jnt_ctr].target_position, jointDataPtr->target_position[jnt_ctr]);
    }
}

void EthercatMaster::handleVelocityMode()
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].modes_of_operation, 9);
    }
}

void EthercatMaster::handleTorqueMode()
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].modes_of_operation, 10);
        EC_WRITE_S16(domainPd + driveOffset[jnt_ctr].target_torque, jointDataPtr->target_torque[jnt_ctr]);
    }
}

void EthercatMaster::handleErrorState()
{
    int all_drives_switched_on = 0;

    // Reset Mode of Operation for all the Drives, maybe not required as

    std::cout<<"Drive is in error state\n";

    for (size_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        StatusWordValues drive_state = readDriveState(jnt_ctr);

        if (drive_state == StatusWordValues::SW_FAULT)
        {
            transitionToState(ControlWordValues::CW_RESET, jnt_ctr);
        }
        else if (drive_state == StatusWordValues::SW_SWITCH_ON_DISABLED)
        {
            all_drives_switched_on++;
        }
        else
        {
            transitionToState(ControlWordValues::CW_DISABLE_VOLTAGE, jnt_ctr);
        }
    }

    if (all_drives_switched_on == NUM_JOINTS)
    {
        systemStateDataPtr->drive_state = DriveState::INITIALIZE;
    }
}

void EthercatMaster::read_data()
{

    for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        jointDataPtr->joint_position[jnt_ctr] = EC_READ_S32(domainPd + driveOffset[jnt_ctr].position_actual_value);
        jointDataPtr->joint_velocity[jnt_ctr] = EC_READ_S32(domainPd + driveOffset[jnt_ctr].velocity_actual_value);
        jointDataPtr->joint_torque[jnt_ctr] = EC_READ_S16(domainPd + driveOffset[jnt_ctr].torque_actual_value);
    }
}