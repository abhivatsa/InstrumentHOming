#pragma once
#include "master.h"

void EthercatMaster::transitionToState(ControlWordValues value, int jnt_ctr)
{
    EC_WRITE_U16(domainPd + driveOffset[jnt_ctr].controlword, value);
}

StatusWordValues EthercatMaster::readDriveState(int joint_num)
{
    uint16_t drive_status = EC_READ_U16(domainPd + driveOffset[joint_num].statusword);

    // cout << "update_state" << endl;
    if (((drive_status | 65456) ^ 65456) == 0)
    {
        std::cout<<"Drive not ready to Switch on \n";
        return StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
    }
    else if (((drive_status | 65456) ^ 65520) == 0)
    {
        return StatusWordValues::SW_SWITCH_ON_DISABLED;
    }
    else if (((drive_status | 65424) ^ 65457) == 0)
    {
        return StatusWordValues::SW_READY_TO_SWITCH_ON;
    }
    else if (((drive_status | 65424) ^ 65459) == 0)
    {
        return StatusWordValues::SW_SWITCHED_ON;
    }
    else if (((drive_status | 65424) ^ 65463) == 0)
    {
        return StatusWordValues::SW_OPERATION_ENABLED;
    }
    else if (((drive_status | 65456) ^ 65471) == 0)
    {
        // Fault Reaction Active
        std::cout<<"Drive is in Fault reaction Active \n";
        return StatusWordValues::SW_FAULT_REACTION_ACTIVE;
    }
    else if (((drive_status | 65456) ^ 65464) == 0)
    {
        // Fault
        std::cout<<"Drive is in Fault \n";
        std::cout<<"error code : "<<EC_READ_U16(domainPd + driveOffset[joint_num].error_code)<<std::endl;

        return StatusWordValues::SW_FAULT;
    }
    else{
        return StatusWordValues::SW_NOT_READY_TO_SWITCH_ON;
    }
}