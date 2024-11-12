#pragma once

#include "instrument_motion_planner.h"

double InstrumentMotionPlanner::jog(int index, int dir, int type)
{
    if (type == 0) // joint space
    {
        /* code */
        double command_pos[NUM_JOINTS];
        double command_vel[NUM_JOINTS] = {0, 0, 0, 0};
        std::copy(std::begin(appDataPtr->actual_position), std::end(appDataPtr->actual_position), std::begin(command_pos));
        command_pos[index] = appDataPtr->actual_position[index] + dir * 0.01;
        write_to_drive(command_pos);
    }
    else if (type == 1)// task space
    {
        double vel = 4.0;
        double ini_pos[NUM_JOINTS];
        double final_pos[NUM_JOINTS];
        double time = 0;
        for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            ini_pos[jnt_ctr] = appDataPtr->actual_position[jnt_ctr];
            final_pos[jnt_ctr] = ini_pos[jnt_ctr];
        }

        
        if (index == 0) // Pitch motion
        {
            if (dir > 0)
            {
                final_pos[0] = ini_pos[0] + vel * time;
                final_pos[1] = ini_pos[1] - 0.7 * vel * time;
                final_pos[2] = ini_pos[2] - 0.7 * vel * time;
                final_pos[3] = ini_pos[3];
            }
            else if (dir < 0)
            {
                final_pos[0] = ini_pos[0] - vel * time;
                final_pos[1] = ini_pos[1] + 0.7 * vel * time;
                final_pos[2] = ini_pos[2] + 0.7 * vel * time;
                final_pos[3] = ini_pos[3];
            }
        }
        else if (index == 1) // Yaw motion
        {
            if (dir > 0)
            {
                final_pos[0] = ini_pos[0];
                final_pos[1] = ini_pos[1] + vel * time;
                final_pos[2] = ini_pos[2] + vel * time;
                final_pos[3] = ini_pos[3];
            }
            else if (dir < 0)
            {
                final_pos[0] = ini_pos[0];
                final_pos[1] = ini_pos[1] - vel * time;
                final_pos[2] = ini_pos[2] - vel * time;
                final_pos[3] = ini_pos[3];
            }
        }

        else if (index == 2) // pinch motion
        {
            if (dir > 0)
            {
                final_pos[0] = ini_pos[0];
                final_pos[1] = ini_pos[1] + vel * time;
                final_pos[2] = ini_pos[2] - vel * time;
                final_pos[3] = ini_pos[3];
            }
            else if (dir < 0)
            {
                final_pos[0] = ini_pos[0];
                final_pos[1] = ini_pos[1] - vel * time;
                final_pos[2] = ini_pos[2] + vel * time;
                final_pos[3] = ini_pos[3];
            }
        }
        else if (index == 3) // roll motion
        {
            if (dir > 0)
            {
                final_pos[0] = ini_pos[0];
                final_pos[1] = ini_pos[1];
                final_pos[2] = ini_pos[2];
                final_pos[3] = ini_pos[3] + vel * time;
            }
            else if (dir < 0)
            {
                final_pos[0] = ini_pos[0];
                final_pos[1] = ini_pos[1];
                final_pos[2] = ini_pos[2];
                final_pos[3] = ini_pos[3] - vel * time;
            }
        }
        else
        {
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1];
            final_pos[2] = ini_pos[2];
            final_pos[3] = ini_pos[3];
        }

        write_to_drive(final_pos);
    }
    else{

    }
    return 1;
}

void InstrumentMotionPlanner::Jog()
{
    appDataPtr->drive_operation_mode = OperationModeState::POSITION_MODE;

    // if (commandDataPtr->jog_data.type == 0 && appDataPtr->trigger_error == false)
    // {
    //     jog(commandDataPtr->jog_data.index, commandDataPtr->jog_data.dir, 0);
    // }

    // if (commandDataPtr->jog_data.type == 1 && appDataPtr->trigger_error == false)
    // {
    //     jog(commandDataPtr->jog_data.index, commandDataPtr->jog_data.dir, 1);
    // }

    while (commandDataPtr->jog_data.type == 0)
    {
        jog(commandDataPtr->jog_data.index, commandDataPtr->jog_data.dir, 0);
        if (appDataPtr->trigger_error)
            break;
    }

    while (commandDataPtr->jog_data.type == 1)
    {
        jog(commandDataPtr->jog_data.index, commandDataPtr->jog_data.dir, 1);
        if (appDataPtr->trigger_error)
            break;
    }

    commandDataPtr->type = CommandType::NONE;
}