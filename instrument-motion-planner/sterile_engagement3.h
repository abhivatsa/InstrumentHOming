#pragma once

#include "instrument_motion_planner.h"

double InstrumentMotionPlanner::sterile_engagement()
{

    double ini_pos[NUM_JOINTS], final_pos[NUM_JOINTS];

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        ini_pos[jnt_ctr] = appDataPtr->actual_position[jnt_ctr];
        final_pos[jnt_ctr] = ini_pos[jnt_ctr] + (4 * M_PI + 0.02);
    }

    bool start_homing = true;
    double sterile_time = 0;
    double command_pos[NUM_JOINTS];
    bool do_home[NUM_JOINTS] = {true, true, true, true};
    std::copy(std::begin(appDataPtr->actual_position), std::end(appDataPtr->actual_position), std::begin(command_pos));

    std::cout<<"Outside start_homing : "<<start_homing<<", !exitFlag : "<<(!exitFlag)<<std::endl;

    while (start_homing && !exitFlag){

        double total_movement = M_PI/3;
        double total_time = 300;
        double time = 0;

        // pitch movement

        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0] + total_movement/total_time*time;
            final_pos[1] = ini_pos[1] - 0.7 * total_movement/total_time*time;
            final_pos[2] = ini_pos[2] - 0.7 * total_movement/total_time*time;
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < 2*total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0] + (total_movement - total_movement/total_time*time);
            final_pos[1] = ini_pos[1] - 0.7 * (total_movement - total_movement/total_time*time);
            final_pos[2] = ini_pos[2] - 0.7 * (total_movement - total_movement/total_time*time);
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0] + (-total_movement + total_movement/total_time*time);
            final_pos[1] = ini_pos[1] - 0.7 * (-total_movement + total_movement/total_time*time);
            final_pos[2] = ini_pos[2] - 0.7 * (-total_movement + total_movement/total_time*time);
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        usleep(100000);

        // yaw movement

        time = 0;
        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1] + total_movement/total_time*time;
            final_pos[2] = ini_pos[2] + total_movement/total_time*time;
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < 2*total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1] + (total_movement - total_movement/total_time*time);
            final_pos[2] = ini_pos[2] + (total_movement - total_movement/total_time*time);
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1] + (-total_movement + total_movement/total_time*time);
            final_pos[2] = ini_pos[2] + (-total_movement + total_movement/total_time*time);
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        usleep(100000);

        // pinch movement

        time = 0;

        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1] + total_movement/total_time*time;
            final_pos[2] = ini_pos[2] - total_movement/total_time*time;
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < 2*total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1] + (total_movement - total_movement/total_time*time);
            final_pos[2] = ini_pos[2] - (total_movement - total_movement/total_time*time);
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1] + (-total_movement + total_movement/total_time*time);
            final_pos[2] = ini_pos[2] - (-total_movement + total_movement/total_time*time);
            final_pos[3] = ini_pos[3];
            write_to_drive(final_pos);
            usleep(1000);
        }

        usleep(100000);

        // roll movement

        time = 0;

        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1];
            final_pos[2] = ini_pos[2];
            final_pos[3] = ini_pos[3] + 3*total_movement/total_time*time;
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < 2*total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1];
            final_pos[2] = ini_pos[2];
            final_pos[3] = ini_pos[3] + 3*(total_movement - total_movement/total_time*time);
            write_to_drive(final_pos);
            usleep(1000);
        }

        time = 0;
        while (time < total_time){
            time = time + 1;
            final_pos[0] = ini_pos[0];
            final_pos[1] = ini_pos[1];
            final_pos[2] = ini_pos[2];
            final_pos[3] = ini_pos[3] + 3*(-total_movement + total_movement/total_time*time);
            write_to_drive(final_pos);
            usleep(1000);
        }
        
        usleep(1000);

    }

    

    commandDataPtr->type = CommandType::NONE;

    return 0;
}