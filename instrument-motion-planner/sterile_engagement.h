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

    std::cout << "Outside start_homing : " << start_homing << ", !exitFlag : " << (!exitFlag) << std::endl;

    bool home_jaws = false;
    bool home_pitch = false;

    bool outer_pitch1_identified = false;
    // bool outer_pitch_2_identified = false;
    double outer_pitch_1_pos;
    double outer_pitch_2_pos;

    bool outer_pos_jaw1_identified = false;
    bool outer_pos_jaw2_identified = false;

    bool inner_pos_jaw1_identified = false;
    bool inner_pos_jaw2_identified = false;

    bool jaw1_backlash_compute = false;
    bool jaw2_backlash_compute = false;

    double outer_pos_jaw1, outer_pos_jaw2, outer_pos_jaw1_ret, outer_pos_jaw2_ret;
    double inner_pos_jaw1, inner_pos_jaw2;

    double jaw1_backlash, jaw2_backlash;

    bool jaw1_center, jaw2_center, pitch_center;

    double center_dist = 0;

    jaw1_center = false;
    jaw2_center = false;
    pitch_center = false;

    bool opening_jaws_step = false;
    bool backlash_jaw1_step = false;
    bool backlash_jaw2_step = false;
    bool centering_step = false;

    bool inner_pos_pitch_identified = false;
    bool outer_pos_pitch_identified = false;
    double inner_pos_pitch = 0;
    double outer_pos_pitch = 0;
    double outer_pos_pitch_ret = 0;
    double pitch_backlash = 0;
    bool pitch_backlash_compute = false;

    std::cout << "homing started \n";

    while (start_homing && !exitFlag)
    {

        sterile_time = sterile_time + 0.001;

        int homing_ctr = 0;

        if (home_jaws == false)
        {

            // Opening the jaws here
            if (opening_jaws_step == false)
            {

                if (outer_pos_jaw1_identified == false)
                {
                    command_pos[1] = command_pos[1] + 0.001;

                    if ((fabs(command_pos[1] - appDataPtr->actual_position[1]) > 0.05))
                    {
                        outer_pos_jaw1 = appDataPtr->actual_position[1];
                        outer_pos_jaw1_identified = true;
                        command_pos[1] = appDataPtr->actual_position[1];
                    }
                }

                if (outer_pos_jaw2_identified == false)
                {
                    command_pos[2] = command_pos[2] - 0.001;

                    if ((fabs(command_pos[2] - appDataPtr->actual_position[2]) > 0.05))
                    {
                        outer_pos_jaw2 = appDataPtr->actual_position[2];
                        outer_pos_jaw2_identified = true;
                        command_pos[2] = appDataPtr->actual_position[2];
                    }
                }
            }

            // Computing the backlash here

            if (outer_pos_jaw1_identified == true && outer_pos_jaw2_identified == true)
            {

                if (backlash_jaw1_step == false)
                {

                    if (jaw1_backlash_compute == false)
                    {

                        if (inner_pos_jaw1_identified == false)
                        {
                            command_pos[1] = command_pos[1] - 0.001;

                            if ((fabs(command_pos[1] - appDataPtr->actual_position[1]) > 0.05))
                            {
                                inner_pos_jaw1 = appDataPtr->actual_position[1];
                                inner_pos_jaw1_identified = true;
                            }
                        }
                        else
                        {

                            command_pos[1] = command_pos[1] + 0.001;

                            if ((fabs(command_pos[1] - appDataPtr->actual_position[1]) > 0.05))
                            {
                                outer_pos_jaw1_ret = appDataPtr->actual_position[1];
                                jaw1_backlash_compute = true;
                                jaw1_backlash = (outer_pos_jaw1 + outer_pos_jaw1_ret) / 2 - inner_pos_jaw1 - (outer_pos_jaw1) / fabs(outer_pos_jaw1) * (33.5 / 180 * 22 / 7);
                                backlash_jaw1_step = true;
                            }
                        }
                    }
                }

                if (backlash_jaw1_step == true && backlash_jaw2_step == false)
                {

                    if (jaw2_backlash_compute == false)
                    {

                        if (inner_pos_jaw2_identified == false)
                        {
                            command_pos[2] = command_pos[2] + 0.001;

                            if ((fabs(command_pos[2] - appDataPtr->actual_position[2]) > 0.05))
                            {
                                inner_pos_jaw2 = appDataPtr->actual_position[2];
                                inner_pos_jaw2_identified = true;
                            }
                        }
                        else
                        {

                            command_pos[2] = command_pos[2] - 0.001;

                            if ((fabs(command_pos[2] - appDataPtr->actual_position[2]) > 0.05))
                            {
                                outer_pos_jaw2_ret = appDataPtr->actual_position[2];
                                jaw2_backlash_compute = true;
                                jaw2_backlash = (outer_pos_jaw2 + outer_pos_jaw2_ret) / 2 - inner_pos_jaw2 - (outer_pos_jaw2) / fabs(outer_pos_jaw2) * (33.5 / 180 * 22 / 7);
                                backlash_jaw2_step = true;
                            }
                        }
                    }
                }
            }

            // Making it center

            if (jaw1_backlash_compute == true && jaw2_backlash_compute == true)
            {

                if (jaw1_center == false && (command_pos[1] > ((outer_pos_jaw1 + outer_pos_jaw1_ret) / 2 - jaw1_backlash)))
                {
                    command_pos[1] = command_pos[1] - 0.001;
                }
                else
                {
                    jaw1_center = true;
                }

                if (jaw2_center == false && (command_pos[2] < ((outer_pos_jaw2 + outer_pos_jaw2_ret) / 2 - jaw2_backlash)))
                {
                    command_pos[2] = command_pos[2] + 0.001;
                }
                else
                {
                    jaw2_center = true;
                }

                if (jaw1_center == true && jaw2_center == true)
                {

                    if (center_dist < (33.5 / 180 * 22 / 7) / 2)
                    {
                        command_pos[1] = command_pos[1] - 0.001;
                        command_pos[2] = command_pos[2] + 0.001;
                        center_dist = center_dist + 0.001;
                    }
                    else
                    {
                        command_pos[1] = appDataPtr->actual_position[1];
                        command_pos[2] = appDataPtr->actual_position[2];
                        home_jaws = true;
                        command_pos[0] = appDataPtr->actual_position[0];
                        std::cout<<" jaws engaged : Command pos[0] : "<<command_pos[0]<<", appDataPtr->actual_position[0] : "<<appDataPtr->actual_position[0]<<std::endl;
                        center_dist = 0;
                    }
                }
            }
        }
        else if (home_pitch == false)
        {

            if (outer_pos_pitch_identified == false)
            {

                command_pos[0] = command_pos[0] + 0.001;

                if ((fabs(command_pos[0]) - appDataPtr->actual_position[0]) > 0.05)
                {
                    // std::cout << "pitch_part_engaged" << std::endl;
                    outer_pos_pitch = appDataPtr->actual_position[0];
                    command_pos[0] = appDataPtr->actual_position[0];
                    outer_pos_pitch_identified = true;
                    // std::cout << "outer_pos_pitch : " << outer_pos_pitch << std::endl;
                }
            }

            if (outer_pos_pitch_identified)
            {

                if (pitch_backlash_compute == false)
                {
                    if (inner_pos_pitch_identified == false)
                    {
                        // std::cout<<"inner_pos_pitch_identified "<<std::endl;
                        command_pos[0] = command_pos[0] - 0.001;

                        if ((fabs(command_pos[0]) - appDataPtr->actual_position[0]) > 0.05)
                        {
                            inner_pos_pitch = appDataPtr->actual_position[0];
                            command_pos[0] = appDataPtr->actual_position[0];
                            inner_pos_pitch_identified = true;
                            // std::cout << "inner_pos_pitch : " << inner_pos_pitch << std::endl;
                        }
                    }
                    else
                    {
                        command_pos[0] = command_pos[0] + 0.001;

                        // std::cout<<"inner_pos_pitch_identified true, command_pos[0]:  "<<command_pos[0]<<", appDataPtr->actual_position[0] : "<<appDataPtr->actual_position[0]<<std::endl;

                        if ((fabs(command_pos[0]) - appDataPtr->actual_position[0]) > 0.05)
                        {
                            outer_pos_pitch_ret = appDataPtr->actual_position[0];
                            command_pos[0] = appDataPtr->actual_position[0];
                            pitch_backlash_compute = true;
                            pitch_backlash = (outer_pos_pitch + outer_pos_pitch_ret) / 2 - inner_pos_pitch - (outer_pos_pitch) / fabs(outer_pos_pitch) * (7.0 / 180.0 * 22.0 / 7.0);
                            // std::cout << ", outer_pos_pitch : " << outer_pos_pitch << ", outer_pos_pitch_ret : " << outer_pos_pitch_ret << ", inner_pos_pitch : " << inner_pos_pitch << ", Comp term : " << (outer_pos_pitch) / fabs(outer_pos_pitch) * (7.0 / 180.0 * 22.0 / 7.0) << std::endl;

                            // std::cout << "pitch_backlash : " << pitch_backlash << std::endl;
                        }
                    }
                }
            }

            if (pitch_backlash_compute == true)
            {
                // std::cout << "Making pitch part straight, pitch_backlash: " << pitch_backlash << std::endl;

                // std::cout << "command_pos[0] : " << command_pos[0] << ", outer_pos_pitch : " << outer_pos_pitch << ", outer_pos_pitch_ret : " << outer_pos_pitch_ret << std::endl;

                // std::cout << "command_pos[0] : " << command_pos[0] << ", outer_pos_pitch : " << (outer_pos_pitch + outer_pos_pitch_ret) / 2 << ", pitch_backlash : " << pitch_backlash << std::endl;

                // std::cout<<"(command_pos[0] > ((outer_pos_pitch + outer_pos_pitch_ret) / 2 - pitch_backlash))"

                if (pitch_center == false && (command_pos[0] > ((outer_pos_pitch + outer_pos_pitch_ret) / 2 - pitch_backlash)))
                {
                    command_pos[0] = command_pos[0] - 0.001;
                }
                else
                {
                    pitch_center = true;
                    // std::cout << "command_pos[0] : " << command_pos[0] << ", outer_pos_pitch : " << (outer_pos_pitch + outer_pos_pitch_ret) / 2 << ", pitch_backlash : " << pitch_backlash << std::endl;
                }

                if (pitch_center == true)
                {

                    if (center_dist < (7 / 180 * 22 / 7) / 2)
                    {
                        command_pos[0] = command_pos[0] - 0.001;
                        center_dist = center_dist + 0.001;
                    }
                    else
                    {
                        command_pos[0] = appDataPtr->actual_position[0];
                        home_pitch = true;
                        start_homing = false;

                        std::cout<<"home_pitch : "<<std::endl;
                    }
                }
            }
        }
        else
        {
            std::cout<<"homing over"<<std::endl;
        }

        // for (int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        // {

        //     if

        //     // // std::cout<<"do_home["<<jnt_ctr<<"]"<<do_home[jnt_ctr]<<std::endl;

        //     // if (do_home[jnt_ctr])
        //     // {
        //     //     command_pos[jnt_ctr] = command_pos[jnt_ctr] + 0.001;

        //     //     if ((fabs(command_pos[jnt_ctr] - appDataPtr->actual_position[jnt_ctr]) > 0.5 && (fabs(appDataPtr->actual_torque[jnt_ctr]) > 1.0)))
        //     //     {
        //     //         std::cout << "fabs(command_pos[jnt_ctr] - appDataPtr->actual_position[jnt_ctr]) > 0.5 : " << (fabs(command_pos[jnt_ctr] - appDataPtr->actual_position[jnt_ctr]) > 0.5) << std::endl;
        //     //         std::cout << "(fabs(appDataPtr->actual_torque[jnt_ctr]) > 1.0 ) : " << (fabs(appDataPtr->actual_torque[jnt_ctr]) > 1.0) << std::endl;
        //     //         std::cout << "(command_pos[jnt_ctr] > final_pos[jnt_ctr]) : " << (command_pos[jnt_ctr] > final_pos[jnt_ctr]) << std::endl;

        //     //         do_home[jnt_ctr] = false;
        //     //     }
        //     // }
        //     // else
        //     // {
        //     //     command_pos[jnt_ctr] = appDataPtr->actual_position[jnt_ctr];
        //     //     homing_ctr++;
        //     // }

        //     // std::cout<<"do_home["<<jnt_ctr<<"]"<<do_home[jnt_ctr]<<", command_pos: "<<command_pos[jnt_ctr]<<", actual_pos : "<<appDataPtr->actual_position[jnt_ctr]<<"actual torq : "<<appDataPtr->actual_torque[jnt_ctr]<<std::endl;
        // }

        // if (homing_ctr == NUM_JOINTS)
        // {
        //     start_homing = false;
        // }

        write_to_drive(command_pos);

        // std::cout<<"start_homing : "<<start_homing<<", !exitFlag : "<<(!exitFlag)<<", homing_ctr : "<<homing_ctr<<std::endl;

        usleep(1000);
    }

    commandDataPtr->type = CommandType::NONE;

    return 0;
}