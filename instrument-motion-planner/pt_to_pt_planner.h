#pragma once

using namespace std;

#include "instrument_motion_planner.h"

int InstrumentMotionPlanner::pt_to_pt_mvmt(double ini_pos[NUM_JOINTS], double final_pos[4])
{

    int num_joints = 4;

    double joint_vel[4] = {0.5, 0.5, 0.5, 0.5};
    double joint_acc[4] = {0.5, 0.5, 0.5, 0.5};

    // taking care of vel and acceleration sign
    for (int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {

        if (final_pos[jnt_ctr] < ini_pos[jnt_ctr])
        {
            joint_vel[jnt_ctr] = -joint_vel[jnt_ctr];
            joint_acc[jnt_ctr] = -joint_acc[jnt_ctr];
        }
    }

    // computing minimum time
    double max_time, global_acc_time, global_cruise_time;
    max_time = 0;

    for (int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        double acc_dist, cruise_dist;

        double acc_time, cruise_time, local_time;

        if (fabs(final_pos[jnt_ctr] - ini_pos[jnt_ctr]) < fabs(joint_vel[jnt_ctr] * joint_vel[jnt_ctr] / (joint_acc[jnt_ctr])))
        {

            double joint_diff = final_pos[jnt_ctr] - ini_pos[jnt_ctr];

            if (fabs(joint_diff) > 1e-4)
            {
                joint_vel[jnt_ctr] = 0.99 * joint_vel[jnt_ctr] / fabs(joint_vel[jnt_ctr]) * sqrt(2 * fabs(joint_diff * joint_acc[jnt_ctr]));

                acc_dist = joint_vel[jnt_ctr] * joint_vel[jnt_ctr] / (2 * joint_acc[jnt_ctr]);
                acc_time = joint_vel[jnt_ctr] / joint_acc[jnt_ctr];
                cruise_time = (joint_diff - 2 * acc_dist) / joint_vel[jnt_ctr];

                local_time = 2 * acc_time + cruise_time;
            }
            else
            {
                local_time = 0;
                acc_time = 0;
                cruise_time = 0;
            }
        }
        else
        {

            double joint_diff = final_pos[jnt_ctr] - ini_pos[jnt_ctr];
            acc_dist = joint_vel[jnt_ctr] * joint_vel[jnt_ctr] / (2 * joint_acc[jnt_ctr]);
            acc_time = joint_vel[jnt_ctr] / joint_acc[jnt_ctr];
            cruise_time = (joint_diff - 2 * acc_dist) / joint_vel[jnt_ctr];
            local_time = 2 * acc_time + cruise_time;
        }

        if (local_time > max_time)
        {
            max_time = local_time;
            global_acc_time = acc_time;
            global_cruise_time = cruise_time;
        }
    }

    // computing joint_vel and joint acc
    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        double joint_diff = final_pos[jnt_ctr] - ini_pos[jnt_ctr];
        joint_acc[jnt_ctr] = joint_diff / (global_acc_time * global_acc_time + global_acc_time * global_cruise_time);
    }

    double t = 0;
    double current_pos[4] = {0};
    double current_vel[4] = {0};
    double current_acc[4] = {0};

    // std::cout << "max_time : " << max_time << std::endl;

    while (t < max_time)
    {
        /* int clock_gettime( clockid_t clock_id, struct
     timespec *tp ); The clock_gettime() function gets
     the current time of the clock specified by clock_id,
     and puts it into the buffer  pointed to by tp.tp
     parameter points to a structure containing
     atleast the following members:
     struct timespec {
               time_t   tv_sec;        // seconds
               long     tv_nsec;       // nanoseconds
           };
    clock id = CLOCK_REALTIME, CLOCK_PROCESS_CPUTIME_ID,
               CLOCK_MONOTONIC ...etc
    CLOCK_REALTIME : clock  that  measures real (i.e., wall-clock) time.
    CLOCK_PROCESS_CPUTIME_ID : High-resolution per-process timer
                               from the CPU.
    CLOCK_MONOTONIC : High resolution timer that is unaffected
                      by system date changes (e.g. NTP daemons).  */
        struct timespec start, end;

        clock_gettime(CLOCK_MONOTONIC, &start);

        // unsync the I/O of C and C++.
        ios_base::sync_with_stdio(false);

        t = t + 0.002;

        if ((fabs(t - max_time) < 1e-4) || t > max_time)
        {
            t = max_time;
        }

        // std::cout<<t<<",";

        for (int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
        {

            if (t < global_acc_time)
            {
                current_acc[jnt_ctr] = joint_acc[jnt_ctr];
                current_vel[jnt_ctr] = joint_acc[jnt_ctr] * t;
                current_pos[jnt_ctr] = ini_pos[jnt_ctr] + 0.5 * joint_acc[jnt_ctr] * t * t;
            }
            else if (t < global_acc_time + global_cruise_time)
            {
                current_acc[jnt_ctr] = 0;
                current_vel[jnt_ctr] = joint_acc[jnt_ctr] * global_acc_time;
                current_pos[jnt_ctr] = ini_pos[jnt_ctr] + 0.5 * joint_acc[jnt_ctr] * global_acc_time * global_acc_time + current_vel[jnt_ctr] * (t - global_acc_time);
            }
            else if (t < 2 * global_acc_time + global_cruise_time)
            {
                current_acc[jnt_ctr] = -joint_acc[jnt_ctr];
                current_vel[jnt_ctr] = joint_acc[jnt_ctr] * (max_time - t);
                current_pos[jnt_ctr] = ini_pos[jnt_ctr] + final_pos[jnt_ctr] - ini_pos[jnt_ctr] - 0.5 * joint_acc[jnt_ctr] * (max_time - t) * (max_time - t);
            }
            else
            {
                current_acc[jnt_ctr] = 0;
                current_vel[jnt_ctr] = 0;
                current_pos[jnt_ctr] = final_pos[jnt_ctr];
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &end);

        // Calculating total time taken by the program.
        double time_taken;
        time_taken = (end.tv_sec - start.tv_sec) * 1e9;
        time_taken = (time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9;

        write_to_drive(current_pos);

        usleep(2000);
    }

    return 0;
}