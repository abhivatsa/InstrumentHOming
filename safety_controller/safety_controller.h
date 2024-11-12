#pragma once

#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>
#include "SharedObject.h"

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

volatile sig_atomic_t exitFlag = 0;

// MOTOR_TYPE = 0 for Faulhaber
// MOTOR_TYPE = 1 for Maxon
#define MOTOR_TYPE 0

#if MOTOR_TYPE == 0
double gear_ratio[NUM_JOINTS] = {50, 50, 50, 50};
double rated_torque[NUM_JOINTS] = { 0.02, 0.02, 0.02, 0.02};
double enc_count[NUM_JOINTS] = {4096, 4096, 4096, 4096};
double pos_limit[NUM_JOINTS] = {10*M_PI, 10*M_PI, 10*M_PI, 10*M_PI};
double vel_limit[NUM_JOINTS] = {M_PI, M_PI, M_PI, M_PI};
double torque_limit[NUM_JOINTS] = {180, 180, 180, 180};
#elif MOTOR_TYPE == 1
double gear_ratio[4] = {50, 50, 50, 50};
double rated_torque[4] = { 0.02, 0.02, 0.02, 0.02};
double enc_count[4] = {4096, 4096, 4096, 4096};
double pos_limit[4] = {10*M_PI, 10*M_PI, 10*M_PI, 10*M_PI};
double vel_limit[4] = {M_PI, M_PI, M_PI, M_PI};
double torque_limit[4] = {180, 180, 180, 50};
#else
double gear_ratio[4] = {50, 50, 50, 50};
double rated_torque[4] = { 0.02, 0.02, 0.02, 0.02};
double enc_count[4] = {4096, 4096, 4096, 4096};
double pos_limit[4] = {10*M_PI, 10*M_PI, 10*M_PI, 10*M_PI};
double vel_limit[4] = {M_PI, M_PI, M_PI, M_PI};
double torque_limit[4] = {180, 180, 180, 50};
#endif

class SafetyController
{
public:
    SafetyController();
    ~SafetyController();
    void run();

private:
    JointData *jointDataPtr;
    SystemStateData *systemStateDataPtr;
    AppData *appDataPtr;
    void stackPrefault();
    void cyclicTask();
    static void signalHandler(int signum);

    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void mapSharedMemory(void *&ptr, int shm_fd, int size);
    void initializeSharedData();
    void write_data();
    void read_data();

    bool check_limits();
    void joint_pos_limit_check();
    void joint_vel_limit_check();
    void joint_torq_limit_check();


    struct period_info
    {
        struct timespec next_period;
        long period_ns;
    };

    static void inc_period(struct period_info *pinfo);
    static void periodic_task_init(struct period_info *pinfo);
    void do_rt_task();
    static void wait_rest_of_period(struct period_info *pinfo);

    int conv_to_target_pos(double rad, int jnt_ctr);
    double conv_to_actual_pos(int count, int jnt_ctr);
    int conv_to_target_velocity(double rad_sec, int jnt_ctr);
    double conv_to_actual_velocity(int rpm, int jnt_ctr);
    int conv_to_target_torque(double torq_val, int jnt_ctr);
    double conv_to_actual_torque(int torq_val, int jnt_ctr);

};