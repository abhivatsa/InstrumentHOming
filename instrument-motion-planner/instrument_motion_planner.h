#pragma once

#include "SharedObject.h"
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

volatile sig_atomic_t exitFlag = 0;

class InstrumentMotionPlanner
{
public:
    InstrumentMotionPlanner();
    ~InstrumentMotionPlanner();
    void run();

private:
    SystemData *systemDataPtr;
    AppData *appDataPtr;
    CommandData *commandDataPtr;
    ForceDimData *forceDataPtr;

    void stackPrefault();
    void cyclicTask();
    static void signalHandler(int signum);

    // double sterile_engagement();
    // int pt_to_pt_mvmt(double ini_pos[NUM_JOINTS], double final_pos[NUM_JOINTS]);
    double jog(int index, int dir, int type);
    void Jog();
    double sterile_engagement();
    int write_to_drive(double joint_pos[4]);
    void configureSharedMemory();
    void createSharedMemory(int &shm_fd, const char *name, int size);
    void mapSharedMemory(void *&ptr, int shm_fd, int size);
    void initializeSharedData();

    struct period_info
    {
        struct timespec next_period;
        long period_ns;
    };

    static void inc_period(struct period_info *pinfo);
    static void periodic_task_init(struct period_info *pinfo);
    void do_rt_task();
    static void wait_rest_of_period(struct period_info *pinfo);

};
