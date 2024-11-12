#include "cyclicTask.h"

int main()
{
    SafetyController safety_ctrl;
    safety_ctrl.run();
    return 0;
}


SafetyController::SafetyController(){
    configureSharedMemory();
}

SafetyController::~SafetyController(){

}

void SafetyController::run(){

    // Set CPU affinity for real-time thread
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // Set to the desired CPU core

    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1)
    {
        perror("Error setting CPU affinity");
        // Handle the error appropriately based on your application's requirements
    }

    // Lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
        // Handle the error appropriately based on your application's requirements
    }

    stackPrefault();

    // Register signal handler to gracefully stop the program
    signal(SIGINT, SafetyController::signalHandler);

    struct sched_param param = {};
    param.sched_priority = 49;

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    systemStateDataPtr->safety_controller_enabled = true;

    cyclicTask();

}

void SafetyController::stackPrefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

void SafetyController::signalHandler(int signum)
{
    if (signum == SIGINT)
    {
        std::cout << "Signal received: " << signum << std::endl;
        exitFlag = 1; // Set the flag to indicate the signal was received
    }
}


void SafetyController::configureSharedMemory()
{
    int shm_fd_jointData;
    int shm_fd_systemStateData;
    int shm_fd_appData;

    createSharedMemory(shm_fd_jointData, "JointData", sizeof(JointData));
    createSharedMemory(shm_fd_systemStateData, "SystemStateData", sizeof(SystemStateData));
    createSharedMemory(shm_fd_appData, "AppData", sizeof(AppData));

    mapSharedMemory((void *&)jointDataPtr, shm_fd_jointData, sizeof(JointData));
    mapSharedMemory((void *&)systemStateDataPtr, shm_fd_systemStateData, sizeof(SystemStateData));
    mapSharedMemory((void *&)appDataPtr, shm_fd_appData, sizeof(AppData));

    initializeSharedData();
}

void SafetyController::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void SafetyController::mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

void SafetyController::initializeSharedData()
{
    jointDataPtr->setZero();
    systemStateDataPtr->setZero();
    appDataPtr->setZero();
}


void SafetyController::read_data()
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        appDataPtr->actual_position[jnt_ctr] = conv_to_actual_pos(jointDataPtr->joint_position[jnt_ctr], jnt_ctr);
        appDataPtr->actual_velocity[jnt_ctr] = conv_to_actual_velocity(jointDataPtr->joint_velocity[jnt_ctr], jnt_ctr);
        appDataPtr->actual_torque[jnt_ctr] = conv_to_actual_torque(jointDataPtr->joint_torque[jnt_ctr], jnt_ctr);

        // std::cout<<"jointDataPtr->joint_position[jnt_ctr] : "<<jointDataPtr->joint_position[jnt_ctr]<<std::endl;
    }

    appDataPtr->sterile_detection = jointDataPtr->sterile_detection_status;
    appDataPtr->instrument_detection = jointDataPtr->instrument_detection_status;
}

void SafetyController::write_data()
{

    if (!systemStateDataPtr->trigger_error_mode && systemStateDataPtr->status_operation_enabled)
    {
        // systemStateDataPtr->drive_operation_mode = appDataPtr->drive_operation_mode;
        // for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        // {
        //     jointDataPtr->target_position[jnt_ctr] = conv_to_target_pos(appDataPtr->target_position[jnt_ctr], jnt_ctr);
        //     jointDataPtr->target_velocity[jnt_ctr] = conv_to_target_velocity(appDataPtr->target_velocity[jnt_ctr], jnt_ctr);
        //     jointDataPtr->target_torque[jnt_ctr] = conv_to_target_torque(appDataPtr->target_torque[jnt_ctr], jnt_ctr);
        // }

        systemStateDataPtr->drive_operation_mode = appDataPtr->drive_operation_mode;
        for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
        {
            // std::cout<<"appDataPtr->target_position "<<jnt_ctr<<" : "<<appDataPtr->target_position[jnt_ctr]<<std::endl;
            // std::cout<<"jointDataPtr->target_position[jnt_ctr] "<<jnt_ctr<<" : "<<conv_to_target_pos(appDataPtr->target_position[jnt_ctr], jnt_ctr)<<std::endl;
            jointDataPtr->target_position[jnt_ctr] = conv_to_target_pos(appDataPtr->target_position[jnt_ctr], jnt_ctr);
            jointDataPtr->target_velocity[jnt_ctr] = conv_to_target_velocity(appDataPtr->target_velocity[jnt_ctr], jnt_ctr);
            jointDataPtr->target_torque[jnt_ctr] = conv_to_target_torque(appDataPtr->target_torque[jnt_ctr], jnt_ctr);
        }
    }
}

int SafetyController::conv_to_target_pos(double rad, int jnt_ctr)
{
    // input in radians, output in encoder count (SEE Object 0x607A)
    if (fabs(rad) > pos_limit[jnt_ctr]){
        return (int)(enc_count[jnt_ctr] * gear_ratio[jnt_ctr] * (rad/fabs(rad) * pos_limit[jnt_ctr]) / (2 * M_PI));  
    }
    else{
        return (int)(enc_count[jnt_ctr] * gear_ratio[jnt_ctr] * rad / (2 * M_PI)); 
    }
}

double SafetyController::conv_to_actual_pos(int count, int jnt_ctr)
{
    // input in encoder count, Output in radians (SEE Object 0x6064)
    return (count / (enc_count[jnt_ctr] * gear_ratio[jnt_ctr]) * (2 * M_PI)); 
}

int SafetyController::conv_to_target_velocity(double rad_sec, int jnt_ctr)
{
    // input in rad/sec, Output in rpm (SEE Object 0X60FF)
    if( fabs(rad_sec) > vel_limit[jnt_ctr]){
        return (int)( (rad_sec/fabs(rad_sec)*vel_limit[jnt_ctr]) / (2 * M_PI) * 60 * gear_ratio[jnt_ctr]);
    }
    else{
        return (int)(rad_sec / (2 * M_PI) * 60 * gear_ratio[jnt_ctr]);
    }
}

double SafetyController::conv_to_actual_velocity(int rpm, int jnt_ctr)
{
    // input in rpm , Output in rad/sec SEE Object (0x606C)
    return (2 * M_PI * rpm / (60 * gear_ratio[jnt_ctr]));
}

int SafetyController::conv_to_target_torque(double torq_val, int jnt_ctr)
{
    // input is torque in N-m, Output is in per thousand of rated torque (SEE Object 0x6071)
    if ( fabs(torq_val) > torque_limit[jnt_ctr]){
        return (int)( (torq_val/fabs(torq_val)*torque_limit[jnt_ctr] ) / (rated_torque[jnt_ctr] * gear_ratio[jnt_ctr]) * 1000);
    }
    return (int)(torq_val / (rated_torque[jnt_ctr] * gear_ratio[jnt_ctr]) * 1000);
}

double SafetyController::conv_to_actual_torque(int torq_val, int jnt_ctr)
{
    // input torq in terms of per thousand of rated torque, Output is in N-m (SEE object 0x6077)
    return (torq_val * rated_torque[jnt_ctr] * gear_ratio[jnt_ctr] / 1000);
}



