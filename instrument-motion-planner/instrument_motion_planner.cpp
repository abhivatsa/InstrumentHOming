#include "cyclicTask.h"
// #include "sterile_engagement.h"


int main(){
    InstrumentMotionPlanner motion_planner;
    motion_planner.run();
    return 0;
}

InstrumentMotionPlanner::InstrumentMotionPlanner(){
    configureSharedMemory();
}

InstrumentMotionPlanner::~InstrumentMotionPlanner(){
}

void InstrumentMotionPlanner::run(){

    // Set CPU affinity for real-time thread
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset); // Set to the desired CPU core

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
    signal(SIGINT, InstrumentMotionPlanner::signalHandler);

    struct sched_param param = {};
    param.sched_priority = 49;

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    cyclicTask();

}

void InstrumentMotionPlanner::stackPrefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

void InstrumentMotionPlanner::signalHandler(int signum)
{
    if (signum == SIGINT)
    {
        std::cout << "Signal received: " << signum << std::endl;
        exitFlag = 1; // Set the flag to indicate the signal was received
    }
}

void InstrumentMotionPlanner::configureSharedMemory()
{
    int shm_fd_systemData;
    int shm_fd_appData;
    int shm_fd_commandData;
    int shm_fd_forceDimData;

    createSharedMemory(shm_fd_systemData, "SystemData", sizeof(SystemData));
    createSharedMemory(shm_fd_appData, "AppData", sizeof(AppData));
    createSharedMemory(shm_fd_commandData, "CommandData", sizeof(CommandData));
    createSharedMemory(shm_fd_forceDimData, "ForceDimData", sizeof(ForceDimData));

    mapSharedMemory((void *&)systemDataPtr, shm_fd_systemData, sizeof(SystemData));
    mapSharedMemory((void *&)appDataPtr, shm_fd_appData, sizeof(AppData));
    mapSharedMemory((void *&)commandDataPtr, shm_fd_commandData, sizeof(CommandData));
    mapSharedMemory((void *&)forceDataPtr, shm_fd_forceDimData, sizeof(ForceDimData));

    initializeSharedData();
}

void InstrumentMotionPlanner::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void InstrumentMotionPlanner::mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

void InstrumentMotionPlanner::initializeSharedData()
{
    // systemDataPtr->setZero();
    appDataPtr->setZero();
    // commandDataPtr->setZero();
    systemDataPtr->setSystemState(SystemState::POWER_OFF);
    systemDataPtr->request = 0;
    appDataPtr->setZero();
    commandDataPtr->type = CommandType::NONE;
    forceDataPtr->setZero();
}

int InstrumentMotionPlanner::write_to_drive(double joint_pos[NUM_JOINTS])
{
    for (unsigned int jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        appDataPtr->target_position[jnt_ctr] = joint_pos[jnt_ctr];
    }
    return 0;
}




