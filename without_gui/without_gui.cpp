#include "without_gui.h"
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>

int main()
{
    configureSharedMemory();
    sleep(2);

    systemDataPtr->request = 1;

    std::cout<<"insuide without gui"<<std::endl;

    while (!(appDataPtr->operation_enable_status)){
        sleep(1);
    }

    commandDataPtr->setHandControl();

}

void configureSharedMemory()
{
    int shm_fd_systemData;
    int shm_fd_appData;
    int shm_fd_commandData;

    createSharedMemory(shm_fd_systemData, "SystemData", sizeof(SystemData));
    createSharedMemory(shm_fd_appData, "AppData", sizeof(AppData));
    createSharedMemory(shm_fd_commandData, "CommandData", sizeof(CommandData));

    mapSharedMemory((void *&)systemDataPtr, shm_fd_systemData, sizeof(SystemData));
    mapSharedMemory((void *&)appDataPtr, shm_fd_appData, sizeof(AppData));
    mapSharedMemory((void *&)commandDataPtr, shm_fd_commandData, sizeof(CommandData));

    initializeSharedData();
}

void createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

void initializeSharedData()
{
    // systemDataPtr->setZero();
    // commandDataPtr->setZero();
    systemDataPtr->setSystemState(SystemState::POWER_OFF);
    systemDataPtr->request = 0;
    appDataPtr->setZero();
    commandDataPtr->type = CommandType::NONE;
}
