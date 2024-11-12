#include "cyclicTask.h"
using namespace std;

int main(int argc, char **argv)
{
    // Create an instance of the Master class
    EthercatMaster ecat_master;

    // Run the main functionality of your program
    ecat_master.run();

    return 0; // Indicate successful program execution
}

EthercatMaster::EthercatMaster()
{
    master = ecrt_request_master(0);
    if (!master)
    {
        throw std::runtime_error("Failed to retrieve Master.");
    }

    /** Creates a new process data domain.
     *
     * For process data exchange, at least one process data domain is needed.
     * This method creates a new process data domain and returns a pointer to the
     * new domain object. This object can be used for registering PDOs and
     * exchanging them in cyclic operation.
     *
     * This method allocates memory and should be called in non-realtime context
     * before ecrt_master_activate().
     *
     * \return Pointer to the new domain on success, else NULL.
     */

    domain = ecrt_master_create_domain(master);
    if (!domain)
    {
        throw std::runtime_error("Failed to create process data domain.");
    }

    for (uint16_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
    {
        ec_slave_config_t *sc;

        std::cout<<"configuring joint jnt_ctr : "<<jnt_ctr<<std::endl;
        uint16_t index_ctr = jnt_ctr + 1;

        if (!(sc = ecrt_master_slave_config(master, 0, index_ctr, ingeniaDenalliXcr)))
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return;
        }

        std::cout<<"Assigning PDOs for jnt_ctr : "<<jnt_ctr<<std::endl;
        pdoMapping(sc);

        ec_pdo_entry_reg_t domain_regs[] = {

            {0, index_ctr, ingeniaDenalliXcr, 0x6041, 0, &driveOffset[jnt_ctr].statusword},                // 6041 0 statusword
            {0, index_ctr, ingeniaDenalliXcr, 0x6061, 0, &driveOffset[jnt_ctr].mode_of_operation_display}, // 6061 0 mode_of_operation_display
            {0, index_ctr, ingeniaDenalliXcr, 0x6064, 0, &driveOffset[jnt_ctr].position_actual_value},     // 6064 0 pos_act_val
            {0, index_ctr, ingeniaDenalliXcr, 0x606C, 0, &driveOffset[jnt_ctr].velocity_actual_value},     // 606C 0 vel_act_val
            {0, index_ctr, ingeniaDenalliXcr, 0x6077, 0, &driveOffset[jnt_ctr].torque_actual_value},       // 6077 0 torq_act_val check this
            {0, index_ctr, ingeniaDenalliXcr, 0x603F, 0, &driveOffset[jnt_ctr].error_code},                // 603F 0 digital_input_value
            {0, index_ctr, ingeniaDenalliXcr, 0x6040, 0, &driveOffset[jnt_ctr].controlword},               // 6040 0 control word
            {0, index_ctr, ingeniaDenalliXcr, 0x6060, 0, &driveOffset[jnt_ctr].modes_of_operation},        // 6060 0 mode_of_operation
            {0, index_ctr, ingeniaDenalliXcr, 0x6071, 0, &driveOffset[jnt_ctr].target_torque},             // 6071 0 target torque
            {0, index_ctr, ingeniaDenalliXcr, 0x607A, 0, &driveOffset[jnt_ctr].target_position},           // 607A 0 target position
            {0, index_ctr, ingeniaDenalliXcr, 0x6073, 0, &driveOffset[jnt_ctr].max_current},               // 6073 0 max current
            {0, index_ctr, ingeniaDenalliXcr, 0x6078, 0, &driveOffset[jnt_ctr].current_actual_value},      // 6078 0 current actual value

            {}};

        // ecrt_slave_config_dc(sc, 0x0300, 1000000, 0, 0, 0);

        /** Registers a bunch of PDO entries for a domain.
         *
         * This method has to be called in non-realtime context before
         * ecrt_master_activate().
         *
         * \see ecrt_slave_config_reg_pdo_entry()
         *
         * \attention The registration array has to be terminated with an empty
         *            structure, or one with the \a index field set to zero!
         * \return 0 on success, else non-zero.
         */

        if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs))
        {
            fprintf(stderr, "PDO entry registration failed!\n");
            return;
        }

        ecrt_slave_config_sdo16(sc, 0x6073, 0, 400);

        std::cout<<ecrt_domain_size(domain)<<std::endl;

        // ecrt_slave_config_dc for assignActivate/sync0,1 cycle and shift values for each drive/slave....
    }

    configureSharedMemory();
}

EthercatMaster::~EthercatMaster()
{

    // Release EtherCAT master resources
    if (master)
    {
        ecrt_master_deactivate(master);
        ecrt_release_master(master);
    }

    // Release shared memory
    if (jointDataPtr != nullptr)
    {
        munmap(jointDataPtr, sizeof(JointData));
    }
    if (systemStateDataPtr != nullptr)
    {
        munmap(systemStateDataPtr, sizeof(SystemStateData));
    }
}

void EthercatMaster::run()
{

    // Activate the master
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        perror("Error activating master");
        // Handle the error appropriately based on your application's requirements
    }

    /** Returns the domain's process data.
     *
     * - In kernel context: If external memory was provided with
     * ecrt_domain_external_memory(), the returned pointer will contain the
     * address of that memory. Otherwise it will point to the internally allocated
     * memory. In the latter case, this method may not be called before
     * ecrt_master_activate().
     *
     * - In userspace context: This method has to be called after
     * ecrt_master_activate() to get the mapped domain process data memory.
     *
     * \return Pointer to the process data memory.
     */

    if (!(domainPd = ecrt_domain_data(domain)))
    {
        return;
    }

    // Set CPU affinity for real-time thread
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset); // Set to the desired CPU core

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
    signal(SIGINT, EthercatMaster::signalHandler);

    struct sched_param param = {};
    param.sched_priority = 49;

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    // Set real-time interval for the master
    ecrt_master_set_send_interval(master, 1000);

    // ecrt_slave_config_state(sc, EC_STATE_SAFE_OP);

    printf("Waiting for Safety Controller to get Started ...\n");
    while (!systemStateDataPtr->safety_controller_enabled && !exitFlag)
    {
        // Make a semaphore lock to open, Easy way out is sleep
        sleep(1);
    }


    

    printf("Safety Controller Started \n");

    cyclicTask();

    
}

void EthercatMaster::stackPrefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

void EthercatMaster::pdoMapping(ec_slave_config_t *sc)
{

//     if (ecrt_slave_config_sync_manager(sc, 2, EC_DIR_OUTPUT, EC_WD_ENABLE) != 0) {
//     printf("Error: ecrt_slave_config_sync_manager failed\n");
// }

// ecrt_slave_config_pdo_assign_clear(sc, 2);

// if (ecrt_slave_config_pdo_assign_add(sc, 2, 0x1600) != 0) {
//     printf("Error: ecrt_slave_config_pdo_assign_add failed for PDO 0x1600\n");
// }

// if (ecrt_slave_config_pdo_assign_add(sc, 2, 0x1601) != 0) {
//     printf("Error: ecrt_slave_config_pdo_assign_add failed for PDO 0x1601\n");
// }

// if (ecrt_slave_config_pdo_assign_add(sc, 2, 0x1602) != 0) {
//     printf("Error: ecrt_slave_config_pdo_assign_add failed for PDO 0x1602\n");
// }

// ecrt_slave_config_pdo_mapping_clear(sc, 0x1600);
// ecrt_slave_config_pdo_mapping_clear(sc, 0x1601);
// ecrt_slave_config_pdo_mapping_clear(sc, 0x1602);

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6040, 0, 16) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6040\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6060, 0, 8) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6060\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6071, 0, 16) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6071\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x607A, 0, 32) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x607A\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x60FF, 0, 32) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x60FF\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x60B2, 0, 32) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x60B2\n");
// }

// // if (ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x6073, 0, 16) != 0) {
// //     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6073\n");
// // }

// /* Define TxPdo */

// if (ecrt_slave_config_sync_manager(sc, 3, EC_DIR_INPUT, EC_WD_ENABLE) != 0) {
//     printf("Error: ecrt_slave_config_sync_manager failed\n");
// }

// ecrt_slave_config_pdo_assign_clear(sc, 3);

// if (ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A00) != 0) {
//     printf("Error: ecrt_slave_config_pdo_assign_add failed for PDO 0x1A00\n");
// }

// if (ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A01) != 0) {
//     printf("Error: ecrt_slave_config_pdo_assign_add failed for PDO 0x1A01\n");
// }

// if (ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A02) != 0) {
//     printf("Error: ecrt_slave_config_pdo_assign_add failed for PDO 0x1A02\n");
// }

// ecrt_slave_config_pdo_mapping_clear(sc, 0x1A00);
// ecrt_slave_config_pdo_mapping_clear(sc, 0x1A01);
// ecrt_slave_config_pdo_mapping_clear(sc, 0x1A02);


// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6041, 0, 16) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6041\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6061, 0, 8) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6061\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6064, 0, 32) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6064\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x606C, 0, 32) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x606C\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6077, 0, 16) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6077\n");
// }

// // if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6078, 0, 16) != 0) {
// //     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x6078\n");
// // }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x2600, 0, 32) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x2600\n");
// }

// if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x603F, 0, 16) != 0) {
//     printf("Error: ecrt_slave_config_pdo_mapping_add failed for mapping 0x603F\n");
// }

    /* Define RxPdo */
    ecrt_slave_config_sync_manager(sc, 2, EC_DIR_OUTPUT, EC_WD_ENABLE);

    ecrt_slave_config_pdo_assign_clear(sc, 2);

    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1600);
    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1601);
    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1602);

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1600);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1601);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1602);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6040, 0, 16); /* 0x6040:0/16bits, control word */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6060, 0, 8);  /* 0x6060:0/8bits, mode_of_operation */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6071, 0, 16); /* 0x6071:0/16bits, target torque */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x607A, 0, 32); /* 0x607a:0/32bits, target position */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x60FF, 0, 32); /* 0x60FF:0/32bits, target velocity */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6073, 0, 16); /* 0x6073:0/16bits, max current */

    /* Define TxPdo */

    ecrt_slave_config_sync_manager(sc, 3, EC_DIR_INPUT, EC_WD_ENABLE);

    ecrt_slave_config_pdo_assign_clear(sc, 3);

    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A00);
    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A01);
    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A02);

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A00);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A01);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A02);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6041, 0, 16); /* 0x6041:0/16bits, Statusword */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6061, 0, 8);  /* 0x6061:0/8bits, Modes of operation display */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6064, 0, 32); /* 0x6064:0/32bits, Position Actual Value */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x606C, 0, 32); /* 0x606C:0/32bits, velocity_actual_value */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6077, 0, 16); /* 0x6077:0/16bits, Torque Actual Value */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6078, 0, 16); /* 0x6077:0/16bits, Torque Actual Value */

    // ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x2600, 0, 32); /* 0x60FD:0/32bits, Digital Inputs */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x603F, 0, 16); /* 0x603F:0/16bits, Error Code */

}

void EthercatMaster::signalHandler(int signum)
{

    if (signum == SIGINT)
    {
        std::cout << "Signal received: " << signum << std::endl;
        exitFlag = 1; // Set the flag to indicate the signal was received
    }

}

void EthercatMaster::configureSharedMemory()
{
    int shm_fd_jointData;
    int shm_fd_systemStateData;

    createSharedMemory(shm_fd_jointData, "JointData", sizeof(JointData));
    createSharedMemory(shm_fd_systemStateData, "SystemStateData", sizeof(SystemStateData));

    mapSharedMemory((void *&)jointDataPtr, shm_fd_jointData, sizeof(JointData));
    mapSharedMemory((void *&)systemStateDataPtr, shm_fd_systemStateData, sizeof(SystemStateData));

    initializeSharedData();
}

void EthercatMaster::createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void EthercatMaster::mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

void EthercatMaster::initializeSharedData()
{
    jointDataPtr->setZero();
    systemStateDataPtr->setZero();
}

void EthercatMaster::checkDomainState()
{
    // cout << "check_domain_state" << endl;
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds); // to do - do for all domains

    if (ds.working_counter != domainState.working_counter)
    {
        // printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domainState.wc_state)
    {
        // printf("Domain1: State %u.\n", ds.wc_state);
    }

    domainState = ds;
}

void EthercatMaster::checkMasterState()
{
    // cout << "check_master_state" << endl;
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != masterState.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != masterState.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != masterState.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    masterState = ms;
}




// #include "cyclicTask.h"
// // #include <iostream>
// // #include <stdexcept>
// // #include <cstring>
// // #include <csignal>
// // #include <sched.h>
// // #include <sys/mman.h>
// // #include <unistd.h>

// #define EC_STATE_INIT        0x01
// #define EC_STATE_PRE_OP      0x02
// #define EC_STATE_BOOT        0x03
// #define EC_STATE_SAFE_OP     0x04
// #define EC_STATE_OPERATIONAL 0x08

// using namespace std;

// int main(int argc, char **argv)
// {
//     try
//     {
//         // Create an instance of the Master class and run the EtherCAT process
//         EthercatMaster ecat_master;
//         ecat_master.run();
//     }
//     catch (const std::runtime_error &e)
//     {
//         cerr << "Error: " << e.what() << endl;
//         return 1;  // Indicate failure
//     }

//     return 0; // Indicate successful program execution
// }

// EthercatMaster::EthercatMaster()
// {
//     // Request EtherCAT master
//     master = ecrt_request_master(0);
//     if (!master)
//     {
//         throw std::runtime_error("Failed to retrieve Master.");
//     }

//     // Create domain for process data exchange
//     domain = ecrt_master_create_domain(master);
//     if (!domain)
//     {
//         throw std::runtime_error("Failed to create process data domain.");
//     }

//     // Configure all EtherCAT slaves (joints)
//     for (uint16_t jnt_ctr = 0; jnt_ctr < NUM_JOINTS; jnt_ctr++)
//     {
//         configureSlave(jnt_ctr);
//     }

//     // Configure shared memory for communication
//     configureSharedMemory();
// }

// void EthercatMaster::configureSlave(uint16_t joint_index)
// {
//     ec_slave_config_t *sc;

//     uint16_t index_ctr = joint_index + 1; // Joint index starts from 1

//     // Retrieve the slave configuration
//     sc = ecrt_master_slave_config(master, 0, index_ctr, ingeniaDenalliXcr);
//     if (!sc)
//     {
//         throw std::runtime_error("Failed to get slave configuration for joint " + to_string(joint_index));
//     }

//     // Configure PDO mappings for the joint
//     configurePDO(sc, joint_index);

//     // Register PDO entries to domain
//     registerPDOEntries(sc, joint_index);
// }

// void EthercatMaster::configurePDO(ec_slave_config_t *sc, uint16_t joint_index)
// {
//     // Clear existing PDO mappings
//     ecrt_slave_config_pdo_mapping_clear(sc, 0x1600);
//     ecrt_slave_config_pdo_mapping_clear(sc, 0x1601);
//     ecrt_slave_config_pdo_mapping_clear(sc, 0x1A00);

//     // Configure RxPDOs (outputs)
//     if (ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6040, 0, 16) ||
//         ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6060, 0, 8) ||
//         ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6071, 0, 16) ||
//         ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x607A, 0, 32))
//     {
//         throw std::runtime_error("Failed to configure RxPDO for joint " + to_string(joint_index));
//     }

//     // Configure TxPDOs (inputs)
//     if (ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6041, 0, 16) ||
//         ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6061, 0, 8) ||
//         ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6064, 0, 32) ||
//         ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x606C, 0, 32))
//     {
//         throw std::runtime_error("Failed to configure TxPDO for joint " + to_string(joint_index));
//     }
// }

// void EthercatMaster::registerPDOEntries(ec_slave_config_t *sc, uint16_t joint_index)
// {
//     // Define PDO entry registration for the joint
//     ec_pdo_entry_reg_t domain_regs[] = {
//         {0, joint_index + 1, ingeniaDenalliXcr, 0x6041, 0, &driveOffset[joint_index].statusword},        // Statusword
//         {0, joint_index + 1, ingeniaDenalliXcr, 0x6061, 0, &driveOffset[joint_index].mode_of_operation_display}, // Mode of operation display
//         {0, joint_index + 1, ingeniaDenalliXcr, 0x6064, 0, &driveOffset[joint_index].position_actual_value},     // Position actual value
//         {0, joint_index + 1, ingeniaDenalliXcr, 0x6077, 0, &driveOffset[joint_index].torque_actual_value},       // Torque actual value
//         {} // End of list
//     };

//     // Register the PDO entries for the domain
//     if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs))
//     {
//         throw std::runtime_error("Failed to register PDO entries for joint " + to_string(joint_index));
//     }
// }

// void EthercatMaster::run()
// {
//     // Activate the master and get domain data
//     printf("Activating master...\n");
//     if (ecrt_master_activate(master))
//     {
//         throw std::runtime_error("Error activating EtherCAT master.");
//     }

//     if (!(domainPd = ecrt_domain_data(domain)))
//     {
//         throw std::runtime_error("Failed to get domain process data pointer.");
//     }

//     // Set CPU affinity for real-time operations
//     setCPUAffinity(3);

//     // Register signal handler to stop the program gracefully
//     signal(SIGINT, EthercatMaster::signalHandler);

//     // Ensure slaves are in operational state before starting the cyclic task
//     setOperationalState();

//     printf("Waiting for Safety Controller to get started...\n");
//     while (!systemStateDataPtr->safety_controller_enabled && !exitFlag)
//     {
//         sleep(1);
//     }

//     printf("Safety Controller Started.\n");

//     // Start cyclic task
//     cyclicTask();
// }

// void EthercatMaster::setCPUAffinity(int core)
// {
//     cpu_set_t cpuset;
//     CPU_ZERO(&cpuset);
//     CPU_SET(core, &cpuset);

//     if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1)
//     {
//         throw std::runtime_error("Error setting CPU affinity: " + string(strerror(errno)));
//     }

//     // Lock memory to prevent paging
//     if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
//     {
//         fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
//     }

//     stackPrefault();
// }

// void EthercatMaster::stackPrefault()
// {
//     unsigned char dummy[MAX_SAFE_STACK];
//     memset(dummy, 0, MAX_SAFE_STACK);
// }

// void EthercatMaster::signalHandler(int signum)
// {
//     if (signum == SIGINT)
//     {
//         std::cout << "Signal received: " << signum << std::endl;
//         exitFlag = 1; // Signal to exit the loop
//     }
// }

// void EthercatMaster::setOperationalState()
// {
//     printf("Transitioning slaves to Operational state...\n");

//     // Check if the slaves are ready to transition to OPERATIONAL state
//     ec_master_state_t master_state;
//     ecrt_master_state(master, &master_state);

//     if (master_state.al_states != EC_STATE_OPERATIONAL)
//     {
//         throw std::runtime_error("Failed to transition slaves to Operational state. Current state: " + to_string(master_state.al_states));
//     }

//     printf("Slaves successfully transitioned to Operational state.\n");
// }

// void EthercatMaster::configureSharedMemory()
// {
//     int shm_fd_jointData;
//     int shm_fd_systemStateData;

//     // Create shared memory for joint and system state data
//     createSharedMemory(shm_fd_jointData, "JointData", sizeof(JointData));
//     createSharedMemory(shm_fd_systemStateData, "SystemStateData", sizeof(SystemStateData));

//     // Map shared memory to pointers
//     mapSharedMemory((void *&)jointDataPtr, shm_fd_jointData, sizeof(JointData));
//     mapSharedMemory((void *&)systemStateDataPtr, shm_fd_systemStateData, sizeof(SystemStateData));

//     // Initialize shared memory data
//     initializeSharedData();
// }

// void EthercatMaster::createSharedMemory(int &shm_fd, const char *name, int size)
// {
//     shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
//     if (shm_fd == -1)
//     {
//         throw std::runtime_error("Failed to create shared memory object.");
//     }
//     ftruncate(shm_fd, size);
// }

// void EthercatMaster::mapSharedMemory(void *&ptr, int shm_fd, int size)
// {
//     ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
//     if (ptr == MAP_FAILED)
//     {
//         throw std::runtime_error("Failed to map shared memory.");
//     }
// }

// void EthercatMaster::initializeSharedData()
// {
//     jointDataPtr->setZero();
//     systemStateDataPtr->setZero();
// }

// void EthercatMaster::checkDomainState()
// {
//     ec_domain_state_t ds;
//     ecrt_domain_state(domain, &ds);

//     if (ds.working_counter != domainState.working_counter || ds.wc_state != domainState.wc_state)
//     {
//         // Optionally handle domain state changes here
//     }

//     domainState = ds;
// }

// void EthercatMaster::checkMasterState()
// {
//     ec_master_state_t ms;
//     ecrt_master_state(master, &ms);

//     if (ms.slaves_responding != masterState.slaves_responding)
//     {
//         printf("%u slave(s) responding.\n", ms.slaves_responding);
//     }

//     if (ms.al_states != masterState.al_states)
//     {
//         printf("AL state: 0x%02X.\n", ms.al_states);
//     }

//     if (ms.link_up != masterState.link_up)
//     {
//         printf("Link is %s.\n", ms.link_up ? "up" : "down");
//     }

//     masterState = ms;
// }
