#pragma once

#include "instrument_jog.h"
#include "sterile_engagement.h"

void InstrumentMotionPlanner::cyclicTask()
{
    // struct period_info pinfo;

    // periodic_task_init(&pinfo);

    while (!exitFlag)
    {
        do_rt_task();
        // wait_rest_of_period(&pinfo);
    }
}

void InstrumentMotionPlanner::inc_period(struct period_info *pinfo)
{

    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

void InstrumentMotionPlanner::periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = 1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void InstrumentMotionPlanner::do_rt_task()
{

    switch (systemDataPtr->getSystemState())
    {
    case SystemState::POWER_OFF:
        // std::cout<<"sys state powered on \n";
        if (systemDataPtr->request == 1)
        {
            appDataPtr->initialize_system = true;

            if (appDataPtr->drive_initialized)
            {
                if (appDataPtr->trigger_error)
                {
                    systemDataPtr->setSystemState(SystemState::ERROR);
                }
                else
                {
                    systemDataPtr->setSystemState(SystemState::READY);
                }
            }
        }
        break;
    case SystemState::READY:
        // std::cout<<"sys state ready \n";
        appDataPtr->switch_to_operation = true;
        if (appDataPtr->operation_enable_status)
        {
            if (commandDataPtr->type != CommandType::NONE)
            {
                systemDataPtr->setSystemState(SystemState::IN_EXECUTION);
            }
        }
        break;
    case SystemState::IN_EXECUTION:
        // std::cout<<"sys state powered in execution \n";
        if (commandDataPtr->type == CommandType::JOG)
        {
            Jog();
        }
        else if (commandDataPtr->type == CommandType::HAND_CONTROL)
        {
            sterile_engagement();
        }
        else
        {
        }
        systemDataPtr->setSystemState(SystemState::READY);
        break;
    case SystemState::RECOVERY:
        // std::cout<<"sys state in recovery \n";
        break;
    case SystemState::ERROR:
        // std::cout<<"sys state error \n";
        appDataPtr->trigger_error = false;
        if (appDataPtr->reset_error)
        {
            systemDataPtr->resetError();
        }
        break;
    default:
        break;
    }

    usleep(1000);

}

void InstrumentMotionPlanner::wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}
