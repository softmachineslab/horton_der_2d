/**
 * rodOpenLoopPWMController.h
 * 
 * Definition(s) for the concrete class rodOpenLoopPWMController.
 * Pipes through the output of PWM blocks, based on a given start time. (open loop.)
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodOpenLoopPWMController.h"
// For exception handling
#include <exception>

// Constructor and destructor call parents
rodOpenLoopPWMController::rodOpenLoopPWMController(int numAct, std::vector<double> m_start_times, std::vector<double> m_periods, 
        std::vector<double> m_duty_cycles) : rodController(numAct), start_times(m_start_times)
{
    // validate: we should have passed in the same number of start times, periods, duty cycles, and numAct.
    if( (numAct != start_times.size()) || (numAct != m_periods.size()) || (numAct != m_duty_cycles.size()) ){
        throw std::invalid_argument("Rod controller received wrong size num of actuators or wrong length lists of start times, duty cycles, or periods. Exiting.");
    }
    // Except that, here, we also create the PWMs.
    for(std::size_t i=0; i < numAct; i++){
        // create and add to the vector at the same time.
        // TO-DO: check and confirm that the duty cycles and periods have the correct dimension!!
        pwm.push_back(make_shared<pwmPeripheral>(m_periods[i], m_duty_cycles[i]));
        // All PWMS are off until their start time.
        // pwm[i]->setOnOff(true);
    }
}

rodOpenLoopPWMController::~rodOpenLoopPWMController()
{
}

// override the base class implementation to include PWMs
void rodOpenLoopPWMController::updateTimestep(double dt)
{
    // call the base class to update current_time
    rodController::updateTimestep(dt);
    // We now need to check if we should start the PWMs. This is pretty inefficient, but passable for now.
    for(std::size_t i=0; i < pwm.size(); i++){
        // If this PWM hasn't been turned on yet,
        if( !pwm[i]->isOn() ){
            // see if it needs turning on
            if( current_time >= start_times[i] ){
                // then turn it on
                pwm[i]->setOnOff(1);
            }
        }
        // MANUAL HACK for a cooldown cycle.
        // double leg_cooldown = 1.0;
        // if( current_time >= leg_cooldown){
        //     pwm[4]->setOnOff(0);
        // }
    }
    // now increment the PWMs also.
    // note this should always happen even if the PWMs are off.
    for(auto p_i : pwm){
        p_i->updateTimestep(dt);
    }
}

// Implementation of the controller.
std::vector<int> rodOpenLoopPWMController::getU(shared_ptr<elasticRod> rod_p)
{
    // Result should be same length as number of actuators
    std::vector<int> u(numActuators, 0);
    // Just check if everyone is on or off
    for (size_t i = 0; i < pwm.size(); i++)
    {
        u[i] = pwm[i]->getHighLow();
    }
    
    return u;
}