/**
 * rodCOMPWMController.h
 * 
 * Definition(s) for the concrete class rodCOMPWMController.
 * Pipes through the output of PWM blocks, based on robot center of mass.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodCOMPWMController.h"

// Constructor and destructor call parents
rodCOMPWMController::rodCOMPWMController(int numAct) : rodController(numAct)
{
    // Except that, here, we also create the PWMs.
    for(std::size_t i=0; i < numAct; i++){
        // create and add to the vector at the same time.
        // assume that we don't specify any period or duty cycle yet.
        // pwm.push_back(new pwmPeripheral(0.0, 0.0));
        pwm.push_back(make_shared<pwmPeripheral>(0.0, 0.0));
        // turn it on. With 0.0, 0.0, the pwm should do nothing.
        pwm[i]->setOnOff(true);
    }
}

rodCOMPWMController::~rodCOMPWMController()
{
}

// override the base class implementation to include PWMs
void rodCOMPWMController::updateTimestep(double dt)
{
    // call the base class to update current_time
    rodController::updateTimestep(dt);
    // now the PWMs also.
    for(auto p_i : pwm){
        p_i->updateTimestep(dt);
    }
}

// Implementation of the controller.
std::vector<int> rodCOMPWMController::getU(shared_ptr<elasticRod> rod_p)
{
    Vector2d com = rod_p->getCOM();
    if( verbosity >= 2) {
        std::cout << "Rod CoM: (" << com[0] << ", " << com[1] << ")" << std::endl; 
    }
    // Result should be same length as number of actuators
    std::vector<int> u(numActuators, 0);
    // Start up the PWM arbitrarily.
    double next_per = 0.0;
    double next_dut = 0.0;
    // Negative is to the left. in meters.
    if( com[0] > 0.01) {
        // msec
        next_per = 0.2;
        // duty cycle
        next_dut = 0.004;

        if( verbosity >= 2){
            std::cout << "Rod PWM Controller ON!" << std::endl;
        }
    }
    // example: if a bit further...
    if( com[0] > 0.025) {
        next_per = 0.2;
        next_dut = 0.04;
    }

    // as an example, do the same to all actuators.
    for (size_t i = 0; i < pwm.size(); i++)
    {
        if (i == 1) {
            pwm[i]->setPeriod(next_per);
            pwm[i]->setDutyCycle(next_dut);
            u[i] = pwm[i]->getHighLow();
        }
    }
    
    return u;
}