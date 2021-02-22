/**
 * rodOpenLoopPWMController.h
 * 
 * Declarations for the concrete class rodOpenLoopPWMController.
 * Pipes through the output of PWM blocks, based on a given start time. (open loop.)
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_OPEN_LOOP_PWM_CONTROLLER_H
#define ROD_OPEN_LOOP_PWM_CONTROLLER_H

// everything comes from parent
#include "rodController.h"

// except the PWM blocks, which are particular to this controller
#include "pwmPeripheral.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

class rodOpenLoopPWMController : public rodController
{

    public:

    /**
     * Constructor and destructor call parents, but now also pass in:
     * @param m_start_times a vector of starting times for all actuators. Must be dimension of numAct.
     * @param m_periods a vector of the periods for all actuators. Must be dimension of numAct.
     * @param m_duty_cycles a vector of the duty cycles for all actuators. Must be dimension of numAct.
     */
    rodOpenLoopPWMController(int numAct, std::vector<double> m_start_times, std::vector<double> m_periods, 
        std::vector<double> m_duty_cycles);
    ~rodOpenLoopPWMController();

    /**
     * Calculate the desired control input, u(x).
     * Now open loop so rod unused.
     * @param rod_p pointer to an elasticRod from which state feedback will occur.
     */
    std::vector<int> getU(shared_ptr<elasticRod> rod_p);

    // redefine the base class' timestepping function to also include the PWMs.
    void updateTimestep(double dt);

    private:

    // We will need to keep track of a set of PWM peripherals.
    std::vector<shared_ptr<pwmPeripheral>> pwm;
    // the controller will need to know when to turn the PWMs on.
    std::vector<double> start_times;

};

#endif //ROD_OPEN_LOOP_PWM_CONTROLLER_H