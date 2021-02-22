/**
 * rodCOMPWMController.h
 * 
 * Declarations for the concrete class rodCOMPWMController.
 * Pipes through the output of PWM blocks, based on robot center of mass.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_COM_PWM_CONTROLLER_H
#define ROD_COM_PWM_CONTROLLER_H

// everything comes from parent
#include "rodController.h"

// except the PWM blocks, which are particular to this controller
#include "pwmPeripheral.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

class rodCOMPWMController : public rodController
{

    public:

    /**
     * Constructor and destructor call parents.
     */
    rodCOMPWMController(int numAct);
    ~rodCOMPWMController();

    /**
     * Calculate the desired control input, u(x).
     * Becomes "1" at some point as a function of rod center of mass.
     * @param rod_p pointer to an elasticRod from which state feedback will occur.
     */
    std::vector<int> getU(shared_ptr<elasticRod> rod_p);

    // redefine the base class' timestepping function to also include the PWMs.
    void updateTimestep(double dt);

    private:

    // We will need to keep track of a set of PWM peripherals.
    std::vector<shared_ptr<pwmPeripheral>> pwm;

};

#endif //ROD_COM_PWM_CONTROLLER_H