/**
 * rodMyFirstController.h
 * 
 * This is a controller based on the centre of mass point. 
 * This controller is based on rodCOMPWMController
 * 
 * @author Zhewei Liu
 */
#ifndef ROD_MY_FIRST_CONTROLLER_H
#define ROD_MY_FIRST_CONTROLLER_H

// everything comes from parent
#include "rodController.h"

// except the PWM blocks, which are particular to this controller
#include "pwmPeripheral.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

class rodMyFirstController : public rodController
{

    public:

    /**
     * Constructor and destructor call parents.
     */
    rodMyFirstController(int numAct);
    ~rodMyFirstController();

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
    // Variables to store the time period
    double lastTime;

};

#endif //ROD_COM_PWM_CONTROLLER_H