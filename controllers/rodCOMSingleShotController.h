/**
 * rodCOMSingleShotController.h
 * 
 * Declarations for the concrete class rodCOMSingleShotController.
 * Turns on an actuator once the CoM of the robot reaches a certain position.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_COM_SINGLE_SHOT_CONTROLLER_H
#define ROD_COM_SINGLE_SHOT_CONTROLLER_H

// everything comes from parent
#include "rodController.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

class rodCOMSingleShotController : public rodController
{

    public:

    /**
     * Constructor and destructor just call parents.
     */
    rodCOMSingleShotController(int numAct);
    ~rodCOMSingleShotController();

    /**
     * Calculate the desired control input, u(x).
     * Becomes "1" at some point as a function of rod center of mass.
     * @param rod_p pointer to an elasticRod from which state feedback will occur.
     */
    std::vector<int> getU(shared_ptr<elasticRod> rod_p);

};

#endif //ROD_COM_SINGLE_SHOT_CONTROLLER_H