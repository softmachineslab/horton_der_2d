/**
 * rodCOMSingleShotController.h
 * 
 * Definition(s) for the concrete class rodCOMSingleShotController.
 * Turns on an actuator once the CoM of the robot reaches a certain position.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodCOMSingleShotController.h"

// Constructor and destructor just call parents
rodCOMSingleShotController::rodCOMSingleShotController(int numAct) : rodController(numAct)
{
}

rodCOMSingleShotController::~rodCOMSingleShotController()
{
}

// Implementation of the controller.
std::vector<int> rodCOMSingleShotController::getU(shared_ptr<elasticRod> rod_p)
{
    Vector2d com = rod_p->getCOM();
    if( verbosity >= 2) {
        std::cout << "Rod CoM: (" << com[0] << ", " << com[1] << ")" << std::endl; 
    }
    // std::vector<int> u = {0, 0};
    // Result should be same length as number of actuators
    std::vector<int> u(numActuators, 0);
    // Single shot: first time robot moves past some point, flick the controller on.
    // Negative is to the left.
    if( com[0] > 0.01) {
        // u = {1,1};
        // make a new vector of ones and reassign to u.
        // std::vector<int> uOn(numActuators, 1);
        // u = uOn;
        u[0] = 1;
        u[1] = 1;
        u[2] = 1;
        u[3] = 1;
        u[4] = 1;
        u[5] = 1;
        u[6] = 1;
        // u[7] = 1;

        if( verbosity >= 2){
            std::cout << "Rod Controller ON!" << std::endl;
        }
    }
    // hack: if a bit further, turn back off.
    if( com[0] > 0.017) {
        u[0] = 0;
        u[1] = 0;
        u[2] = 0;
        u[3] = 0;
        u[4] = 0;
        u[5] = 0;
        u[6] = 0;
    }
    return u;
}