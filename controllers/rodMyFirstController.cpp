/**
 * rodMyFirstController.h
 * 
 * This is a controller based on the centre of mass point. 
 * This controller is based on rodCOMPWMController
 * 
 * @author Zhewei Liu
 */
#include "rodMyFirstController.h"

// Constructor and destructor call parents
rodMyFirstController::rodMyFirstController(int numAct) : rodController(numAct)
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
    lastTime = -0.1;
}

rodMyFirstController::~rodMyFirstController()
{
}

// override the base class implementation to include PWMs
void rodMyFirstController::updateTimestep(double dt)
{
    // call the base class to update current_time
    rodController::updateTimestep(dt);
    // now the PWMs also.
    for(auto p_i : pwm){
        p_i->updateTimestep(dt);
    }
}

// Implementation of the controller.
std::vector<int> rodMyFirstController::getU(shared_ptr<elasticRod> rod_p)
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
    if( com[0] > 0.0) {
        // msec
        next_per = 0.5;
        // duty cycle
        next_dut = 0.004;
        if (current_time < 1.59) {
            pwm[1]->setPeriod(next_per);
            pwm[1]->setDutyCycle(next_dut);
            u[1] = pwm[1]->getHighLow();
            pwm[3]->setPeriod(next_per);
            pwm[3]->setDutyCycle(next_dut);
            u[3] = pwm[3]->getHighLow();
            pwm[5]->setPeriod(next_per);
            pwm[5]->setDutyCycle(next_dut);
            u[5] = pwm[5]->getHighLow();
        } else if (current_time < 3) {
            pwm[0]->setPeriod(next_per);
            pwm[0]->setDutyCycle(next_dut);
            u[0] = pwm[0]->getHighLow();
            pwm[4]->setPeriod(next_per);
            pwm[4]->setDutyCycle(next_dut);
            u[4] = pwm[4]->getHighLow();
            pwm[2]->setPeriod(next_per);
            pwm[2]->setDutyCycle(next_dut);
            u[2] = pwm[2]->getHighLow();
            // pwm[8]->setPeriod(next_per);
            // pwm[8]->setDutyCycle(next_dut);
            // u[8] = pwm[8]->getHighLow();
        } 
    }
    
    
    return u;
}