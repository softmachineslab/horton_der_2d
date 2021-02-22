/**
 * shapeMemoryAlloy.h
 * 
 * Declarations for the class shapeMemoryAlloy.
 * A shape memory alloy stores the amount of time it has been actuated,
 * and provides functions that return the intrinsic curvature (kappa bar) 
 * and bending stiffness (EI) of the actuator as a function of that time.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef SHAPE_MEMORY_ALLOY_H
#define SHAPE_MEMORY_ALLOY_H

// Some constants used throughout this program
#include "../global_const.h"
// we do some math here
#include "math.h"

// SMAs function independently of the rod, so we actually don't need to include anything
// except i/o
#include <iostream>

class shapeMemoryAlloy
{

    public:

    /**
     * Constructor for a shapeMemoryAlloy actuator.
     * Takes in the parameters for the curve fit from Huang et al.
     * @param kappa_bar_0 Unactuated intrinsic curvature.
     * @param ei_0 Unactuated bending stiffness (EI)
     * @param kappa_bar_ratio Maximum ratio for the kappaBar increase, i.e., n1.
     * @param ei_ratio Maximum ratio for the elastic modulus decrease, i.e., n2.
     * @param act_kB_slope The slope of the linear decrease for the heating-up region, kappa. (Defines t0 from the paper.)
     * @param act_EI_slope The slope of the linear increase for the heating-up region, bending stiffness. (also via t0.)
     * @param log_scaling Scaling constant for the logistic function applied during the cooling-down region, i.e., tau.
     * @param log_offset Scaling offset for the logistic function applied during the cooling-down region, i.e., t_bar.
     */
    shapeMemoryAlloy(double kappa_bar_0, double ei_0, double kappa_bar_ratio, double ei_ratio, 
                     double act_kB_slope, double act_EI_slope, double log_scaling, double log_offset);
    ~shapeMemoryAlloy();

    /**
     * Return the intrinsic curvature for this SMA.
     * @return kappaBar, the intrinsic curvature, according to the curve fit and amount of time actuated.
     */
    double getKappaBar();

    /**
     * Return the bending stiffness for the actuator with this SMA.
     * @return EI, the bending stiffness, according to the curve fit and amount of time actuated.
     */
    double getEI();

    /**
     * Update the timestep for the SMA. Recalculates kappa bar, E at each timestep.
     * @param dt time increment from world
     */
    void updateTimestep(double dt);

    /**
     * Turn on or off this SMA. 
     * Switches between the two functions for getting kappa bar and E, i.e., heating/cooling.
     * @param on_off Setting for the SMA (heating = 1, cooling = 0)
     */
    void setActState(bool on_off);

    protected:

    /**
     * Get the actuator's kappa bar (intrinsic curvature) if cooling down or heating up
     * Stores the results in local variable, no need to return.
     */
    virtual void setKappaBarOff();
    virtual void setKappaBarOn();
    // same for the bending stiffness
    virtual void setEIOff();
    virtual void setEIOn();

    // Local variables for the heating and cooling models.
    // These correspond to parameters in the constructor.
    double kB0;
    double EI0;
    double n1;
    double n2;
    double dkBdt;
    double dEIdt;
    double tau;
    double tbar;

    /**
     * Three variables used to determine the SMA's location in its heating/cooling curves.
     * current_time = the total simulation time from start of the program
     * last_sw_time = time (vs. total time) when the SMA was last switched from on to off or vice verse
     * last_sw_kB = the kappa bar value at last_sw_time. 
     * last_sw_EI = the bending stiffness value at last_sw_time.
     */
    double current_time;
    double last_sw_time;
    double last_sw_kb;
    double last_sw_EI;

    // ...and a helper that we'll use every time step, the difference between current and last.
    double dt_switch;

    // Minimum kappa bar, maximum bending stiffness
    // This is the staturation point for heatup.
    // Can be calculated directly from kB0 and n1, or equivalently EI0 and n2
    double kB_sat;
    double EI_sat;

    /**
     * Store local copies of kappa bar and elastic modulus. These can be returned arbitrarily.
     */
    double kB;
    double EI;
    
    // The SMA is always either heating up or cooling down.
    double is_on;

};

#endif // SHAPE_MEMORY_ALLOY_H