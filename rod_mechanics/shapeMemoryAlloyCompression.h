/**
 * shapeMemoryAlloyCompression.h
 * 
 * Declarations for the class shapeMemoryAlloyCompression.
 * A shape memory alloy stores the amount of time it has been actuated,
 * and provides functions that return the intrinsic curvature (kappa bar) 
 * and bending stiffness (EI) of the actuator as a function of that time.
 * The "compression" SMA is the spring that shortens, not the wire that straightens out.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef SHAPE_MEMORY_ALLOY_COMPRESSION_H
#define SHAPE_MEMORY_ALLOY_COMPRESSION_H

// Some constants used throughout this program
#include "../global_const.h"
// we do some math here
#include "math.h"

// SMAs function independently of the rod, so we actually don't need to include anything except the base class
#include "shapeMemoryAlloy.h"
// except i/o
#include <iostream>

class shapeMemoryAlloyCompression : public shapeMemoryAlloy
{

    public:

    /**
     * Constructor for a shapeMemoryAlloyCompression actuator.
     * Takes in the parameters for the curve fit from Huang et al.
     * @param kappa_bar_0 Unactuated intrinsic curvature.
     * @param ei_0 Unactuated bending stiffness (EI)
     * @param kappa_bar_sat Saturation kappaBar. We're doing compression now. This is a hack which stinks, TO-DO refactor the base SMA class
     * @param ei_ratio Maximum ratio for the elastic modulus decrease, i.e., n2.
     * @param act_kB_slope The slope of the linear increase for the heating-up region, kappa. (Defines t0 from the paper.)
     * @param act_EI_slope The slope of the linear increase for the heating-up region, bending stiffness. (also via t0.)
     * @param log_scaling Scaling constant for the logistic function applied during the cooling-down region, i.e., tau.
     * @param log_offset Scaling offset for the logistic function applied during the cooling-down region, i.e., t_bar.
     */
    shapeMemoryAlloyCompression(double kappa_bar_0, double ei_0, double kappa_bar_sat, double ei_ratio, 
                     double act_kB_slope, double act_EI_slope, double log_scaling, double log_offset);
    ~shapeMemoryAlloyCompression();

    // The base class handles all public get/set.

    protected:

    /**
     * Get the actuator's kappa bar (intrinsic curvature) if cooling down or heating up
     * Stores the results in local variable, no need to return.
     */
    // For compression, looks like only ON changes. The logistic cooldown works in either direction (compression/extension.)
    virtual void setKappaBarOn();
    // same for the bending stiffness
    virtual void setEIOn();

    // The base class has all its locals, except we need one here to enable backwards (negative kappaBar) motions.
    bool backwards = false;
};

#endif // SHAPE_MEMORY_ALLOY_COMPRESSION_H