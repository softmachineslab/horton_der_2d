/**
 * shapeMemoryAlloyCompression.cpp
 * 
 * Definitions for the class shapeMemoryAlloyCompression. 
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "shapeMemoryAlloyCompression.h"

// Constructor. See .h file
shapeMemoryAlloyCompression::shapeMemoryAlloyCompression(double kappa_bar_0, double ei_0, double kappa_bar_sat, double ei_ratio, 
                                   double act_kB_slope, double act_EI_slope, double log_scaling, double log_offset) : 
                                   shapeMemoryAlloy(kappa_bar_0, ei_0, kappa_bar_sat, ei_ratio, act_kB_slope, act_EI_slope,
                                        log_scaling, log_offset)
{
    if( verbosity >= 2){
        std::cout << "Constructing a shapeMemoryAlloyCompression with parameters..." << std::endl;
    }
    // Override the base class' definition of kB_sat, now we're increasing not decreasing and specify max directly
    kB_sat = kappa_bar_sat;
    // Check: is this actuator bending backwards? That would be the case if...
    if(kB_sat < kappa_bar_0)
    { 
        backwards=true; 
    }    
}

shapeMemoryAlloyCompression::~shapeMemoryAlloyCompression()
{
}

void shapeMemoryAlloyCompression::setKappaBarOn()
{
    // Need to check if saturated.
    // Model is linear, with offset of wherever the actuator was last switched on.
    double kB_next = dkBdt * dt_switch + last_sw_kb;
    // for compression SMA, this will depend on actuation direction.
    // If forward, less than saturation.
    if(!backwards)
    {
        if( kB_next <= kB_sat ){
            kB = kB_next;
        }
        else {
            kB = kB_sat;
        }
    }
    else
    {
        // same as the extension SMA: curvature becomes more negative with time.
        if( kB_next >= kB_sat ){
            kB = kB_next;
        }
        else {
            kB = kB_sat;
        }
    }
}

void shapeMemoryAlloyCompression::setEIOn()
{
    // as with kappa bar, check saturation.
    double EI_next = dEIdt * dt_switch + last_sw_EI;
    // we'll let EI increase during actuation here also... TO-DO check with Sean about reasoning for increased EI?
    if( EI_next <= EI_sat ){
        EI = EI_next;
    }
    else {
        EI = EI_sat;
    }
}