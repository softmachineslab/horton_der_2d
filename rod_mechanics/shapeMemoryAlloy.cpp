/**
 * shapeMemoryAlloy.cpp
 * 
 * Definitions for the class shapeMemoryAlloy. 
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "shapeMemoryAlloy.h"

// Constructor. See .h file
shapeMemoryAlloy::shapeMemoryAlloy(double kappa_bar_0, double ei_0, double kappa_bar_ratio, double ei_ratio, 
                                   double act_kB_slope, double act_EI_slope, double log_scaling, double log_offset) :
                                   kB0(kappa_bar_0), EI0(ei_0), n1(kappa_bar_ratio), n2(ei_ratio), 
                                   dkBdt(act_kB_slope), dEIdt(act_EI_slope), tau(log_scaling), tbar(log_offset), current_time(0.0), 
                                   last_sw_time(0.0), last_sw_kb(kappa_bar_0), last_sw_EI(ei_0), dt_switch(0.0), kB(kappa_bar_0),
                                   EI(ei_0), is_on(false)
{
    if( verbosity >= 2){
        std::cout << "Constructing a shapeMemoryAlloy with parameters..." << std::endl;
    }
    // ...initialized time (temperature) in the declaration above,
    // but calculate the saturation point here.
    kB_sat = n1 * kappa_bar_0; // less than KB0
    EI_sat = n2 * ei_0; // greater than EI0
}

shapeMemoryAlloy::~shapeMemoryAlloy()
{
}

void shapeMemoryAlloy::updateTimestep(double dt)
{
    current_time += dt;
    // The time difference since the last switch. Needed in both on or off.
    dt_switch = current_time - last_sw_time;
    // Then, calculate based on state
    if( is_on ){
        setKappaBarOn();
        setEIOn();
    }
    else
    {
        setKappaBarOff();
        setEIOff();
    }
}

void shapeMemoryAlloy::setActState(bool on_off)
{
    // Is different than before?
    if( on_off != is_on ) {
        // Update the switching times and other constants
        last_sw_time = current_time;
        last_sw_kb = kB;
        last_sw_EI = EI;
        // and turn on or off
        is_on = on_off;
    }
}

double shapeMemoryAlloy::getKappaBar()
{
    // Since this is recalculated at each updateTime now, just return.
    return kB;
}

double shapeMemoryAlloy::getEI()
{
    // Since this is recalculated at each updateTime now, just return.
    return EI;
}

// to-do: probably cleaner if these are singletons, and the local variables are
// passed in as parameters. Maybe less efficient though?

void shapeMemoryAlloy::setKappaBarOff()
{
    // Logistic curve. From the Huang et al. NatComm paper, adjusting for the constants used here,
    // and from the starting point 
    // L / (1 + exp(-tau(dt - tbar))) + offset
    // ...where L and offset are according to last kappa from which we're now cooling down.
    kB = (kB0 - last_sw_kb) / (1 + exp( -tau*( dt_switch - tbar ) )) + last_sw_kb;
}

void shapeMemoryAlloy::setEIOff()
{
    // also a logistic curve. Similar to kappa bar, but decreases instead of increases (i.e. L < last_sw_EI, since n2 > 1).
    // L / (1 + exp(-tau(dt - tbar))) + offset
    EI = (EI0 - last_sw_EI) / (1 + exp( -tau*(dt_switch - tbar ) )) + last_sw_EI;
}

void shapeMemoryAlloy::setKappaBarOn()
{
    // Need to check if saturated.
    // Model is linear, with offset of wherever the actuator was last switched on.
    double kB_next = dkBdt * dt_switch + last_sw_kb;
    if( kB_next >= kB_sat ){
        kB = kB_next;
    }
    else {
        kB = kB_sat;
    }
}

void shapeMemoryAlloy::setEIOn()
{
    // as with kappa bar, check saturation.
    double EI_next = dEIdt * dt_switch + last_sw_EI;
    if( EI_next <= EI_sat ){
        EI = EI_next;
    }
    else {
        EI = EI_sat;
    }
}