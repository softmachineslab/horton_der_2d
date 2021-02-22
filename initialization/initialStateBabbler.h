/**
 * initialStateBabbler.h
 * 
 * Declarations for the class initialStateBabbler.
 * A really basic wrapper for sampling over initial states.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef INITIAL_STATE_BABBLER_H
#define INITIAL_STATE_BABBLER_H

// for randomness
#include <random>
// because we have more than one dimension
#include <vector>
// because memory management
#include <memory>
// easier for vectors to be in Eigen for playing nice with the rest of the DER simulation
#include "../eigenIncludes.h"

class initialStateBabbler
{
    public: 

    /** 
     * An initialStateBabbler returns an initial state for the robot,
     * sampled from a normal (to-do: better choice here!) distribution.
     * HACK: NOW A UNIFORM DISTRIBUTION
     * @param means the mean of the normal distribution for each state. Must be same dimension as stddevs
     * @param stddevs the standard deviations of the dist to sample from. Must be same dimension as means
     */
    initialStateBabbler(std::vector<double> means, std::vector<double> stddevs);
    ~initialStateBabbler();

    /**
     * Get a sample.
     */
    // std::vector<double> getSample();
    VectorXd getSample();

    protected:

    // store the normal distribution objects for each dimension for sampling from repeatedly
    std::default_random_engine gen;
    // std::vector<std::unique_ptr<std::normal_distribution<double>>> dists;
    // HACK: TO-DO, separate class.
    std::vector<std::unique_ptr<std::uniform_real_distribution<double>>> dists;

};

#endif // INITIAL_STATE_BABBLER_H