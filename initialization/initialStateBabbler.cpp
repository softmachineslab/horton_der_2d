/**
 * initialStateBabbler.cpp
 * 
 * Definitions for the class initialStateBabbler.
 * A really basic wrapper for sampling over initial states.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

// all includes in this' header
#include "initialStateBabbler.h"

initialStateBabbler::initialStateBabbler(std::vector<double> means, std::vector<double> stddevs)
{
    // Assuming the means and stddevs are ordered according to their respective state
    for(size_t i=0; i < means.size(); i++){
        // smart pointers here
        // dists.push_back(std::make_unique<std::normal_distribution<double>>(means[i], stddevs[i]));
        // HACK: TO-DO: separate class
        dists.push_back(std::make_unique<std::uniform_real_distribution<double>>(means[i], stddevs[i]));
    }
}

// constructor does nothing, smart pointers take care of the normal_distributions
initialStateBabbler::~initialStateBabbler()
{
}

// return a sample, presumably of the correct size
// std::vector<double> initialStateBabbler::getSample()
VectorXd initialStateBabbler::getSample()
{
    // iteratively. Probably faster if vector BUT this function is called rarely so who cares
    // std::vector<double> samples;
    VectorXd samples = VectorXd::Zero(dists.size());
    for(size_t i=0; i < dists.size(); i++){
        // This is the least ugly way to call the () operator on the normal_distribution class
        // random engine is declared (constructed??) in header
        samples[i] = (*dists[i])(gen);
    }
    return samples;
}