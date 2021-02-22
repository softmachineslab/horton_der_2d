/**
 * rodKappaLogger.cpp
 * 
 * Concrete data logging class that records the curvatures for each rod segment.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodKappaLogger.h"

// Constructor: the parent does everything for us
rodKappaLogger::rodKappaLogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, shared_ptr<world> w, int per) : 
                            worldLogger(fileNamePrefix, logfile_base, df, w, per)
{
    if(verbosity >= 2) {
        std::cout << "Constructing a rodKappaLogger..." << std::endl;
    }
}

// destructor does nothing
rodKappaLogger::~rodKappaLogger()
{
}

// Redefine the parent's aux setup function.
void rodKappaLogger::setupHelper()
{
    if(verbosity >= 1) {
        std::cout << "Setting up a rodKappaLogger..." << std::endl;
    }
    // Collect a set of vertex numbers to report their properties. 
    for(int i=0; i < m_world_p->getNumLimbs(); i++){
        // Store the vertex number for a representative example
        // ASSUMES that the rod and world are in agreement about number of limbs!
        vert_nums.push_back(m_world_p->getRodP()->getAnyVertexNumForLimb(i));
    }
}

// Just define the header and data to return.

std::string rodKappaLogger::getLogHeader()
{
    // dynamically create header based on number of vertices.
    std::ostringstream hdr;
    hdr << "rodKappaLogger.,Columns are limb_property_vert." << std::endl << "Simulation time (sec),";
    // For each of the representative vertices, we'll be reporting its curvature and intrinsic curvature.
    for(std::size_t i=0, max = vert_nums.size(); i < max; i++){
        hdr << i << "_" << "kappaBar_" << vert_nums[i] << ",";
    }
    // base class appends an endline
    // hdr << std::endl;
    return hdr.str();
}

std::string rodKappaLogger::getLogData()
{
    // return the kappa and kappa bar for each limb
    std::ostringstream logdata;
    logdata << m_world_p->getCurrentTime() << ",";
    // Collect for each representative vertex.
    for(std::size_t i=0, max = vert_nums.size(); i < max; i++){
        // World is a reference, but rod is a pointer within world.
        // logdata << m_world_p->getRodP()->kappa[vert_nums[i]] << "," << m_world_p->getRodP()->kappaBar[vert_nums[i]] << ",";
        // We took out the "kappa" here, since it's a dummy variable that isn't updated!
        logdata << m_world_p->getRodP()->kappaBar[vert_nums[i]] << ",";
    }
    // base class appends an endline
    // logdata << std::endl;
    return logdata.str();
}