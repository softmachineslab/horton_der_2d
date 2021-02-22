/**
 * rodRBStateFileLogger.cpp
 * 
 * Concrete data logging class that records the rigid body approx for the rolling star.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodRBStateFileLogger.h"

// Constructor: the parent does everything for us
rodRBStateFileLogger::rodRBStateFileLogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, shared_ptr<world> w, int per) : 
                            worldLogger(fileNamePrefix, logfile_base, df, w, per)
{
    if(verbosity >= 2) {
        std::cout << "Constructing a rodRBStateFileLogger..." << std::endl;
    }
}

// destructor does nothing
rodRBStateFileLogger::~rodRBStateFileLogger()
{
}

// Just define the header and data to return.
std::string rodRBStateFileLogger::getLogHeader()
{
    // it'll always be (x,y,theta)
    std::ostringstream hdr;
    hdr << "rodRBStateFileLogger.," << std::endl << "Simulation time (sec),CoM_X,CoM_Y,Theta,dCoM_X_dt,dCoM_Y_dt,dTheta_dt";
    // base class appends an endline
    // hdr << std::endl;
    return hdr.str();
}

std::string rodRBStateFileLogger::getLogData()
{
    // return the kappa and kappa bar for each limb
    std::ostringstream logdata;
    logdata << m_world_p->getCurrentTime() << ",";
    // X, Y
    Vector2d com = m_world_p->getRodP()->getCOM();
    logdata << com[0] << "," << com[1] << ",";
    // Theta, see elasticRod for more info
    logdata << m_world_p->getRodP()->getRBRotation() << ",";
    // dot X, dot Y (velocities)
    Vector2d com_vel = m_world_p->getRodP()->getVelocityCOM();
    logdata << com_vel[0] << "," << com_vel[1] << ",";
    // and angular velocity too. 
    logdata << m_world_p->getRodP()->getVelocityAngular();
    // logdata << std::endl;
    return logdata.str();
}