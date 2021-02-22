/**
 * rodAllNodeLogger.cpp
 * 
 * Concrete data logging class that records the nodal locations for all nodes in the rolling star
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "rodAllNodeLogger.h"

// Constructor: the parent does everything for us
rodAllNodeLogger::rodAllNodeLogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, shared_ptr<world> w, int per) : 
                            worldLogger(fileNamePrefix, logfile_base, df, w, per)
{
    if(verbosity >= 2) {
        std::cout << "Constructing a rodAllNodeLogger..." << std::endl;
    }
}

// destructor does nothing
rodAllNodeLogger::~rodAllNodeLogger()
{
}

// Just define the header and data to return.
std::string rodAllNodeLogger::getLogHeader()
{
    std::ostringstream hdr;
    hdr << "rodAllNodeLogger.,Simulation Setup:" << std::endl;
    hdr << "limb_length,num_limbs,num_vert_per_limb" << std::endl;
    // Some quick math:
    // Number of vertices per circular section of limb: from elasticRod,
    int num_vert_per_limb = (m_world_p->getRodP()->nv)/(m_world_p->getNumLimbs());
    // and the number per flat section ("groove") already calculated in rod
    // int num_vert_per_flat = m_world_p->getRodP()->getnPerGroove();
    // The vertices have the structure of (circular_1, circular_2, ... circular_N, flat_1, flat_2, ...) so the total per "limb" is
    // int num_vert_per_limb = num_vert_per_circular + num_ver_per_flat;
    // ...not reporting, since we shouldn't be dividing 126 / 18, that gives the wrong allocation of nodes to limbs.
    hdr << m_world_p->getLimbLength() << "," << m_world_p->getNumLimbs() << "," << num_vert_per_limb << "," << std::endl;
    hdr << "Simulation Time,";

    // Generate the header for all vertices. First, the circular sections, since they come at the beginning of the vector.
    for(std::size_t i=0; i < m_world_p->getNumLimbs(); i++){
        // For the i-th limb, we have a certain number of vertices
        for(std::size_t j=0; j < num_vert_per_limb; j++){
            // both an x and a y for this vertex, this limb
            hdr << i << "_" << j << "_x," << i << "_" << j << "_y,";
        }
    }
    // // repeat the process, now for the flat "groove" sections
    // for(std::size_t i=0; i < m_world_p->getNumLimbs(); i++){
    //     // For the i-th limb, we have a certain number of vertices
    //     for(std::size_t j=0; j < num_vert_per_flat; j++){
    //         // both an x and a y for this vertex, this limb
    //         hdr << "flat_" << i << "_" << j << "_x,flat_" << i << "_" << j << "_y,";
    //     }
    // }

    // CoM_X,CoM_Y,Theta,dCoM_X_dt,dCoM_Y_dt,dTheta_dt";
    // base class appends an endline
    // hdr << std::endl;
    return hdr.str();
}

std::string rodAllNodeLogger::getLogData()
{
    // return the kappa and kappa bar for each limb
    std::ostringstream logdata;
    logdata << m_world_p->getCurrentTime();
    // Return the x, y position of each node. This should be arranged correctly in the vector x in elasticRod.
    for(std::size_t i=0; i < m_world_p->getRodP()->ndof; i++){
        logdata << "," << m_world_p->getRodP()->x(i);
    }
    // X, Y
    // Vector2d com = m_world_p->getRodP()->getCOM();
    // logdata << com[0] << "," << com[1] << ",";
    // // Theta, see elasticRod for more info
    // logdata << m_world_p->getRodP()->getRBRotation() << ",";
    // // dot X, dot Y (velocities)
    // Vector2d com_vel = m_world_p->getRodP()->getVelocityCOM();
    // logdata << com_vel[0] << "," << com_vel[1] << ",";
    // // and angular velocity too. 
    // logdata << m_world_p->getRodP()->getVelocityAngular();
    // logdata << std::endl;
    return logdata.str();
}