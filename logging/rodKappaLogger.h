/**
 * rodKappaLogger.h
 * 
 * Declarations for the concrete class rodKappaLogger.
 * Records the curvature and intrinsic curvature of a rod,
 * optionall specifying a range of vertices to record in particular.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_KAPPA_LOGGER_H
#define ROD_KAPPA_LOGGER_H

// ideally, we only need what's in the superclass
#include "worldLogger.h"

// subclass of worldLogger.
class rodKappaLogger : public worldLogger
{
    public:

    // Constructor which will call the parent's constructor also.
    rodKappaLogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, shared_ptr<world> w, int per);
    // destructor will also just call parent
    ~rodKappaLogger();

    protected:

    // redefining the parent's aux setup function
    void setupHelper();

    private:

    // Function for header for this logger.
    std::string getLogHeader();

    // Function for data to be returned by this logger.
    std::string getLogData();

    // Store the vertex numbers for a representative example from each limb.
    // We'll report only these, not all vertices.
    std::vector<int> vert_nums;

    // aside, the world knows its numLimbs, so we don't need to store it here.

};

#endif // ROD_KAPPA_LOGGER_H