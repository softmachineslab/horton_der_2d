/**
 * rodKbEILogger.h
 * 
 * Declarations for the concrete class rodKbEILogger.
 * Records the intrinsic curvature and bending stiffness of a rod,
 * with representative vertex for each limb.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_KB_EI_LOGGER_H
#define ROD_KB_EI_LOGGER_H

// ideally, we only need what's in the superclass
#include "worldLogger.h"

// subclass of worldLogger.
class rodKbEILogger : public worldLogger
{
    public:

    // Constructor which will call the parent's constructor also.
    rodKbEILogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, shared_ptr<world> w, int per);
    // destructor will also just call parent
    ~rodKbEILogger();

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

#endif // ROD_KB_EI_LOGGER_H