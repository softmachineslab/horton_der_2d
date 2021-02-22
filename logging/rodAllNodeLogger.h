/**
 * rodAllNodeLogger.h
 * 
 * Declarations for the concrete class rodAllNodeLogger.
 * Records the nodal locations for all nodes, alongside some options,
 * logging to a file.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_ALL_NODE_LOGGER_H
#define ROD_ALL_NODE_LOGGER_H

// ideally, we only need what's in the superclass
#include "worldLogger.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

// subclass of worldLogger.
class rodAllNodeLogger : public worldLogger
{
    public:

    // Constructor which will call the parent's constructor also.
    rodAllNodeLogger(std::string fileNamePrefix, std::string logfile_base, std::ofstream& df, shared_ptr<world> w, int per);
    // destructor will also just call parent
    ~rodAllNodeLogger();

    protected:

    // redefining the parent's aux setup function
    // void setupHelper();

    private:

    // Function for header for this logger.
    std::string getLogHeader();

    // Function for data to be returned by this logger.
    std::string getLogData();

};

#endif // ROD_ALL_NODE_LOGGER_H