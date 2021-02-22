/**
 * horton_2d_der, based on simDER
 * Simulates a 2D planar motion of the underwater tripod robot
 * 
 * simDER stands for "[sim]plified [D]iscrete [E]lastic [R]ods"
 * Copyright M.K. Jawed et al., Dec 2017,
 * and Andrew P. Sabelhaus, 2020-
 * This code is based on previous iterations. 
 * */

// Usage: INSERT

//This line is for mac
//#include <GLUT/glut.h>

//This is for linux
#include <GL/glut.h>

// the C++ standard library
#include <iostream>
#include <fstream>
#include <string>
#include <exception> // for error handling

// Eigen
#include "eigenIncludes.h"

// The "simulation environments" are going to control time stepping.
#include "simulation_environments/derSimulationEnvironment.h"
#include "simulation_environments/openglDERSimulationEnvironment.h"
#include "simulation_environments/headlessDERSimulationEnvironment.h"
// Rod and stepper are included in the world
#include "world.h"
// Initialization needs both the input options (.txt file) and rod state initialization
#include "initialization/setInput.h"
#include "initialization/initialStateBabbler.h"
// The logging framework is separate from world.
#include "logging/worldLogger.h"
#include "logging/rodKappaLogger.h"
#include "logging/rodKbEILogger.h"
#include "logging/rodRBStateFileLogger.h"
#include "logging/rodAllNodeLogger.h"
// The control framework is used in the world, but the controller object itself is created here.
#include "controllers/rodController.h"
#include "controllers/rodCOMSingleShotController.h"
#include "controllers/rodCOMPWMController.h"
#include "controllers/rodEmptyController.h"
#include "controllers/rodOpenLoopPWMController.h"
#include "controllers/rodOpenLoopFileController.h"
// Some constants used throughout this program
#include "global_const.h"

// #include <unistd.h> // for sleep(1)

// global variables
ofstream kappaOutFile;
// Here is where we declare verbosity as a variable.
// the global_const.h file makes it "extern" for everyone else
int verbosity;

int main(int argc,char *argv[])
{
	// Startup: load the options file.
	setInput inputData;
	inputData = setInput();
	// Check: throw an error unless at least one argument is passed (requires an options file!)
	if(argc < 2) {
		throw std::runtime_error("Not enough arguments! Must pass at least an options file.");
	}
	inputData.LoadOptions(argv[1]);
	inputData.LoadOptions(argc,argv);
	//read input parameters from txt file and cmd

	// Some variables used outside of the world:
	// WE ARE NOW DOING TWO ACTUATORS PER LIMB
	int numAct = 2 * inputData.GetIntOpt("num-limbs");
	// Walkers must have an odd number of limbs.
	if(inputData.GetIntOpt("num-limbs") % 2 == 0)
	{
		throw std::invalid_argument("Error! Number of limbs must be an odd number for the walker.");
	}
	int logging_period = inputData.GetIntOpt("data-frequency");
	// debugging verbosity, used in many classes.
	verbosity = inputData.GetIntOpt("debug-verbosity");
	// for the simulation environment to output periodic updates
	int cmdline_per = inputData.GetIntOpt("cmdline-period");
	// folder for the logger
	std::string logfile_base = inputData.GetStringOpt("logfile-base");
	// where to read the actuation timepoints from
	std::string act_csv_path = inputData.GetStringOpt("ol-control-filepath");
	// number of simulations to run (for the loop below.)
	// should be 1 if deterministic!
	int num_simulations = inputData.GetIntOpt("num-simulations");

	// TO-DO: Any manual edits to the input data? For example, maximum simulation time?

	// Greeting.
	if(verbosity >= 1){
		std::cout << "simTripod." << std::endl << "Copyright 2020 Huang et al., M.K. Jawed et al., modifications by A.P. Sabelhaus." << std::endl;
		std::cout << "Will run " << to_string(num_simulations) << " simulations, each ";
		std::cout << to_string(inputData.GetScalarOpt("totalTime")) << " seconds long." << std::endl;
	}

	// Since we don't anticipate needing to set the rigid body initial position via command line,
	// i.e. it will only be done programatically, we can pass it in as another argument to the world constructor.
	// VectorXd rb_state_0(6);
	// rb_state_0 << 0.0, 0.03, 0.0, 0.0, 0.0, 0.0;
	VectorXd rb_state_0 = VectorXd::Zero(6);
	rb_state_0(1) = 0.0; // y offset
	rb_state_0(2) = 0.0; // initial rotation, degrees
	rb_state_0(3) = 0.0; // x-velocity 
	rb_state_0(4) = 0.0; // y-velocity
	rb_state_0(4) = 0.0; // y-velocity
	rb_state_0(5) = 0.0; // angular velocity, deg/sec.

	// Alternatively, sample from a normal distribution for the initial states.
	// Means and standard deviations for each state:
	// [x, y, theta, dx_dt, dy_dt, dtheta_dt], units of:
	// [m, m, deg, m/s, m/s, deg/sec]
	// vector<double> means = {0.0, 0.035, 0.0, 0.0, 0.0, 0.0};
	// vector<double> stddevs = {0.015, 0.005, 180, 0.2, 0.2, 200};
	// HACK: this is now a uniform distribution, with "means" as lower and "stddevs" as upper bound.
	// vector<double> means = {-0.01, 0.025, -180.0, -0.02, -0.02, -300};
	// vector<double> stddevs = {0.02, 0.035, 180.0, 0.02, 0.02, 300};
	// unique_ptr<initialStateBabbler> st_babbler_p = make_unique<initialStateBabbler>(means, stddevs);

	// Now we can run the simulation iteratively.
	// ***NOTE that the GUI is broken for multiple iterations!!!! Must find a way to exit GLUT...
	// int num_simulations = 10;

	for(int i=0; i < num_simulations; i++) {

		// Initialize the pose of the robot
		// VectorXd rb_state_0 = st_babbler_p->getSample();

		if( verbosity >= 1){
			std::cout << std::endl << "Iteration number: " << to_string(i+1) << std::endl;
			std::cout << "RB state sample: " << std::endl;
			std::cout << rb_state_0 << std::endl;
		}

		shared_ptr<world> myworld_p = make_shared<world>(inputData, rb_state_0);
		// Primary tie-in to set up basically everything.
		myworld_p->setRodStepper();

		// Set up the logging infrastructure
		// rodKappaLogger kLogger = rodKappaLogger("simTripodKappa", kappaOutFile, myworld_p, logging_period);
		// rodKbEILogger kLogger = rodKbEILogger("simTripodKbEI", kappaOutFile, myworld_p, logging_period);
		// rodRBStateFileLogger kLogger = rodRBStateFileLogger("simTripodRBState", kappaOutFile, myworld_p, logging_period);
		// worldLogger *logger_p = new rodRBStateFileLogger("simTripodRBState", kappaOutFile, myworld_p, logging_period);
		// attempting to use smart pointers. will be passed around so... shared
		// shared_ptr<worldLogger> logger_p = make_shared<rodRBStateFileLogger>("simTripodRBState", logfile_base, kappaOutFile, myworld_p, logging_period);
		// shared_ptr<worldLogger> logger_p = make_shared<rodAllNodeLogger>("simBipedAllNodes", logfile_base, kappaOutFile, myworld_p, logging_period);

		// This writes the header.
		// logger_p->setup();

		// Create a controller and assign it to the world.
		// rodCOMSingleShotController ctlr = rodCOMSingleShotController(numAct);
		// controller_p = new rodCOMSingleShotController(numAct);
		// controller_p = new rodCOMPWMController(numAct);
		// controller_p = new rodEmptyController(numAct);
		// rodController *controller_p = new rodEmptyController(numAct);
		// shared_ptr<rodController> controller_p = make_shared<rodEmptyController>(numAct);

		// For the open-loop controller: specify the seven start times, periods, and duty cycles.
		// double st_t = 0.1;
		double per_t = 0.08; // short period with low duty cycle means slow pulses to heat up the SMA
		// Same start time
		// std::vector<double> act_starts = {st_t, st_t, st_t, st_t, st_t, st_t, st_t};
		// Turning on some later 
		// std::vector<double> act_starts = {1.0, st_t, st_t, st_t, st_t, st_t};
		// std::vector<double> act_pers = {per_t, per_t, per_t, per_t, per_t, per_t, per_t};

		// duty cycle as a percent, 0 to 1
		// For "fast" inertial rolling motions
		// std::vector<double> act_dutys = {0.0, 0.01, 0.01, 0.0, 0.0, 0.0, 0.01}; // funky deformation 1
		// std::vector<double> act_dutys = {0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0}; // one leg deformation
		// std::vector<double> act_dutys = {0.0, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0}; // two leg deformation
		// For "slow" pseudo-static motions, *not* reliant on frictional properties/hybrid transitions
		// std::vector<double> act_dutys = {0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0}; // one leg
		// std::vector<double> act_dutys = {0.0, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0}; // two leg
		// std::vector<double> act_dutys = {0.05, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0}; // two PLUS 0th leg
		// std::vector<double> act_dutys = {0.0, 0.05, 0.05, 0.05, 0.0, 0.0, 0.0}; // three leg
		// std::vector<double> act_dutys = {0.05, 0.05, 0.05, 0.05, 0.0, 0.0, 0.0}; // three PLUS 0th leg
		// std::vector<double> act_dutys = {0.0, 0.05, 0.05, 0.05, 0.05, 0.0, 0.0}; // four leg
		// std::vector<double> act_dutys = {0.0, 0.05, 0.05, 0.05, 0.05, 0.05, 0.0}; // five leg
		// std::vector<double> act_dutys = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.0}; // five PLUS 0th leg - should have delay for 0 here

		// Now initialized to match num-limbs in the config text file.
		// NOTE that there are now TWO ACTUATORS per limb: 
		// [0, 1], [2, 3], ..., [2(N-2), 2(N-1)]
		// std::vector<double> act_starts(numAct, st_t);
		std::vector<double> act_pers(numAct, per_t);
		// std::vector<double> act_dutys(numAct, 0.0);
		// and we can manually modify as desired.
		// NOTE, this can cause a segfault if misaligned with numAct!
		// The tripod has 5 limbs, 0-through-4,
		// which are actuators (0,1) - (2,3) - (4,5) - (6,7) - (8,9)
		// where each are (right, left) bending
		// act_dutys[0] = 0.02;
		// act_dutys[1] = 0.02;
		// act_dutys[2] = 0.02;
		// act_dutys[3] = 0.02;
		// act_dutys[4] = 0.02;
		// act_dutys[8] = 0.02;
		// act_dutys[9] = 0.02;

		// To make the middle leg wiggle back and forth like a PATRICK limb, let's set different start times,
		// and then hack a hard cutoff time in the controller for one direction.
		// act_dutys[4] = 0.05;
		// act_dutys[5] = 0.05;
		// act_starts[5] = 2;

		// shared_ptr<rodController> controller_p = make_shared<rodOpenLoopPWMController>(numAct, act_starts, act_pers, act_dutys);
		
		// Now, use a CSV file.
		// std::string act_csv_path = "openloop_control_trajectories/tripod_ol_control_example_2020-09-02.csv";
		shared_ptr<rodController> controller_p = make_shared<rodOpenLoopFileController>(numAct,  act_pers, act_csv_path);

		myworld_p->setRodController(controller_p);

		// Make a simulation environment based on render yes/no from the inputData
		unique_ptr<derSimulationEnvironment> env_p = nullptr;
		if(myworld_p->isRender()){
			// NOTE this is BROKEN for num_simulations > 1!
			// env_p = make_unique<openglDERSimulationEnvironment>(myworld_p, cmdline_per, logger_p, argc, argv);
			// no log for some testing for now
			env_p = make_unique<openglDERSimulationEnvironment>(myworld_p, cmdline_per, argc, argv);
		}
		else
		{
			// env_p = make_unique<headlessDERSimulationEnvironment>(myworld_p, cmdline_per, logger_p);
			// no log for some testing for now
			env_p = make_unique<headlessDERSimulationEnvironment>(myworld_p, cmdline_per);
		}
		
		// bool render = myworld_p->isRender();

		// Use the "simulation environment" paradigm. Only main will ever have a sim env.
		// unique_ptr<derSimulationEnvironment> env_p = make_unique<openglDERSimulationEnvironment>(myworld_p, cmdline_per, logger_p, argc, argv);
		// unique_ptr<derSimulationEnvironment> env_p = make_unique<headlessDERSimulationEnvironment>(myworld_p, cmdline_per, logger_p);

		// run until world's total time from inputData
		env_p->runSimulation();

	}
	
	return 0;

	// smart pointers should take care of all memory management now, no delete()s.
}
