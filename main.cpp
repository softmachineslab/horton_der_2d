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

// Hack: main creates the output file for logging
ofstream logging_output_file;

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
	// turn the logger on or off
	bool enable_logging = inputData.GetBoolOpt("enable-logging");
	// folder for the logger
	std::string logfile_base = inputData.GetStringOpt("logfile-base");
	// If using the rodOpenLoopFileController: where to read the actuation timepoints from
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

	/**
	 * Initialize the location of the robot (the discrete elastic rod) in space.
	 */

	// Since we don't anticipate needing to set the rigid body initial position via command line,
	// i.e. it will only be done programatically, we can pass it in as another argument to the world constructor.
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

	// ***NOTE that the GUI is broken for multiple iterations!!!! Must find a way to exit GLUT...
	for(int i=0; i < num_simulations; i++) {

		// UNCOMMENT if using the initial state sampler
		// VectorXd rb_state_0 = st_babbler_p->getSample();

		if( verbosity >= 1){
			std::cout << std::endl << "Iteration number: " << to_string(i+1) << std::endl;
			std::cout << "RB state sample: " << std::endl;
			std::cout << rb_state_0 << std::endl;
		}

		shared_ptr<world> myworld_p = make_shared<world>(inputData, rb_state_0);
		// Primary tie-in to set up basically everything.
		myworld_p->setRodStepper();

		/**
		 * Set up the logging infrastructure if you want to save data.
		 */

		shared_ptr<worldLogger> logger_p = nullptr;
		// We use a flag from the options text file to enable/disable logging
		if(enable_logging){
			logger_p = make_shared<rodRBStateFileLogger>("simTripodRBState", logfile_base, logging_output_file, myworld_p, logging_period);
			// logger_p = make_shared<rodAllNodeLogger>("simBipedAllNodes", logfile_base, logging_output_file, myworld_p, logging_period);
			
			// This writes the header. You must call it here!
			logger_p->setup();
		}

		/**
		 * Create a controller for the rod, and assign it to the world.
		 */

		// Option 1) Controller that does nothing:
		// shared_ptr<rodController> controller_p = make_shared<rodEmptyController>(numAct);

		// Option 2) Open-loop controller for the pulse-width-modulation (PWM) input to the shape memory alloy (SMA) actuator model.
		// PWMs take a duty cycle and frequency, and will "power on" our imaginary SMA wires intermittently.
		// The SMAs change the rod mechanics parameters (specifically, the intrinsic curvature, kappaBar) that move the robot.
		// **NOTE** you must specify the correct number of PWM inputs as the number of SMAs on the robot.
		// Below are examples for the "rolling star" robot with 7-10 SMAs. Horton uses 10.

		// Period:
		double per_t = 0.08; // short period with low duty cycle means slow pulses to heat up the SMA
		// Start time:
		// double st_t = 0.1;
		// All the same start time:
		// std::vector<double> act_starts = {st_t, st_t, st_t, st_t, st_t, st_t, st_t};
		// Turning on some later 
		// std::vector<double> act_starts = {1.0, st_t, st_t, st_t, st_t, st_t};

		// Now initialized to match num-limbs in the options text file.
		// NOTE that there are now TWO ACTUATORS per limb: 
		// [0, 1], [2, 3], ..., [2(N-2), 2(N-1)]
		// std::vector<double> act_starts(numAct, st_t);
		std::vector<double> act_pers(numAct, per_t);
		// std::vector<double> act_dutys(numAct, 0.0);
		// and we can manually modify as desired.
		// NOTE, this can cause a segfault if misaligned with numAct!
		// Horton has 5 limbs, 0-through-4,
		// which are actuators (0,1) - (2,3) - (4,5) - (6,7) - (8,9)
		// where each are (right, left) bending
		// act_dutys[0] = 0.02;
		// act_dutys[1] = 0.02;
		// act_dutys[2] = 0.02;
		// act_dutys[3] = 0.02;
		// act_dutys[4] = 0.02;
		// act_dutys[8] = 0.02;
		// act_dutys[9] = 0.02;

		// Finally: create the manually-defined open-loop controller
		// shared_ptr<rodController> controller_p = make_shared<rodOpenLoopPWMController>(numAct, act_starts, act_pers, act_dutys);
		
		// Option 3) Open-loop controller, reading inputs from a comma-separated-value (CSV) file.
		// Note that act_csv_path is specified via the options file
		shared_ptr<rodController> controller_p = make_shared<rodOpenLoopFileController>(numAct,  act_pers, act_csv_path);

		// Attach the controller to the world.
		myworld_p->setRodController(controller_p);

		/**
		 * Create a "simulation environment" that will manage logging, command line output, etc.
		 * The "render" flag from the options file determines if we will use a graphical interface or not.
		 */
		unique_ptr<derSimulationEnvironment> env_p = nullptr;
		if(myworld_p->isRender()){
			// NOTE this is BROKEN for num_simulations > 1!
			if(enable_logging){
				env_p = make_unique<openglDERSimulationEnvironment>(myworld_p, cmdline_per, logger_p, argc, argv);
			}
			else{
				env_p = make_unique<openglDERSimulationEnvironment>(myworld_p, cmdline_per, argc, argv);
			}
		}
		else
		{
			if(enable_logging){
				env_p = make_unique<headlessDERSimulationEnvironment>(myworld_p, cmdline_per, logger_p);
			}
			else{
				env_p = make_unique<headlessDERSimulationEnvironment>(myworld_p, cmdline_per);
			}
		}

		// run until world's total time from inputData
		env_p->runSimulation();
	}
	
	return 0;

	// smart pointers should take care of all memory management now, no delete()s.
}
