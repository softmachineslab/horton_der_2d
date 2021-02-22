#ifndef WORLD_H
#define WORLD_H

#include "eigenIncludes.h"

// include elastic rod class
#include "rod_mechanics/elasticRod.h"

#include <time.h>

// For throwing errors related to convergence.
#include <stdexcept>

// include force classes
#include "rod_mechanics/elasticStretchingForce.h"
#include "rod_mechanics/elasticBendingForce.h"
#include "rod_mechanics/externalGravityForce.h"
#include "rod_mechanics/inertialForce.h"

// include external force
#include "rod_mechanics/dampingForce.h"
#include "rod_mechanics/externalContactForce.h"

// include time stepper
#include "rod_mechanics/timeStepper.h"

// include input file and option
#include "initialization/setInput.h"

// SMAs for actuation model
#include "rod_mechanics/shapeMemoryAlloy.h"
#include "rod_mechanics/shapeMemoryAlloyCompression.h"

// Controller framework
#include "controllers/rodController.h"
#include "controllers/rodCOMSingleShotController.h"

class world
{
public:
	world();
	// Constructor can include an initial pose of the rod. Otherwise will be assumed zero.
	world(setInput &m_inputData);
	world(setInput &m_inputData, VectorXd m_rb_state_0);
	// a helper to the constructor will take care of the SetInput initialization.
	void worldConstructorHelper(setInput &m_inputData);
	//
	~world();
	void setRodStepper();
	void updateTimeStep();
	int simulationRunning();
	int numPoints();
	Vector3d getScaledCoordinate(int i, int j);
	Vector3d getScaledBendingCoordinate(int i, int j); // for plotting bending pairs also. 
	double getScaledBoundary(double x);
	double getCurrentTime();
	double getTotalTime();
	
	bool isRender();

	// Some helpers for interfacing with callers.
	shared_ptr<elasticRod> getRodP();
	int getTimeStep();

	// Controller framework: must assign a rodController after instantiation.
	// void setRodController(rodController* c_p);
	void setRodController(shared_ptr<rodController> c_p);

	// Some outward facing functions for the private variables.
	int getNumLimbs();

	// some getters for use with logging
	double getLimbLength();
		
private:

	// Physical parameters
	double limbLength, RodLength;
	double footLength;
	double rodRadius;
	int numVertices;
	double youngM;
	double Poisson;
	double shearM;
	double deltaTime;
	double totalTime;
	double density;
	Vector3d gVector;
	double viscositya;
	double viscosityb;
	double dropHeight;
	double delta_l;
	int numLimbs;
	bool enableAutoDrop;
	// Initial rigid body pose of the rod, passed through to the elasticRod constructor.
	VectorXd rb_state_0;

	double tol, stol;
	int maxIter; // maximum number of iterations
	double characteristicForce;
	double forceTol;
	
	// Geometry, passed in to rod
	MatrixXd vertices;
	
	// Rod
	shared_ptr<elasticRod> rod = nullptr;
	
	// set up the time stepper
	// timeStepper* stepper;
	shared_ptr<timeStepper> stepper = nullptr;
	double* totalForce; // this is owned by the stepper, so the stepper delete[]s it.
	double currentTime;
	
	// declare the forces
	unique_ptr<elasticStretchingForce> m_stretchForce = nullptr;
	unique_ptr<elasticBendingForce> m_bendingForce = nullptr;
	unique_ptr<inertialForce> m_inertialForce = nullptr;
	unique_ptr<externalGravityForce> m_gravityForce = nullptr;
	unique_ptr<dampingForce> m_dampingForce = nullptr;
	unique_ptr<externalContactForce> m_externalContactForce = nullptr;
	
	// int Nstep;
	// count number of iterations.
	int timeStep;
	int iter;

	void rodGeometryChain();
	void rodGeometryWalker();
    
	bool render; // should the OpenGL rendering be included?
	
	bool updateTimeStep_singleRod();
	void addRod();
	// The SMAs are also added during setup. Note, must be AFTER rod!!
	void addSMAs();
	
	// Friction coefficients. NOTE that static isn't used...!
	double dynamicFric;
	
	void checkJacobian();
	void computeReactionForce();
	
	// double FN, Ff; // normal and friction force
	
	// compute average velocity
	// double initialX, finalX, avgVelocity;
	Vector2d computeCenterMass();
	// std::string actuationDirName;
	
	// was simulation successful?
	bool successfulSim;
	
	int boundaryType; // type of boundary

	int inputNv; // number of nodes per limb, counting inclusively (includes joints). From options file. Drew's notation: M
	int nvEachLimb; // number of nodes per limb, counting only ONE of its two attached joint locations. = inputNv-1. Drew's notation: P
	int nvEachLimbNoJoints; // number of nodes per limb, counting NEITHER joint. = inputNV-2. Drew's notation: unused I think...

	double nTimeCurvature;
	double nTimeStiffness;
	double tau;
	double Tinitial;

	// For the compression SMA model
	double compression_kB_max;

	VectorXi ifAddDynamicFriction;

	// Rod controller pointer, to be used (e.g.) in updateTimeStep
	// rodController* ctlr_p = NULL;
	shared_ptr<rodController> ctlr_p = nullptr;

	// The world contains the SMA actuators. The SMAs are manipulated by the world, then
	// their resulting kB, EI get passed to the rods for mechanics calculations.
	std::vector<shared_ptr<shapeMemoryAlloy>> sma;
};

#endif
