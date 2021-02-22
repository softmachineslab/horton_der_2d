#ifndef ELASTICROD_H
#define ELASTICROD_H

#include "../eigenIncludes.h"

// for some error checking
// #include <assert.h>
// used for generating a range of integers
#include <numeric> // std::iota 

class elasticRod
{
	public:
	
	// The default constructor assumes that initialNodes is the desired pose
	// (subject to enable-auto-drop) with zero velocity.
	elasticRod(MatrixXd initialNodes, 
		double m_rho, double m_rodRadius, double m_dt,
		double m_youngM, double m_shearM, 
		double m_dropHeight, double m_Aboundary,
		double m_lambdaBoundary, int m_numLimbs, double m_limbLength,
		int m_boundaryType, int m_nvEachLimb, bool m_enableAutoDrop);
	// But, we can also pass in a desired initial pose, in terms of rigid body states,
	// and the constructor will adjust the vertices for us.
	elasticRod(MatrixXd initialNodes, 
		double m_rho, double m_rodRadius, double m_dt,
		double m_youngM, double m_shearM, 
		double m_dropHeight, double m_Aboundary,
		double m_lambdaBoundary, int m_numLimbs, double m_limbLength,
		int m_boundaryType, int m_nvEachLimb, bool m_enableAutoDrop, VectorXd m_rb_state_0);
	~elasticRod();

	// called externally from world - set up the rod
	void setup();
	// called internally - set mass array
	void setMass();
	
	// called externally from world - boundary things
	void setVertexBoundaryCondition(Vector2d position, int i);
	void releaseVertexBoundaryCondition(int i);

	// called externally from world - moves to next timestep
	void updateTimeStep();
	// used in setup, internal
	void computeElasticStiffness();
	// called externally by world, part of timestepping, updateTimestep_SingleRod
	void updateNewtonX(double *dc);
	// called externally by world, part of timestepping
	void updateGuess();
	
	// boundary: used locally here
	double Aboundary, lambdaBoundary;
	// called externally by world
	double thetaNormal(double xPos);
	double boundary(double xPos);
	
	// utility functions
	Vector2d getVertex(int k);
	Vector2d getVelocity(int k);
	// used internally. Implements the equation for kappa
	double computeCurvature(double e0e1, double xkm1, double ykm1, double xk, double yk, double xkp1, double ykp1);
	
	// used for initialization of kappaBarOriginal in comparison to the actuated kappaBar
	void computeKappaBar();
	// used internally, adjusts bendingNodes during setup. Then, bendingNodes used in elasticBendingForce.
	void fixBendingData();

	// used in world during the collision checking.
	void setOneVertexConstraintCurve(Vector2d position, int k);
	
	// Elastic stiffness values
	double youngM, shearM; // Young's and shear modulus
	double EA; // stretching stiffness
	double EI_scalar; // bending stiffness
	VectorXd EI; // bending stiffness vector
	
	int nv; // number of vertices
	int ndof; // number of degrees of freedom = 3*nv + ne
	double rho; // density
	double rodRadius; // cross-sectional radius of the rod
	double crossSectionalArea; // cross-sectional area of the rod
	double dt; // time step
	double dm; // mass per segment
	double dropHeight; // height above ground
	bool enableAutoDrop; // turn on/off the automatic alignment. Overrides any \bx_0 passed to constructor.

	// dof vector before time step
	VectorXd x0; 
	// dof vector after time step
	VectorXd x;
	// velocity vector
	VectorXd u;
	// previous velocity vector
	VectorXd u0;
	// Projected/estimated velocity vector (to handle contact)
	VectorXd uProjected;
	// change in velocity vector
	VectorXd dV;
	// required change in velocity vector
	VectorXd dVReqd;
	
	// nodes passed in. Reorganized into x during instantiation.
	MatrixXd nodes;
	// We'll store the reference pose of the robot, automatically centered at the origin,
	// useful for e.g. rotational RB state. Used after reorganizing from 2xN matrix to 1x? vector.
	VectorXd x_ref;
	// Store an initial pose of the robot, in terms of rigid body states, at time 0.
	// Used during construction.
	VectorXd rb_state_0;

	// lumped mass
	VectorXd massArray;
	// inverse lumped mass
	std::vector<Matrix2d> W_Nodes;
	// Curvature. These also used in SMAs and actuation code in world, as well as in elasticBendingForce!
	VectorXd kappaBar;
	VectorXd kappaBarOriginal;
	
	// Nodes for bending
	// To allow arbitrary connectivity, we specify "previous" and "after" nodes
	int nBends;
	std::vector< std::array<int, 3> > bendingNodes;
	std::vector< double > e0e1; // multiplication of the two edge lengths
	std::vector< double > l_k_bending; // reference length for bending

	// Edges for stretching
	int nEdges;
	std::vector< std::array<int, 2> > stretchingNodes;
	std::vector< double > l_k_stretching; // reference length for stretching

	int nvEachLimb; // number of nodes per limb, see world for more info
	
	// Sum of force so that we can compute the reaction forces. Used in world
	VectorXd sumForces;

	// constraint. Used in updateTimeStep.
	VectorXi nConstraint;
	VectorXi nConstraintPrevious;

	// CoM computation should be done by the rod itself, not the world. Copied here.
	Vector2d getCOM();
	// need the center of mass velocity for learning also
	Vector2d getVelocityCOM();
	// For the rigid body state, we need the "rotation" of the rod (around its CoM).
	double getRBRotation();
	// Will also need the time derivative of this state. Let's do angular velocity of
	// the system of particles around its CoM. TO-DO: is this consistent? Have we chosen
	// \dot \theta such that d/dt theta = \dot \theta?
	double getVelocityAngular();

	// The rod should be able to tell us which of its vertices correspond to a limb.
	// Used in the world for correlating the rodController, SMA, and rod.
	// Needs to be the list of all vertex numbers, since we've got some limbs
	// that have out-of-order numbering.
	// @returns list of all vertices for a limb.
	// NOTE that this does NOT include the joint nodes - you have to calculate those yourself.
	std::vector<int> getVertexNumsForLimb(int limb_num);

	// Some parts of the world, controller, etc., just need to query one vertex, any vertex,
	// within a limb. Return a representative vertex number. 
	int getAnyVertexNumForLimb(int limb_num);
	// Similarly, the SMA will need representative kappaBar and/or kappaBarOriginal per limb.
	double getAnyKbForLimb(int limb_num);
	double getAnyKbOrigForLimb(int limb_num);
	// And will need to set the kB and EI for all nodes within that limb.
	void setKbForLimb(int limb_num, double kB);
	void setEIForLimb(int limb_num, double EI_next);
	
	private:	

	// a helper for the constructor(s), since we'll now have two of them.
	// Constructor takes care of assigning locals,
	void constructorHelper();
	// Splitting the helper into a handful of smaller functions so it's no so incredibly long.
	void constructorHelperAssignNodes(); // sets up local variables related to the stretching, bending, etc. which use node numbers.
	// void constructorHelperCenterInX(); // for help with dropping, center the robot in the x-direction.
	void constructorHelperAddDropOffset(); // for help with dropping, move the robot such that it's above the boundary.
	VectorXd centerAt(VectorXd nodes_in, double x_center, double y_center); // more generic helper that moves a collection of points such that its CoM is as specified
	// void setInitialRBState(); // given an rb_state_0 stored locally, center the robot at that point.
	// as used in the constructor, set both u and u0 based on a translational velocity
	void addLinearVelocity(double dxdt, double dydt);
	// We'll need to rotate the rod around its center of mass.
	// angle is in degrees
	void rotateRod(double theta);
	// and also to add an initial angular velocity. Interpreted as applied to each particle around the CoM.
	void addAngularVelocity(double omega);

	// no private variable is likely a bad idea. But let's roll with it for now
	// 2d identity matrix, useful in various places
	Matrix2d Id2;
	double delta_l; // discretization length in the "circular" part
	int numLimbs; // number of limbs
	double limbLength; // length per limb, used to calculated discretization things. Could be inferred from nodes but this is easier
	
	// variables for setVertexBoundaryCondition()
	Vector2d u0Point, vAlongNormal;

	// used in constraint calcs
	double thetaWall;
	Vector2d nWall;
	int boundaryType;

	// Now that bendingNodes does not have the same indexing as nodes, we need a mapping between the two
	// Used as bendVtxIdxMap[vertex_no] = index_into_kappaBar
	std::map<int, int> bendVtxIdxMap;
	// and a map between limb number and list of "inner" nodes: the ones whose kB and EI will change upon actuation.
	std::map<int, std::vector<int>> limbVtxMap;

};

#endif
