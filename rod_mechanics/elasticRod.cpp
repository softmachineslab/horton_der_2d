#include "elasticRod.h"

elasticRod::elasticRod(MatrixXd initialNodes,
	double m_rho, double m_rodRadius, double m_dt,
	double m_youngM, double m_shearM,
	double m_dropHeight, double m_Aboundary,
	double m_lambdaBoundary, int m_numLimbs, double m_limbLength, int m_boundaryType, 
	int m_nvEachLimb, bool m_enableAutoDrop) :
	nodes(initialNodes), rho(m_rho), rodRadius(m_rodRadius), dt(m_dt),
	youngM(m_youngM), shearM(m_shearM), dropHeight(m_dropHeight),
	Aboundary(m_Aboundary), lambdaBoundary(m_lambdaBoundary), numLimbs(m_numLimbs), limbLength(m_limbLength),
	boundaryType(m_boundaryType), nvEachLimb(m_nvEachLimb), enableAutoDrop(m_enableAutoDrop)
{
	// Initialize the rigid body pose to 0 at time 0.
	// We are hard-coding the number of states to be 6, as in x y theta and dot x, dot y, dot theta.
	rb_state_0 = VectorXd::Zero(6);
	// Everything is in the helper now.
	constructorHelper();
}

// Testing: is nodesUndeformed used at all? what happens if we don't initialize it?

// If the rigid body state is passed as an argument... assign it to the local variable
elasticRod::elasticRod(MatrixXd initialNodes,
	double m_rho, double m_rodRadius, double m_dt,
	double m_youngM, double m_shearM,
	double m_dropHeight, double m_Aboundary,
	double m_lambdaBoundary, int m_numLimbs, double m_limbLength, int m_boundaryType, 
	int m_nvEachLimb, bool m_enableAutoDrop, VectorXd m_rb_state_0) :
	nodes(initialNodes), rho(m_rho), rodRadius(m_rodRadius), dt(m_dt),
	youngM(m_youngM), shearM(m_shearM), dropHeight(m_dropHeight),
	Aboundary(m_Aboundary), lambdaBoundary(m_lambdaBoundary), numLimbs(m_numLimbs), limbLength(m_limbLength),
	boundaryType(m_boundaryType), nvEachLimb(m_nvEachLimb), enableAutoDrop(m_enableAutoDrop), rb_state_0(m_rb_state_0)
{
	// a quick check that the rigid body state is the correct size.
	if(rb_state_0.size() != 6){
		throw std::invalid_argument("Error! Rigid body state specified in elasticRod constructor must be 6 rows.");
	}
	// Everything is in the helper now.
	constructorHelper();
}

// All parameters passed in to the constructor are assigned to locals there,
// nothing to pass in to the helper.
void elasticRod::constructorHelper()
{
	// First, some basic setup:
	// (1) Parameters for the constraints(?)

	Id2<<1.0,0.0,
		0.0,1.0;

	// (2) other local variables: length, cross sectional area, etc.

	nv = nodes.rows(); // ...technically could have been obtained via numLimbs and nvEachLimb, but...
	ndof = 2*nv;
	crossSectionalArea = 1 * rodRadius; // ...this seems to hard-code a constant somehow...
	
	// Compute the discretization length
	// For the tripod/walker, assume that all limbs have the same length for now
	double totalLen = limbLength * numLimbs;
	delta_l = totalLen / ((double) nv - 1);
	
	// (3) Reorganize the nodes.

	// Combine the 2D nodes matrix into a single column vector.
	// The variable x is what's used throughout the simulation. TO-DO: do "nodes" ever change in value??
	x = VectorXd(ndof);

	for (int i=0; i < nv; i++)
	{
		x(2*i) = nodes(i, 0);
		x(2*i + 1) = nodes(i, 1);
	}

	// (4) Calculate which nodes are pairs for bending, stretching, etc.
	// Here, we'll also populate limbVtxMap, so we don't need to keep calling getVertexNumsForLimb.
	for(int ell=0; ell < numLimbs; ell++)
	{
		limbVtxMap[ell] = getVertexNumsForLimb(ell);
	}
	// do the node allocation into bendingNodes and stretchingNodes
	constructorHelperAssignNodes();

	// (5) Adjustments to the initial pose.
	// save the reference pose
	x_ref = centerAt(x, 0.0, 0.0);
	// Here, we know that rb_state_0 exists, and if not specified elsewhere, is zero.
	x = centerAt(x, rb_state_0(0), rb_state_0(1));
	// If an initial angle is specified, rotate the rod that amount. Recall, [x y theta dxdt dydt dtheta_dt]
	rotateRod(rb_state_0(2));

	// IF BEING DROPPED, i.e., if this constructor is changing the initial pose,
	if(enableAutoDrop){
		// To-Do: make this function take arguments and return result
		constructorHelperAddDropOffset();
	}
	// else, we'll assume that the reference pose was as intended and make no adjustments.
	
	// (6) Now that we have the initial pose fully specified, save it as "previous" also,
	//	   and set the velocities to zero.

	x0 = x;
	u = VectorXd::Zero(ndof); // velocity
	u0 = u;

	// (7) Change the initial velocity based on the rigid body state.
	
	// Note: this does nothing if rb_state_0 == zeros.
	// States are x y theta, dxdt dydt dthetadt, so indices 3 and 4 counting from 0.
	addLinearVelocity(rb_state_0(3), rb_state_0(4));
	// Same with angular, last index (6-th entry).
	addAngularVelocity(rb_state_0(5));

	dV = VectorXd::Zero(ndof); // change in velocity
	dVReqd = VectorXd::Zero(ndof); // imposed change in velocity
	uProjected = VectorXd::Zero(ndof);
	
	// set mass array
	setMass();

	nConstraint = VectorXi::Zero(nv);
	nConstraintPrevious = VectorXi::Zero(nv);
}

// This helper function calculates which pairs of nodes are stretching, bending, etc.
// Result: values now exist in bendingNodes, stretchingNodes
void elasticRod::constructorHelperAssignNodes()
{
	// Edges for stretching
	nEdges = 0;
	stretchingNodes.clear();
	// For the tripod: via Drew's notes, we must add the following stretching pairs.
	// (1) everything that's inclusive inside one limb ...for all limbs
	for(int ell=0; ell < numLimbs; ell++)
	{
		std::vector<int> v_nums = limbVtxMap[ell];
		// stop just before the end so we only do edges within this set of indices
		for(int i=0; i < v_nums.size()-1; i++)
		{
			std::array<int, 2> a = {v_nums[i], v_nums[i+1]};
			stretchingNodes.push_back(a);
			nEdges++;
		}
	}
	// (2) manual hack for joint between limbs 0-to-1: connecting at the top-left shoulder joint
	std::array<int, 2> topleft1 = {nvEachLimb-1, nvEachLimb};
	std::array<int, 2> topleft2 = {nvEachLimb, nvEachLimb+1};
	stretchingNodes.push_back(topleft1);
	stretchingNodes.push_back(topleft2);
	nEdges += 2;
	// (3) the joints for all remaining limbs. Starting at limb 2 because we did the hack for 0 and 1.
	for(int ell=2; ell < numLimbs; ell++)
	{
		// Here, we split according to even/odd. Odd are spine, even are legs.
		if(ell % 2 == 0)
		{
			// connecting the (ell-1)-th spine segment to the ell-th leg:
			std::array<int, 2> a1 = {ell*nvEachLimb-1, ell*nvEachLimb};
			std::array<int, 2> a2 = {ell*nvEachLimb, ell*nvEachLimb+1};
			stretchingNodes.push_back(a1);
			stretchingNodes.push_back(a2);
			nEdges += 2;
		}
		else
		{
			// connecting the (ell-1)-th leg to the ell-th spine segment:
			// HERE is where the chain breaks: we connect the previous spine segment to this one, effectively.
			std::array<int, 2> a = {(ell-1)*nvEachLimb, ell*nvEachLimb+1};
			stretchingNodes.push_back(a);
			nEdges++;
		}
	}
	// (4) the triangular feet: see below after bendingNodes, we'll do both at the same time because indexing is difficult at this point

	// Triplets for bending
	nBends = 0;
	bendingNodes.clear();
	// For the tripod: via Drew's notes, we must add the following bending triplets
	// (1) everything that's inclusive inside one limb ...for all limbs
	for(int ell=0; ell < numLimbs; ell++)
	{
		std::vector<int> v_nums = limbVtxMap[ell];
		// For example, if the vertices are [Y, Y+1, Y+2, ..., Z-1, Z],
		// then we want to add [Y, Y+1, Y+2], [Y+1, Y+2, Y+3], ... [Z-2, Z-1, Z]
		// ...here, Y = v_nums[0] and Z = v_nums[v_nums.size()-1]
		for(int i=1; i < v_nums.size()-1; i++)
		{
			std::array<int, 3> a = {v_nums[i-1], v_nums[i], v_nums[i+1]};
			bendingNodes.push_back(a);
			// Store the mapping from the "middle" vertex into its place in kB.
			bendVtxIdxMap[v_nums[i]] = nBends;
			// NOTE that we should only ever change these "within limb" kappaBars, throw an error otherwise!
			nBends++;
		}
	}
	// (2) manual hack for joint between limbs 0-to-1: connecting at the top-left shoulder joint
	std::array<int, 3> topleft1b = {nvEachLimb-2, nvEachLimb-1, nvEachLimb};
	std::array<int, 3> topleft2b = {nvEachLimb-1, nvEachLimb, nvEachLimb+1};
	std::array<int, 3> topleft3b = {nvEachLimb, nvEachLimb+1, nvEachLimb+2};
	bendingNodes.push_back(topleft1b);
	bendingNodes.push_back(topleft2b);
	bendingNodes.push_back(topleft3b);
	nBends += 3;
	// (3) the joints for all remaining limbs. Starting at limb 2 because we did the hack for 0 and 1.
	for(int ell=2; ell < numLimbs; ell++)
	{
		// Here, we split according to even/odd. Odd are spine, even are legs.
		if(ell % 2 == 0)
		{
			// connecting the (ell-1)-th spine segment to the ell-th leg:
			std::array<int, 3> a1 = {ell*nvEachLimb-2, ell*nvEachLimb-1, ell*nvEachLimb};
			std::array<int, 3> a2 = {ell*nvEachLimb-1, ell*nvEachLimb, ell*nvEachLimb+1};
			std::array<int, 3> a3 = {ell*nvEachLimb, ell*nvEachLimb+1, ell*nvEachLimb+2};
			bendingNodes.push_back(a1);
			bendingNodes.push_back(a2);
			bendingNodes.push_back(a3);
			nBends += 3;
		}
		else
		{
			// connecting the (ell-1)-th leg AND the (ell-2)-th spine segment to the ell-th spine segment
			// HERE is where the chain breaks.
			// From spine to spine:
			std::array<int, 3> a1 = {(ell-1)*nvEachLimb-1, (ell-1)*nvEachLimb, ell*nvEachLimb+1};
			// From leg to spine:
			std::array<int, 3> a2 = {(ell-1)*nvEachLimb+1, (ell-1)*nvEachLimb, ell*nvEachLimb+1};
			std::array<int, 3> a3 = {(ell-1)*nvEachLimb, ell*nvEachLimb+1, ell*nvEachLimb+2};
			bendingNodes.push_back(a1);
			bendingNodes.push_back(a2);
			bendingNodes.push_back(a3);
			nBends += 3;
		}
	}

	// (4) The feet. Here we'll do both bending and stretching at the same time.
	// Easiest to index according to limb,
	for(int ell=0; ell < numLimbs; ell++)
	{
		// ...where legs are even limbs
		if( ell % 2 == 0 )
		{
			// Three relevant vertices for stretching: base, heel, toe. All stored as indices here.
			int base_idx;
			int heel_idx;
			int toe_idx;
			// For bending, we also need the vertex just above the base. Call it ankle.
			int ankle_idx;
			// Because of the ordering, we need a different formula for first leg vs. others.
			if( ell == 0 )
			{
				base_idx = 0;
				ankle_idx = 1;
			}
			else
			{
				// recalling that nvEachLimb = "P" in Drew's notes, which is numVertices-1 (as per text file)
				base_idx = (ell+1)*nvEachLimb;
				ankle_idx = base_idx - 1;
			}
			// in either case, the heel and toe are same indexing
			heel_idx = numLimbs*nvEachLimb + ell + 1;			
			toe_idx = numLimbs*nvEachLimb + ell + 2;
			// Stretching pairs: there are three to add, for the triangle.
			std::array<int, 2> bh = {base_idx, heel_idx};
			std::array<int, 2> ht = {heel_idx, toe_idx};
			std::array<int, 2> tb = {toe_idx, base_idx};
			stretchingNodes.push_back(bh);
			stretchingNodes.push_back(ht);
			stretchingNodes.push_back(tb);
			nEdges += 3;
			// Bending triples: there are four to add. Two for the heel/toe, two for the base on each side.
			std::array<int, 3> abh = {ankle_idx, base_idx, heel_idx};
			std::array<int, 3> bht = {base_idx, heel_idx, toe_idx};
			std::array<int, 3> htb = {heel_idx, toe_idx, base_idx};
			std::array<int, 3> tba = {toe_idx, base_idx, ankle_idx};
			bendingNodes.push_back(abh);
			bendingNodes.push_back(bht);
			bendingNodes.push_back(htb);
			bendingNodes.push_back(tba);
			nBends += 4;
		}
	}
}

// Adding a velocity to the rod based on its rigid body state.
void elasticRod::addLinearVelocity(double dxdt, double dydt)
{
	// TO-DO: good idea to add to both u and u0, or just u?
	// ...assuming the same indexing as x:
	for(int i=0; i < nv; i++){
		// "previous"
		u0(2*i) += dxdt;
		u0(2*i+1) += dydt;
		// "current"?
		u(2*i) += dxdt;
		u(2*i+1) += dydt;
	}
}

// Now, taking in and returning arguments reduces the assumptions we're making
VectorXd elasticRod::centerAt(VectorXd nodes_in, double x_center, double y_center)
{
	// We'll loop over the passed-in argument.
	// Since it's a vector, size is scalar. BUT, we assume that nodes_in goes [x0, y0, x1, y1, ...],
	// so it's really *half* the size is number of nodes. Assumes nodes_in is even number.
	int n_in = nodes_in.size()/2;
	// Preallocate the output, will be same dimension as input.
	// To-Do: will performance suffer here? How often is this function called?
	VectorXd nodes_out = VectorXd::Zero(n_in*2);
	// Averaging in both the X and Y directions.
	double comX = 0.0;
	double comY = 0.0;
	// loop over all nodes
	for(int i=0; i < n_in; i++){
		comX += nodes_in(2*i);
		comY += nodes_in(2*i+1);
	}
	// Average to get CoM
	comX = comX / ((double)n_in);
	comY = comY / ((double)n_in);
	// and apply offset to all nodes.
	for(int i=0; i < n_in; i++){
		nodes_out(2*i) = nodes_in(2*i) - comX + x_center;
		nodes_out(2*i+1) = nodes_in(2*i+1) - comY + y_center;
		// x(2*i) += rb_state_0(0) - comX;
		// x(2*i+1) += rb_state_0(1) - comY;
	}
	return nodes_out;
}

void elasticRod::constructorHelperAddDropOffset()
{
	//
	// Apply offset such that the robot's minimum point is on ground (in Y)
	//
	// delta_l = totalLen / ((double) nCircular - 1);
	double totalLen = delta_l * (nv - 1);
	// a fudge factor? Why not just 0????
	double minY = totalLen * 10;
	// Find the y-coordinate where the robot would be contacting the boundary
	for (int i=0; i < nv; i++)
	{
		double xPos = x(2*i);
		double bdPos = boundary(xPos);
		if ( (x(2*i+1) - bdPos) < minY)
		{
			minY = x(2*i+1) - bdPos;
		}
	}
	
	// Add the offset.
	// Note, this only acts on the vertex y-positions, which are 2*i+1
	for (int i=0; i < nv; i++)
	{
		x(2*i+1) = x(2*i+1) - minY + dropHeight;	
	}
}

// The rod setup function is called AFTER construction, for functionality that
// can't go in the constructor.
void elasticRod::setup()
{
	//
	// BENDING
	// Compute the reference lengths for each bending nodes
	//
	e0e1.clear();
	l_k_bending.clear();
	for (int i=0; i < nBends; i++)
	{
		Vector2d p0 = getVertex(bendingNodes[i][0]);
		Vector2d p  = getVertex(bendingNodes[i][1]);
		Vector2d p1 = getVertex(bendingNodes[i][2]);
		double e0e1L = ((p-p0).norm()) * ((p1-p).norm());
		e0e1.push_back(e0e1L);
		
		double bendingLen = 0.5 * ( (p-p0).norm() + (p1-p).norm() );
		l_k_bending.push_back(bendingLen);
	}
		
	//
	// STRETCHING
	// Compute the reference lengths for each edge
	//
	l_k_stretching.clear();
	for (int i=0; i < nEdges; i++)
	{
		Vector2d p0 = getVertex(stretchingNodes[i][0]);
		Vector2d p  = getVertex(stretchingNodes[i][1]);
		double stretchLen = (p-p0).norm();
		l_k_stretching.push_back(stretchLen);
		
		if (stretchLen > 1.5 * delta_l)
		{
			// TO-DO: throw an error here that's more informative
			cout << "Error in stretching length in initial pose\n";
			// exit(1);
		}		
	}

	// compute natural curvature.
	kappaBar = VectorXd::Zero(nBends);
	// adjusts bendingNodes for use elsewhere (e.g. elasticBendingForce)
	// TO-DO: do we need/want to call this nowadays???
	// fixBendingData();
	// initialize kappaBar
	computeKappaBar();
	kappaBarOriginal = kappaBar; // this stays fixed while kappaBar changes upon actuation
	
	// compute elastic stiffness
	computeElasticStiffness();
	
	// values at the beginning of time step
	x0 = x;
	
	return;
}

void elasticRod::setVertexBoundaryCondition(Vector2d position, int i)
{
	W_Nodes[i].setZero();

	x(2*i) = position(0);
	x(2*i+1) = position(1);

	dVReqd(2*i) = 2 * (position(0) - x0(2*i)) / dt - 2 * u0(2*i);
	dVReqd(2*i + 1) = 2 * (position(1) - x0(2*i + 1)) / dt - 2 * u0(2*i + 1);

	nConstraint(i) = 2;
}

void elasticRod::setOneVertexConstraintCurve(Vector2d position, int k)
{
	// get point norm
	double xPos = position(0);
	// double yPos = position(1);

	double thetaWall = thetaNormal(xPos);
	Vector2d nWall( cos(thetaWall), sin(thetaWall) );

	x(2*k) = position(0);
	x(2*k+1) = position(1);

	W_Nodes[k] = Id2 / massArray(2*k);
	W_Nodes[k] = (Id2 - nWall * nWall.transpose()) / massArray(2*k);

	Vector2d u0Point = Vector2d(u0(2*k), u0(2*k+1));
	Vector2d x0Point = Vector2d(x0(2*k), x0(2*k+1));

	Vector2d vAlongNormal = nWall * u0Point.dot(nWall);

	dVReqd(2*k) = 2 * (position(0) - x0Point(0)) / dt - 2 * vAlongNormal(0);
	dVReqd(2*k+1) = 2 * (position(1) - x0Point(1)) / dt - 2 * vAlongNormal(1);

	nConstraint(k) = 1;
}

void elasticRod::releaseVertexBoundaryCondition(int i)
{
	W_Nodes[i] = Id2 / massArray(2*i);
	
	dVReqd(2*i) = 0;
	dVReqd(2*i+1) = 0;

	nConstraint(i) = 0;
}

elasticRod::~elasticRod()
{
	;
}

void elasticRod::setMass()
{
	massArray = VectorXd::Zero(ndof);	
	for (int i=0; i<nv; i++)
	{
		dm = delta_l * crossSectionalArea * rho;

		for (int k=0; k <2; k++)
		{
			massArray(2*i+k) = dm;
		}
	}
	
	for (int i=0; i<nv; i++)
	{
		Matrix2d Wlocal = Id2 / massArray(2*i);
		W_Nodes.push_back(Wlocal);
	}
}

Vector2d elasticRod::getVertex(int k)
{
	return Vector2d(x(2*k), x(2*k+1));
}

Vector2d elasticRod::getVelocity(int k)
{
	Vector2d uCurrent;
	uCurrent(0) = u(2*k+0);
	uCurrent(1) = u(2*k+1);

	return uCurrent;
}

void elasticRod::fixBendingData()
{
    for(int i=0; i<nBends; i++)
    {
		Vector2d Xkm1 = getVertex( bendingNodes[i][0] );
		Vector2d Xk = getVertex( bendingNodes[i][1] );
		Vector2d Xkp1 = getVertex( bendingNodes[i][2] );
		double kappaT = computeCurvature(e0e1[i], Xkm1[0], Xkm1[1], Xk[0], Xk[1], Xkp1[0], Xkp1[1]);
		if (kappaT < 0)
		{
			std::array<int, 3> a = {bendingNodes[i][2],  bendingNodes[i][1], bendingNodes[i][0]};
			bendingNodes[i] = a;
			cout << "Bending data fixed at i=" << i << endl;
		}
    }
}

void elasticRod::computeKappaBar()
{
    for(int i=0; i<nBends; i++)
    {
		Vector2d Xkm1 = getVertex( bendingNodes[i][0] );
		Vector2d Xk = getVertex( bendingNodes[i][1] );
		Vector2d Xkp1 = getVertex( bendingNodes[i][2] );
		// Now, we make it explicit: this is for kappaBar only! kappa is not updated.
		kappaBar(i) = computeCurvature(e0e1[i], Xkm1[0], Xkm1[1], Xk[0], Xk[1], Xkp1[0], Xkp1[1]);
    }
}

void elasticRod::computeElasticStiffness()
{
	EI_scalar = (youngM*1*rodRadius*rodRadius*rodRadius)/12;
	EA =  youngM*crossSectionalArea;
	
	EI = VectorXd::Zero(nBends);
	for (int i=0; i < nBends; i++) 
	{
		EI(i) = EI_scalar;
	}
}

void elasticRod::updateNewtonX(double *dc)
{
	// update change in velocity
	for (int c=0; c < ndof; c++)
	{
		dV[c] -= dc[c];
		u[c] = u0[c] + dV[c];
	}
	
	// update coordinates
	for (int c=0; c < ndof; c++)
	{
		x[c] = x0[c] + (u0[c] + u[c]) * dt / 2;
	}
}

void elasticRod::updateTimeStep()
{
	for (int i = 0; i < nv; i++)
	{
		if (nConstraint(i) == 1)
		{
			double xPos = x(2*i);

			double thetaWall = thetaNormal(xPos);
			Vector2d nWall( cos(thetaWall), sin(thetaWall) );

			Vector2d uPoint;
			uPoint(0) = u(2*i);
			uPoint(1) = u(2*i+1);

			Vector2d vAlongNormal = nWall * uPoint.dot(nWall);

			u(2*i) = u(2*i) - vAlongNormal(0);
			u(2*i+1) = u(2*i+1) - vAlongNormal(1);
		}

		if (nConstraint(i) == 2)
		{
			u(2*i) = 0;
			u(2*i+1) = 0;
		}
	}

	// update x
	x0 = x;

	// update u
	u0 = u;

	// set dVReqd zero
	dVReqd = VectorXd::Zero(ndof);

	nConstraintPrevious = nConstraint;
}

void elasticRod::updateGuess()
{
	for (int c=0; c < ndof; c++)
	{
		dV[c] =  0.0;
		
		{
			x[c] =  x0[c] + u0[c] * dt;
		}
	}
}

// TO-DO: THIS DOES NOT USE e0e1, WHICH MAY BE ENTIRELY UNUSED IN THIS SIMULATION, NOT USED in elasticBendingForce EITHER!!!
double elasticRod::computeCurvature(double e0e1, double xkm1, double ykm1, double xk, double yk, double xkp1, double ykp1)
{
	return 0.2e1 * tan(atan((-(xk - xkm1) * (ykp1 - yk) + (yk - ykm1) * (xkp1 - xk)) / ((xk - xkm1) * (xkp1 - xk) + (yk - ykm1) * (ykp1 - yk))) / 0.2e1);
}

double elasticRod::thetaNormal(double xPos)
{
	if (boundaryType==1) 
	{
		return M_PI/2;
	}
	else if (boundaryType==2)
	{
		return atan( - Aboundary * 2.0 * M_PI / lambdaBoundary * sin( 2.0 * M_PI / lambdaBoundary * xPos ) ) + M_PI/2.0; // robot is at the max-point
	}
	else if (boundaryType==3)
	{
		return atan( Aboundary * 2.0 * M_PI / lambdaBoundary * sin( 2.0 * M_PI / lambdaBoundary * xPos ) ) + M_PI/2.0; // robot is at the min-point
	}
	else if (boundaryType == 4)
	{
		// Slope, line: y = mx + b, with {m, b} = {-0.1..., 0}.
		double m = -0.16;
		// For a line, angle from x-axis is arctan(m), so normal is
		return M_PI/2.0 - atan(m);
	}
	else
	{
		throw std::runtime_error("Error in boundary definition");
	}
}

double elasticRod::boundary(double xPos)
{
	if (boundaryType==1) 
	{
		return tan(thetaNormal(xPos) + M_PI/2);
	}
	else if (boundaryType==2)
	{ 
		return Aboundary * cos ( 2.0 * M_PI / lambdaBoundary * xPos );
	}
	else if (boundaryType==3) 
	{
		return (- Aboundary * cos ( 2.0 * M_PI / lambdaBoundary * xPos ) );
	}
	else if (boundaryType == 4)
	{
		// Slope, line: y = mx + b, with {m, b} =  {-0.1..., 0}.
		double m = -0.16;
		return xPos * m;
	}
	else
	{
		throw std::runtime_error("Error in boundary definition");
	}
}

// Center of mass is weight avg of all vertex positions.
// TO-DO: THIS ASSUMES SAME MASS DISTRIBUTION AT ALL VERTICES
Vector2d elasticRod::getCOM()
{
	Vector2d Pos;
	Pos.setZero(2);

	for (int j=0; j < nv; j++)
	{
		Vector2d p = getVertex(j);
		Pos+= p;
	}
	return Pos / nv;
}

// Velocity of CoM is weighted average of all vertex velocities
// TO-DO: THIS ASSUMES SAME MASS DISTRIBUTION AT ALL VERTICES
Vector2d elasticRod::getVelocityCOM()
{
	Vector2d com_vel;
	com_vel.setZero(2);

	for (int j=0; j < nv; j++)
	{
		Vector2d vel_j = getVelocity(j);
		com_vel += vel_j;
	}
	return com_vel / nv;
}

std::vector<int> elasticRod::getVertexNumsForLimb(int limb_num)
{
	// if improper limb number, throw an error instead to alert the user to their problem
	if((limb_num < 0) || (limb_num >= numLimbs)) {
		throw std::runtime_error("Error! Invalid limb number in elasticRod::getVertexNumForLimb.");
	}
	// This function now returns nodes *only inside* the limb, not including joints.
	// For limb L, with P = nvEachNode, the list should be [LP+1, LP+2, ..., LP+P-1], which contains P-1 indices
	std::vector<int> indices(nvEachLimb-1);
	int startNode = limb_num*nvEachLimb+1;
	int endNode = limb_num*nvEachLimb + nvEachLimb - 1;
	
	// will populate from startNode to startNode + nvEachLimb
	std::iota(indices.begin(), indices.end(), startNode);

	// HOWEVER, if we're working with foot limbs (which are even), must append the foot index.
	if(limb_num % 2 == 0)
	{
		// For the 0-th limb, the foot is index 0, inserted at the beginning of the list
		if(limb_num == 0)
		{
			indices.insert(indices.begin(), 0);
		}
		else
		{
			// For all others, the foot gets added to the end.
			indices.push_back((limb_num+1) * nvEachLimb);
		}
	}

	return indices;
}

// Return a representative vertex number for a limb. Callers are mainly concerned 
// with macroscropic limb properties here (example, controllers and loggers with kappa bar.)
int elasticRod::getAnyVertexNumForLimb(int limb_num)
{
	// Get the list of nodes for this limb
	std::vector<int> limb_verts = limbVtxMap[limb_num];
	// Would probably be fine to do the first node within the rod's verticesForLimb, but... I'm not confident
	// about that, with the wraparound nodes (-1 -> 90) and the overlap, so choose the second vertex number from the list
	// for each limb.
	if(limb_verts.size() < 2){
		throw std::runtime_error("Error - you need more than one vertex in a limb!");
	}
	// otherwise, the second vertex for this limb:
	return limb_verts[1];
}

double elasticRod::getAnyKbForLimb(int limb_num)
{
	// Return the kappaBar for the second vertex (chosen arbitrarily) within this limb.
	return kappaBar[ bendVtxIdxMap[limbVtxMap[limb_num][1]] ];
}

double elasticRod::getAnyKbOrigForLimb(int limb_num)
{
	// Return the kappaBarOriginal for the second vertex (chosen arbitrarily) within this limb.
	return kappaBarOriginal[ bendVtxIdxMap[limbVtxMap[limb_num][1]] ];
}

void elasticRod::setKbForLimb(int limb_num, double kB)
{
	// Iterate over all the "inner" vertices within this limb
	vector<int>::iterator v_ptr;
	for(v_ptr = limbVtxMap[limb_num].begin(); v_ptr < limbVtxMap[limb_num].end(); v_ptr++)
	{
		kappaBar[bendVtxIdxMap[*v_ptr]] = kB;
		// Debugging
		// std::cout << "Set kB for limb " << limb_num << ", vertex " << *v_ptr << ", to " << kB << std::endl;
	}
}

void elasticRod::setEIForLimb(int limb_num, double EI_next)
{
	// Iterate over all the "inner" vertices within this limb
	vector<int>::iterator v_ptr;
	for(v_ptr = limbVtxMap[limb_num].begin(); v_ptr < limbVtxMap[limb_num].end(); v_ptr++)
	{
		EI[bendVtxIdxMap[*v_ptr]] = EI_next;
		// Debugging
		// std::cout << "Set EI for limb " << limb_num << ", vertex " << *v_ptr << ", to " << EI_next << std::endl;
	}
}

// Return a representation of the rod "rotation" around its
// center of mass. 
// Note, we're doing counterclockwise = positive
double elasticRod::getRBRotation()
{
	// Procedure:
	// (1) translate the current node position vectors to the origin
	VectorXd x_curr = centerAt(x, 0.0, 0.0);
	// (2) Calculate the angle between the i-th node's reference pose and its current pose.
	double theta = 0.0;
	// placeholders for reorganizing nodes
	Vector2d x_i, x_ref_i;
	// and one where we'll need to check for +/- correction of arc cos
	Matrix2d x_check_mat;
	double x_check_det;
	int reverse_dir; // will be 1 if negative or 0 if positive (signed bit is 1 if neg).
	// preventing the nan returned by acos if argument is greater than 1.0
	double theta_cos_i;
	// we actually need to track each individual theta because of the +/- correction
	double theta_i;
	// x and x_ref are of size ndof, but indexing should be half that, which is nv
	for(int i=0; i < nv; i++){
		// The vector that's the i-th point is
		x_i << x_curr(2*i), x_curr(2*i+1);
		x_ref_i << x_ref(2*i), x_ref(2*i+1);
		// From some hand calculations, the acute angle between these two vectors is
		// arccos( x \cdot x_ref / (||x|| ||x_ref||))
		theta_cos_i = x_i.dot(x_ref_i) / (x_i.norm() * x_ref_i.norm());
		// Sometimes, numerical issues cause theta_cos_i to be slightly greater than 1.
		if(theta_cos_i > 1.0){ 
			theta_cos_i = 1.0;
		}
		theta_i = std::acos( theta_cos_i ) * 180/M_PI;
		// However, acos has a range from 0 to 180, while we need -180 to 180.
		// To check which "direction", we're looking for the cross product to be pointing one way or another.
		// In 2D, this is the determinant, a scalar value which we can check +/-.
		x_check_mat << x_i, x_ref_i;
		x_check_det = x_check_mat.determinant();
		// NOTE! signbit returns 1 if NEGATIVE!
		reverse_dir = signbit(x_check_det);
		// std::cout << "reverse_dir = " << reverse_dir << std::endl;
		// If counterclockwise is positive, then clockwise should be negative...
		if(reverse_dir == 0){
			// std::cout << "Reversing theta_i..." << std::endl;
			// theta_i += -360.0;
			theta_i = -theta_i;
		}
		// std::cout << "theta_i = " << theta_i << std::endl;
		theta += theta_i;
	}
	// average
	theta = theta / ((double)nv);
	return theta;
}

// Get the angular velocity of this system of particles around its center of mass.
double elasticRod::getVelocityAngular()
{
	// Procedure: 
	// (1) shift the rod to the origin (omega around centroid!)
	VectorXd x_shifted = centerAt(x, 0.0, 0.0);
	// (2) For each node, calc its angular vel (a scalar)
	double omega = 0.0;
	double omega_i;
	Vector2d x_shifted_i;
	// will need a placeholder matrix
	Matrix2d x_shifted_v;
	for(int i=0; i < nv; i++){
		// placeholder for better linear algebra with Eigen
		x_shifted_i << x_shifted(2*i), x_shifted(2*i+1);
		// w = r x v / ||r||^2 in 3D, which here, is just a determinant, det[r, v] / ||r||^2.
		// here r is in x_shifted, and velocity is in u. Needs to be columns.
		x_shifted_v << x_shifted_i(0), u(2*i),
					   x_shifted_i(1), u(2*i+1);
		// debugging.
		// std::cout << "[x_i, u_i] = " << x_shifted_v << std::endl;
		// result should be scalar. Normalized by radial distance squared
		omega_i = (x_shifted_v.determinant()) / (pow(x_shifted_i.norm(),2));
		// debugging.
		// std::cout << "omega_i = " << omega_i << std::endl;
		omega += omega_i;
	}
	// (3) Average them out. 
	// std::cout << "Omega sum = " << omega << std::endl;
	omega = omega / ((double)nv);
	// (4) For consistency with getRBRotation, report angular velocity in degrees/sec.
	omega = omega * (180/M_PI);
	// debugging
	// std::cout << "Omega = " << omega << std::endl;
	return omega;
}

// Rotate the rod around its CoM.
void elasticRod::rotateRod(double theta)
{
	// Procedure:
	// (i) change units of theta into radians for later.
	theta = theta * M_PI/180.0;
	// (1) translate the rod to the origin
	VectorXd x_shifted = centerAt(x, 0.0, 0.0);
	// (2) apply rotation to all points
	// the rotation matrix can be reused for all points
	Vector2d x_i;
	Matrix2d rot_mat;
	rot_mat << cos(theta), -sin(theta),
			   sin(theta), cos(theta);
	for(int i=0; i < nv; i++){
		// reconstruct the i-th position vector
		x_i << x_shifted(2*i), x_shifted(2*i+1);
		// rotate around origin
		x_i = rot_mat * x_i;
		// reinsert
		x_shifted(2*i) = x_i(0);
		x_shifted(2*i+1) = x_i(1);
	}
	// (3) translate the rod back to where it was before
	VectorXd rod_com = getCOM();
	x = centerAt(x_shifted, rod_com(0), rod_com(1));
}

// Add an angular velocity: each node around the centroid. Input is deg/sec, consistent with getVelocityAngular
void elasticRod::addAngularVelocity(double omega)
{
	// Generally:
	// u_i += omega * (x_i - CoM)^\perp
	// (i) change units back to rad
	omega = omega * M_PI / 180.0;
	// (1) Shift all nodes 
	VectorXd x_shifted = centerAt(x, 0.0, 0.0);
	// (2) Apply to each node
	for(int i=0; i < nv; i++){
		// Noting that (in the plane), if r = [x; y], then r^\perp = [-y; x],
		// we can do this element-wise.
		// x-velocity: omega * -y
		u(2*i) += - omega * x_shifted(2*i+1);
		// y-velocity: omega * x
		u(2*i+1) += omega * x_shifted(2*i);
	}
	// that's all, nothing to return.
}