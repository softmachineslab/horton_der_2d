#include "world.h"

world::world()
{
	;
}

world::world(setInput &m_inputData)
{
	// Since not passing in the initial pose to this constructor, init it here.
	// We are hard-coding the number of states to be 6, as in x y theta and dot x, dot y, dot theta.
	rb_state_0 = VectorXd::Zero(6);
	// call the helper.
	worldConstructorHelper(m_inputData);
}

// if a rigid body state is passed in, record it.
world::world(setInput &m_inputData, VectorXd m_rb_state_0) : rb_state_0(m_rb_state_0)
{
	// call the helper.
	worldConstructorHelper(m_inputData);
}

// a helper to the constructor offloads all the SetInput initialization.
void world::worldConstructorHelper(setInput &m_inputData)
{
	// hello! depending on verbosity.
	if( verbosity >= 2){
		std::cout << "Creating the world..." << std::endl;
	}

	// At this point, we assume what's in inputData.
	render = m_inputData.GetBoolOpt("render");				// boolean

	// Physical parameters
	limbLength = m_inputData.GetScalarOpt("limb-length");	// meter
    gVector = m_inputData.GetVecOpt("gVector");             // m/s^2
    maxIter = m_inputData.GetIntOpt("maxIter");             // maximum number of iterations
    rodRadius = m_inputData.GetScalarOpt("rodRadius");      // meter
	numVertices = m_inputData.GetIntOpt("numVertices");     // int_num, this is PER LIMB inclusive of the node to join together limbs

	// stored for passing in to elasticRod
	inputNv = numVertices;
	// for our purposes with geometry, etc.
	nvEachLimb = inputNv-1;
	nvEachLimbNoJoints = inputNv-2;

	youngM = m_inputData.GetScalarOpt("youngM");            // Pa
	Poisson = m_inputData.GetScalarOpt("Poisson");          // dimensionless
	deltaTime = m_inputData.GetScalarOpt("deltaTime");      // seconds
	totalTime= m_inputData.GetScalarOpt("totalTime");       // seconds
	tol = m_inputData.GetScalarOpt("tol");                  // small number like 10e-7
	stol = m_inputData.GetScalarOpt("stol");				// small number, e.g. 0.1%
	density = m_inputData.GetScalarOpt("density");          // kg/m^3
	viscositya = m_inputData.GetScalarOpt("viscositya");    // viscosity a
	viscosityb = m_inputData.GetScalarOpt("viscosityb");    // viscosity b
	dropHeight = m_inputData.GetScalarOpt("drop-height");	// height above ground
	numLimbs = m_inputData.GetIntOpt("num-limbs");			// number of limbs
	dynamicFric = m_inputData.GetScalarOpt("dynamic-friction"); // dynamic friction coefficient
	boundaryType = m_inputData.GetIntOpt("boundary-type");	// 1=plane, 2=max point, 3=min point, 4=...0
	enableAutoDrop = m_inputData.GetBoolOpt("enable-auto-drop"); // enabling or disabling the auto alignment mechanism for t=0.
	compression_kB_max = m_inputData.GetScalarOpt("compression-kB-max"); // For the compression SMA model, manually specify kBmax.
	
	shearM = youngM/(2.0*(1.0+Poisson));					// shear modulus
    delta_l = limbLength / (numVertices - 1);
	RodLength = limbLength * numLimbs;						// length of the robot, used for characteristic force and graphics scaling

	// These constants given in the NatComm paper (ratios of min to max kappaBar and EI.)
	nTimeCurvature = 0.21; // was 0.21 ...sec?
	nTimeStiffness = 2.68;
	tau = 3.4;
	Tinitial = 1.4;

	// For the robot with triangular feet,
	footLength = limbLength * m_inputData.GetScalarOpt("foot-ratio");
}

world::~world()
{
	;
}

bool world::isRender()
{
	return render;
}

void world::setRodStepper()
{
	// Find out the tolerance, e.g. how small is enough?
	double dm = limbLength * M_PI * pow(rodRadius, 2.0) * density / ((double) numVertices - 1);
	characteristicForce = M_PI * pow(rodRadius ,4)/4.0 * youngM / pow(RodLength, 2);
	forceTol = tol * characteristicForce / dm;
	
	// Set up the initial configuration
	// For the rolling-star-like closed chain:
	// rodGeometryChain();	
	// For the walker (tripod or otherwise):
	rodGeometryWalker();

	// Add one rod.
	addRod();
	// Now that the rod has been setup, create the SMAs also.
	addSMAs();
	
	// Nstep = totalTime/deltaTime;
	timeStep = 0;
	currentTime = 0.0;
	
	// Allocate space
	rod->sumForces = VectorXd::Zero(rod->ndof);
}

Vector2d world::computeCenterMass()
{
	Vector2d Pos;
	Pos.setZero(2);

	for (int j=0; j < rod->nv; j++)
	{
		Vector2d p = rod->getVertex(j);
		Pos+= p;
	}
	return Pos / rod->nv;
}

// Setup geometry for the rolling-star-like closed chain
void world::rodGeometryChain()
{
	vertices = MatrixXd( numLimbs * (numVertices-1), 2);
    double turningAngle = 2.0 * M_PI / ((double) numLimbs);
    int totalvertexNo = 0;

	// The start and end nodes for each limb.
	Vector2d xL = Vector2d::Zero(2);
	Vector2d xLp1 = Vector2d::Zero(2);
	// For reference, vector of limblength along horiz axis
	Vector2d horizs = Vector2d::Zero(2);
	horizs(0) = limbLength;
	// Discretized length along the straight limb
	double ds = limbLength / (numVertices-1);
    
    for (int iR = 0; iR < numLimbs; iR++)
    {
		// Update the start and end nodes for this limb.
		// Manually specify the first two before recurrence
		if(iR == 0){
			// xL = [0, 0]
			// xLp1 = [s, 0]
			xLp1 = horizs;
		}
		else{
			// New first node is last node from previous
			xL = xLp1;
			// New last node is xLp1 = Rz(L*turningangle) * [s, 0] + xL
			xLp1 = Rotation2D<double>(iR*turningAngle) * horizs + xL;
		}
		// Get the unit vector along this limb
		Vector2d dirvec = (xLp1-xL);
		Vector2d uL = dirvec / dirvec.norm();
		// counting from 1 means we exclude the "first" node for this limb (chain closure takes care of this)
        for (int n = 1; n < numVertices; n++)
        {
			// Each node is a translation along the unit vector, translated w.r.t. the first vertex of this limb.
			Vector2d xLn = n*ds*uL + xL;
			// inserted element-wise, TO-DO eigen indexing here
			vertices(totalvertexNo, 0) = xLn(0);
			vertices(totalvertexNo, 1) = xLn(1);
			totalvertexNo++;
        }
    }
	// TO-DO: this overrides the option passed in! Was num vert per limb, now is total num vert! BAD!
    numVertices = totalvertexNo;
}

// Setup geometry for the tripod (or multi-pod) walker.
void world::rodGeometryWalker()
{
	// With the new node numbering, we've got a total of (numLimbs*nvEachLimb) + 1 vertices to add for the body,
	// then an additional 2 nodes for each foot which is ((numLimbs+1)/2) feet times two verts per foot = numLimbs+1
	// total of (numLimbs*nvEachLimb) + 1 + numLimbs+1 = (numLimbs*nvEachLimb) + numLimbs + 2
	// example, with L=5 and numVertices=7, there are 31 nodes in the body (indexed 0 to 30), with the first limb having one extra in comparison to the others,
	// and a total of 37 nodes for the whole robot.
	// vertices = MatrixXd( numLimbs * nvEachLimb + 1, 2);
	int numverts_estimate = (numLimbs*nvEachLimb) + numLimbs + 2;
	// DEBUGGING
	// std::cout << "Estimated number of vertices in Tripod Walker: " << numverts_estimate << std::endl;
	vertices = MatrixXd( numverts_estimate, 2);
    int totalvertexNo = 0;
	// The start and end nodes for each limb.
	Vector2d xL = Vector2d::Zero(2);
	Vector2d xLp1 = Vector2d::Zero(2);
	// For reference, vector of limblength along horiz axis
	Vector2d horizs = Vector2d::Zero(2);
	horizs(0) = limbLength;
	// Discretized length along the straight limb.
	double ds = limbLength / nvEachLimb;
	// The 0th limb contains a vertex at (0,0) which must be added manually.
	vertices(totalvertexNo, 0) = 0.0;
	vertices(totalvertexNo, 1) = 0.0;
	totalvertexNo++;
	// Add each limb in sequence...
	for(int ell=0; ell < numLimbs; ell++)
	{
		// ...with the 0-th limb having a hard-coded correction for its endpoint (goes vertically in +y)
		if(ell==0)
		{
			xLp1 = Rotation2D<double>(M_PI_2) * horizs;
		}
		else if(ell==1)
		{
			// ...as well as a hard-coded correction for limb 1, which doesn't need the weird offset for 3, 5, etc.
			xLp1 = horizs + xL;
		}
		else
		{
			// Here, we split according to even/odd. Odd are spine, even are legs.
			if(ell % 2 == 0)
			{
				// Even: leg points down from the spine's last node
				xLp1 = Rotation2D<double>(-M_PI_2) * horizs + xL;
			}
			else
			{
				// Odd: spine points out from the previous spine segment's last node,
				// which is back "up" from the current location. Easiest to reset the starting point here
				xL = xL + Rotation2D<double>(M_PI_2) * horizs;
				xLp1 = horizs + xL;
			}	
		}
		// Then just iterate along this line
		// Get the unit vector along this limb
		Vector2d dirvec = (xLp1-xL);
		Vector2d uL = dirvec / dirvec.norm();
		// counting from 1 means we exclude the "first" node for this limb
		Vector2d xLn;
		// SEE BELOW: this counts to second-to-last vertex only
        for (int n = 1; n < inputNv-1; n++)
        {
			// Each node is a translation along the unit vector, translated w.r.t. the first vertex of this limb.
			xLn = n*ds*uL + xL;
			// inserted element-wise, TO-DO eigen indexing here
			vertices(totalvertexNo, 0) = xLn(0);
			vertices(totalvertexNo, 1) = xLn(1);
			totalvertexNo++;
        }
		// HACK / BUG FIX: Since the 90 degree bends have a bug in the physics,
		// adjust the last vertex position so there are no 90 degree bends.
		double lastv_rot;
		// Depending on which leg (only do this for leg 0 and the spines, which are odd numbers)
		if( (ell == 0) || (ell % 2 == 1) )
		{
			// rotate the last translation by...
			lastv_rot = -20.0 * M_PI/180;
		}
		else
		{
			lastv_rot = 0.0;
		}

		Vector2d trans = ds*uL;
		// translated last vertex + translation up to second-to-last + start
		xLn = Rotation2D<double>(lastv_rot)*trans + (inputNv-2)*ds*uL + xL;
		// inserted element-wise, TO-DO eigen indexing here
		vertices(totalvertexNo, 0) = xLn(0);
		vertices(totalvertexNo, 1) = xLn(1);
		totalvertexNo++;
		// The last node of this limb is therefore the "first" node of the next limb
		// xL = xLn;
		// Actually, now with the hack to remove 90 degree joints, we need to take xL as if the robot was indeed rectangular.
		// Note that this xL is NOT added to vertices, it just serves as a placeholder when incrementing ell.
		xL = (inputNv-1)*ds*uL + xL;
	}
	// Finally, add a foot to each leg.
	// The height offset due to the feet will be 
	double foot_h = footLength * sqrt(3.0)/2.0;
	// adjust all the current vertices in the Y direction by this amount
	for(int i=0; i < totalvertexNo; i++)
	{
		vertices(i, 1) += foot_h;
	}
	// then add the feet
	for(int ell=0; ell < numLimbs; ell++)
	{
		// legs are even limbs
		if( ell % 2 == 0 )
		{
			// current foot = node touching the ground at this limb
			Vector2d foot_ell = Vector2d::Zero(2);
			// we have a special case for the 0-th limb, which is vertex 0
			if( ell == 0 )
			{
				foot_ell(0) = vertices(0, 0);
				foot_ell(1) = vertices(0, 1);
			}
			else
			{
				// the tip index is ell+1 * number vertices
				foot_ell(0) = vertices( (ell+1)*(inputNv-1), 0);
				foot_ell(1) = vertices( (ell+1)*(inputNv-1), 1);
			}
			// debugging
			// std::cout << "Leg tip for limb " << ell << " is " << std::endl << foot_ell << std::endl;

			// Now, the coordinates of the two lower points of this foot's triangle are
			Vector2d f_ell_a;
			Vector2d f_ell_b;
			// footx - length/2, footy - height
			f_ell_a << foot_ell(0) - footLength/2.0, foot_ell(1) - foot_h; 
			// footx + length/2, footy - height
			f_ell_b << foot_ell(0) + footLength/2.0, foot_ell(1) - foot_h;
			// add them both.
			vertices(totalvertexNo, 0) = f_ell_a(0);
			vertices(totalvertexNo, 1) = f_ell_a(1);
			totalvertexNo++;
			vertices(totalvertexNo, 0) = f_ell_b(0);
			vertices(totalvertexNo, 1) = f_ell_b(1);
			totalvertexNo++;
		}
	}
	// DEBUGGING
	// std::cout << "Final number of vertices in Tripod Walker: " << totalvertexNo << std::endl;

	// TO-DO: this overrides the option passed in! Was num vert per limb, now is total num vert! BAD!
    numVertices = totalvertexNo;
}

void world::addRod()
{
	// Create the rod 
	// double Aboundary = 0.01;
	// double Aboundary = 0.1;
	// double lambdaBoundary = 0.5;

	// NOTE: THIS APPEARS TO OVERRIDE INPUT!!!
	double Aboundary = 0.0065; // for correct simulation
	double lambdaBoundary = 0.2;
	
	// Now, with the rigid body initialization passed in:
	// rod = new elasticRod(vertices, density, rodRadius, deltaTime,
	rod = make_shared<elasticRod>(vertices, density, rodRadius, deltaTime,
		youngM, shearM, dropHeight, Aboundary, lambdaBoundary,
		numLimbs, limbLength, boundaryType, nvEachLimb, enableAutoDrop, rb_state_0);


	// Make sure the "connectivity" is implemented before setup
	
	// setup the rod so that all the relevant variables are populated
	rod->setup();

	// set up the time stepper
	// stepper = new timeStepper(*rod);
	stepper = make_shared<timeStepper>(rod);
	totalForce = stepper->getForce();

	// declare the forces
	m_stretchForce = make_unique<elasticStretchingForce>( rod, stepper );
	m_bendingForce = make_unique<elasticBendingForce>( rod, stepper );
	m_inertialForce = make_unique<inertialForce>( rod, stepper );
	m_gravityForce = make_unique<externalGravityForce>( rod, stepper, Vector2d(gVector(0), gVector(1)) );
	m_dampingForce = make_unique<dampingForce>( rod, stepper, viscositya, viscosityb);
	m_externalContactForce = make_unique<externalContactForce>( rod, stepper, dynamicFric );
	
	// Allocate every thing to prepare for the first iteration
	rod->updateTimeStep();
	
	//
	// Fire up PARDISO and reserve space for all the forces
	//
	m_stretchForce->reserveJacobianSpace();
	m_bendingForce->reserveJacobianSpace();
	m_inertialForce->reserveJacobianSpace();
	m_dampingForce->reserveJacobianSpace();
	// Gravity doesn't need Jacobian
	stepper->first_time_PARDISO();

	ifAddDynamicFriction = VectorXi::Zero(rod->nv);

	// TO-DO: looks like this block of code isn't used anymore...
	double maxY = -100;
	double minY = 100;

	for (int i = 0; i < rod->nv; i++)
	{
		Vector2d xCurrent = rod->getVertex(i);

		if (xCurrent(1) < minY)
		{
			minY = xCurrent(1);
		}

		if (xCurrent(1) > maxY)
		{
			maxY = xCurrent(1);
		}
	}
}

void world::addSMAs()
{
	// Create the SMA based on the rod parameters.
	for(std::size_t i=0; i < numLimbs; i++){
		// WE ARE NOW MAKING TWO SMAs PER LIMB: one forward and one backwards.

		// Debugging the compression SMA: hard-code a new kB max.
		// Positive is bending "to the right" along the direction vector between nodes, negative is "to the left"
		// This is a parameter passed from the options file now.
		// double kB_max = 0.5;
		// the EI_scalar variable is really "EI0 for all vertices"
		double EI_max = nTimeStiffness * rod->EI_scalar;
		// Hard-coded the etime = 0.05. TO-DO: MAKE THIS A PARAMETER???
		double act_Kb_slope = (compression_kB_max - rod->getAnyKbOrigForLimb(i))/0.05;
		double act_EI_slope = (EI_max - rod->EI_scalar)/0.05;
		// Construct a new shapeMemoryAlloy and add it to our vector at the same time
		// With the compression SMA, the constructor will be
		// (kB0, EI0, kB_sat, nTimeStiffness, etc)
		// Rightward bending:
		sma.push_back(make_shared<shapeMemoryAlloyCompression>(rod->getAnyKbOrigForLimb(i), rod->EI_scalar, compression_kB_max,
							nTimeStiffness, act_Kb_slope, act_EI_slope, tau, Tinitial));
		// Leftward bending: just do a negative max value and negative kB slope
		sma.push_back(make_shared<shapeMemoryAlloyCompression>(rod->getAnyKbOrigForLimb(i), rod->EI_scalar, -compression_kB_max,
							nTimeStiffness, -act_Kb_slope, act_EI_slope, tau, Tinitial));
	}
}

void world::updateTimeStep()
{
	// DEBUGGING: is the rod's kappaBar kept around??

	// Controller framework: confirm one has been assigned to this world,
	// then get the feedback signal.
	if( ctlr_p == NULL ){
		throw std::runtime_error("Error, simulation started but no controller assigned to world! Aborting.");
	}
	// controllers need to be timestepped. Example, in case they contain other logic, like in the PWM implementation.
	ctlr_p->updateTimestep(rod->dt);
	// and finally, the controller's response.
	std::vector<int> u_input = ctlr_p->getU(rod);

	// Turn on or off each SMA based on the controller.
	for(std::size_t i=0; i < numLimbs; i++){
		// WE ARE NOW DOING TWO LIMBS PER SMA

		// Assuming that everyone played nice so far and that u_input is the same dimension as 2 x numLimb
		// which is also the same number of SMAs (TO-DO: ERROR CHECKING HERE!!!)
		sma[2*i]->setActState(u_input[2*i]);
		sma[2*i+1]->setActState(u_input[2*i+1]);
		// NOTE: updating the SMA timestep actually needs to happen after the rod moves:
		// logically, if the SMA is turned "on", it will take a timestep to start to heat up.
		// This matches the NatComm paper model: the first timestep that's "on" should have the default kB.

		// ...at this point, the SMAs should be able to deliver kB and E for each limb.
		// The rod takes care of mapping limb->vertex_number->index_into_kappaBar.
		// WITH TWO LIMBS, we'll assume that the SMA contributions average out.
		rod->setKbForLimb(i, (sma[2*i]->getKappaBar() + sma[2*i+1]->getKappaBar()) / 2.0);
		rod->setEIForLimb(i, (sma[2*i]->getEI() + sma[2*i+1]->getEI()) / 2.0);
		// rod->setEIForLimb(i, sma[i]->getEI());
	}

	// Some output: compare a representative SMA vs. the rod itself.
	// std::cout << "SMA0 kB: " << sma[0]->getKappaBar() << ", Rod kB: " << (rod->kappaBar[1]) << std::endl;

	// Integrate equations of motion
	bool solved = false;

	int itr = 0;

	int withFriction = 0;

	m_externalContactForce->setForceZero();
	ifAddDynamicFriction = VectorXi::Zero(rod->nv);

	while (solved == false && itr < 10)
	{
		solved = updateTimeStep_singleRod();

		// Compute reaction force
		computeReactionForce();

		// check if underground
		if (solved == true)
		{
			for (int i = 0; i < rod->nv; i++)
			{
				Vector2d xCurrent = rod->getVertex(i);

				double xPos = xCurrent[0];
				double yPos = xCurrent[1];

				double desiredPos = rod->boundary(xPos);

				if (yPos <= desiredPos && rod->nConstraint(i) == 0)
				{
					double dY = desiredPos - yPos;
					double thetaWall = rod->thetaNormal(xPos);
					double dRDesired = dY * sin(thetaWall);
					Vector2d nWall( cos(thetaWall), sin(thetaWall) );

					Vector2d qDesired = xCurrent + nWall * dRDesired;

					if (rod->nConstraintPrevious(i) == 1)
					{
						rod->setOneVertexConstraintCurve(qDesired, i);
					}
					else
					{
						rod->setVertexBoundaryCondition(qDesired, i);
					}

					solved = false;
				}
			}
		}

		// check if take off
		if (solved == true && itr < 5)
		{
			for (int i = 0; i < rod->nv; i++)
			{
				Vector2d localForce;
				localForce(0) = rod->sumForces(2*i);
				localForce(1) = rod->sumForces(2*i+1);

				Vector2d xCurrent = rod->getVertex(i);

				double thetaWall = rod->thetaNormal(xCurrent(0));
				Vector2d nWall( cos(thetaWall), sin(thetaWall) );

				double ForceN = localForce.dot(nWall);

				if (ForceN < 0 && rod->nConstraint(i) != 0)
				{
					rod->releaseVertexBoundaryCondition(i);

					solved = false;
				}
			}
		}

		// add dynamic friction force
		if (withFriction == 0 && solved == true)
		{
			for (int i = 0; i < rod->nv; i++)
			{
				Vector2d localForce;
				localForce(0) = rod->sumForces(2*i);
				localForce(1) = rod->sumForces(2*i+1);

				Vector2d xCurrent = rod->getVertex(i);
				double thetaWall = rod->thetaNormal(xCurrent(0));
				Vector2d nWall( cos(thetaWall), sin(thetaWall) );
				Vector2d tWall( sin(thetaWall), -cos(thetaWall) );

				double ForceN = localForce.dot(nWall);
				double ForceT = localForce.dot(tWall);

				Vector2d uCurrent( rod->u(2*i), rod->u(2*i+1) );

				double VelocityN = uCurrent.dot(nWall);
				double VelocityT = uCurrent.dot(tWall);

				if (rod->nConstraint(i) == 1 && VelocityT != 0 && ifAddDynamicFriction(i) == 0 )
				{
					if (VelocityT>0)
					{
						Vector2d FrictionForce = - ForceN * dynamicFric * tWall;
						m_externalContactForce->addContactFriction(FrictionForce(0), 2*i);
						m_externalContactForce->addContactFriction(FrictionForce(1), 2*i+1);

						ifAddDynamicFriction(i) = 1;
						solved = false;
					}

					if (VelocityT<0)
					{
						Vector2d FrictionForce = ForceN * dynamicFric * tWall;
						m_externalContactForce->addContactFriction(FrictionForce(0), 2*i);
						m_externalContactForce->addContactFriction(FrictionForce(1), 2*i+1);

						ifAddDynamicFriction(i) = 1;
						solved = false;
					}
				}
			}

			withFriction = 1;
		}

		// check if slide
		if (solved == true)
		{
			for (int i = 0; i < rod->nv; i++)
			{
				if (rod->nConstraint(i) == 2)
				{
					Vector2d localForce;
					localForce(0) = rod->sumForces(2*i);
					localForce(1) = rod->sumForces(2*i+1);

					Vector2d xCurrent = rod->getVertex(i);
					double thetaWall = rod->thetaNormal(xCurrent(0));
					Vector2d nWall( cos(thetaWall), sin(thetaWall) );
					Vector2d tWall( sin(thetaWall), -cos(thetaWall) );

					double ForceN = localForce.dot(nWall);
					double ForceT = localForce.dot(tWall);

					if ( abs(ForceN) * dynamicFric < abs(ForceT) && ifAddDynamicFriction(i) == 0 )
					{
						if (ForceT>0)
						{
							rod->setOneVertexConstraintCurve(xCurrent, i);

							Vector2d FrictionForce = ForceN * dynamicFric * tWall;
							m_externalContactForce->addContactFriction(FrictionForce(0), 2*i);
							m_externalContactForce->addContactFriction(FrictionForce(1), 2*i+1);

							ifAddDynamicFriction(i) = 1;
							solved = false;
						}

						if (ForceT<0)
						{
							rod->setOneVertexConstraintCurve(xCurrent, i);

							Vector2d FrictionForce = - ForceN * dynamicFric * tWall;
							m_externalContactForce->addContactFriction(FrictionForce(0), 2*i);
							m_externalContactForce->addContactFriction(FrictionForce(1), 2*i+1);

							ifAddDynamicFriction(i) = 1;
							solved = false;
						}
					}
				}
			}
		}

		//check if tangent velocity change direction
		if (solved == true && itr < 5)
		{
			for (int i = 0; i < rod->nv; i++)
			{
				if (rod->nConstraint(i) == 1)
				{
					Vector2d xCurrent = rod->getVertex(i);
					double thetaWall = rod->thetaNormal(xCurrent(0));
					
					Vector2d tWall( sin(thetaWall), -cos(thetaWall) );

					Vector2d uCurrent( rod->u(2*i), rod->u(2*i+1) );
					double VelocityTcurrent = uCurrent.dot(tWall);

					Vector2d uPrevious( rod->u0(2*i), rod->u0(2*i+1) );
					double VelocityTprevious = uPrevious.dot(tWall);

					Vector2d xPrevious(rod->x0(2*i), rod->x0(2*i+1));

					if ( VelocityTcurrent * VelocityTprevious < 0)
					{
						rod->u(2*i) = 0;
						rod->u(2*i) = 0;
						rod->u0(2*i+1) = 0;
						rod->u0(2*i+1) = 0;

						rod->setVertexBoundaryCondition((xCurrent+xPrevious)/2, i);

						solved = false;
					}
				}
			}
		}

		itr++;
	}

	for (int i = 0; i < rod->nv; i++)
	{
		if (rod->nConstraint(i) == 1)
		{
			Vector2d xCurrent = rod->getVertex(i);
			double thetaWall = rod->thetaNormal(xCurrent(0));

			Vector2d tWall( sin(thetaWall), -cos(thetaWall) );

			Vector2d uCurrent( rod->u(2*i), rod->u(2*i+1) );
			double VelocityTcurrent = uCurrent.dot(tWall);

			Vector2d uPrevious( rod->u0(2*i), rod->u0(2*i+1) );
			double VelocityTprevious = uPrevious.dot(tWall);

			Vector2d xPrevious(rod->x0(2*i), rod->x0(2*i+1));

			if ( VelocityTcurrent * VelocityTprevious < 0)
			{

				rod->setVertexBoundaryCondition((xCurrent+xPrevious)/2, i);
			}
		}
	}
	
	//
	// update the variables of the rod
	//
	rod->updateTimeStep();

	// compute bt for next time step
	stepper->prepareForBt();

	m_stretchForce->computeJs();
	m_bendingForce->computeJb();
	m_gravityForce->computeJg();
	m_dampingForce->computeJd();

	stepper->computeBt();

	// Increment the SMAs (now that the rod is solved for this timestep)
	for( auto sma_i : sma){
		sma_i->updateTimestep(rod->dt);
	}

	// move to next time step
	timeStep++;
	currentTime += rod->dt;
	
	// Detect whether a successful simulation or not
	successfulSim = solved;
	// exit if bad simulation
	if (successfulSim == false)
	{
		cout << "numerical issue!" << endl;
		currentTime = totalTime * 1.1;
	}
}

bool world::updateTimeStep_singleRod()
{
	// Start by assuming the simulation converged	
	double normf = forceTol * 10.0;
	double normf0 = 0;
	
	bool solved = false;
	
	iter = 0;

	// Start with a trial solution for our solution x
	rod->updateGuess(); // x = x0 + u * dt
		
	while (solved == false)
	{
		iter++;

		stepper->setZero();

		// Compute the forces
		m_inertialForce->computeFi();	
		m_stretchForce->computeFs();		
		m_bendingForce->computeFb();		
		m_gravityForce->computeFg();	
		m_externalContactForce->computeFc();

		// Compute the jacobians
		m_inertialForce->computeJi();
		m_stretchForce->computeJs();
		m_bendingForce->computeJb();
		m_gravityForce->computeJg();

		// compute damping force and jacobian
		m_dampingForce->computeFd();
		m_dampingForce->computeJd();

		// set up force
		stepper->finishForceSetup();

		// Compute norm of the force equations.
		normf = 0;
		for (int i=0; i < rod->ndof; i++)
		{
			normf += totalForce[i] * totalForce[i];
		}
		normf = sqrt(normf);
		
		if (iter == 0) 
		{
			normf0 = normf;
		}
		
		if (normf <= forceTol)
		{
			solved = true;
		}
		else if(iter > 0 && normf <= normf0 * stol)
		{
			solved = true;
		}
		
		if (solved == false)
		{
			stepper->integrator(); // Solve equations of motion
			rod->updateNewtonX(totalForce); // new q = old q + Delta q
		}

		if (iter > maxIter)
		{
			// Throw an error so that the simulation environment can catch it and do a clean shutdown
			throw std::runtime_error("Error. Could not converge.");
			// cout << "Error. Could not converge.\n";

			return false;
		}
	}	
	
	return solved;
}

void world::computeReactionForce()
{		
	// Get forces
	rod->sumForces = VectorXd::Zero(rod->ndof);
	m_inertialForce->returnFi(rod->sumForces);			
	m_stretchForce->returnFs(rod->sumForces);			
	m_bendingForce->returnFb(rod->sumForces);		
	m_gravityForce->returnFg(rod->sumForces);	
	m_dampingForce->returnFd(rod->sumForces);
}

void world::checkJacobian()
{	
	rod->updateGuess(); // x = x0 + u * dt

	stepper->setZero();
	// Compute the forces
	m_inertialForce->computeFi();	
	m_stretchForce->computeFs();		
	m_bendingForce->computeFb();		
	m_gravityForce->computeFg();	
	m_dampingForce->computeFd();
	
	// Compute the jacobians
	m_inertialForce->computeJi();
	m_stretchForce->computeJs();
	m_bendingForce->computeJb();
	m_gravityForce->computeJg();
	m_dampingForce->computeJd();
					
	stepper->finishForceSetup();
	stepper->modifyMassJacobian();
	MatrixXd jAnalytical = stepper->getJacMat(); // CHECK TO MAKE SURE
	
	VectorXd f0(rod->ndof);
	for (int i=0; i < rod->ndof; i++)
		f0(i) = totalForce[i];
	
	MatrixXd jNumerical(rod->ndof, rod->ndof);
	
	double epsL = delta_l / 10000;
	double epsDV = epsL / rod->dt;
	for (int j=0; j < rod->ndof; j++)
	{

		rod->updateGuess(); // x = x0 + u * dt
		rod->dV(j) += epsDV;
		rod->x(j) =  rod->x(j) + epsDV * rod->dt;
		
		stepper->setZero();
		
		// Compute the forces
		m_inertialForce->computeFi();	
		m_stretchForce->computeFs();		
		m_bendingForce->computeFb();		
		m_gravityForce->computeFg();	
		m_dampingForce->computeFd();
			
		stepper->finishForceSetup();
		stepper->modifyMassJacobian();
	
		VectorXd f1(rod->ndof);
		for (int i=0; i < rod->ndof; i++)
			f1(i) = totalForce[i];
		
		for (int i=0; i < rod->ndof; i++)
		{
			jNumerical(j,i) = (f1(i) - f0(i)) / epsDV;
		}
	}
	
	cout << "Jacobian: k1 k2 analytical numerical diff\n";
	for (int k1=0; k1 < rod->ndof; k1++)
		for (int k2=0; k2 < rod->ndof; k2++)
		{
			if ( fabs(jNumerical(k1,k2)) > 0)
			{
				double diffPerc = (jNumerical(k1,k2) - jAnalytical(k1,k2)) / jNumerical(k1,k2) * 100;
				if (fabs(diffPerc) > 2)
				{
					cout << k1 << " " << k2 << " jA " << jAnalytical(k1,k2) 
						<< " jN " << jNumerical(k1,k2) << " " << fabs(diffPerc) << "%%" << endl;
				}
			}
		}
}

int world::simulationRunning()
{
	if (currentTime <= totalTime) 
		return 1;
	else 
	{
		return -1;
	}
}

int world::numPoints()
{
	return rod->nEdges;
}
	
Vector3d world::getScaledCoordinate(int i, int j)
{
	Vector2d p = rod->getVertex(rod->stretchingNodes[i][j]);
	// Scaled by a lot to make the visualization easier to see
	return Vector3d(p(0), p(1), 0) / ( 0.8 * RodLength );
}

Vector3d world::getScaledBendingCoordinate(int i, int j)
{
	// here, j is 0, 1, 2 since we have three vertices in a bending pair.
	Vector2d p = rod->getVertex(rod->bendingNodes[i][j]);
	// Scaled by a lot to make the visualization easier to see
	return Vector3d(p(0), p(1), 0) / ( 0.8 * RodLength );
}

double world::getScaledBoundary(double x)
{
	// Scaled by a lot to make the visualization easier to see
	return rod->boundary(x * 3 * RodLength) / ( 0.8 * RodLength );
}

double world::getCurrentTime()
{
	return currentTime;
}

double world::getTotalTime()
{
	return totalTime;
}

// Some helpers for interfacing with callers.

// returns a pointer to the rod, not the rod itself.
// elasticRod* world::getRodP()
shared_ptr<elasticRod> world::getRodP()
{
	return rod;
}

int world::getTimeStep()
{
	return timeStep;
}

// Rod controller must be set after instantiation.
// void world::setRodController(rodController* c_p)
void world::setRodController(shared_ptr<rodController> c_p)
{
	ctlr_p = c_p;
}

int world::getNumLimbs()
{
	return numLimbs;
}

double world::getLimbLength()
{
	return limbLength;
}