#include "timeStepper.h"
#include <fstream>

// timeStepper::timeStepper(elasticRod &m_rod)
timeStepper::timeStepper(shared_ptr<elasticRod> m_rod)
{
	// rod = &m_rod;
	rod = m_rod;
    ndof = rod->ndof;
	totalForce = new double[ndof];

	nrhs = 1;

    jacMat = MatrixXd::Zero(ndof, ndof);

    ElasticJacobian.setZero(ndof, ndof);
    InertialJacobian.setZero(ndof, ndof);
    DampingJacobian.setZero(ndof, ndof);

    forceB.setZero(ndof);
}

timeStepper::~timeStepper()
{	    
    /* -------------------------------------------------------------------- */    
	/* ..  Termination and release of memory.                               */
	/* -------------------------------------------------------------------- */ 
    phase = -1;                 /* Release internal memory. */

    pardiso (pt, &maxfct, &mnum, &mtype, &phase,
             &n, &ddum, ia, ja, &idum, &nrhs,
             iparm, &msglvl, &ddum, &ddum, &error,  dparm);
	
	// We also need to delete any locals.
	delete[] totalForce;
}

double* timeStepper::getForce()
{
	return totalForce;
}

MatrixXd timeStepper::getJacMat()
{
	return jacMat;
}

void timeStepper::addForceInertia(VectorXd &m_f)
{
	finertia = m_f;
}

void timeStepper::addForce(VectorXd &m_f)
{
	f = f + m_f;
}

void timeStepper::subtractForce(VectorXd &m_f)
{
	f = f - m_f;
}

void timeStepper::addJacobianMat(int ind1, int ind2, double p)
{
	jacMat(ind1, ind2) += p;
}

void timeStepper::addElaticJacobian(int ind1, int ind2, double p)
{
	ElasticJacobian(ind1, ind2) += p;
}

void timeStepper::addDampingJacobian(int ind1, int ind2, double p)
{
	DampingJacobian(ind1, ind2) += p;
}

void timeStepper::setZero()
{
    f = VectorXd::Zero(ndof);
    
    jacMat = MatrixXd::Zero(ndof, ndof);

    ElasticJacobian.setZero(ndof, ndof);
    InertialJacobian.setZero(ndof, ndof);
    DampingJacobian.setZero(ndof, ndof);
}

void timeStepper::finishForceSetup()
{
	// Update it using W_Nodes
	for (int i=0; i < rod->nv; i++)
	{
		Vector2d flocal(f(2*i), f(2*i+1));
		Vector2d fblocal(forceB(2*i), forceB(2*i+1));
		Vector2d fw = rod->W_Nodes[i] * (flocal + fblocal) / 2;
		
		totalForce[2*i]   = finertia(2*i)   + fw(0) - rod->dVReqd(2*i)   / rod->dt;
		totalForce[2*i+1] = finertia(2*i+1) + fw(1) - rod->dVReqd(2*i+1) / rod->dt;
	}
}

void timeStepper::integrator()
{
	modifyMassJacobian();
	
	pardisoSolve();
	
    for (int i = 0; i < ndof; i++) 
    {
        totalForce[i] = x[i];
    }
}

void timeStepper::modifyMassJacobian()
{
	for (int i=0; i < rod->nv; i++)
	{
		int row_i = 2*i;
		
		// Option 1
		jacMat.block(row_i,0,2,ndof) = rod->W_Nodes[i] * (rod->dt * ElasticJacobian.block(row_i,0,2,ndof) / 4 + DampingJacobian.block(row_i,0,2,ndof) / 2 );
	}

	// include inertia
	for (int i=0; i < rod->ndof; i++)
	{
		jacMat(i,i) += 1.0 / rod->dt;
	}
}

void timeStepper::first_time_PARDISO()
{	
	n = ndof;
	ia = new int[n+1];
	
	nnz = 0; // number of non-zeros
	for (int i=0; i < ndof; i++)
	{
		for (int j=0; j < ndof; j++)
		{
			if ( jacMat(i,j) != 0 )
			{
				nnz++;
			}
		}
	}
	
	ia[0] = 1; // FORTRAN's 1-based notation
	ja = new int[nnz];
	jr = new int[nnz];
	a = new double[nnz];
	nnz = 0; // reset and recompute
	for (int i=0; i < ndof; i++)
	{
		for (int j=0; j < ndof; j++)
		{
			if ( jacMat(i,j) != 0 )
			{
				jr[nnz] = i + 1; // row index - FORTRAN's 1-based notation
				ja[nnz] = j + 1; // column index - FORTRAN's 1-based notation
				nnz++;
			}
		}

		ia[i+1] = nnz + 1; // FORTRAN's 1-based notation
	}
	
	mtype = 11;        /* Real unsymmetric matrix */
	b = new double[n]; // right hand side
	x = new double[n]; // solution
		
/* -------------------------------------------------------------------- */
/* ..  Setup Pardiso control parameters and initialize the solvers      */
/*     internal adress pointers. This is only necessary for the FIRST   */
/*     call of the PARDISO solver.                                      */
/* ---------------------------------------------------------------------*/
      
    error = 0;
    solver = 0; /* use sparse direct solver */
    pardisoinit (pt,  &mtype, &solver, iparm, dparm, &error);

    if (error != 0)
    {
        if (error == -10 )
           printf("No license file found \n");
        if (error == -11 )
           printf("License is expired \n");
        if (error == -12 )
           printf("Wrong username or hostname \n");
         return;
    }
    else
        printf("[PARDISO]: License check was successful ... \n");
 

    /* Numbers of processors, value of OMP_NUM_THREADS */
    var = getenv("OMP_NUM_THREADS");
    if(var != NULL)
        sscanf( var, "%d", &num_procs );
    else {
        printf("Set environment OMP_NUM_THREADS to 1");
        exit(1);
    }
    iparm[2]  = num_procs;   
    
    // If small number is an issue, consider changing the iparm parameter
    // representing "eps" pivot.
    
/* -------------------------------------------------------------------- */    
/* ..  Reordering and Symbolic Factorization.  This step also allocates */
/*     all memory that is necessary for the factorization.              */
/* -------------------------------------------------------------------- */ 
	
	maxfct = 1;         /* Maximum number of numerical factorizations.  */
    mnum   = 1;         /* Which factorization to use. */
    
    msglvl = 0;         /* Print statistical information  */
    error  = 0;         /* Initialize error flag */
    
    phase = 11; 

	for (int i=0; i < nnz; i++)
	{
		int indr = jr[i]-1;
		int indc = ja[i]-1;
		a[i] = jacMat(indr,indc);
	}
	
    pardiso (pt, &maxfct, &mnum, &mtype, &phase,
             &n, a, ia, ja, &idum, &nrhs,
             iparm, &msglvl, &ddum, &ddum, &error,  dparm);
    
    if (error != 0) {
        printf("\nERROR during symbolic factorization: %d", error);
        exit(1);
    }
    printf("\nReordering completed ... ");
    printf("\nNumber of nonzeros in factors  = %d", iparm[17]);
    printf("\nNumber of factorization MFLOPS = %d", iparm[18]);
    printf("\n");
}

void timeStepper::pardisoSolve()
{			
    /* Set right hand side. */
    for (int i = 0; i < ndof; i++) 
    {
        b[i] = totalForce[i];
    }

	// jacMat is not symmetric
	for (int i=0; i < nnz; i++)
	{
		int indr = jr[i]-1;
		int indc = ja[i]-1;
		a[i] = jacMat(indr,indc);
	}
    
/* -------------------------------------------------------------------- */    
/* ..  Numerical factorization.                                         */
/* -------------------------------------------------------------------- */    
    phase = 22;

    pardiso (pt, &maxfct, &mnum, &mtype, &phase,
             &n, a, ia, ja, &idum, &nrhs,
             iparm, &msglvl, &ddum, &ddum, &error, dparm);
   
    if (error != 0) {
        printf("\nERROR during numerical factorization: %d", error);
        exit(2);
    }
    // printf("\nFactorization completed ...\n ");

/* -------------------------------------------------------------------- */    
/* ..  Back substitution and iterative refinement.                      */
/* -------------------------------------------------------------------- */    
    phase = 33;

    iparm[7] = 1;       /* Max numbers of iterative refinement steps. */

    pardiso (pt, &maxfct, &mnum, &mtype, &phase,
             &n, a, ia, ja, &idum, &nrhs,
             iparm, &msglvl, b, x, &error,  dparm);
   
    if (error != 0) {
        printf("\nERROR during solution: %d", error);
        exit(3);
    }             
}

void timeStepper::prepareForBt()
{
	f.setZero(ndof);
}

void timeStepper::computeBt()
{
	forceB.setZero(ndof);

	forceB = f;
}