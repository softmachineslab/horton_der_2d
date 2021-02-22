#include "dampingForce.h"

dampingForce::dampingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, double m_viscositya, double m_viscosityb)
{
	rod = m_rod;
	stepper = m_stepper;
	viscositya = m_viscositya;
	viscosityb = m_viscosityb;
	
	ndof = rod->ndof;

	Id2<<1,0,
		0,1;
}

dampingForce::~dampingForce()
{
	;
}

void dampingForce::computeFd()
{
	f = VectorXd::Zero(ndof);

	// beta damping
	f = - viscosityb * stepper->ElasticJacobian * rod->u;

	// alpha damping 
	for (int i = 0; i < ndof; i++)
	{
		f(i) = f(i) - viscositya * rod->massArray(i) * rod->u(i);
	}

	stepper->subtractForce(f);
}

void dampingForce::computeJd()
{
	// alpha damping 
	for (int i = 0; i < ndof; i++)
	{
		stepper->addDampingJacobian(i, i, viscositya * rod->massArray(i) );
	}

	// beta damping
	for (int i = 0; i < rod->ndof; i++)
	{
		for (int j = 0; j < rod->ndof; j++)
		{
			stepper->addDampingJacobian(i, j, viscosityb * stepper->ElasticJacobian(i, j) );
		}
	}
}

// Set the non-zero entries in Jacobian to one
void dampingForce::reserveJacobianSpace()
{
	for (int i=0;i<rod->nv;i++)
	{
		for (int kx = 0; kx < 2; kx++)
		{
			indx = 2 * i + kx;
			for (int ky = 0; ky < 2; ky++)
			{
				indy = 2 * i + ky;
				stepper->addJacobianMat(indx, indy, 1.0 );
			}
		}
	}
}

void dampingForce::returnFd(VectorXd &m_f)
{
	m_f = m_f - f;
}
