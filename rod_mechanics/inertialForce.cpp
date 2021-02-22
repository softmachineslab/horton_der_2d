#include "inertialForce.h"

inertialForce::inertialForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper)
{
	rod = m_rod;
	stepper = m_stepper;
	ndof = rod->ndof;
}

inertialForce::~inertialForce()
{
	;
}

void inertialForce::computeFi()
{
	f = rod->dV / rod->dt;
	
	stepper->addForceInertia(f);
}

void inertialForce::computeJi()
{
	;
}

void inertialForce::reserveJacobianSpace()
{
	for (int i=0; i<rod->nv; i++)
    {
		stepper->addJacobianMat(2*i, 2*i, 1.0);
		stepper->addJacobianMat(2*i, 2*i+1, 1.0);
		stepper->addJacobianMat(2*i+1, 2*i, 1.0);
		stepper->addJacobianMat(2*i+1, 2*i+1, 1.0);
	}
}

void inertialForce::returnFi(VectorXd &m_f)
{
	// double tt = 0;
	for (int i=0; i<ndof; i++)
	{
		m_f(i) += rod->massArray(i) * rod->dV(i) / rod->dt;
	}
}
