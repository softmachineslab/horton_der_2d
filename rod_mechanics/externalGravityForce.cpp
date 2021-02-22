#include "externalGravityForce.h"

externalGravityForce::externalGravityForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, Vector2d m_gVector)
{
	rod = m_rod;
	stepper = m_stepper;
	gVector = m_gVector;

	setGravity();
}

externalGravityForce::~externalGravityForce()
{
	;
}

void externalGravityForce::computeFg()
{
	stepper->subtractForce(f); // subtracting gravity force
}

void externalGravityForce::computeJg()
{
	;
}

void externalGravityForce::setGravity()
{
	f = VectorXd::Zero(rod->ndof);
	for (int i=0; i < rod->nv; i++)
	{
		for (int k=0; k < 2; k++)
		{
			int ind = 2*i + k;
			f(ind) = gVector[k] * rod->massArray(ind);
		}
	}
}

void externalGravityForce::returnFg(VectorXd &m_f)
{
	m_f = m_f - f;
}
