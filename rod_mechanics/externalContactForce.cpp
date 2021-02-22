#include "externalContactForce.h"

externalContactForce::externalContactForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, double m_dynamicFric)
{
	rod = m_rod;
	stepper = m_stepper;
	dynamicFric = m_dynamicFric;
	f = VectorXd::Zero(rod->ndof);
	
	Id2<<1,0,
		 0,1;	
}

externalContactForce::~externalContactForce()
{
	;
}

void externalContactForce::setForceZero()
{
	f.setZero();
}

void externalContactForce::addContactFriction(double m_force, int i)
{
	f(i) = m_force;
}

void externalContactForce::computeFc()
{
	stepper->subtractForce(f);
}

void externalContactForce::computeJc()
{
	;
}

void externalContactForce::setFc(VectorXd &m_f)
{
	f = m_f;
}

void externalContactForce::returnFc(VectorXd &m_f)
{
	m_f = m_f - f;
}
