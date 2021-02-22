#ifndef EXTERNALCONTACTFORCE_H
#define EXTERNALCONTACTFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class externalContactForce
{
public:
	externalContactForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, double m_dynamicFric);
	~externalContactForce();
	void computeFc();
	void computeJc();
	void setFc(VectorXd &m_f);
	void returnFc(VectorXd &m_f);

	void addContactFriction(double m_force, int i);
	void setForceZero();

private:
	shared_ptr<elasticRod> rod;
	shared_ptr<timeStepper> stepper;
	double dynamicFric;
    VectorXd f;
    Vector2d u, flocal;
    int ind, indx, indy;
    Matrix2d jac, Id2;
};

#endif
