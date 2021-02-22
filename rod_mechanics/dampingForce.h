#ifndef DAMPINGFORCE_H
#define DAMPINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class dampingForce
{
public:
	dampingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, double m_viscositya, double m_viscosityb);
	~dampingForce();
	void computeFd();
	void computeJd();
	void returnFd(VectorXd &m_f);
	void reserveJacobianSpace();
	
private:
	shared_ptr<elasticRod> rod;
	shared_ptr<timeStepper> stepper;
	double viscositya;
	double viscosityb;
	
    Vector2d t, u, flocal, p;
    VectorXd f;
    int ind, indx, indy;
    Matrix2d Id2, jac;
    int ndof;
};

#endif
