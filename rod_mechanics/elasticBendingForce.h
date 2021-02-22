#ifndef ELASTICBENDINGFORCE_H
#define ELASTICBENDINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class elasticBendingForce
{
public:
	elasticBendingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper);
	~elasticBendingForce();
	void computeFb();
	void computeJb();
	void returnFb(VectorXd &m_f);
	void reserveJacobianSpace();
	
private:

	shared_ptr<elasticRod> rod;
	shared_ptr<timeStepper> stepper;

	double xkm1, ykm1, xk, yk, xkp1, ykp1, l_k, e0e1, curvature0;
    VectorXd flocal;
    VectorXd f;
    MatrixXd Jbb;
    int ind, ind0, ind1, ind2;
    
    void localForce();
    void localJacobian();
};

#endif
