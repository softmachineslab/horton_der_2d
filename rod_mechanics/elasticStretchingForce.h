#ifndef ELASTICSTRETCHINGFORCE_H
#define ELASTICSTRETCHINGFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class elasticStretchingForce
{
public:
	elasticStretchingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper);
	~elasticStretchingForce();
	void computeFs();
	void computeJs();
	void returnFs(VectorXd &m_f);
	void reserveJacobianSpace();
	
private:
	shared_ptr<elasticRod> rod;
	shared_ptr<timeStepper> stepper;
    
    double EA;
    int ndof;
    
    double xk, yk, xkp1, ykp1, l_k;
    VectorXd flocal;
    VectorXd f;
    MatrixXd Jss;
    int ind, ind0, ind1, ind2;
    Vector2d p, p1;
    
    void localForce();
    void localJacobian();
};

#endif
