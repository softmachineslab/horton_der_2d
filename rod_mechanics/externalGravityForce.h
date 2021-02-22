#ifndef EXTERNALGRAVITYFORCE_H
#define EXTERNALGRAVITYFORCE_H

#include "../eigenIncludes.h"
#include "elasticRod.h"
#include "timeStepper.h"

class externalGravityForce
{
public:
	externalGravityForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper, Vector2d m_gVector);
	~externalGravityForce();
	void computeFg();
	void computeJg();
	void returnFg(VectorXd &m_f);

private:
	shared_ptr<elasticRod> rod;
	shared_ptr<timeStepper> stepper;
    Vector2d gVector;
    VectorXd f;
    void setGravity();
};

#endif
