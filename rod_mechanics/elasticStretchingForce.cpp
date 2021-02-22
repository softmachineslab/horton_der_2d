#include "elasticStretchingForce.h"
#include <iostream>

elasticStretchingForce::elasticStretchingForce(shared_ptr<elasticRod> m_rod, shared_ptr<timeStepper> m_stepper)
{
	rod = m_rod;
	stepper = m_stepper;
	ndof = rod->ndof;
	
	Jss.setZero(4,4);
	flocal = VectorXd::Zero(4);
	EA = rod->EA;
}

elasticStretchingForce::~elasticStretchingForce()
{
	;
}

void elasticStretchingForce::computeFs()
{
	f = VectorXd::Zero(rod->ndof);

	for (int k=0; k<rod->nEdges; k++)
	{
		ind1 = rod->stretchingNodes[k][0];
		ind2 = rod->stretchingNodes[k][1];
		p = rod->getVertex(ind1);
		p1 = rod->getVertex(ind2);
		xk = p[0];
		yk = p[1];
		xkp1 = p1[0];
		ykp1 = p1[1];
		l_k = rod->l_k_stretching[k];
		
		localForce(); // populate flocal: first derivative of (axial stretch)^2
		flocal = (- 0.5 * EA * l_k) * flocal; // scale with stiffness
		
		f(2*ind1)   += flocal[0];
		f(2*ind1+1) += flocal[1];
		f(2*ind2)   += flocal[2];
		f(2*ind2+1) += flocal[3];
	}

    stepper->subtractForce(f); // subtracting elastic force
}

void elasticStretchingForce::computeJs()
{	
	for (int k=0; k<rod->nEdges; k++)
	{
		ind1 = rod->stretchingNodes[k][0];
		ind2 = rod->stretchingNodes[k][1];
		p = rod->getVertex(ind1);
		p1 = rod->getVertex(ind2);
		xk = p[0];
		yk = p[1];
		xkp1 = p1[0];
		ykp1 = p1[1];
		l_k = rod->l_k_stretching[k];
		
		localJacobian();
		Jss = (0.5 * EA * l_k) * Jss; // scale with stiffness
        
        for (int i=0; i < 2; i++)
        {
        	for (int j=0; j < 2; j++)
			{
				ind0 = rod->stretchingNodes[k][i];
				ind1 = rod->stretchingNodes[k][j];

				stepper->addElaticJacobian( 2*ind0,   2*ind1,   Jss(2*i,2*j) );				
				stepper->addElaticJacobian( 2*ind0,   2*ind1+1, Jss(2*i,2*j+1) );				
				stepper->addElaticJacobian( 2*ind0+1, 2*ind1,   Jss(2*i+1,2*j) );				
				stepper->addElaticJacobian( 2*ind0+1, 2*ind1+1, Jss(2*i+1,2*j+1) );				
			}
        }
	}
}

void elasticStretchingForce::reserveJacobianSpace()
{
	for (int k=0; k<rod->nEdges; k++)
	{
        for (int i=0; i < 2; i++)
			for (int j=0; j < 2; j++)
			{
				ind0 = rod->stretchingNodes[k][i];
				ind1 = rod->stretchingNodes[k][j];

				stepper->addJacobianMat( 2*ind0,   2*ind1,   1.0 );				
				stepper->addJacobianMat( 2*ind0,   2*ind1+1, 1.0 );				
				stepper->addJacobianMat( 2*ind0+1, 2*ind1,   1.0 );				
				stepper->addJacobianMat( 2*ind0+1, 2*ind1+1, 1.0 );				
			}
	}
}

void elasticStretchingForce::returnFs(VectorXd &m_f)
{
	m_f = m_f - f;
}

void elasticStretchingForce::localForce()
{
	flocal(0) = -(0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k * (-0.2e1 * xkp1 + 0.2e1 * xk);
	flocal(1) = -(0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k * (-0.2e1 * ykp1 + 0.2e1 * yk);
	flocal(2) = -(0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k * (0.2e1 * xkp1 - 0.2e1 * xk);
	flocal(3) = -(0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k * (0.2e1 * ykp1 - 0.2e1 * yk);
}

void elasticStretchingForce::localJacobian()
{
	Jss(0,0) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * pow(-0.2e1 * xkp1 + 0.2e1 * xk, 0.2e1) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * pow(-0.2e1 * xkp1 + 0.2e1 * xk, 0.2e1) / 0.2e1 - 0.2e1 * (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k;
	Jss(0,1) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * (-0.2e1 * ykp1 + 0.2e1 * yk) * (-0.2e1 * xkp1 + 0.2e1 * xk) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * (-0.2e1 * xkp1 + 0.2e1 * xk) * (-0.2e1 * ykp1 + 0.2e1 * yk) / 0.2e1;
	Jss(0,2) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * (0.2e1 * xkp1 - 0.2e1 * xk) * (-0.2e1 * xkp1 + 0.2e1 * xk) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * (-0.2e1 * xkp1 + 0.2e1 * xk) * (0.2e1 * xkp1 - 0.2e1 * xk) / 0.2e1 + 0.2e1 * (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k;
	Jss(0,3) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * (0.2e1 * ykp1 - 0.2e1 * yk) * (-0.2e1 * xkp1 + 0.2e1 * xk) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * (-0.2e1 * xkp1 + 0.2e1 * xk) * (0.2e1 * ykp1 - 0.2e1 * yk) / 0.2e1;
	Jss(1,0) = Jss(0,1);
	Jss(1,1) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * pow(-0.2e1 * ykp1 + 0.2e1 * yk, 0.2e1) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * pow(-0.2e1 * ykp1 + 0.2e1 * yk, 0.2e1) / 0.2e1 - 0.2e1 * (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k;
	Jss(1,2) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * (0.2e1 * xkp1 - 0.2e1 * xk) * (-0.2e1 * ykp1 + 0.2e1 * yk) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * (-0.2e1 * ykp1 + 0.2e1 * yk) * (0.2e1 * xkp1 - 0.2e1 * xk) / 0.2e1;
	Jss(1,3) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * (0.2e1 * ykp1 - 0.2e1 * yk) * (-0.2e1 * ykp1 + 0.2e1 * yk) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * (-0.2e1 * ykp1 + 0.2e1 * yk) * (0.2e1 * ykp1 - 0.2e1 * yk) / 0.2e1 + 0.2e1 * (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k;
	Jss(2,0) = Jss(0,2);
	Jss(2,1) = Jss(1,2);
	Jss(2,2) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * pow(0.2e1 * xkp1 - 0.2e1 * xk, 0.2e1) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * pow(0.2e1 * xkp1 - 0.2e1 * xk, 0.2e1) / 0.2e1 - 0.2e1 * (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k;
	Jss(2,3) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * (0.2e1 * ykp1 - 0.2e1 * yk) * (0.2e1 * xkp1 - 0.2e1 * xk) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * (0.2e1 * xkp1 - 0.2e1 * xk) * (0.2e1 * ykp1 - 0.2e1 * yk) / 0.2e1;
	Jss(3,0) = Jss(0,3);
	Jss(3,1) = Jss(1,3);
	Jss(3,2) = Jss(2,3);
	Jss(3,3) = 0.1e1 / (pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) * pow(l_k, -0.2e1) * pow(0.2e1 * ykp1 - 0.2e1 * yk, 0.2e1) / 0.2e1 + (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.3e1 / 0.2e1) / l_k * pow(0.2e1 * ykp1 - 0.2e1 * yk, 0.2e1) / 0.2e1 - 0.2e1 * (0.1e1 - sqrt(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1)) / l_k) * pow(pow(xkp1 - xk, 0.2e1) + pow(ykp1 - yk, 0.2e1), -0.1e1 / 0.2e1) / l_k;
}
