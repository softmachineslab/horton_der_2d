#ifndef TIMESTEPPER_H
#define TIMESTEPPER_H

#include "elasticRod.h"

extern "C" void dgbsv_( int* n, int* kl, int* ku, int* nrhs, double* ab, int* ldab, int* ipiv, double* b, int* ldb, int* info );

/* PARDISO prototype. */
extern "C" void pardisoinit (void   *, int    *,   int *, int *, double *, int *);
extern "C" void pardiso     (void   *, int    *,   int *, int *,    int *, int *, 
                  double *, int    *,    int *, int *,   int *, int *,
                     int *, double *, double *, int *, double *);
extern "C" void pardiso_chkmatrix  (int *, int *, double *, int *, int *, int *);
extern "C" void pardiso_chkvec     (int *, int *, double *, int *);
extern "C" void pardiso_printstats (int *, int *, double *, int *, int *, int *, double *, int *);
/* END PARDISO prototype. */

class timeStepper
{
public:
	// timeStepper(elasticRod &m_rod);
    timeStepper(shared_ptr<elasticRod> m_rod);
	~timeStepper();
	double* getForce();
	void setZero();
	void addForce(VectorXd &f);
	void addForceInertia(VectorXd &m_f);
	void subtractForce(VectorXd &f);
	void addJacobian(int ind1, int ind2, double p);
	void addJacobianMat(int ind1, int ind2, double p);
	void addJacobianInertia(int ind1, int ind2, double p);
	void integrator();
	void finishForceSetup();
    void first_time_PARDISO();
	MatrixXd getJacMat();
    void modifyMassJacobian(); // make it private

    void addElaticJacobian(int ind1, int ind2, double p);
	void addInertialJacobian(int ind1, int ind2, double p);
	void addDampingJacobian(int ind1, int ind2, double p);

    MatrixXd ElasticJacobian;
    MatrixXd InertialJacobian;
    MatrixXd DampingJacobian;

    void prepareForBt();
    void computeBt();
	
private:
	// elasticRod *rod;
    shared_ptr<elasticRod> rod;
	int ndof;
	
	double *totalForce;
	VectorXd f;
	VectorXd finertia;
    
    MatrixXd jacMat;
    MatrixXd jacMatBackup;
    
    //
    // PARDISO
    //
    double *b, *x;
    int nrhs;          /* Number of right hand sides. */
	int *ja, *jr;
	double *a;
	int nnz;
	int n;
	int *ia;
	
	void    *pt[64];

    /* Pardiso control parameters. */
    int      iparm[64];
    double   dparm[64];
    int      solver;
    int      maxfct, mnum, phase, error, msglvl;

    /* Number of processors. */
    int      num_procs;

    /* Auxiliary variables. */
    char    *var;
    int      i;

    double   ddum;              /* Double dummy */
    int      idum;              /* Integer dummy. */
    
    int mtype;
    
    void pardisoSolve();

    VectorXd forceB;
};

#endif
