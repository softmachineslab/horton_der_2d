/**
 * openglDERSimulationEnvironment.cpp
 * 
 * Definitions for the class openglDERSimulationEnvironment.
 * Creates, updates, manages, etc., a graphical interface to the DER simulation.
 * Uses GLUT, code originally from main.cpp from Huang et al.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

// all includes should be in header
#include "openglDERSimulationEnvironment.h"

// the callbacks for openGL.
// Note this is a C function now
extern "C" void keyHandler(unsigned char key, int x, int y)
{
	switch (key) // ESCAPE to quit
  	{
		case 27:
			exit(0);
  	}
}


// Constructors just call parents and store command-line arguments from main
openglDERSimulationEnvironment::openglDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per, int m_argc, char **m_argv) : 
									derSimulationEnvironment(m_world, m_cmdline_per), argc_main(m_argc), argv_main(m_argv)
{
	// also make static copies of the passed-in pointers.
	openglWorld_p = m_world;
	opengl_is_logging = false;
	opengl_cmdline_per = m_cmdline_per;
}

openglDERSimulationEnvironment::openglDERSimulationEnvironment(shared_ptr<world> m_world, int m_cmdline_per, shared_ptr<worldLogger> m_logger, int m_argc, char **m_argv) : 
	derSimulationEnvironment(m_world, m_cmdline_per, m_logger), argc_main(m_argc), argv_main(m_argv)
{
	openglWorld_p = m_world;
	openglWorldLogger_p = m_logger;
	opengl_is_logging = true;
	opengl_cmdline_per = m_cmdline_per;
}

openglDERSimulationEnvironment::~openglDERSimulationEnvironment()
{
}

/* Initialize OpenGL Graphics */
void openglDERSimulationEnvironment::initGL() 
{
	// Initialize GLUT
	glutInit(&argc_main, argv_main);
	// When the window closes, return control back to main
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	// Create the window.
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize (1000, 1000);
	glutInitWindowPosition (100, 100);
	glutCreateWindow ("simRollingStar");

	// set window properties
	glClearColor(0.7f, 0.7f, 0.7f, 0.0f); // Set background color to black and opaque
	glClearDepth(10.0f);                   // Set background depth to farthest
	//glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	//glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	//glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

	glLoadIdentity();
	// gluLookAt(0.05, 0.05, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
	gluLookAt(0.00, 0.00, 0.3, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	// void gluLookAt(	GLdouble eyeX, 	GLdouble eyeY, 	GLdouble eyeZ, 	GLdouble centerX, 	GLdouble centerY, 	GLdouble centerZ, 	GLdouble upX, 	GLdouble upY, 	GLdouble upZ);
	glPushMatrix();

	//glMatrixMode(GL_MODELVIEW);

	// Other OpenGL setup:
	// keyboard callback. This is a C function
	glutKeyboardFunc(keyHandler);
	// the function that's called by OpenGL to actually run things. This is a static C++ function which can work as a C function in a pinch, as is here
	glutDisplayFunc(derOpenGLDisplay);

	// Here, we take an initial reading for the x0 state.
	// log if specified.
	if(opengl_is_logging){
		openglWorldLogger_p->logWorldData();
	}
}

void openglDERSimulationEnvironment::derOpenGLDisplay(void)
{
	// openglWorld_p is static.
	// world knows its max time from setInput
	while ( openglWorld_p->simulationRunning() > 0)
	{
		//  Clear screen and Z-buffer
		glClear(GL_COLOR_BUFFER_BIT);

		// draw axis
		double axisLen = 1;
		// glLineWidth(0.5);
		
		/*
		// Draw axes.
		glBegin(GL_LINES);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(axisLen, 0.0, 0.0);

			glColor3f(0.0, 1.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, axisLen, 0.0);

			glColor3f(0.0, 0.0, 1.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, axisLen);
		glEnd();
		*/

		// much thicker lines
		glLineWidth(2.0);
		
		//draw a line
		glBegin(GL_LINES);

		glColor3f(0.1, 0.1, 0.1);
		// glLineWidth(3.0);
		// much thicker
		// glLineWidth(6.0);

		// number of points? more efficient to call only once
		// int nPts = openglWorld_p->numPoints();
		// actually, we want the number of stretching pairs to plot.
		int nPts = openglWorld_p->getRodP()->nEdges;
		
		// plot the stretching edges
		for (int i=0; i < nPts; i++)
		{
			// The function getScaledCoordinate(i, j) returns (x,y,z=0) for stretching pair i, j=0 first node j=1 second node.
			Vector3d p = openglWorld_p->getScaledCoordinate(i, 0);
			glVertex3f( p(0), p(1), p(2));
			p = openglWorld_p->getScaledCoordinate(i, 1);
			glVertex3f( p(0), p(1), p(2));
			
			// Flush the buffer so we can see the lines plotted one after another
			// glEnd();		
			// glFlush();
			// glBegin(GL_LINES);
			// glColor3f(0.1, 0.1, 0.1);
			// glLineWidth(3.0);
		}

		// // plot the bending edges
		// int nPtsBending = openglWorld_p->getRodP()->nBends;
		// for (int i=0; i < nPtsBending; i++)
		// {
		// 	// previous edge will be blue
		// 	glColor3f(0.0, 0.0, 1.0);
		// 	// The function getScaledBendingCoordinate(i, j) returns (x,y,z=0) for stretching pair i, j=0 first node j=1 second node j=2 third node
		// 	Vector3d p0 = openglWorld_p->getScaledBendingCoordinate(i, 0);
		// 	glVertex3f( p0(0), p0(1), p0(2));
		// 	Vector3d p1 = openglWorld_p->getScaledBendingCoordinate(i, 1);
		// 	glVertex3f( p1(0), p1(1), p1(2));
		// 	// next edge will be red
		// 	glColor3f(1.0, 0.0, 0.0);
		// 	glVertex3f( p1(0), p1(1), p1(2));
		// 	Vector3d p2 = openglWorld_p->getScaledBendingCoordinate(i, 2);
		// 	glVertex3f( p2(0), p2(1), p2(2));
			
		// 	// Flush the buffer so we can see the lines plotted one after another
		// 	glEnd();		
		// 	glFlush();
		// 	// clear screen so we can see exactly what we're plotting
		// 	glClear(GL_COLOR_BUFFER_BIT);
		// 	// then restart
		// 	glBegin(GL_LINES);
		// 	// reset to black lines
		// 	glColor3f(0.1, 0.1, 0.1);
		// 	glLineWidth(3.0);
		// 	// // overwrite the last lines back to black so the next bend will display nicely
		// 	// glVertex3f( p0(0), p0(1), p0(2));
		// 	// glVertex3f( p1(0), p1(1), p1(2));
		// 	// glVertex3f( p2(0), p2(1), p2(2));
		// }
		// reset to black lines
		// glColor3f(0.1, 0.1, 0.1);

		double wallAngle = 6.0 * 3.141592654 / 180.0;
		
		for (double xb = - axisLen; xb < axisLen; xb += axisLen/100.0)
		{
			glVertex3f( xb, openglWorld_p->getScaledBoundary(xb), 0);
			glVertex3f( xb + axisLen/100.0, openglWorld_p->getScaledBoundary(xb+axisLen/100.0), 0);
		}

		// cleanup
		glEnd();		
		glFlush();
		
		// Step the world forward. This takes care of the SMAs, controller, rod, etc., ...
		// openglWorld_p->updateTimeStep();
		// catch convergence errors
		try {
			openglWorld_p->updateTimeStep(); // update time step
		}
		catch(std::runtime_error& excep){
			if(verbosity >= 1){
				std::cout << "Caught a runtime_error when trying to world->updateTimeStep: " << excep.what() << std::endl;
				std::cout << "Attempting clean shutdown..." << std::endl;
			}
			// superclass has the method
			cleanShutdown(openglWorldLogger_p, opengl_is_logging);
			// ugly to return here, but that's life
			// exit(1);
			// return;
			// FreeGLUT has a routine for actually stopping the GUI and returning cleanly
			glutLeaveMainLoop();
		}

		// log if specified.
		if(opengl_is_logging){
			openglWorldLogger_p->logWorldData();
		}

		// The helper from our superclass handles command line output.
		cmdlineOutputHelper(openglWorld_p, opengl_cmdline_per);

		// sleep(0.1);
	}
	// exit(1);
	// return;
	// FreeGLUT has a routine for actually stopping the GUI and returning cleanly
	glutLeaveMainLoop();
}

// Simply start up openGL.
void openglDERSimulationEnvironment::runSimulation()
{
	// let our users know to expect a GUI
	if( verbosity >= 1){
		std::cout << "Using a graphical simulation environment. Opening an openGL window..." << std::endl;
	}
	// First, setup the openGL window
	initGL();
	// then start up openGL.
	glutMainLoop();
}

