all:
	g++ -I eigen/eigen-eigen-323c052e1731/ -L /usr/local/lib/pardiso600 main.cpp world.cpp elasticRod.cpp elasticStretchingForce.cpp elasticBendingForce.cpp externalGravityForce.cpp inertialForce.cpp dampingForce.cpp externalContactForce.cpp timeStepper.cpp setInput.cpp -lGL -lglut -lGLU -lpardiso600-GNU720-X86-64 -llapack -lgfortran -fopenmp -lpthread -lm -Ofast -o horton_der_2d

clean:

	rm horton_der_2d
