# horton_der_2d
Discrete Elastic Rod (DER) Simulation of Horton the Robot. Two-dimensional (in-plane).

This code simulates the Horton robot using a discrete elastic rod (DER) model, with shape memory alloy (SMA) wires controlling the rod's parameters to induce motion. It is written in C++, and is based on code from M. Khalid Jawed. 

See his article [Dynamic Simulation of Articulated Soft Robots](https://www.nature.com/articles/s41467-020-15651-9) for the specific details. Information about the actual calculations is included in the paper's supplementary information.

## Installing horton_der_2d

This code is designed to prevent any hard-coded paths to files on your computer. We should not be pushing any changes to Github that would only work on one computer but not others.

Code designed to work on Ubuntu 18.04.

If attempting to run on Mac OS X, you will need to figure out how to install all the prerequisite packages below. 
Unsure if other changes need to be made.
**At least one change** is in `main.c`, when including the OpenGL library.
Replace the line
```
#include <GL/glut.h>
```
which is used for Linux, with
```
#include <GLUT/glut.h>
```
to use with Mac OS X.

### Prerequisites

Download and install the following:

1. LAPACK, for linear algebra.
```
sudo apt-get install liblapack-dev liblapack3 libopenblas-base libopenblas-dev
```
2. Compilers. This includes g++ and gfortran.
```
sudo apt-get install g++ gfortran gfortran-7 libgfortran-7-dev gdb
```
3. OpenGL.
```
sudo apt-get install freeglut3 freeglut3-dev
```
4. PARDISO.
	1. Download from [pardiso-project.org](pardiso-project.org). You want the gcc/gfortran 7.2.0, libparadiso600-GNU720-X86-64
	2. Accept the license and download the .so file.
	3. Place the .so file in:
	```
	/usr/local/lib/pardiso600/
	```
	4. From your email that you received, copy the license key into a text file pardiso.lic in your home directory (`~`)
	5. If you have questions, see the [PARDISO manual](https://pardiso-project.org/manual/manual.pdf).

**Note:** This code also requires the Eigen library. However, Drew has placed this library in the repository itself, and configured the compiler to look for it there. If you want to use your own version of Eigen, you can, but switch to your own branch to do it there.

### Set up your environment

We need to add a few environment variables for g++ and PARDISO. First, we will determine how many CPU cores for PARDISO to use during its calculations. Run the following command:
```
cat /proc/cpuinfo | grep processor | wc -l
```
It's probably 8 cores. Use whatever number you have in place of 8 below.

Open the file `~/.bashrc`, for example, `gedit ~/.bashrc`. Add the following lines at the end of the file:

1. Set the library path (for the linker, runtime) to look in the folder where you put the pardiso .so file:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/pardiso600
``` 
2. Add the number of threads you'd like to use for PARDISO.
```
export OMP_NUM_THREADS=8
```

### Set up your compiler or your develoment environment

You have the option of either (1) compiling this code directly from the command line, (2) using the Visual Studio Code configuration that Drew provides here, or (3) figuring out how to compile the code on your own. We'll explain the first two options. **Drew recommends using Visual Studio Code.**

1. Compile from the command line. We provide an example `Makefile`. Right now, it does not include the correct .cpp files, Drew has not edited it in a while. You will need to edit it to include all .cpp files in all folders. To use: copy `example_setup/Makefile` to this folder. To compile your code, navigate to this folder in a terminal, and run 
	```
	make
	```
2. Compile within Visual Studio Code.
	1. Download and install [Visual Studio Code](https://code.visualstudio.com/download). Follow the instructions for Ubuntu.
	2. Set up `gdb` to correctly display `eigen` matrices. 
		1. Copy `example_setup/example_gdbinit` to your home folder and rename it `.gdbinit`. Edit the file and change line three to the correct location of Eigen wherever you cloned this repository.
	3. Install the C/C++ Plugin for VS Code (see their website). Hint, VS Code's "getting started with debugging for C/C++" website is really helpful.
	4. Open VS Code and either open the repository as a folder (`File -> Open Folder...`) or open a terminal, navigate to this folder, and run `vscode .`
	5. Go to the "Run" tab and run one of the compile tasks (green arrow at the top).

## Running horton_der_2d

Develop code in your own branch in Github then merge back to main once confirmed it works/compiles.

The DER simulation takes one argument: an "options" file. This is implemented so that you may change certain parameters of the simulation without recompiling. Information about these parameters is included in `initialization/SetInput.*`. The main function reads these parameters, and passes them around to other classes, primarily the `world` class.

There is currently NO ERROR HANDLING for loading in options, so you'll get a segmentation fault (pointer error) if run incorrectly, no debugging output.

If you are changing these parameters, make a copy of the default options file and use that one.

You can either run your code directly from the command line or within Visual Studio Code:

* Execute with the options file as an argument: 
```
./horton_der_2d options_default.txt
```
* Execute within VS Code: go to the "Run" tab and select one of the tasks at the top, such as `g++ build and debug active file`.
	* If you want to use a different options file, change this setting in `.vscode/launch.json`.

Drew provides two examples of build options in VS Code (see `.vscode/tasks.json`.) The default build settings do not include compiler optimizations. This is useful for debugging your code, but runs slower. The "optimized" task includes compiler optimizations, which makes debugging impractical, but the code runs faster.

## Documentation

We currently do not have documentation for any code in this repository. I sincerely apologize. If anyone would like to document parts of this code, your help is appreciated.

At a high level:
1. `main` creates a `world`. Calling `world->setRodStepper()` initializes most things in the world, including the `elasticRod` and the classes which calculate rod forces, as well as the `timeStepper` which does the math for the simulation, and the `shapeMemoryAlloy` wires for the actuator model. 
1. Next, in `main`, you create a `rodController` and pass it to the world via `world->setRodController`. 
1. Finally, based on the `isRender` parameter from the options file, `main` creates a `simulation_environment` that is either graphical (OpenGL) or command-line only. **Note** that the graphical interface will only run for one iteration: to run the simulation many times, you must use `headlessDERSimulationEnvironment`.

## More information about simulation options.

Copied from Khalid Jawed. This may be outdated, and certain parameters are missing. You will need to look through the code to be sure.

1. You can edit the parameters of the simulation by editing "option.txt" file. You can also specify an option using the following syntax:
./horton_der_2d option.txt -- option_name option_value
Example: ./horton_der_2d option.txt -- RodLength 0.2

1. Details on the options (we use SI units): 
	1. "boundary-type" = 1 for planar, 2 for sinusoidal (with robot at maximum point), and 3 for sinusoidal (with robot at minimum point)
    1. "limb-length" is the length of the circular part of each limb.
    1. "groove-length" is the length of the straight part of each limb.
    1. "rodRadius" is the cross-sectional radius (assuming the rod is circular).
    1. "youngM" is the young's modulus.
    1. "Poisson" is the Poisson ratio.
    1. "deltaTime" is the time step size.
    1. "totalTime" is the time at which the simulation ends.
    1. "tol" and "stol" are small numbers used in solving the linear system. Fraction of a percent, e.g. 1.0e-3, is often a good choice.
    1. "maxIter" is the maximum number of iterations allowed before the solver quits.
    1. "density" is the mass per unit volume.
    1. "gVector" is the vector specifying acceleration due to gravity.
    1. "viscosity" is the viscosity of the fluid medium.
    1. "numVertices" is the number of nodes per limb (circular part only).
    1. "limb-curvature" is the curvature of the circular part of the limbs.
    1. "num-limbs" is the number of limbs.
    1. "static-friction" is the coefficient of static friction.
    1. "dynamic-friction" is the coefficient of dynamic friction (NOT IMPLEMENTED YET).
    1. "render" (0 or 1) indicates whether OpenGL visualization should be displayed.
    1. "saveData" (0 or 1) indicates whether the location of the head should be saved in "datafiles/" folder (this folder will be created by the program).
    1. "data-frequency" is the number of steps per data dump. This is used to prevent huge data files.
    1. "enable-auto-drop" is on/off for automatic alignment of the rod, ignored if a \bx_0 is passed to its constructor.