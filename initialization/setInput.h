#ifndef SETINPUT_H
#define SETINPUT_H

#include <iostream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

#include "Option.h"
#include "../eigenIncludes.h"

class setInput
{
public:

  typedef std::map<std::string, Option> OptionMap;
  OptionMap m_options;

  setInput();
  ~setInput();

  template <typename T>
	int AddOption(const std::string& name, const std::string& desc, const T& def);

  Option* GetOption(const std::string& name);
  bool& GetBoolOpt(const std::string& name);
  int& GetIntOpt(const std::string& name);
  double& GetScalarOpt(const std::string& name);
  Vector3d& GetVecOpt(const std::string& name);
  string& GetStringOpt(const std::string& name);

  int LoadOptions(const char* filename);
  int LoadOptions(const std::string& filename)
  {
    return LoadOptions(filename.c_str());
  }
  int LoadOptions(int argc, char** argv);

private:
	double limbLength;
	double rodRadius;
	int numVertices;
	double youngM;
	double Poisson;
	double shearM;
	double deltaTime;
	double totalTime;
	double tol, stol;
	int maxIter; // maximum number of iterations
	double density;
	Vector3d gVector;
	double viscositya;
	double viscosityb;
	bool render;
	double dropHeight;
	double curvature_limb;
	int boundaryIter;
	int numLimbs;
	double grooveLength;
	double staticFric, dynamicFric; // friction coefficients
	int dataFreq;
	int boundaryType;
	int verbosity;
	int cmdline_per;
	std::string logfile_base;
	std::string ol_control_filepath;
	bool enable_logging;
	int num_simulations;
	double foot_ratio;
	double compression_kB_max;

	// To enable/disable the auto-drop feature, so we can deterministically specify
	// the initial pose of the robot at instantiation.
	bool enableAutoDrop;
};

#include "setInput.tcc"

#endif // PROBLEMBASE_H
