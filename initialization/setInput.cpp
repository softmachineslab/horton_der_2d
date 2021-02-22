#include "setInput.h"
using namespace std;

setInput::setInput()
{
	AddOption("render", "visualization", render);
	AddOption("data-frequency", "Number of steps per data output", dataFreq);

	AddOption("limb-length", "length of limb", limbLength);
	AddOption("rodRadius", "Radius of Rod", rodRadius);
	AddOption("numVertices", "Number of Vertices", numVertices);
	AddOption("youngM", "Young's Modulus", youngM);
	AddOption("Poisson", "Poisson Ratio", Poisson);
	AddOption("deltaTime", "Time Step Length", deltaTime);
	AddOption("totalTime", "Total Running Time", totalTime);
	AddOption("tol", "Tolerance of Newton Method", tol);
	AddOption("stol", "Ratio between initial and final error", stol);
	AddOption("maxIter", "Maximum Running Times of Each Stepper", maxIter);
	AddOption("boundary-iter", "Maximum number of iterations to resolve boundary", boundaryIter);
	AddOption("density", "Density of the Rod", density);
	AddOption("viscositya", "viscositya", viscositya);
  AddOption("viscosityb", "viscosityb", viscosityb);
	AddOption("gVector", "Gravity", gVector);
	AddOption("drop-height", "Height of the box", dropHeight);
	AddOption("limb-curvature", "Curvature of the limb", curvature_limb);
	AddOption("num-limbs", "Number of limbs", numLimbs);
	AddOption("groove-length", "Length of each groove", grooveLength);
	AddOption("static-friction", "Static friction coefficient", staticFric);
	AddOption("dynamic-friction", "Dynamic friction coefficient", dynamicFric);
	AddOption("boundary-type", "1=plane, 2=max point, 3=min point", boundaryType);
  AddOption("enable-auto-drop", "Enable automatic alignment and dropping of the robot at t0", enableAutoDrop);
  AddOption("debug-verbosity", "Verbosity of output to the command line. 0=none, 2=max", verbosity);
  AddOption("cmdline-period", "Period of output to command line for check-in updates", cmdline_per);
  AddOption("logfile-base", "Base directory to store the logfiles in.", logfile_base);
  AddOption("ol-control-filepath", "CSV file with actuation timepoints for rodOpenLoopFileController.", ol_control_filepath);
  AddOption("num-simulations", "Number of simulations to execute. For deterministic simulations, choose = 1.", num_simulations);
  AddOption("foot-ratio", "Foot segment length as a percent of limb length", foot_ratio);
  AddOption("compression-kB-max", "Maximum kappaBar value for the compression SMA model", compression_kB_max);
}

setInput::~setInput()
{
	;
}

Option* setInput::GetOption(const string& name)
{
  if (m_options.find(name) == m_options.end()) 
  {
    cerr << "Option " << name << " does not exist" << endl;
  }
  return &(m_options.find(name)->second);
}

bool& setInput::GetBoolOpt(const string& name)
{
  return GetOption(name)->b;
}

int& setInput::GetIntOpt(const string& name)
{
  return GetOption(name)->i;
}

double& setInput::GetScalarOpt(const string& name)
{
  return GetOption(name)->r;
}

Vector3d& setInput::GetVecOpt(const string& name)
{
  return GetOption(name)->v;
}

string& setInput::GetStringOpt(const string& name)
{
  return GetOption(name)->s;
}

int setInput::LoadOptions(const char* filename)
{
  ifstream input(filename);
  if (!input.is_open()) 
  {
    cerr << "ERROR: File " << filename << " not found" << endl;
    return -1;
  }

  string line, option;
  istringstream sIn;
  string tmp;
  for (getline(input, line); !input.eof(); getline(input, line)) 
  {
    sIn.clear();
    option.clear();
    sIn.str(line);
    sIn >> option;
    if (option.size() == 0 || option.c_str()[0] == '#') continue;
    OptionMap::iterator itr;
    itr = m_options.find(option);
    if (itr == m_options.end()) 
    {
      cerr << "Invalid option: " << option << endl;
      continue;
    }
    if (itr->second.type == Option::BOOL) 
    {
      sIn >> tmp;
      if (tmp == "true" || tmp == "1") itr->second.b = true;
      else if (tmp == "false" || tmp == "0") itr->second.b = false;
    } 
    else if (itr->second.type == Option::INT) 
    {
      sIn >> itr->second.i;
    } 
    else if (itr->second.type == Option::DOUBLE) 
    {
      sIn >> itr->second.r;
    } 
    else if (itr->second.type == Option::VEC) 
    {
      Vector3d& v = itr->second.v;
      sIn >> v[0];
      sIn >> v[1];
      sIn >> v[2];
    } 
    else if (itr->second.type == Option::STRING) 
    {
      sIn >> itr->second.s;
    } else 
    {
      cerr << "Invalid option type" << endl;
    }
  }
  input.close();

  return 0;
}

int setInput::LoadOptions(int argc, char** argv)
{
  string option, tmp;
  int start = 0;
  while (start < argc && string(argv[start]) != "--") ++start;
  for (int i = start + 1; i < argc; ++i) 
  {
    option = argv[i];
    OptionMap::iterator itr;
    itr = m_options.find(option);
    if (itr == m_options.end()) 
    {
      cerr << "Invalid option on command line: " << option << endl;
      continue;
    }
    if (i == argc - 1) 
    {
      cerr << "Too few arguments on command line" << endl;
      break;
    }
    if (itr->second.type == Option::BOOL) 
    {
      tmp = argv[i+1]; ++i;
      if (tmp == "true" || tmp == "1") itr->second.b = true;
      if (tmp == "false" || tmp == "0") itr->second.b = false;
    } 
    else if (itr->second.type == Option::INT) 
    {
      itr->second.i = atoi(argv[i+1]); ++i;
    } 
    else if (itr->second.type == Option::DOUBLE) 
    {
      itr->second.r = atof(argv[i+1]); ++i;
    } 
    else if (itr->second.type == Option::VEC) 
    {
      if (i >= argc - 3) 
      {
        cerr << "Too few arguments on command line" << endl;
        break;
      }
      Vector3d& v = itr->second.v;
      v[0] = atof(argv[i+1]); ++i;
      v[1] = atof(argv[i+1]); ++i;
      v[2] = atof(argv[i+1]); ++i;
    } 
    else if (itr->second.type == Option::STRING) 
    {
      itr->second.s = argv[i+1]; ++i;
    } 
    else 
    {
      //cerr << "Invalid option type" << endl;
    }
  }
  return 0;
}
