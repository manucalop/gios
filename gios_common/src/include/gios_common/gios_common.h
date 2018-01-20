#ifndef GIOS_COMMON_H_
#define GIOS_COMMON_H_

#include <vector>
#include <iostream>
#include <memory>

namespace gios{//{{{

// Typedefs {{{

typedef double Variable;
typedef Variable * VariablePtr;
typedef std::vector<Variable> VariableVector; //Column Vector of variables
typedef std::vector<VariablePtr> VariablePtrVector;
typedef std::vector<VariableVector> VariableMatrix;
typedef std::vector<VariablePtrVector> VariablePtrMatrix;

typedef struct Parameters_{
unsigned n;   //Number of steps
unsigned nx;  //Number of differential states 
unsigned nu;  //Number of controls
unsigned np;  //Number of parameters
unsigned nr;  //Number of references
unsigned nrn; //Number of references at end
} Parameters;

//}}}

class Solver{//{{{
  public:
  explicit Solver();
  virtual ~Solver();
  virtual void linkState(         VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
  virtual void linkControl(       VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
  virtual void linkReference(     VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
  virtual void linkWeight(        VariablePtr &var, const unsigned &pos) = 0;
  virtual void linkDynamicWeight( VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
  virtual void linkParameter(     VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;

  virtual void linkFeedbackState( VariablePtr &var, const unsigned &pos) = 0;
  virtual void linkEndReference(  VariablePtr &var, const unsigned &pos) = 0;
  virtual void linkEndWeight(     VariablePtr &var, const unsigned &pos) = 0;

  virtual void getParameters(Parameters &p) = 0;
  virtual void test_init();

  virtual void solve()=0;
  virtual void simulate()=0;
};//}}}

}// Namespace gios}}}

#endif

