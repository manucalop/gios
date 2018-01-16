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
  VariablePtrMatrix x, u, r, w, p;
  VariablePtrVector xf, rn, wn;
  protected:
  void setX( const VariablePtr &var, const unsigned &step, const unsigned &pos);
  void setU( const VariablePtr &var, const unsigned &step, const unsigned &pos);
  void setR( const VariablePtr &var, const unsigned &step, const unsigned &pos);
  void setW( const VariablePtr &var, const unsigned &step, const unsigned &pos);
  void setP( const VariablePtr &var, const unsigned &step, const unsigned &pos);
  void setXF(const VariablePtr &var, const unsigned &pos);
  void setRN(const VariablePtr &var, const unsigned &pos);
  void setWN(const VariablePtr &var, const unsigned &pos);
  
  public:
  explicit Solver(const Parameters &params);
  virtual ~Solver();
  void linkState(     VariablePtr &var, const unsigned &step, const unsigned &pos) const;
  void linkControl(   VariablePtr &var, const unsigned &step, const unsigned &pos) const;
  void linkReference( VariablePtr &var, const unsigned &step, const unsigned &pos) const;
  void linkWeight(    VariablePtr &var, const unsigned &step, const unsigned &pos) const;
  void linkParameter( VariablePtr &var, const unsigned &step, const unsigned &pos) const;

  void linkFeedbackState( VariablePtr &var, const unsigned &pos) const;
  void linkEndReference(  VariablePtr &var, const unsigned &pos) const;
  void linkEndWeight(     VariablePtr &var, const unsigned &pos) const;

  void getParameters(Parameters &p) const;
  void test_init();

  virtual void solve()=0;
  virtual void simulate()=0;
};//}}}

}// Namespace gios}}}

#endif

