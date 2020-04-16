#ifndef GIOS_ACADO_H_
#define GIOS_ACADO_H_

#include "gios/gios.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace gios{

//TODO: Add safety measurements to ensure that step and pos doesn't go out of range

  class AcadoSolver : public Solver{//{{{
    public:
      AcadoSolver();
      ~AcadoSolver();
    public:
      void linkState(         VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkControl(       VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkReference(     VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkWeight(        VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkParameter(     VariablePtr &var, const unsigned &step, const unsigned &pos) override;

      void linkFeedbackState( VariablePtr &var, const unsigned &pos) override;
      void linkEndReference(  VariablePtr &var, const unsigned &pos) override;
      void linkEndWeight(     VariablePtr &var, const unsigned &pos) override;

      void getParameters(Parameters &p) override;
      unsigned getN() override;
      double getKKTTolerance() override;

      void solve() override;
      void simulate() override;
      void reset() override;
  };//}}}

AcadoSolver::AcadoSolver(){//{{{
  acado_initializeSolver();
  // Initialize States
  for (unsigned i = 0; i < ACADO_NX * (ACADO_N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
  // Initialize Controls
  for (unsigned i = 0; i < ACADO_NU * ACADO_N; ++i)  acadoVariables.u[ i ] = 0.0;
  // Initialize online data
  for (unsigned i = 0; i < ACADO_NOD * (ACADO_N + 1); ++i)  acadoVariables.od[ i ] = 0.0;
  // Initialize weights (all weights have to be at least 1!!!
  for (unsigned i = 0; i < ACADO_NY*ACADO_NY; ++i)  acadoVariables.W[ i ] = 0.0;
  //for( unsigned i=0; i<ACADO_NY; i++){
	// 	acadoVariables.W[ ( i )*(ACADO_NY+1) ] = 1.0;
  //}
  //for (unsigned i = 0; i < ny*ny*n; ++i)  acadoVariables.W[ i ] = 0.0;
  for (unsigned i = 0; i < ACADO_NYN*ACADO_NYN; ++i)  acadoVariables.WN[ i ] = 0.0;
  // Initialize the reference. 
  for (unsigned i = 0; i < ACADO_NY * ACADO_N; ++i)  acadoVariables.y[ i ] = 0.0;
  for (unsigned i = 0; i < ACADO_NYN; ++i)  acadoVariables.yN[ i ] = 0.0;
  //Initialize the current state feedback. 
  for (unsigned i = 0; i < ACADO_NX; ++i) acadoVariables.x0[ i ] = 0.0;
  std::cout<<"Acado Solver Created"<<std::endl;
}//}}}

AcadoSolver::~AcadoSolver(){//{{{
  std::cout<<"Acado Solver Deleted"<<std::endl;
};//}}}

void AcadoSolver::linkState(VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.x[step*ACADO_NX + pos];
}//}}}

void AcadoSolver::linkFeedbackState(VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.x0[pos];
}//}}}

void AcadoSolver::linkControl(VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.u[step*ACADO_NU + pos];
}//}}}

void AcadoSolver::linkReference(VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.y[step*ACADO_NY + pos];
}//}}}

void AcadoSolver::linkWeight(VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  var = &acadoVariables.W[step*ACADO_NY*ACADO_NY + pos*(ACADO_NY +1)];
}//}}}

void AcadoSolver::linkParameter(VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.od[step*ACADO_NOD + pos];
}//}}}

void AcadoSolver::linkEndReference(VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.yN[pos];
}//}}}

void AcadoSolver::linkEndWeight(VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.WN[pos*(ACADO_NYN + 1)];
}//}}}

void AcadoSolver::getParameters(Parameters &p){//{{{
  p.n   = ACADO_N;
  p.nx  = ACADO_NX;  //Number of differential states 
  p.nu  = ACADO_NU;  //Number of controls
  p.np  = ACADO_NOD;  //Number of parameters
  p.nr  = ACADO_NY;  //Number of references
  p.nrn = ACADO_NYN; //Number of references at end
}//}}}

unsigned AcadoSolver::getN(){/*{{{*/
  return ACADO_N;
}/*}}}*/

double AcadoSolver::getKKTTolerance(){/*{{{*/
  return acado_getKKT();
}/*}}}*/

void AcadoSolver::solve(){//{{{
  acado_feedbackStep();
};//}}}

void AcadoSolver::simulate(){//{{{
  acado_preparationStep();
};//}}}

void AcadoSolver::reset(){//{{{
 acado_initializeSolver();
 for (unsigned i = 0; i < ACADO_NX * (ACADO_N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
 // Initialize Controls
 for (unsigned i = 0; i < ACADO_NU * ACADO_N; ++i)  acadoVariables.u[ i ] = 0.0;

 acado_initializeNodesByForwardSimulation();
};//}}}

}
#endif
