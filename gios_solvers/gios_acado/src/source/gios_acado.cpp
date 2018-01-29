#include "gios_acado/gios_acado.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

//TODO: Add safety measurements to ensure that step and pos doesn't go out of range

namespace gios{

AcadoSolver::AcadoSolver(){//{{{
  acado_initializeSolver();
  std::cout<<"Acado Solver Created"<<std::endl;
}//}}}

AcadoSolver::~AcadoSolver(){//{{{
  std::cout<<"Acado Solver Deleted"<<std::endl;
};//}}}

void AcadoSolver::linkState(gios::VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.x[step*ACADO_NX + pos];
}//}}}

void AcadoSolver::linkFeedbackState(gios::VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.x0[pos];
}//}}}

void AcadoSolver::linkControl(gios::VariablePtr &var, const unsigned &pos, const unsigned &step){//{{{
    var = &acadoVariables.u[step*ACADO_NU + pos];
}//}}}

void AcadoSolver::linkReference(gios::VariablePtr &var, const unsigned &pos, const unsigned &step){//{{{
    var = &acadoVariables.y[step*ACADO_NY + pos];
}//}}}

void AcadoSolver::linkWeight(gios::VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  var = &acadoVariables.W[step*ACADO_NY*ACADO_NY + pos*(ACADO_NY +1)];
}//}}}

void AcadoSolver::linkParameter(gios::VariablePtr &var,const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.od[step*ACADO_NOD + pos];
}//}}}

void AcadoSolver::linkEndReference(gios::VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.yN[pos];
}//}}}

void AcadoSolver::linkEndWeight(gios::VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.WN[pos*(ACADO_NYN + 1)];
}//}}}

void AcadoSolver::getParameters(gios::Parameters &p){//{{{
  p.n   = ACADO_N;
  p.nx  = ACADO_NX;  //Number of differential states 
  p.nu  = ACADO_NU;  //Number of controls
  p.np  = ACADO_NOD;  //Number of parameters
  p.nr  = ACADO_NY;  //Number of references
  p.nrn = ACADO_NYN; //Number of references at end
}//}}}

unsigned AcadoSolver::getN(){
  return ACADO_N;
}

void AcadoSolver::solve(){//{{{
  acado_feedbackStep();
};//}}}

void AcadoSolver::simulate(){//{{{
  acado_preparationStep();
};//}}}

}
