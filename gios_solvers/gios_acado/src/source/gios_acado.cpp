#include "gios_acado/gios_acado.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

gios_acado::AcadoSolver::AcadoSolver(){//{{{
  acado_initializeSolver();
  std::cout<<"Acado Solver Created"<<std::endl;
}//}}}

gios_acado::AcadoSolver::~AcadoSolver(){//{{{
  std::cout<<"Acado Solver Deleted"<<std::endl;
};//}}}

void gios_acado::AcadoSolver::linkState(gios::VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.x[step*ACADO_NX + pos];
}//}}}

void gios_acado::AcadoSolver::linkFeedbackState(gios::VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.x0[pos];
}//}}}

void gios_acado::AcadoSolver::linkControl(gios::VariablePtr &var, const unsigned &pos, const unsigned &step){//{{{
    var = &acadoVariables.u[step*ACADO_NU + pos];
}//}}}

void gios_acado::AcadoSolver::linkReference(gios::VariablePtr &var, const unsigned &pos, const unsigned &step){//{{{
    var = &acadoVariables.y[step*ACADO_NY + pos];
}//}}}

void gios_acado::AcadoSolver::linkDynamicWeight(gios::VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  var = &acadoVariables.W[step*ACADO_NY*ACADO_NY + pos*(ACADO_NY +1)];
}//}}}

void gios_acado::AcadoSolver::linkWeight(gios::VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.W[pos*(ACADO_NY + 1)];
}//}}}

void gios_acado::AcadoSolver::linkParameter(gios::VariablePtr &var,const unsigned &step, const unsigned &pos){//{{{
    var = &acadoVariables.od[step*ACADO_NOD + pos];
}//}}}

void gios_acado::AcadoSolver::linkEndReference(gios::VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.yN[pos];
}//}}}

void gios_acado::AcadoSolver::linkEndWeight(gios::VariablePtr &var, const unsigned &pos){//{{{
    var = &acadoVariables.WN[pos*(ACADO_NYN + 1)];
}//}}}

void gios_acado::AcadoSolver::getParameters(gios::Parameters &p){//{{{
  p.n   = ACADO_N;
  p.nx  = ACADO_NX;  //Number of differential states 
  p.nu  = ACADO_NU;  //Number of controls
  p.np  = ACADO_NOD;  //Number of parameters
  p.nr  = ACADO_NY;  //Number of references
  p.nrn = ACADO_NYN; //Number of references at end
}//}}}

void gios_acado::AcadoSolver::solve(){//{{{
  acado_feedbackStep();
};//}}}

void gios_acado::AcadoSolver::simulate(){//{{{
  acado_preparationStep();
};//}}}

