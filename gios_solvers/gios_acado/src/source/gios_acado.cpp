#include "gios_acado/gios_acado.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

gios_acado::AcadoSolver::AcadoSolver(): Solver(//{{{
    gios::Parameters{ .n   = ACADO_N, 
                            .nx  = ACADO_NX, 
                            .nu  = ACADO_NU, 
                            .np  = ACADO_NOD, 
                            .nr  = ACADO_NY, 
                            .nrn = ACADO_NYN }
                                              ){
init();
std::cout<<"Acado Solver Created"<<std::endl;
}//}}}

gios_acado::AcadoSolver::~AcadoSolver(){//{{{
  std::cout<<"Acado Solver Deleted"<<std::endl;
};//}}}

void gios_acado::AcadoSolver::init(){//{{{
  printf("Initializing solver...\n");
  // Initialize the solver. 
  acado_initializeSolver();

  for(unsigned i = 0; i < ACADO_N + 1; i++){
    
    for (unsigned j = 0; j < ACADO_NX; j++ ){
      setX(&acadoVariables.x[i*ACADO_NX + j], i, j);
    }
    
    for (unsigned j = 0; j < ACADO_NOD; j++ ){
      setP(&acadoVariables.od[i*ACADO_NOD + j], i, j);
    }
  }
  for(unsigned i = 0; i < ACADO_N; i++){

    for (unsigned j = 0; j < ACADO_NU; j++ ){
      setU(&acadoVariables.u[i*ACADO_NU + j], i, j);
    }
    
    for (unsigned j = 0; j < ACADO_NY; j++ ){
      setR(&acadoVariables.y[i*ACADO_NY + j], i, j);
      setW(&acadoVariables.W[j*(ACADO_NY + 1)], i, j);
    }
  }
  for (unsigned j = 0; j < ACADO_NX; j++ ){
    setXF(&acadoVariables.x0[j], j);
  }
  for (unsigned j = 0; j < ACADO_NYN; j++ ){
    setWN(&acadoVariables.WN[j*(ACADO_NYN + 1)], j);
    setRN(&acadoVariables.yN[j], j);
  }
  test_init();
}//}}}

void gios_acado::AcadoSolver::solve(){//{{{
  acado_feedbackStep();
};//}}}

void gios_acado::AcadoSolver::simulate(){//{{{
  acado_preparationStep();
};//}}}

