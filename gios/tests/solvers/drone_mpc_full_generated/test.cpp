//mpc_solver.h {{{
#ifndef MPC_SOLVER_H_
#define MPC_SOLVER_H_
#include <vector>
#include <iostream>
#include <memory>

namespace mpc_solver{//{{{

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

}// Namespace mpc_solver}}}

#endif
// }}}

// mpc_solver.cpp {{{

//#include "mpc_solver.h
  
mpc_solver::Solver::Solver( const Parameters &p)://{{{
  x ( p.n + 1, VariablePtrVector(p.nx, nullptr) ),
  u ( p.n    , VariablePtrVector(p.nu, nullptr) ),
  r ( p.n    , VariablePtrVector(p.nr, nullptr) ),
  w ( p.n    , VariablePtrVector(p.nr, nullptr) ),
  p ( p.n + 1, VariablePtrVector(p.np, nullptr) ),
  xf( p.nx,  nullptr),
  rn( p.nrn, nullptr),
  wn( p.nrn, nullptr){
  std::cout<<"Solver Created"<<std::endl;
  }//}}}

mpc_solver::Solver::~Solver(){//{{{
  std::cout<<"Solver Deleted"<<std::endl;
};//}}}

void mpc_solver::Solver::setX( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  x[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void mpc_solver::Solver::setXF(const VariablePtr &var, const unsigned &pos){//{{{
  xf[pos] = var;                                                              
}//}}}                                                                              
                                                                                    
void mpc_solver::Solver::setU( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  u[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void mpc_solver::Solver::setR( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  r[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void mpc_solver::Solver::setW( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  w[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void mpc_solver::Solver::setP( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  p[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void mpc_solver::Solver::setRN(const VariablePtr &var, const unsigned &pos){//{{{
  rn[pos] = var;                                                                    
}//}}}                                                                              
                                                                                    
void mpc_solver::Solver::setWN(const VariablePtr &var, const unsigned &pos){//{{{
  wn[pos] = var;
}//}}}

void mpc_solver::Solver::linkState(VariablePtr &var, const unsigned &step, const unsigned &pos) const{//{{{
    var = x[step][pos];
}//}}}

void mpc_solver::Solver::linkFeedbackState(VariablePtr &var, const unsigned &pos) const{//{{{
    var = xf[pos];
}//}}}

void mpc_solver::Solver::linkControl(VariablePtr &var, const unsigned &pos, const unsigned &step) const{//{{{
    var = u[step][pos];
}//}}}

void mpc_solver::Solver::linkReference(VariablePtr &var, const unsigned &pos, const unsigned &step) const{//{{{
    var = r[step][pos];
}//}}}

void mpc_solver::Solver::linkWeight(VariablePtr &var,const unsigned &step, const unsigned &pos) const{//{{{
    var = w[step][pos];
}//}}}

void mpc_solver::Solver::linkParameter(VariablePtr &var,const unsigned &step, const unsigned &pos) const{//{{{
    var = p[step][pos];
}//}}}

void mpc_solver::Solver::linkEndReference(VariablePtr &var, const unsigned &pos) const{//{{{
    var = rn[pos];
}//}}}

void mpc_solver::Solver::linkEndWeight(VariablePtr &var, const unsigned &pos) const{//{{{
    var = wn[pos];
}//}}}

void mpc_solver::Solver::getParameters(Parameters &params) const{//{{{
params.n    = u.size();  
params.nx   = x[0].size();  //Number of differential states 
params.nu   = u[0].size();  //Number of controls
params.np   = p[0].size();  //Number of parameters
params.nr   = r[0].size();  //Number of references
params.nrn  = rn.size(); //Number of references at end
}//}}}

void mpc_solver::Solver::test_init(){//{{{
  bool ok = true;
  std::cout<<"Testing Solver initialization ...\n";
  for(unsigned i = 0; i < x.size(); i++)
    for(unsigned j = 0;j < x[0].size(); j++)
      if (x[i][j] == nullptr){
        ok = false;
        std::cout<<"Found nullptr in x["<<i<<"]["<<j<<"]"<<std::endl;
      }
  
  for(unsigned i = 0; i < u.size(); i++)
    for(unsigned j = 0;j < u[0].size(); j++)
      if (u[i][j] == nullptr){
        std::cout<<"Found nullptr in u["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }


  for(unsigned i = 0; i < r.size(); i++)
    for(unsigned j = 0;j < r[0].size(); j++)
      if (r[i][j] == nullptr){
        std::cout<<"Found nullptr in r["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }

  for(unsigned i = 0; i < w.size(); i++)
    for(unsigned j = 0;j < w[0].size(); j++)
      if (w[i][j] == nullptr){
        std::cout<<"Found nullptr in w["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < p.size(); i++)
    for(unsigned j = 0;j < p[0].size(); j++)
      if (p[i][j] == nullptr){
        std::cout<<"Found nullptr in p["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < xf.size(); i++)
      if (xf[i] == nullptr){
        std::cout<<"Found nullptr in xf["<<i<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < rn.size(); i++)
      if (rn[i] == nullptr){
        std::cout<<"Found nullptr in rn["<<i<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < wn.size(); i++)
      if (wn[i] == nullptr){
        std::cout<<"Found nullptr in wn["<<i<<"]"<<std::endl;
        ok = false;
      }
  if (ok == true)
    std::cout<<"Initialization tested successfully"<<std::endl;
}//}}}

//}}}

// mpc_acado_solver.h {{{
#ifndef MPC_ACADO_SOLVER_H_
#define MPC_ACADO_SOLVER_H_
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

namespace mpc_acado_solver{

  class AcadoSolver : public mpc_solver::Solver{//{{{
    private:
      void init();
    public:
      AcadoSolver();
      ~AcadoSolver();
    public:
      void solve() override;
      void simulate() override;
  };//}}}

}

#endif

//}}}

// mpc_acado_solver.cpp {{{

//#include "mpc_acado_solver.h

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

mpc_acado_solver::AcadoSolver::AcadoSolver(): Solver(//{{{
    mpc_solver::Parameters{ .n   = ACADO_N, 
                            .nx  = ACADO_NX, 
                            .nu  = ACADO_NU, 
                            .np  = ACADO_NOD, 
                            .nr  = ACADO_NY, 
                            .nrn = ACADO_NYN }
                                              ){
init();
std::cout<<"Acado Solver Created"<<std::endl;
}//}}}

mpc_acado_solver::AcadoSolver::~AcadoSolver(){//{{{
  std::cout<<"Acado Solver Deleted"<<std::endl;
};//}}}

void mpc_acado_solver::AcadoSolver::init(){//{{{
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

void mpc_acado_solver::AcadoSolver::solve(){//{{{
  acado_feedbackStep();
};//}}}

void mpc_acado_solver::AcadoSolver::simulate(){//{{{
  acado_preparationStep();
};//}}}

//}}}

// drone_mpc_controller.h {{{

#ifndef DRONE_MPC_CONTROLLER_H_
#define DRONE_MPC_CONTROLLER_H_
#include <array>
#include <vector>

namespace drone_mpc_controller{//{{{

// Typedefs {{{
  typedef mpc_solver::VariablePtr VariablePtr;
  typedef struct Vector3_{
    VariablePtr x = nullptr, y = nullptr, z = nullptr;
  } Vector3;
  typedef struct EulerAngles_{
    VariablePtr roll = nullptr, pitch = nullptr, yaw = nullptr;
  } EulerAngles;
  typedef struct Pose_{
    Vector3 position;
    EulerAngles orientation;
  } Pose;
  typedef struct Twist_{
    Vector3 linear;
    Vector3 angular;
  } Twist;
  typedef struct Accel_{
    Vector3 linear;
    Vector3 angular;
  } Accel;
  typedef struct Jerk_{
    Vector3 linear;
    Vector3 angular;
  } Jerk;
  typedef struct State_{
    Pose pose;
    Twist twist;
    Accel accel;
    Jerk jerk;
  } State;
  typedef std::vector<State> StateArray;
  typedef Twist Control;
  typedef std::vector<Control> ControlArray;
  typedef std::array<std::array<VariablePtr,3>,3> RotationMatrix;
  typedef struct Ellipsoid_{
    Vector3 position;
    Vector3 size;
    RotationMatrix rotation_matrix; 
  } Ellipsoid;
  typedef struct EllipsoidalObstacle_{
    Ellipsoid ellipsoid;
    VariablePtr s_k, s_w;//Sensitivity and weight of the slack variable
  } EllipsoidalObstacle;//Static obstacle
  typedef std::vector<EllipsoidalObstacle> EllipsoidalObstacleArray;//Dynamic Obstacle
  typedef std::vector<EllipsoidalObstacleArray> EllipsoidalObstacleMatrix;//Multiple dynamic obstacles
  typedef struct Plane_{
    Vector3 position;
    Vector3 n_vector;//Normal vector
  } Plane;
  typedef struct PlanarObstacle_{
    Plane plane;
    VariablePtr s_k, s_w;//Sensitivity and weight of the slack variable
  } PlanarObstacle;
  typedef std::vector<PlanarObstacle> PlanarObstacleArray;
  typedef std::vector<PlanarObstacleArray> PlanarObstacleMatrix;
  typedef struct Fod_{
    VariablePtr k, tau, delay;
  } Fod;
  typedef struct DroneFodModel_{
    Fod x, y, z, yaw;
    VariablePtr sin_yaw, cos_yaw;
  } DroneFodModel;
  typedef std::vector<DroneFodModel> DroneFodModelArray;

//}}}

  class DroneMPCController{//{{{
    std::unique_ptr<mpc_solver::Solver> solver;
    StateArray states;
    StateArray r_states;
    StateArray max_states;
    State f_state;
    State rn_state;
    ControlArray controls;
    ControlArray r_controls;
    ControlArray max_controls;
    DroneFodModelArray model;
    EllipsoidalObstacleMatrix e_obstacles;
    PlanarObstacleMatrix p_obstacles;
    mpc_solver::VariablePtrVector ss, es, ps;
    mpc_solver::VariablePtrVector r_ss, r_es, r_ps;
    mpc_solver::VariablePtrVector sks;
    public:
    DroneMPCController();
    void linkData();
    ~DroneMPCController();
  };//}}}

}//}}}

#endif

//}}}

// drone_mpc_controller.cpp {{{

//#include "drone_mpc_controller.h"

drone_mpc_controller::DroneMPCController::DroneMPCController(){//{{{
  
  solver = std::unique_ptr<mpc_solver::Solver>(new mpc_acado_solver::AcadoSolver);

  linkData();

  //Add the publisher/subscribers

  solver->solve();

  std::cout<<"DroneMPCController created"<<std::endl;

}//}}}

drone_mpc_controller::DroneMPCController::~DroneMPCController(){//{{{
  std::cout<<"Drone MPC Controller deleted"<<std::endl;
}//}}}

void drone_mpc_controller::DroneMPCController::linkData(){//{{{
  mpc_solver::Parameters p;
  solver->getParameters(p);

  unsigned num_eobs = 10, num_pobs = 6;
  
  states.resize(p.n + 1);
  r_states.resize(p.n);
  max_states.resize(p.n + 1);
  controls.resize(p.n);
  r_controls.resize(p.n);
  max_controls.resize(p.n + 1);
  model.resize(p.n +1);
  e_obstacles.resize(p.n + 1, EllipsoidalObstacleArray(num_eobs));
  p_obstacles.resize(p.n + 1, PlanarObstacleArray(num_pobs));
  ss.resize(p.n);
  es.resize(p.n);
  ps.resize(p.n);
  r_ss.resize(p.n);
  r_es.resize(p.n);
  r_ps.resize(p.n);
  sks.resize(p.n + 1);
  
  unsigned count = 0;

  solver->linkFeedbackState(f_state.pose.position.x,       count++);
  solver->linkFeedbackState(f_state.pose.position.y,       count++);
  solver->linkFeedbackState(f_state.pose.position.z,       count++);
  solver->linkFeedbackState(f_state.pose.orientation.yaw,  count++);
  solver->linkFeedbackState(f_state.twist.linear.x,        count++);
  solver->linkFeedbackState(f_state.twist.linear.y,        count++);
  solver->linkFeedbackState(f_state.twist.linear.z,        count++);
  solver->linkFeedbackState(f_state.twist.angular.z,       count++);

  for( unsigned i = 0; i< p.n + 1; i++ ){
    count = 0;
    solver->linkState(states[i].pose.position.x,      i, count++);
    solver->linkState(states[i].pose.position.y,      i, count++);
    solver->linkState(states[i].pose.position.z,      i, count++);
    solver->linkState(states[i].pose.orientation.yaw, i, count++);
    solver->linkState(states[i].twist.linear.x,       i, count++);
    solver->linkState(states[i].twist.linear.y,       i, count++);
    solver->linkState(states[i].twist.linear.z,       i, count++);
    solver->linkState(states[i].twist.angular.z,      i, count++);
  }
  
  for( unsigned i = 0; i< p.n ; i++ ){
    count = 0;
    solver->linkControl(controls[i].linear.x,  i, count++);
    solver->linkControl(controls[i].linear.y,  i, count++);
    solver->linkControl(controls[i].linear.z,  i, count++);
    solver->linkControl(controls[i].angular.z, i, count++);

    solver->linkControl(ss[i], i, count++);
    solver->linkControl(es[i], i, count++);
    solver->linkControl(ps[i], i, count++);
  }

  for( unsigned i = 0; i < p.n +1; i++){
    count = 0;
    solver->linkParameter(model[i].sin_yaw,   i, count++);
    solver->linkParameter(model[i].cos_yaw,   i, count++);
    solver->linkParameter(model[i].x.k,       i, count++);
    solver->linkParameter(model[i].x.tau,     i, count++);
    solver->linkParameter(model[i].y.k,       i, count++);
    solver->linkParameter(model[i].y.tau,     i, count++);
    solver->linkParameter(model[i].z.k,       i, count++);
    solver->linkParameter(model[i].z.tau,     i, count++);
    solver->linkParameter(model[i].yaw.k,     i, count++);
    solver->linkParameter(model[i].yaw.tau,   i, count++);

    solver->linkParameter(max_states[i].twist.linear.x,  i, count++);
    solver->linkParameter(max_states[i].twist.linear.y,  i, count++);
    solver->linkParameter(max_states[i].twist.linear.z,  i, count++);
    solver->linkParameter(max_states[i].twist.angular.z, i, count++);

    solver->linkParameter(max_states[i].accel.linear.x,  i, count++);
    solver->linkParameter(max_states[i].accel.linear.y,  i, count++);
    solver->linkParameter(max_states[i].accel.linear.z,  i, count++);
    solver->linkParameter(max_states[i].accel.angular.z, i, count++);

    solver->linkParameter(sks[i], i, count++);

    solver->linkParameter(max_controls[i].linear.x,  i, count++);
    solver->linkParameter(max_controls[i].linear.y,  i, count++);
    solver->linkParameter(max_controls[i].linear.z,  i, count++);
    solver->linkParameter(max_controls[i].angular.z, i, count++);

    for(unsigned j = 0; j < num_eobs; j++)
      solver->linkParameter(e_obstacles[i][j].s_k, i, count);
    count++;
    
    for(unsigned j = 0; j < num_pobs; j++)
      solver->linkParameter(p_obstacles[i][j].s_k, i, count);
    count++;
    
    for(unsigned j = 0; j < num_eobs; j++){
      solver->linkParameter(e_obstacles[i][j].ellipsoid.position.x, i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.position.y, i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.position.z, i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.size.x, i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.size.y, i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.size.z, i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[0][0], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[0][1], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[0][2], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[1][0], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[1][1], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[1][2], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[2][0], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[2][1], i, count++);
      solver->linkParameter(e_obstacles[i][j].ellipsoid.rotation_matrix[2][2], i, count++);
    }
    for(unsigned j = 0; j < num_pobs; j++){
      solver->linkParameter(p_obstacles[i][j].plane.position.x, i, count++);
      solver->linkParameter(p_obstacles[i][j].plane.position.y, i, count++);
      solver->linkParameter(p_obstacles[i][j].plane.position.z, i, count++);
      solver->linkParameter(p_obstacles[i][j].plane.n_vector.x, i, count++);
      solver->linkParameter(p_obstacles[i][j].plane.n_vector.y, i, count++);
      solver->linkParameter(p_obstacles[i][j].plane.n_vector.z, i, count++);
    }

  }
  for( unsigned i = 0; i< p.n; i++ ){
    count = 0;
    solver->linkReference(r_states[i].pose.position.x,      i, count++);
    solver->linkReference(r_states[i].pose.position.y,      i, count++);
    solver->linkReference(r_states[i].pose.position.z,      i, count++);
    solver->linkReference(r_states[i].pose.orientation.yaw, i, count++);
    solver->linkReference(r_states[i].twist.linear.x,       i, count++);
    solver->linkReference(r_states[i].twist.linear.y,       i, count++);
    solver->linkReference(r_states[i].twist.linear.z,       i, count++);
    solver->linkReference(r_states[i].twist.angular.z,      i, count++);
    solver->linkReference(r_states[i].accel.linear.x,       i, count++);
    solver->linkReference(r_states[i].accel.linear.y,       i, count++);
    solver->linkReference(r_states[i].accel.linear.z,       i, count++);
    solver->linkReference(r_states[i].accel.angular.z,      i, count++);
  
    solver->linkReference(r_controls[i].linear.x,  i, count++);
    solver->linkReference(r_controls[i].linear.y,  i, count++);
    solver->linkReference(r_controls[i].linear.z,  i, count++);
    solver->linkReference(r_controls[i].angular.z, i, count++);
    
    solver->linkReference(r_ss[i], i, count++);
    solver->linkReference(r_es[i], i, count++);
    solver->linkReference(r_ps[i], i, count++);
  }
}//}}}

//}}}

#include <memory>

int main(){

  drone_mpc_controller::DroneMPCController mpc;

  //drone_mpc_controller::State feedback_state;
  //drone_mpc_controller::StateArray states;
  //states.resize(10);
  ////std::unique_ptr<mpc_solver::Solver> solver(new mpc_acado_solver::AcadoSolver);
  //std::unique_ptr<mpc_solver::Solver> solver;
  //solver = std::unique_ptr<mpc_solver::Solver>(new mpc_acado_solver::AcadoSolver);

  //solver->linkFeedbackState(feedback_state.pose.position.x,0);

  //*feedback_state.pose.position.x = 1;

  //std::cout<<"Test "<<acadoVariables.x0[0]<<std::endl;

  std::cout<<"Test "<<std::endl;

  return 0;
}
