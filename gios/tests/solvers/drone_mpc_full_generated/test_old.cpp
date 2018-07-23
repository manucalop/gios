#include <vector>
#include <array>

class Solver{/*{{{*/
  public:  //Solver data (as close as possible to the real data)
  //Dynamic data structures
  typedef struct State_{
    double x, y, z;
    double roll, pitch, yaw;
    double v_x, v_y, v_z;
    double v_roll, v_pitch, v_yaw;
    double a_x, a_y, a_z;
    double a_roll, a_pitch, a_yaw;
    double j_x, j_y, j_z;
    double j_roll, j_pitch, j_yaw;
  } State;
  typedef std::vector<State> StateArray;
  typedef struct Controls_{
    double u_x, u_y, u_z, u_yaw;
  } Controls;
  typedef std::vector<Controls> ControlsArray;
    //Static data structures
  typedef std::array<std::array<double,3>,3> RotationMatrix;
  typedef struct Ellipsoid_{
    double x, y, z;
    double r_x, r_y, r_z;
    RotationMatrix rotation_matrix;
  } Ellipsoid;
  typedef struct Plane_{
    double x, y, z;
    double a, b, c;
  } Plane;
  typedef struct EllipsoidalObstacle_{
    unsigned id;
    Ellipsoid ellipsoid;
    double s_k;//Sensitivity of the soft variable 1.0,2.5,...1000.0.
    double s_w;//Weight of the soft variable penalty
  } EllipsoidalObstacle;
  typedef std::vector<EllipsoidalObstacle> EllipsoidalObstacleArray;
  typedef struct PlanarObstacle_{
    unsigned id;
    Plane plane;
    double s_k;//Sensitivity of the soft variable 1.0,2.5,...1000.0.
    double s_w;//Weight of the soft variable penalty
  } PlanarObstacle;
  typedef std::vector<PlanarObstacle> PlanarObstacleArray;
  typedef struct Fod_{
    double k, tau, delay;
  } Fod;
  typedef struct DroneFodModel_{
    Fod x, y, z, yaw;
  } DroneFodModel;
  typedef std::vector<DroneFodModel> DroneFodModelArray;
  public:  
   // virtual ~Solver()=0;
  public:
    virtual void setState(const State &state_)=0;//Sensed state

    virtual void setReference(const State         &reference_)=0;//Static state reference
    virtual void setReference(const StateArray    &reference_)=0;//Dynamic state reference
    virtual void setReference(const Controls      &reference_)=0;//Static control reference
    virtual void setReference(const ControlsArray &reference_)=0;//Dynamic control reference

    virtual void setObstacle(const EllipsoidalObstacle      &obstacle_)=0;//Static obstacles
    virtual void setObstacle(const PlanarObstacle           &obstacle_)=0;
    virtual void setObstacle(const EllipsoidalObstacleArray &obstacle_)=0;//Dynamic obstacles
    virtual void setObstacle(const PlanarObstacleArray      &obstacle_)=0;

    virtual void setModel(const DroneFodModel       &model_)=0;//Static model
    virtual void setModel(const DroneFodModelArray  &model_)=0;//Dynamic model

    virtual void setBoundsMax(const State         &max_state_)=0;//Static bounds
    virtual void setBoundsMax(const Controls      &max_state_)=0;
    virtual void setBoundsMax(const StateArray    &max_state_)=0;//Dynamic bounds
    virtual void setBoundsMax(const ControlsArray &max_state_)=0;
    
    virtual void setWeights(const State                     &state_weights_)=0;//Static weights
    virtual void setWeights(const Controls                  &controls_weights_)=0;
    virtual void setWeights(const StateArray                &state_weights_)=0;//Dynamic weights
    virtual void setWeights(const ControlsArray             &controls_weights_)=0;
  public:
    virtual void solve()=0;
    virtual void simulate()=0;
  public:
    virtual void getControls(Controls      &controls_)const=0;
    virtual void getControls(ControlsArray &controls_)const=0;

    virtual void getTrajectory(StateArray  &states_)const=0;
};/*}}}*/

//Acado Solver libraries
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

//Needs to be global for cross compiling with C
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class AcadoSolver final : public Solver{//{{{
  public:
    typedef struct Parameters_{
      DroneFodModel model;  //Model of the System
      unsigned  sminmax_p;  //Soft minmax constraint parameters
      unsigned  hminmax_p;  //Hard minmax constraint parameters
      unsigned  model_p;    //Model parameters
      unsigned  eobs_p;     //Ellipsoidal obstacle parameters
      unsigned  pobs_p;     //Planar obstacle parameters

      unsigned  num_controls; //Number of controls
      unsigned  num_extra_s;  //Number of extra states
      unsigned  num_sminmax;  //Number of soft minmax constraints
      unsigned  num_hminmax;  //Number of hard minmax constraints
      unsigned  num_uminmax;  //Number of control minmax constraints
      unsigned  num_eobs;     //Number of ellipsoidal obstacles
      unsigned  num_pobs;     //Number of planar obstacles
    } Parameters;
  private:
    //Solver values
    unsigned n;   //Number of intervals in the horizon
    unsigned nx;  //Number of differential state variables
    unsigned nxa; //Number of algebraic variables
    unsigned nu;  //Number of controls
    unsigned nod; //Number of parameters (online data)
    unsigned ny;  //Number of references on nodes (0 ... n-1) 
    unsigned nyn; //Number of references on node n
    float t,dt;   //Time horizon and Dt


    //Set the order of the parameters in ACADOvariables.u[] << controls << minmax_slacks << obstacles_slacks <<...
    unsigned u_controls_offset;
    unsigned u_minmax_offset;
    unsigned u_eobs_offset;
    unsigned u_pobs_offset;
    
    //Set the order of the parameters in ACADOvariables.od[] << model << minmax << uminmax << eobs << pobs << ...
    unsigned od_model_offset;
    unsigned od_minmax_offset;
    unsigned od_uminmax_offset;
    unsigned od_eobs_offset;
    unsigned od_pobs_offset;
    unsigned od_eobs_p;
    unsigned od_pobs_p;
    
    //Position of the variables in ACADOvariables.y[] << states << controls << minmax << eobs << pobs <<...
    unsigned y_states_offset;
    unsigned y_controls_offset;
    unsigned y_minmax_offset;
    unsigned y_eobs_offset;
    unsigned y_pobs_offset;
  public:  
    explicit AcadoSolver(const AcadoSolver::Parameters parameters);
    ~AcadoSolver();
  public:
    void setState(const State &state_) override;//Sensed state

    void setReference(const State         &reference_) override;//Static state reference
    void setReference(const StateArray    &reference_) override;//Dynamic state reference
    void setReference(const Controls      &reference_) override;//Static control reference
    void setReference(const ControlsArray &reference_) override;//Dynamic control reference

    void setObstacle(const EllipsoidalObstacle      &obstacle_) override;//Static obstacles
    void setObstacle(const PlanarObstacle           &obstacle_) override;
    void setObstacle(const EllipsoidalObstacleArray &obstacle_) override;//Dynamic obstacles
    void setObstacle(const PlanarObstacleArray      &obstacle_) override;

    void setModel(const DroneFodModel       &model_) override;//Static model
    void setModel(const DroneFodModelArray  &model_) override;//Dynamic model

    void setBoundsMax(const State         &max_state_) override;//Static bounds
    void setBoundsMax(const Controls      &max_state_) override;
    void setBoundsMax(const StateArray    &max_state_) override;//Dynamic bounds
    void setBoundsMax(const ControlsArray &max_state_) override;
    
    void setWeights(const State                     &state_weights_) override;//Static weights
    void setWeights(const Controls                  &controls_weights_) override;
    void setWeights(const StateArray                &state_weights_) override;//Dynamic weights
    void setWeights(const ControlsArray             &controls_weights_) override;
  public:
    virtual void solve() override;
    virtual void simulate() override;
  public:
    virtual void getControls(Controls      &controls_)const override;
    virtual void getControls(ControlsArray &controls_)const override;

    virtual void getTrajectory(StateArray  &states_)const override;
};/*}}}*/

// acado_solver.cpp

AcadoSolver::AcadoSolver(const AcadoSolver::Parameters parameters){//{{{ Constructor

  n   = ACADO_N;
  nx  = ACADO_NX;
  nu  = ACADO_NU;
  nod = ACADO_NOD;
  ny  = ACADO_NY; 
  nyn = ACADO_NYN;
  
  unsigned u_num_controls  = parameters.num_controls;
  unsigned u_num_minmax    = parameters.num_sminmax;
  unsigned u_num_eobs      = parameters.num_eobs;
  unsigned u_num_pobs      = parameters.num_pobs;
  
  unsigned od_num_model_p  = parameters.model_p;
  unsigned od_num_minmax   = parameters.sminmax_p*parameters.num_sminmax + 
                             parameters.hminmax_p*parameters.num_hminmax;
  unsigned od_num_uminmax  = parameters.num_uminmax;
  unsigned od_num_eobs     = parameters.num_eobs;
  unsigned od_num_pobs     = parameters.num_pobs;

  unsigned y_num_states    = nx + parameters.num_extra_s;
  unsigned y_num_controls  = parameters.num_controls;
  unsigned y_num_minmax    = parameters.num_sminmax;
  unsigned y_num_eobs      = parameters.num_eobs;
  unsigned y_num_pobs      = parameters.num_pobs;

  //Set the order of the parameters in ACADOvariables.u[] << controls << minmax_slacks << obstacles_slacks <<...
  u_controls_offset = 0;
  u_minmax_offset   = u_controls_offset + u_num_controls;
  u_eobs_offset     = u_minmax_offset   + u_num_minmax;
  u_pobs_offset     = u_eobs_offset     + u_num_eobs;
  
  //Set the order of the parameters in ACADOvariables.od[] << model << minmax << uminmax << eobs << pobs << ...
  od_model_offset   = 0;
  od_minmax_offset  = od_model_offset   + od_num_model_p;
  od_uminmax_offset = od_minmax_offset  + od_num_minmax;
  od_eobs_offset    = od_uminmax_offset + od_num_uminmax;
  od_pobs_offset    = od_eobs_offset    + od_eobs_p*od_num_eobs;
  od_eobs_p         = parameters.eobs_p;
  od_pobs_p         = parameters.pobs_p;
  
  //Position of the variables in ACADOvariables.y[] << states << controls << minmax << eobs << pobs <<...
  y_states_offset   = 0;
  y_controls_offset = y_states_offset   + y_num_states;
  y_minmax_offset   = y_controls_offset + y_num_controls;
  y_eobs_offset     = y_minmax_offset   + y_num_minmax;
  y_pobs_offset     = y_eobs_offset     + y_num_eobs;

  // Initialize the solver. 
  acado_initializeSolver();
  // Initialize Trajectory
  for (unsigned i = 0; i < nx * (n + 1); ++i)  acadoVariables.x[ i ] = 0.0;
  // Initialize Controls
  for (unsigned i = 0; i < nu * n; ++i)  acadoVariables.u[ i ] = 0.0;
  // Initialize online data
  for (unsigned i = 0; i < nod * (n + 1); ++i)  acadoVariables.od[ i ] = 0.0;
  // Initialize weights
  for (unsigned i = 0; i < ny*ny*n; ++i)  acadoVariables.W[ i ] = 0.0;
  for (unsigned i = 0; i < nyn*nyn; ++i)  acadoVariables.WN[ i ] = 0.0;
  // Initialize the reference. 
  for (unsigned i = 0; i < ny * n; ++i)  acadoVariables.y[ i ] = 0.0;
  for (unsigned i = 0; i < nyn; ++i)  acadoVariables.yN[ i ] = 0.0;
  //Initialize the current state feedback. 
  for (unsigned i = 0; i < nx; ++i) acadoVariables.x0[ i ] = 0.0;

  State initial_state = {};
  initial_state.x   = -10;
  initial_state.y   = -10;
  initial_state.z   = -10;
  initial_state.yaw = 0;
  initial_state.v_x = 0;
  initial_state.v_y = 0;
  initial_state.v_z = 0;
  initial_state.v_yaw = 0;

  setState(initial_state);

  State initial_reference = {};
  initial_reference.x     = 10;
  initial_reference.y     = 10;
  initial_reference.z     = 10;
  initial_reference.yaw   = 0;
  initial_reference.v_x   = 0;
  initial_reference.v_y   = 0;
  initial_reference.v_z   = 0;
  initial_reference.v_yaw = 0;

  setReference(initial_reference);

  EllipsoidalObstacle initial_eobstacle ={};
  initial_eobstacle.ellipsoid.x = 0;
  initial_eobstacle.ellipsoid.y = 0;
  initial_eobstacle.ellipsoid.z = -1000;
  initial_eobstacle.ellipsoid.r_x = 1;
  initial_eobstacle.ellipsoid.r_y = 1;
  initial_eobstacle.ellipsoid.r_z = 1;
  initial_eobstacle.ellipsoid.rotation_matrix[0][0] = 1;
  initial_eobstacle.ellipsoid.rotation_matrix[1][1] = 1;
  initial_eobstacle.ellipsoid.rotation_matrix[2][2] = 1;
  initial_eobstacle.s_k = 1;
  initial_eobstacle.s_w = 1;

  for (unsigned i= 0; i<= parameters.num_eobs; i++){
    initial_eobstacle.id = i;
    setObstacle(initial_eobstacle);
  }

  PlanarObstacle initial_pobstacle = {};
  initial_pobstacle.plane.x = 0;
  initial_pobstacle.plane.y = 0;
  initial_pobstacle.plane.z = -1000;
  initial_pobstacle.plane.a = 0;
  initial_pobstacle.plane.b = 0;
  initial_pobstacle.plane.c = 1;
  initial_pobstacle.s_k = 1;
  initial_pobstacle.s_w = 1;
  for (unsigned i= 0; i<= parameters.num_pobs; i++){
    initial_pobstacle.id = i;
    setObstacle(initial_eobstacle);
  }

  setModel(parameters.model);

  State max_state = {1000};
  max_state.v_x = 1000;
  max_state.v_y = 1000;
  max_state.v_z = 1000;
  max_state.v_yaw = 1000;
  max_state.a_x = 1000;
  max_state.a_y = 1000;
  max_state.a_z = 1000;
  max_state.a_yaw = 1000;
  setBoundsMax(max_state);

  Controls max_controls = {1000};
  max_controls.u_x = 1000;
  max_controls.u_y = 1000;
  max_controls.u_z = 1000;
  max_controls.u_yaw = 1000;
  setBoundsMax(max_controls);

//  State state_weights = {1};
//  setWeights(state_weights);
//  state_weights.x     = 1;
//  state_weights.y     = 1;
//  state_weights.z     = 1;
//  state_weights.yaw   = 1;
//  state_weights.v_x   = 0;
//  state_weights.v_y   = 0;
//  state_weights.v_z   = 0;
//  state_weights.v_yaw = 0;
//  state_weights.a_x   = 0;
//  state_weights.a_y   = 0;
//  state_weights.a_z   = 0;
//  state_weights.a_yaw = 0;
//  Controls controls_weights = {1};
//  controls_weights.u_x   = 1;
//  controls_weights.u_y   = 1;
//  controls_weights.u_z   = 1;
//  controls_weights.u_yaw = 1;
//  setWeights(controls_weights);

};//}}}

AcadoSolver::~AcadoSolver(){};

void AcadoSolver::setState(const State &state_){//{{{ Sensed state
  acadoVariables.x0[ 0 ] = state_.x;
  acadoVariables.x0[ 1 ] = state_.y;
  acadoVariables.x0[ 2 ] = state_.z;
  acadoVariables.x0[ 3 ] = state_.yaw;
  acadoVariables.x0[ 4 ] = state_.v_x;
  acadoVariables.x0[ 5 ] = state_.v_y;
  acadoVariables.x0[ 6 ] = state_.v_z;
  acadoVariables.x0[ 7 ] = state_.v_yaw;
	for (unsigned i = 0; i < n + 1; ++i)
	{
		acadoVariables.od[ od_model_offset + 0 + i*nod ] = sin(state_.yaw);
		acadoVariables.od[ od_model_offset + 1 + i*nod ] = cos(state_.yaw);
	}
};//}}}

void AcadoSolver::setReference(const State &reference_){//{{{ Static state reference
  for (unsigned i = 0; i < n; ++i)
	{
	 	acadoVariables.y[ y_states_offset + 0  + i*ny ] = reference_.x;//x_r
	 	acadoVariables.y[ y_states_offset + 1  + i*ny ] = reference_.y;//y_r
		acadoVariables.y[ y_states_offset + 2  + i*ny ] = reference_.z;//z_r
		acadoVariables.y[ y_states_offset + 3  + i*ny ] = reference_.yaw;//yaw_r
		acadoVariables.y[ y_states_offset + 4  + i*ny ] = reference_.v_x;//vf_r
		acadoVariables.y[ y_states_offset + 5  + i*ny ] = reference_.v_y;//vs_r
		acadoVariables.y[ y_states_offset + 6  + i*ny ] = reference_.v_z;//vu_r
		acadoVariables.y[ y_states_offset + 7  + i*ny ] = reference_.v_yaw;//vh_r
		acadoVariables.y[ y_states_offset + 8  + i*ny ] = reference_.a_x;//vf_r
		acadoVariables.y[ y_states_offset + 9  + i*ny ] = reference_.a_y;//vs_r
		acadoVariables.y[ y_states_offset + 10 + i*ny ] = reference_.a_z;//vu_r
		acadoVariables.y[ y_states_offset + 11 + i*ny ] = reference_.a_yaw;//vh_r
	}

  acadoVariables.yN[ y_states_offset + 0 ] = reference_.x;//x_r
  acadoVariables.yN[ y_states_offset + 1 ] = reference_.y;//y_r
  acadoVariables.yN[ y_states_offset + 2 ] = reference_.z;//z_r
  acadoVariables.yN[ y_states_offset + 3 ] = reference_.yaw;//yaw_r
  acadoVariables.yN[ y_states_offset + 4 ] = reference_.v_x;//vf_r
  acadoVariables.yN[ y_states_offset + 5 ] = reference_.v_y;//vs_r
  acadoVariables.yN[ y_states_offset + 6 ] = reference_.v_z;//vu_r
  acadoVariables.yN[ y_states_offset + 7 ] = reference_.v_yaw;//vh_r
};//}}}

void AcadoSolver::setReference(const StateArray &reference_){//{{{ Dynamic state reference
  for (unsigned i = 0; i < n; ++i)
	{
	 	acadoVariables.y[ y_states_offset + 0  + i*ny ] = reference_[i].x;//x_r
	 	acadoVariables.y[ y_states_offset + 1  + i*ny ] = reference_[i].y;//y_r
		acadoVariables.y[ y_states_offset + 2  + i*ny ] = reference_[i].z;//z_r
		acadoVariables.y[ y_states_offset + 3  + i*ny ] = reference_[i].yaw;//yaw_r
		acadoVariables.y[ y_states_offset + 4  + i*ny ] = reference_[i].v_x;//vf_r
		acadoVariables.y[ y_states_offset + 5  + i*ny ] = reference_[i].v_y;//vs_r
		acadoVariables.y[ y_states_offset + 6  + i*ny ] = reference_[i].v_z;//vu_r
		acadoVariables.y[ y_states_offset + 7  + i*ny ] = reference_[i].v_yaw;//vh_r
		acadoVariables.y[ y_states_offset + 8  + i*ny ] = reference_[i].a_x;//vf_r
		acadoVariables.y[ y_states_offset + 9  + i*ny ] = reference_[i].a_y;//vs_r
		acadoVariables.y[ y_states_offset + 10 + i*ny ] = reference_[i].a_z;//vu_r
		acadoVariables.y[ y_states_offset + 11 + i*ny ] = reference_[i].a_yaw;//vh_r
	}

  acadoVariables.yN[ y_states_offset + 0 ] = reference_[n].x;//x_r
  acadoVariables.yN[ y_states_offset + 1 ] = reference_[n].y;//y_r
  acadoVariables.yN[ y_states_offset + 2 ] = reference_[n].z;//z_r
  acadoVariables.yN[ y_states_offset + 3 ] = reference_[n].yaw;//yaw_r
  acadoVariables.yN[ y_states_offset + 4 ] = reference_[n].v_x;//vf_r
  acadoVariables.yN[ y_states_offset + 5 ] = reference_[n].v_y;//vs_r
  acadoVariables.yN[ y_states_offset + 6 ] = reference_[n].v_z;//vu_r
  acadoVariables.yN[ y_states_offset + 7 ] = reference_[n].v_yaw;//vh_r
};//}}}

void AcadoSolver::setReference(const Controls &reference_){//{{{ Static control reference
  for (unsigned i = 0; i < n; ++i)
	{
		acadoVariables.y[ y_controls_offset + 0 + i*ny ] = reference_.u_x;//ux_r
		acadoVariables.y[ y_controls_offset + 1 + i*ny ] = reference_.u_y;//uy_r
		acadoVariables.y[ y_controls_offset + 2 + i*ny ] = reference_.u_z;//uz_r
		acadoVariables.y[ y_controls_offset + 3 + i*ny ] = reference_.u_yaw;//uyaw_r
	}
};//}}}

void AcadoSolver::setReference(const ControlsArray &reference_){//{{{ Dynamic control reference
  for (unsigned i = 0; i < n; ++i)
	{
		acadoVariables.y[ y_controls_offset + 0 + i*ny ] = reference_[i].u_x;//ux_r
		acadoVariables.y[ y_controls_offset + 1 + i*ny ] = reference_[i].u_y;//uy_r
		acadoVariables.y[ y_controls_offset + 2 + i*ny ] = reference_[i].u_z;//uz_r
		acadoVariables.y[ y_controls_offset + 3 + i*ny ] = reference_[i].u_yaw;//uyaw_r
	}
};///}}}

void AcadoSolver::setObstacle(const EllipsoidalObstacle &obstacle_){//{{{ Static obstacle
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_eobs_offset + 0  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.x;
		acadoVariables.od[ od_eobs_offset + 1  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.y;
		acadoVariables.od[ od_eobs_offset + 2  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.z;
		acadoVariables.od[ od_eobs_offset + 3  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.r_x;
		acadoVariables.od[ od_eobs_offset + 4  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.r_y;
		acadoVariables.od[ od_eobs_offset + 5  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.r_z;
		acadoVariables.od[ od_eobs_offset + 6  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[0][0];
		acadoVariables.od[ od_eobs_offset + 7  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[0][1];
		acadoVariables.od[ od_eobs_offset + 8  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[0][2];
		acadoVariables.od[ od_eobs_offset + 9  + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[1][0];
		acadoVariables.od[ od_eobs_offset + 10 + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[1][1];
		acadoVariables.od[ od_eobs_offset + 11 + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[1][2];
		acadoVariables.od[ od_eobs_offset + 12 + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[2][0];
		acadoVariables.od[ od_eobs_offset + 13 + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[2][1];
		acadoVariables.od[ od_eobs_offset + 14 + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.ellipsoid.rotation_matrix[2][2];
    acadoVariables.od[ od_eobs_offset + 15 + od_eobs_p*obstacle_.id + i*nod ] = obstacle_.s_k;//Sensitivity
  }
  for (unsigned i = 0 ; i < n ; i++){
		acadoVariables.W[ i*(ny*ny) + (y_eobs_offset + obstacle_.id)*(ny+1) ] = obstacle_.s_w;//Weight
  }
};//}}}

void AcadoSolver::setObstacle(const PlanarObstacle &obstacle_){//{{{
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_pobs_offset + 0 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.x;
		acadoVariables.od[ od_pobs_offset + 1 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.y;
		acadoVariables.od[ od_pobs_offset + 2 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.z;
		acadoVariables.od[ od_pobs_offset + 3 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.a;
		acadoVariables.od[ od_pobs_offset + 4 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.b;
		acadoVariables.od[ od_pobs_offset + 5 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.c;
		acadoVariables.od[ od_pobs_offset + 6 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.s_k;//Sensitivity
  }	
  for (unsigned i = 0 ; i < n ; i++){
    acadoVariables.W[ i*(ny*ny) + (y_pobs_offset + obstacle_.id)*(ny+1) ] = obstacle_.s_w;//Weight
  }
};//}}}

void AcadoSolver::setObstacle(const EllipsoidalObstacleArray &obstacle_){//{{{ Dynamic obstacle
  for (unsigned i = 0 ; i < n + 1 ; i++){//We use the same id since is the same obstacle propagated through the horizon
		acadoVariables.od[ od_eobs_offset + 0  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.x;
		acadoVariables.od[ od_eobs_offset + 1  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.y;
		acadoVariables.od[ od_eobs_offset + 2  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.z;
		acadoVariables.od[ od_eobs_offset + 3  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.r_x;
		acadoVariables.od[ od_eobs_offset + 4  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.r_y;
		acadoVariables.od[ od_eobs_offset + 5  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.r_z;
		acadoVariables.od[ od_eobs_offset + 6  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[0][0];
		acadoVariables.od[ od_eobs_offset + 7  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[0][1];
		acadoVariables.od[ od_eobs_offset + 8  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[0][2];
		acadoVariables.od[ od_eobs_offset + 9  + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[1][0];
		acadoVariables.od[ od_eobs_offset + 10 + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[1][1];
		acadoVariables.od[ od_eobs_offset + 11 + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[1][2];
		acadoVariables.od[ od_eobs_offset + 12 + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[2][0];
		acadoVariables.od[ od_eobs_offset + 13 + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[2][1];
		acadoVariables.od[ od_eobs_offset + 14 + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].ellipsoid.rotation_matrix[2][2];
    acadoVariables.od[ od_eobs_offset + 15 + od_eobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].s_k;//Sensitivity
  }	
  for (unsigned i = 0 ; i < n ; i++){
    acadoVariables.W[ i*(ny*ny) + (y_eobs_offset + obstacle_[0].id)*(ny+1) ]=obstacle_[i].s_w;//Weight
  }
};//}}}

void AcadoSolver::setObstacle(const PlanarObstacleArray &obstacle_){//{{{
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_pobs_offset + 0 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.x;
		acadoVariables.od[ od_pobs_offset + 1 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.y;
		acadoVariables.od[ od_pobs_offset + 2 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.z;
		acadoVariables.od[ od_pobs_offset + 3 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.a;
		acadoVariables.od[ od_pobs_offset + 4 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.b;
		acadoVariables.od[ od_pobs_offset + 5 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.c;
		acadoVariables.od[ od_pobs_offset + 6 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].s_k;//Sensitivity
  }  
  for (unsigned i = 0 ; i < n ; i++){
    acadoVariables.W[ i*(ny*ny) + (y_pobs_offset + obstacle_[0].id)*(ny+1) ]=obstacle_[i].s_w;//Weight
  }
};//}}}

void AcadoSolver::setModel(const DroneFodModel &model_){//{{{ Static model
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_model_offset + 2  + i*nod ] = model_.x.k;
		acadoVariables.od[ od_model_offset + 3  + i*nod ] = model_.x.tau;
		
    acadoVariables.od[ od_model_offset + 4  + i*nod ] = model_.y.k;
		acadoVariables.od[ od_model_offset + 5  + i*nod ] = model_.y.tau;
		
    acadoVariables.od[ od_model_offset + 6  + i*nod ] = model_.z.k;
		acadoVariables.od[ od_model_offset + 7  + i*nod ] = model_.z.tau;
    
		acadoVariables.od[ od_model_offset + 8  + i*nod ] = model_.yaw.k;
		acadoVariables.od[ od_model_offset + 9  + i*nod ] = model_.yaw.tau;
  }
};//}}}

void AcadoSolver::setModel(const DroneFodModelArray &model_){//{{{ Dynamic model
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_model_offset + 2  + i*nod ] = model_[i].x.k;
		acadoVariables.od[ od_model_offset + 3  + i*nod ] = model_[i].x.tau;
		
    acadoVariables.od[ od_model_offset + 4  + i*nod ] = model_[i].y.k;
		acadoVariables.od[ od_model_offset + 5  + i*nod ] = model_[i].y.tau;
		
    acadoVariables.od[ od_model_offset + 6  + i*nod ] = model_[i].z.k;
		acadoVariables.od[ od_model_offset + 7  + i*nod ] = model_[i].z.tau;
    
		acadoVariables.od[ od_model_offset + 8  + i*nod ] = model_[i].yaw.k;
		acadoVariables.od[ od_model_offset + 9  + i*nod ] = model_[i].yaw.tau;
  }
};//}}}

void AcadoSolver::setBoundsMax(const State &max_state_){//{{{Static bounds
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_minmax_offset + 0  + i*nod ] = max_state_.v_x;
		acadoVariables.od[ od_minmax_offset + 1  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 2  + i*nod ] = max_state_.v_y;
		acadoVariables.od[ od_minmax_offset + 3  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 4  + i*nod ] = max_state_.v_z;
		acadoVariables.od[ od_minmax_offset + 5  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 6  + i*nod ] = max_state_.v_yaw;
		acadoVariables.od[ od_minmax_offset + 7  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 8  + i*nod ] = max_state_.a_x;
		acadoVariables.od[ od_minmax_offset + 9  + i*nod ] = max_state_.a_y;
		acadoVariables.od[ od_minmax_offset + 10 + i*nod ] = max_state_.a_z;
		acadoVariables.od[ od_minmax_offset + 11 + i*nod ] = max_state_.a_yaw;
  }
};//}}}

void AcadoSolver::setBoundsMax(const StateArray &max_state_){//{{{Dynamic State bounds
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_minmax_offset + 0  + i*nod ] = max_state_[i].v_x;
		acadoVariables.od[ od_minmax_offset + 1  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 2  + i*nod ] = max_state_[i].v_y;
		acadoVariables.od[ od_minmax_offset + 3  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 4  + i*nod ] = max_state_[i].v_z;
		acadoVariables.od[ od_minmax_offset + 5  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 6  + i*nod ] = max_state_[i].v_yaw;
		acadoVariables.od[ od_minmax_offset + 7  + i*nod ] = 10;//Sensitivity
		acadoVariables.od[ od_minmax_offset + 8  + i*nod ] = max_state_[i].a_x;
		acadoVariables.od[ od_minmax_offset + 9  + i*nod ] = max_state_[i].a_y;
		acadoVariables.od[ od_minmax_offset + 10 + i*nod ] = max_state_[i].a_z;
		acadoVariables.od[ od_minmax_offset + 11 + i*nod ] = max_state_[i].a_yaw;
  }
};//}}}

void AcadoSolver::setBoundsMax(const Controls &max_controls_){//{{{
  for (unsigned i = 0 ; i < n + 1; i++){
		acadoVariables.od[ od_uminmax_offset + 0 + i*nod ] = max_controls_.u_x;
		acadoVariables.od[ od_uminmax_offset + 1 + i*nod ] = max_controls_.u_y;
		acadoVariables.od[ od_uminmax_offset + 2 + i*nod ] = max_controls_.u_z;
		acadoVariables.od[ od_uminmax_offset + 3 + i*nod ] = max_controls_.u_yaw;
  }
};//}}}

void AcadoSolver::setBoundsMax(const ControlsArray &max_controls_){//{{{
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_uminmax_offset + 0 + i*nod ] = max_controls_[i].u_x;
		acadoVariables.od[ od_uminmax_offset + 1 + i*nod ] = max_controls_[i].u_y;
		acadoVariables.od[ od_uminmax_offset + 2 + i*nod ] = max_controls_[i].u_z;
		acadoVariables.od[ od_uminmax_offset + 3 + i*nod ] = max_controls_[i].u_yaw;
  }
};//}}}

void AcadoSolver::setWeights(const State &state_weights_){//{{{Static weights
  for (unsigned i = 0; i < n; ++i)
	{
	 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 0 )*(ny+1) ] = state_weights_.x;//x_r
	 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 1 )*(ny+1) ] = state_weights_.y;//y_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 2 )*(ny+1) ] = state_weights_.z;//z_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 3 )*(ny+1) ] = state_weights_.yaw;//yaw_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 4 )*(ny+1) ] = state_weights_.v_x;//vf_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 5 )*(ny+1) ] = state_weights_.v_y;//vs_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 6 )*(ny+1) ] = state_weights_.v_z;//vu_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 7 )*(ny+1) ] = state_weights_.v_yaw;//vh_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 8 )*(ny+1) ] = state_weights_.a_x;//vf_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 9 )*(ny+1) ] = state_weights_.a_y;//vs_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 10)*(ny+1) ] = state_weights_.a_z;//vu_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 11)*(ny+1) ] = state_weights_.a_yaw;//vh_r
	}

	acadoVariables.WN[ (y_states_offset + 0 )*(ny+1) ] = state_weights_.x;//x_r
	acadoVariables.WN[ (y_states_offset + 1 )*(ny+1) ] = state_weights_.y;//y_r
	acadoVariables.WN[ (y_states_offset + 2 )*(ny+1) ] = state_weights_.z;//z_r
	acadoVariables.WN[ (y_states_offset + 3 )*(ny+1) ] = state_weights_.yaw;//yaw_r
	acadoVariables.WN[ (y_states_offset + 4 )*(ny+1) ] = state_weights_.v_x;//vf_r
	acadoVariables.WN[ (y_states_offset + 5 )*(ny+1) ] = state_weights_.v_y;//vs_r
	acadoVariables.WN[ (y_states_offset + 6 )*(ny+1) ] = state_weights_.v_z;//vu_r
	acadoVariables.WN[ (y_states_offset + 7 )*(ny+1) ] = state_weights_.v_yaw;//vh_r
	acadoVariables.WN[ (y_states_offset + 8 )*(ny+1) ] = state_weights_.a_x;//vf_r
	acadoVariables.WN[ (y_states_offset + 9 )*(ny+1) ] = state_weights_.a_y;//vs_r
	acadoVariables.WN[ (y_states_offset + 10)*(ny+1) ] = state_weights_.a_z;//vu_r
	acadoVariables.WN[ (y_states_offset + 11)*(ny+1) ] = state_weights_.a_yaw;//vh_r

};//}}}

void AcadoSolver::setWeights(const StateArray &state_weights_){//{{{Static weights
  for (unsigned i = 0; i < n; ++i)
	{
	 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 0 )*(ny+1) ] = state_weights_[i].x;//x_r
	 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 1 )*(ny+1) ] = state_weights_[i].y;//y_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 2 )*(ny+1) ] = state_weights_[i].z;//z_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 3 )*(ny+1) ] = state_weights_[i].yaw;//yaw_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 4 )*(ny+1) ] = state_weights_[i].v_x;//vf_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 5 )*(ny+1) ] = state_weights_[i].v_y;//vs_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 6 )*(ny+1) ] = state_weights_[i].v_z;//vu_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 7 )*(ny+1) ] = state_weights_[i].v_yaw;//vh_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 8 )*(ny+1) ] = state_weights_[i].a_x;//vf_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 9 )*(ny+1) ] = state_weights_[i].a_y;//vs_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 10)*(ny+1) ] = state_weights_[i].a_z;//vu_r
		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 11)*(ny+1) ] = state_weights_[i].a_yaw;//vh_r
	}

	acadoVariables.WN[ (y_states_offset + 0 )*(ny+1) ] = state_weights_[n].x;//x_r
	acadoVariables.WN[ (y_states_offset + 1 )*(ny+1) ] = state_weights_[n].y;//y_r
	acadoVariables.WN[ (y_states_offset + 2 )*(ny+1) ] = state_weights_[n].z;//z_r
	acadoVariables.WN[ (y_states_offset + 3 )*(ny+1) ] = state_weights_[n].yaw;//yaw_r
	acadoVariables.WN[ (y_states_offset + 4 )*(ny+1) ] = state_weights_[n].v_x;//vf_r
	acadoVariables.WN[ (y_states_offset + 5 )*(ny+1) ] = state_weights_[n].v_y;//vs_r
	acadoVariables.WN[ (y_states_offset + 6 )*(ny+1) ] = state_weights_[n].v_z;//vu_r
	acadoVariables.WN[ (y_states_offset + 7 )*(ny+1) ] = state_weights_[n].v_yaw;//vh_r
	acadoVariables.WN[ (y_states_offset + 8 )*(ny+1) ] = state_weights_[n].a_x;//vf_r
	acadoVariables.WN[ (y_states_offset + 9 )*(ny+1) ] = state_weights_[n].a_y;//vs_r
	acadoVariables.WN[ (y_states_offset + 10)*(ny+1) ] = state_weights_[n].a_z;//vu_r
	acadoVariables.WN[ (y_states_offset + 11)*(ny+1) ] = state_weights_[n].a_yaw;//vh_r

};//}}}

void AcadoSolver::setWeights(const Controls &controls_weights_){//{{{ 
  for (unsigned i = 0; i < n; ++i)
	{
	 	acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 0)*(ny+1) ] = controls_weights_.u_x;//x_r
	 	acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 1)*(ny+1) ] = controls_weights_.u_y;//y_r
		acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 2)*(ny+1) ] = controls_weights_.u_z;//z_r
		acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 3)*(ny+1) ] = controls_weights_.u_yaw;//yaw_r
	}
};//}}}

void AcadoSolver::setWeights(const ControlsArray &controls_weights_){//{{{ 
  for (unsigned i = 0; i < n; ++i)
	{
	 	acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 0)*(ny+1) ] = controls_weights_[i].u_x;//x_r
	 	acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 1)*(ny+1) ] = controls_weights_[i].u_y;//y_r
		acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 2)*(ny+1) ] = controls_weights_[i].u_z;//z_r
		acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 3)*(ny+1) ] = controls_weights_[i].u_yaw;//yaw_r
	}
};//}}}

void AcadoSolver::solve(){//{{{
  acado_feedbackStep();
};//}}}

void AcadoSolver::simulate(){//{{{
  acado_preparationStep();
};//}}}

void AcadoSolver::getControls(Controls &controls_)const{//{{{
  controls_.u_x   = acadoVariables.u[0];
  controls_.u_y   = acadoVariables.u[1];
  controls_.u_z   = acadoVariables.u[2];
  controls_.u_yaw = acadoVariables.u[3];
};//}}}

void AcadoSolver::getControls(ControlsArray &controls_)const{//{{{
  for( unsigned i = 0; i < n; i++ ){
    controls_[i].u_x   = acadoVariables.u[ 0 + i*nu ];
    controls_[i].u_y   = acadoVariables.u[ 1 + i*nu ];
    controls_[i].u_z   = acadoVariables.u[ 2 + i*nu ];
    controls_[i].u_yaw = acadoVariables.u[ 3 + i*nu ];
  }
};//}}}

void AcadoSolver::getTrajectory(StateArray  &states_)const{//{{{
  for( unsigned i = 0; i < n + 1; i++ ){
    states_[i].x     = acadoVariables.x[ 0 + i*nx ];
    states_[i].y     = acadoVariables.x[ 1 + i*nx ];
    states_[i].z     = acadoVariables.x[ 2 + i*nx ];
    states_[i].yaw   = acadoVariables.x[ 3 + i*nx ];
    states_[i].v_x   = acadoVariables.x[ 4 + i*nx ];
    states_[i].v_y   = acadoVariables.x[ 5 + i*nx ];
    states_[i].v_z   = acadoVariables.x[ 6 + i*nx ];
    states_[i].v_yaw = acadoVariables.x[ 7 + i*nx ];
    
  }
};//}}}



//void AcadoSolver::setState(State state_){};
//void AcadoSolver::setState(drone_mpc_msgs::State::ConstPtr& state_){//{{{
//  drone_state = *state_;
//  double roll, pitch, yaw;
//  double roll_ref, pitch_ref, yaw_ref;
//  //Set position
//	acadoVariables.x0[ 0 ] = drone_state.pose.position.x;
//	acadoVariables.x0[ 1 ] = drone_state.pose.position.y;
//	acadoVariables.x0[ 2 ] = drone_state.pose.position.z;
//  //Set relative orientation
//  tf2::Quaternion robot_orientation(\
//			drone_state.pose.orientation.x,\
//			drone_state.pose.orientation.y,\
//			drone_state.pose.orientation.z,\
//			drone_state.pose.orientation.w);
//	tf2::Matrix3x3(robot_orientation).getEulerYPR(yaw, pitch, roll);
//  
//  tf2::Quaternion reference_orientation(\
//			drone_state_reference.states[0].state.pose.orientation.x,\
//			drone_state_reference.states[0].state.pose.orientation.y,\
//			drone_state_reference.states[0].state.pose.orientation.z,\
//			drone_state_reference.states[0].state.pose.orientation.w);
//	tf2::Matrix3x3(reference_orientation).getEulerYPR(yaw_ref, pitch_ref, roll_ref);
//  
//  acadoVariables.x0[ 3 ] = atan2( sin(yaw - yaw_ref), cos(yaw - yaw_ref));
//  //Set body frame velocities
//	acadoVariables.x0[ 4 ] = drone_state.twist.linear.x;
//	acadoVariables.x0[ 5 ] = drone_state.twist.linear.y;
//	acadoVariables.x0[ 6 ] = drone_state.twist.linear.z;
//	acadoVariables.x0[ 7 ] = drone_state.twist.angular.z; 
//
//	//Update yaw (constant for each iteration -- Linearized model) 
//	for (unsigned i = 0; i < n; ++i)
//	{
//		acadoVariables.od[ model_offset + 0 + i*nod ] = sin(yaw);
//		acadoVariables.od[ model_offset + 1 + i*nod ] = cos(yaw);
//	}
//};//}}}

//void AcadoSolver::setReference(Trajectory reference_){};
//void AcadoSolver::setReference(drone_mpc_msgs::Trajectory::ConstPtr& reference_){//{{{
//	drone_state_reference = *reference_;
//
//	for (unsigned i = 0; i < n; ++i)
//	{
//	 	acadoVariables.y[ i*n + 0 ] = drone_state_reference.states[i].state.pose.position.x;//x_r
//	 	acadoVariables.y[ i*n + 1 ] = drone_state_reference.states[i].state.pose.position.y;//y_r
//		acadoVariables.y[ i*n + 2 ] = drone_state_reference.states[i].state.pose.position.z;//z_r
//		acadoVariables.y[ i*n + 3 ] = 0;//yaw_r
//		acadoVariables.y[ i*n + 4 ] = drone_state_reference.states[i].state.twist.linear.x;//vf_r
//		acadoVariables.y[ i*n + 5 ] = drone_state_reference.states[i].state.twist.linear.y;//vs_r
//		acadoVariables.y[ i*n + 6 ] = drone_state_reference.states[i].state.twist.linear.z;//vf_r
//		acadoVariables.y[ i*n + 7 ] = drone_state_reference.states[i].state.twist.angular.z;//vs_r
//		acadoVariables.y[ i*n + 8 ] = 0;//uf_r
//		acadoVariables.y[ i*n + 9 ] = 0;//us_r
//		acadoVariables.y[ i*n + 10 ] = 0;//uz_r
//		acadoVariables.y[ i*n + 11 ] = 0;//uyaw_r
//	}
//
//  acadoVariables.yN[ 0 ] = drone_state_reference.states[n].state.pose.position.x;//x_r
//  acadoVariables.yN[ 1 ] = drone_state_reference.states[n].state.pose.position.y;//y_r
//  acadoVariables.yN[ 2 ] = drone_state_reference.states[n].state.pose.position.z;//z_r
//  acadoVariables.yN[ 3 ] = 0;//yaw_r
//  acadoVariables.yN[ 4 ] = drone_state_reference.states[n].state.twist.linear.x;//vf_r
//  acadoVariables.yN[ 5 ] = drone_state_reference.states[n].state.twist.linear.y;//vs_r
//  acadoVariables.yN[ 6 ] = drone_state_reference.states[n].state.twist.linear.z;//vf_r
//  acadoVariables.yN[ 7 ] = drone_state_reference.states[n].state.twist.angular.z;//vs_r
//};//}}}
//
//void AcadoSolver::setObstacle(Obstacle obstacle_){};
//void AcadoSolver::setObstacle(drone_mpc_msgs::Obstacle::ConstPtr& obstacle_){//{{{
//  //Obstacles in world frame
//  //obstacle->id starts in 0
//  switch( obstacle_->type ){
//    case 1: //Plane
//	    for (unsigned i = 0 ; i < n ; i++){
//          //Position along the horizon
//		      acadoVariables.od[pobs_offset + 0 + pobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->trajectory.states[i].state.pose.position.x;
//		      acadoVariables.od[pobs_offset + 1 + pobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->trajectory.states[i].state.pose.position.y;
//		      acadoVariables.od[pobs_offset + 2 + pobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->trajectory.states[i].state.pose.position.z;
//          //Normal vector (A B C)
//          acadoVariables.od[pobs_offset + 3 + pobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->scale.x;
//          acadoVariables.od[pobs_offset + 4 + pobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->scale.y;
//         	acadoVariables.od[pobs_offset + 5 + pobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->scale.z;
//      }
//
//    case 2: //Sphere
//	    for (unsigned i = 0 ; i < n ; i++){
//          //Position along the horizon
//		      acadoVariables.od[eobs_offset + 0 + eobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->trajectory.states[i].state.pose.position.x;
//		      acadoVariables.od[eobs_offset + 1 + eobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->trajectory.states[i].state.pose.position.y;
//		      acadoVariables.od[eobs_offset + 2 + eobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->trajectory.states[i].state.pose.position.z;
//          //Size (rx ry rz)
//          acadoVariables.od[eobs_offset + 3 + eobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->scale.x;
//          acadoVariables.od[eobs_offset + 4 + eobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->scale.y;
//         	acadoVariables.od[eobs_offset + 5 + eobs_p*obstacle_->id + i*nod ] = 
//            obstacle_->scale.z;
//          //Rotation matrix
//          tf2::Quaternion rot(
//            obstacle_->trajectory.states[i].state.pose.orientation.x,
//            obstacle_->trajectory.states[i].state.pose.orientation.y,
//            obstacle_->trajectory.states[i].state.pose.orientation.z,
//            obstacle_->trajectory.states[i].state.pose.orientation.w
//          );
//          tf2::Matrix3x3 mrot(rot);
//          acadoVariables.od[eobs_offset + 6 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(0)[0];//kxx
//          acadoVariables.od[eobs_offset + 7 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(0)[1];//kxy
//          acadoVariables.od[eobs_offset + 8 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(0)[2];//kxz
//          acadoVariables.od[eobs_offset + 9 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(1)[0];//kyx
//          acadoVariables.od[eobs_offset + 10 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(1)[1];//kyy
//          acadoVariables.od[eobs_offset + 11 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(1)[2];//kyz
//          acadoVariables.od[eobs_offset + 12 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(2)[0];//kzx
//          acadoVariables.od[eobs_offset + 13 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(2)[1];//kzy
//          acadoVariables.od[eobs_offset + 14 + eobs_p*obstacle_->id + i*nod ] = 
//            mrot.getRow(2)[2];//kzz
//	    }
//    break;
//  }
//};//}}}
//
//void AcadoSolver::setModel(DroneFodModel model_){};
//void AcadoSolver::setModel(drone_mpc_msgs::DroneFodModel::ConstPtr& model_){//{{{
//  model = *model_;
//	for (unsigned i = 0 ; i < n ; i++)
//	{
//		acadoVariables.od[model_offset + 2 + i*nod ] = model_->x.k;
//		acadoVariables.od[model_offset + 3 + i*nod ] = model_->x.tau;
//
//		acadoVariables.od[model_offset + 4 + i*nod ] = model_->y.k;
//		acadoVariables.od[model_offset + 5 + i*nod ] = model_->y.tau;
//
//		acadoVariables.od[model_offset + 6 + i*nod ] = model_->z.k;
//		acadoVariables.od[model_offset + 7 + i*nod ] = model_->z.tau;
//
//		acadoVariables.od[model_offset + 8 + i*nod ] = model_->yaw.k;
//		acadoVariables.od[model_offset + 9 + i*nod ] = model_->yaw.tau;
//	}
//};//}}}
//
//void AcadoSolver::setBounds(Bounds bounds_){};
//void AcadoSolver::setBounds(drone_mpc_msgs::Bounds::ConstPtr& bounds_){};
//
//void AcadoSolver::setWeights(Weights weights_){};
//void AcadoSolver::setWeights(drone_mpc_msgs::Weights::ConstPtr& weights_){};
//
//void AcadoSolver::solve(){};
//
//void AcadoSolver::getControls(ControlsArray &controls_) {};
//void AcadoSolver::getControls(drone_mpc_msgs::ControlsArray &controls_){};
//
//void AcadoSolver::getTrajectory(Trajectory &trajectory_){};
//void AcadoSolver::getTrajectory(drone_mpc_msgs::Trajectory &trajectory_){};
//}}}

/*
template<class SolverClass>
class MPC{//{{{
    //std::unique_ptr<Solver> solver;
    //std::shared_ptr<Solver> solver;
  private:
    SolverClass solver;
    std_msgs::Float64 sample_time;
    std_msgs::Float32 reference_limit;
    std::queue<std::vector<double> > delay_u_forward, delay_u_sideward, delay_u_upward, delay_u_yaw;
    std_msgs::Float64 control_latency;
    std_msgs::Float64 total_latency;
  private:
    ros::NodeHandle n;
    ros::Subscriber state_sub;
    std::string     state_topic_name;
    ros::Subscriber reference_sub;
    ros::Subscriber weights_sub;
    ros::Subscriber obstacles_sub;
    ros::Subscriber bounds_sub;
    ros::Subscriber model_sub;
    ros::Subscriber sample_time_sub;
    ros::Subscriber reference_limit_sub;
    ros::Publisher  controls_pub;
    ros::Publisher  planned_trajectory_pub;
    ros::Publisher  planned_controls_pub;
    ros::Publisher  control_latency_pub;
    ros::Publisher  total_latency_pub;
  public:
    MPC();
};/*}}}*/
/*
template<class SolverClass> MPC<SolverClass>::MPC(){
  state_sub =  n.subscribe<drone_mpc_msgs::State>(state_topic_name, 1, &Solver::setState, &solver);
}
*/
int main(){
  AcadoSolver::Parameters parameters = {0};
  parameters.sminmax_p =  2;//Soft minmax constraint parameters
  parameters.hminmax_p =  1;//Hard minmax constraint parameters
  parameters.model_p   = 10;//Model parameters
  parameters.eobs_p    = 16;//Ellipsoidal obstacle parameters
  parameters.pobs_p    =  7;//Planar obstacle parameters
  
  parameters.num_controls =  4;//Number of controls
  parameters.num_extra_s  =  4;//Number of extra states
  parameters.num_sminmax  =  4;//Number of soft minmax constraints
  parameters.num_hminmax  =  4;//Number of hard minmax constraints
  parameters.num_uminmax  =  4;//Number of control minmax constraints
  parameters.num_eobs     = 10;//Number of ellipsoidal obstacles
  parameters.num_pobs     = 10;//Number of planar obstacles
  
  parameters.model.x.k = 1;
  parameters.model.x.tau = 1;
  parameters.model.y.k = 1;
  parameters.model.y.tau = 1;
  parameters.model.z.k = 1;
  parameters.model.z.tau = 0.5;
  parameters.model.yaw.k = 0.0175;
  parameters.model.yaw.tau = 0.2;

  unsigned ny = ACADO_NY;

 // for( unsigned i=0; i< ny*ny; i++) acadoVariables.W[i] = 0;
    
  //for( unsigned i=0; i< ny; i++)
  //{
  //  acadoVariables.W[i*(ny +1)] = 1;
  //}
  AcadoSolver current_solver(parameters);
  Solver* solver(&current_solver);

  //AcadoSolver* solver = new AcadoSolver(parameters);
	
 // acado_printDifferentialVariables();
 //	acado_printControlVariables();
  //Initial conditions
  unsigned num_iters = 4;
//  Solver::State initial_reference = {};
//  initial_reference.x = 100;
//  solver->setReference(initial_reference);
    Solver::StateArray states(ACADO_N + 1);
    Solver::ControlsArray controls(ACADO_N);
//  states[0].x = -10;
//  states[0].y = -10;
//  states[0].z = -10;
//  solver->setState(states[0]);
  solver->simulate();
  printf("Hello %f \n",acadoVariables.x0[0]);
  printf("Hello %f \n",acadoVariables.y[0]);
  printf("Hello %f \n",acadoVariables.W[0 + ACADO_NY ]);
  acado_preparationStep();
  acado_feedbackStep();
  acado_printDifferentialVariables();
 // acado_printControlVariables();
  for( unsigned i =0; i<num_iters; i++){
    //solver->setState(states[0]);
    solver->solve();
    solver->getControls(controls);//Get and apply controls
    solver->getTrajectory(states);//Get the predicted trajectory
    printf("x:  %f\t y:  %f\t z:  %f\t  yaw:  %f\n",states[0].x, states[0].y, states[0].z, states[0].yaw);
    printf("ux: %f\t uy: %f\t uz: %f\t  uyaw: %f\n",controls[0].u_x, controls[0].u_y, controls[0].u_z, controls[0].u_yaw);
    states[0] = states[1];
    controls[0] = controls[1];
    solver->simulate();
  }
 // acado_printDifferentialVariables();
 // acado_printControlVariables();
 // delete[] solver;
  
//  unsigned n   = ACADO_N;
//  unsigned nx  = ACADO_NX;
//  unsigned nu  = ACADO_NU;
//  unsigned nod = ACADO_NOD;
//  unsigned ny  = ACADO_NY; 
//  unsigned nyn = ACADO_NYN;
//  // Initialize the solver. 
//  acado_initializeSolver();
//  // Initialize Trajectory
//  for (unsigned i = 0; i < nx * (n + 1); ++i)  acadoVariables.x[ i ] = 0.0;
//  // Initialize Controls
//  for (unsigned i = 0; i < nu * n; ++i)  acadoVariables.u[ i ] = 0.0;
//  // Initialize online data
//  for (unsigned i = 0; i < nod * (n + 1); ++i)  acadoVariables.od[ i ] = 1.0;
//  // Initialize weights
//  for (unsigned i = 0; i < ny*ny*n; ++i)  acadoVariables.W[ i ] = 0.0;
//  for (unsigned i = 0; i < ny*ny*n; ++i)  acadoVariables.W[ i ] = 0.0;
//  for (unsigned i = 0; i < nyn*nyn; ++i)  acadoVariables.WN[ i ] = 0.0;
//  // Initialize the reference. 
//  for (unsigned i = 0; i < ny * n; ++i)  acadoVariables.y[ i ] = 100.0;
//  for (unsigned i = 0; i < nyn; ++i)  acadoVariables.yN[ i ] = 0.0;
//  //Initialize the current state feedback. 
//  for (unsigned i = 0; i < nx; ++i) acadoVariables.x0[ i ] = 1.0;
//
//  unsigned y_states_offset = 0;
//  Solver::State state_weights_ = {1};
//  for (unsigned i = 0; i < n; ++i){
//    acadoVariables.od[0 + i*nod] = 0;
//    acadoVariables.od[1 + i*nod] = 1;
//  }
//  
//  for (unsigned i = 0; i < n; ++i)
//	{
//	 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 0 )*(ny+1) ] = 1;//x_r
//	 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 1 )*(ny+1) ] = 1;//y_r
//		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 2 )*(ny+1) ] = 1;//z_r
//		acadoVariables.W[ i*(ny*ny) + (y_states_offset + 3 )*(ny+1) ] = 1;//yaw_r
//	}
//
//	acadoVariables.WN[ (y_states_offset + 0 )*(ny+1) ] = 1;//x_r
//	acadoVariables.WN[ (y_states_offset + 1 )*(ny+1) ] = 1;//y_r
//	acadoVariables.WN[ (y_states_offset + 2 )*(ny+1) ] = 1;//z_r
//	acadoVariables.WN[ (y_states_offset + 3 )*(ny+1) ] = 1;//yaw_r
//  
//  acado_preparationStep();
//  acado_feedbackStep();
//  acado_printDifferentialVariables();
  return 0;
}
