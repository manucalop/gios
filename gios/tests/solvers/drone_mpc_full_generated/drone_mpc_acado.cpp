#include "drone_mpc_acado.h"

//Needs to be global for cross compiling with C
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

AcadoSolver::AcadoSolver(){//{{{ Constructor

  printf("Starting Constructor Method...\n");

  n   = ACADO_N;
  nx  = ACADO_NX;
  nu  = ACADO_NU;
  nod = ACADO_NOD;
  ny  = ACADO_NY; 
  nyn = ACADO_NYN;
  
  Parameters parameters = {};
  parameters.sminmax_p =  1;//Soft minmax constraint parameters
  parameters.hminmax_p =  1;//Hard minmax constraint parameters
  parameters.model_p   = 10;//Model parameters
  parameters.eobs_p    = 15;//Ellipsoidal obstacle parameters
  parameters.pobs_p    =  6;//Planar obstacle parameters
  
  parameters.num_controls =  4;//Number of controls
  parameters.num_extra_s  =  4;//Number of extra states
  parameters.num_sminmax  =  4;//Number of soft minmax constraints
  parameters.num_hminmax  =  4;//Number of hard minmax constraints
  parameters.num_uminmax  =  4;//Number of control minmax constraints
  parameters.num_eobs     =  10;//Number of ellipsoidal obstacles
  parameters.num_pobs     =  6;//Number of planar obstacles
  
  parameters.model.x.k = 1;
  parameters.model.x.tau = 1;
  parameters.model.y.k = 1;
  parameters.model.y.tau = 1;
  parameters.model.z.k = 1;
  parameters.model.z.tau = 0.5;
  parameters.model.yaw.k = 0.0175;
  parameters.model.yaw.tau = 0.2;
  
  u_num_controls  = parameters.num_controls;//4
  u_num_minmax    = 1;
  u_num_eobs      = 1;
  u_num_pobs      = 1;
  
  od_num_model_p  = parameters.model_p;//10
  od_num_minmax   = parameters.sminmax_p*parameters.num_sminmax + 
                    parameters.hminmax_p*parameters.num_hminmax +1;
  od_num_uminmax  = parameters.num_uminmax;
  od_num_eobs     = parameters.num_eobs;
  od_num_pobs     = parameters.num_pobs;
  od_num_obs_slacks = 2;

  y_num_states    = nx + parameters.num_extra_s;
  y_num_controls  = parameters.num_controls;
  y_num_minmax    = 1;
  y_num_eobs      = 1;
  y_num_pobs      = 1;

  //Set the order of the parameters in ACADOvariables.u[] << controls << minmax_slacks << obstacles_slacks <<...
  u_controls_offset = 0;
  u_minmax_offset   = u_controls_offset + u_num_controls;
  u_eobs_offset     = u_minmax_offset   + u_num_minmax;
  u_pobs_offset     = u_eobs_offset     + u_num_eobs;
  
  //Set the order of the parameters in ACADOvariables.od[] << model << minmax << uminmax << eobs << pobs << ...
  od_model_offset   = 0;
  od_minmax_offset  = od_model_offset   + od_num_model_p;
  od_uminmax_offset = od_minmax_offset  + od_num_minmax;
  od_slacks_offset  = od_uminmax_offset + od_num_uminmax;
  od_eobs_p         = parameters.eobs_p;
  od_pobs_p         = parameters.pobs_p;
  od_eobs_offset    = od_slacks_offset  + od_num_obs_slacks;
  od_pobs_offset    = od_eobs_offset    + od_eobs_p*od_num_eobs;
  
  //Position of the variables in ACADOvariables.y[] << states << controls << minmax << eobs << pobs <<...
  y_states_offset   = 0;
  y_controls_offset = y_states_offset   + y_num_states;
  y_minmax_offset   = y_controls_offset + y_num_controls;
  y_eobs_offset     = y_minmax_offset   + y_num_minmax;
  y_pobs_offset     = y_eobs_offset     + y_num_eobs;
  
  printf("Offsets initialized:\n");
  printf("\nu_controls_offset = %d\n",u_controls_offset);
  printf("u_minmax_offset = %d\n",u_minmax_offset);
  printf("u_eobs_offset = %d\n",u_eobs_offset);
  printf("u_pobs_offset = %d\n",u_pobs_offset);
  
  printf("\nod_model_offset = %d\n",od_model_offset);
  printf("od_minmax_offset = %d\n",od_minmax_offset);
  printf("od_uminmax_offset = %d\n",od_uminmax_offset);
  printf("od_slacks_offset = %d\n",od_slacks_offset);
  printf("od_eobs_offset = %d\n",od_eobs_offset);
  printf("od_pobs_offset = %d\n",od_pobs_offset);
  printf("od_eobs_p = %d\n",od_eobs_p);
  printf("od_pobs_p = %d\n",od_pobs_p);

  printf("\ny_states_offset = %d\n",y_states_offset);
  printf("y_controls_offset = %d\n",y_controls_offset);
  printf("y_minmax_offset = %d\n",y_minmax_offset);
  printf("y_eobs_offset = %d\n",y_eobs_offset);
  printf("y_pobs_offset = %d\n",y_pobs_offset);

  printf("Initializing solver...\n");
  // Initialize the solver. 
  acado_initializeSolver();
  // Initialize Trajectory
  for (unsigned i = 0; i < nx * (n + 1); ++i)  acadoVariables.x[ i ] = 0.0;
  // Initialize Controls
  for (unsigned i = 0; i < nu * n; ++i)  acadoVariables.u[ i ] = 0.0;
  // Initialize online data
  for (unsigned i = 0; i < nod * (n + 1); ++i)  acadoVariables.od[ i ] = 0.0;
  // Initialize weights (all weights have to be at least 1!!!
  for (unsigned i = 0; i < ny*ny; ++i)  acadoVariables.W[ i ] = 0.0;
  //for( unsigned i=0; i<ACADO_NY; i++){
	// 	acadoVariables.W[ ( i )*(ACADO_NY+1) ] = 0.0;
  //}
  //for (unsigned i = 0; i < ny*ny*n; ++i)  acadoVariables.W[ i ] = 0.0;
  for (unsigned i = 0; i < nyn*nyn; ++i)  acadoVariables.WN[ i ] = 0.0;
  // Initialize the reference. 
  for (unsigned i = 0; i < ny * n; ++i)  acadoVariables.y[ i ] = 0.0;
  for (unsigned i = 0; i < nyn; ++i)  acadoVariables.yN[ i ] = 0.0;
  //Initialize the current state feedback. 
  for (unsigned i = 0; i < nx; ++i) acadoVariables.x0[ i ] = 0.0;

  printf("Setting initial state...\n");
  State initial_state = {};
  initial_state.x   = 0;
  initial_state.y   = 0;
  initial_state.z   = 0;
  initial_state.yaw = 0;
  initial_state.v_x = 0;
  initial_state.v_y = 0;
  initial_state.v_z = 0;
  initial_state.v_yaw = 0;

  setState(initial_state);

  printf("Setting initial reference...\n");
  State initial_reference = {};
  initial_reference.x     = 0;
  initial_reference.y     = 0;
  initial_reference.z     = 0;
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
  initial_eobstacle.ellipsoid.rotation_matrix[0][1] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[0][2] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[1][0] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[1][1] = 1;
  initial_eobstacle.ellipsoid.rotation_matrix[1][2] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[2][0] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[2][1] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[2][2] = 1;
  initial_eobstacle.s_k = 1;
  initial_eobstacle.s_w = 1;

  for (unsigned i= 0; i< parameters.num_eobs; i++){
    initial_eobstacle.id = i;
    printf("Setting Ellipsoidal obstacle %i\n",initial_eobstacle.id);
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

  for (unsigned i= 0; i< parameters.num_pobs; i++){
    initial_pobstacle.id = i;
    printf("Setting Planar obstacle %i\n",initial_pobstacle.id);
    setObstacle(initial_pobstacle);
  }
  
  printf("Setting model parameters...\n");
  setModel(parameters.model);

  
  printf("Setting velocity and acceleration limits...\n");
  State max_state = {1000};
  max_state.v_x = 1000;
  max_state.v_y = 1000;
  max_state.v_z = 1000;
  max_state.v_yaw = 1000;
  max_state.a_x = 0.1;
  max_state.a_y = 1000;
  max_state.a_z = 1000;
  max_state.a_yaw = 1000;
  setBoundsMax(max_state);

  printf("Setting controls limits...\n");
  Controls max_controls = {1000};
  max_controls.u_x = 1000;
  max_controls.u_y = 1000;
  max_controls.u_z = 1000;
  max_controls.u_yaw = 1000;
  setBoundsMax(max_controls);

  printf("Setting state weights...\n");
  State state_weights = {};
  state_weights.x     = 1;
  state_weights.y     = 1;
  state_weights.z     = 1;
  state_weights.yaw   = 1;
  state_weights.v_x   = 1;
  state_weights.v_y   = 1;
  state_weights.v_z   = 1;
  state_weights.v_yaw = 1;
  state_weights.a_x   = 1;
  state_weights.a_y   = 1;
  state_weights.a_z   = 1;
  state_weights.a_yaw = 1;
  
  setWeights(state_weights);


  printf("Setting controls weights...\n");
  Controls controls_weights = {};
  controls_weights.u_x   = 1;
  controls_weights.u_y   = 1;
  controls_weights.u_z   = 1;
  controls_weights.u_yaw = 1;

  setWeights(controls_weights);

  std::cout<<"Running some tests...\n";
  
  runTests();

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
  try{
    if( reference_.size() < n + 1 ) throw 1;
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
  }
  catch(int i){
    std::cout<<"ERROR setting reference. Reference size is "<<reference_.size()<<" and should be "<<n+1<<std::endl;
  };
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
  try{
    if( reference_.size() < n ) throw 1;
    for (unsigned i = 0; i < n; ++i)
    {
      acadoVariables.y[ y_controls_offset + 0 + i*ny ] = reference_[i].u_x;//ux_r
      acadoVariables.y[ y_controls_offset + 1 + i*ny ] = reference_[i].u_y;//uy_r
      acadoVariables.y[ y_controls_offset + 2 + i*ny ] = reference_[i].u_z;//uz_r
      acadoVariables.y[ y_controls_offset + 3 + i*ny ] = reference_[i].u_yaw;//uyaw_r
    }
  }
  catch(int i){
    std::cout<<"ERROR setting reference. Reference size is "<<reference_.size()<<" and should be "<<n<<std::endl;
  };
};///}}}

void AcadoSolver::setObstacle(const EllipsoidalObstacle &obstacle_){//{{{ Static obstacle
  try{
    if( obstacle_.id < 0 || obstacle_.id >= od_num_eobs ) throw 2;
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
      acadoVariables.od[ od_slacks_offset + 0 + i*nod ] = obstacle_.s_k;//Sensitivity
    }
      acadoVariables.W[ (y_eobs_offset)*(ny+1) ] = obstacle_.s_w;//Weight
  }
  catch(int i){
    std::cout<<"Not valid Obstacle id ( "<<obstacle_.id<<" )"<<std::endl;
  }
 // for (unsigned i = 0 ; i < n ; i++){
 // 	acadoVariables.W[ i*(ny*ny) + (y_eobs_offset + obstacle_.id)*(ny+1) ] = obstacle_.s_w;//Weight
 // }
};//}}}

void AcadoSolver::setObstacle(const PlanarObstacle &obstacle_){//{{{
  try{
    if( obstacle_.id < 0 || obstacle_.id >= od_num_pobs ) throw 2;
    for (unsigned i = 0 ; i < n + 1 ; i++){
      acadoVariables.od[ od_pobs_offset + 0 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.x;
      acadoVariables.od[ od_pobs_offset + 1 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.y;
      acadoVariables.od[ od_pobs_offset + 2 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.z;
      acadoVariables.od[ od_pobs_offset + 3 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.a;
      acadoVariables.od[ od_pobs_offset + 4 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.b;
      acadoVariables.od[ od_pobs_offset + 5 + od_pobs_p*obstacle_.id + i*nod ] = obstacle_.plane.c;
      acadoVariables.od[ od_slacks_offset + 1 + i*nod ] = obstacle_.s_k;//Sensitivity
    }	
    acadoVariables.W[ (y_pobs_offset)*(ny+1) ] = obstacle_.s_w;//Weight
  //  for (unsigned i = 0 ; i < n ; i++){
  //    acadoVariables.W[ i*(ny*ny) + (y_pobs_offset + obstacle_.id)*(ny+1) ] = obstacle_.s_w;//Weight
  //  }
  }
  catch(int i){
    std::cout<<"Not valid Obstacle id ( "<<obstacle_.id<<" )"<<std::endl;
  }
};//}}}

void AcadoSolver::setObstacle(const EllipsoidalObstacleArray &obstacle_){//{{{ Dynamic obstacle
  try{
    if( obstacle_.size() < n + 1 ) throw 1;
    if( obstacle_[0].id < 0 || obstacle_[0].id >= od_num_eobs ) throw 2;
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
        acadoVariables.W[ (y_eobs_offset)*(ny+1) ] = obstacle_[0].s_w;//Weight
    //    acadoVariables.W[ (y_eobs_offset + obstacle_[0].id)*(ny+1) ]=obstacle_[0].s_w;//Weight
    //  for (unsigned i = 0 ; i < n ; i++){
    //    acadoVariables.W[ i*(ny*ny) + (y_eobs_offset + obstacle_[0].id)*(ny+1) ]=obstacle_[i].s_w;//Weight
    //  }
  }
  catch(int i){
    if(i == 1) std::cout<<"Obstacle Array size not valid ( "<<obstacle_.size()<<" )"<<std::endl;
    if(i == 2) std::cout<<"Obstacle id not valid ( "<<obstacle_[0].id<<" )"<<std::endl;
  }
};//}}}

void AcadoSolver::setObstacle(const PlanarObstacleArray &obstacle_){//{{{
  try{
    if( obstacle_.size() < n + 1 ) throw 1;
    if( obstacle_[0].id < 0 || obstacle_[0].id >= od_num_pobs ) throw 2;
    for (unsigned i = 0 ; i < n + 1 ; i++){
      acadoVariables.od[ od_pobs_offset + 0 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.x;
      acadoVariables.od[ od_pobs_offset + 1 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.y;
      acadoVariables.od[ od_pobs_offset + 2 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.z;
      acadoVariables.od[ od_pobs_offset + 3 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.a;
      acadoVariables.od[ od_pobs_offset + 4 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.b;
      acadoVariables.od[ od_pobs_offset + 5 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].plane.c;
      acadoVariables.od[ od_pobs_offset + 6 + od_pobs_p*obstacle_[0].id + i*nod ] = obstacle_[i].s_k;//Sensitivity
    }  
      acadoVariables.W[ (y_pobs_offset + obstacle_[0].id)*(ny+1) ]=obstacle_[0].s_w;//Weight
  //  for (unsigned i = 0 ; i < n ; i++){
  //    acadoVariables.W[ i*(ny*ny) + (y_pobs_offset + obstacle_[0].id)*(ny+1) ]=obstacle_[i].s_w;//Weight
  //  }
  }
  catch(int i){
    if(i == 1) std::cout<<"Obstacle Array size not valid ( "<<obstacle_.size()<<" )"<<std::endl;
    if(i == 2) std::cout<<"Obstacle id not valid ( "<<obstacle_[0].id<<" )"<<std::endl;
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
  try{
    if( model_.size() < n + 1 ) throw 1;
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
  }
  catch(int i){
    std::cout<<"Model Array size not valid ( "<<model_.size()<<" )"<<std::endl;
  }
};//}}}

void AcadoSolver::setBoundsMax(const State &max_state_){//{{{Static bounds
  for (unsigned i = 0 ; i < n + 1 ; i++){
		acadoVariables.od[ od_minmax_offset + 0  + i*nod ] = max_state_.v_x;
		acadoVariables.od[ od_minmax_offset + 1  + i*nod ] = max_state_.v_y;
		acadoVariables.od[ od_minmax_offset + 2  + i*nod ] = max_state_.v_z;
		acadoVariables.od[ od_minmax_offset + 3  + i*nod ] = max_state_.v_yaw;
		acadoVariables.od[ od_minmax_offset + 4  + i*nod ] = max_state_.a_x;
		acadoVariables.od[ od_minmax_offset + 5  + i*nod ] = max_state_.a_y;
		acadoVariables.od[ od_minmax_offset + 6  + i*nod ] = max_state_.a_z;
		acadoVariables.od[ od_minmax_offset + 7  + i*nod ] = max_state_.a_yaw;
		acadoVariables.od[ od_minmax_offset + 8  + i*nod ] = 0.01;//Sensitivity
  }
  acadoVariables.W[ (y_minmax_offset)*(ny+1) ] = 1;//Weight
};//}}}

void AcadoSolver::setBoundsMax(const StateArray &max_state_){//{{{Dynamic State bounds
  try{
    if( max_state_.size() < n + 1 ) throw 1;
    for (unsigned i = 0 ; i < n + 1 ; i++){
      acadoVariables.od[ od_minmax_offset + 0  + i*nod ] = max_state_[i].v_x;
      acadoVariables.od[ od_minmax_offset + 1  + i*nod ] = max_state_[i].v_y;
      acadoVariables.od[ od_minmax_offset + 2  + i*nod ] = max_state_[i].v_z;
      acadoVariables.od[ od_minmax_offset + 3  + i*nod ] = max_state_[i].v_yaw;
      acadoVariables.od[ od_minmax_offset + 4  + i*nod ] = max_state_[i].a_x;
      acadoVariables.od[ od_minmax_offset + 5  + i*nod ] = max_state_[i].a_y;
      acadoVariables.od[ od_minmax_offset + 6  + i*nod ] = max_state_[i].a_z;
      acadoVariables.od[ od_minmax_offset + 7  + i*nod ] = max_state_[i].a_yaw;
      acadoVariables.od[ od_minmax_offset + 8  + i*nod ] = 0.01;//Sensitivity
    }
    acadoVariables.W[ (y_minmax_offset)*(ny+1) ] = 1;//Weight
  }
  catch(int i){
   std::cout<<"Max State Array size not valid ( "<<max_state_.size()<<" )"<<std::endl;
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
  try{
    if( max_controls_.size() < n + 1 ) throw 1;
    for (unsigned i = 0 ; i < n + 1 ; i++){
      acadoVariables.od[ od_uminmax_offset + 0 + i*nod ] = max_controls_[i].u_x;
      acadoVariables.od[ od_uminmax_offset + 1 + i*nod ] = max_controls_[i].u_y;
      acadoVariables.od[ od_uminmax_offset + 2 + i*nod ] = max_controls_[i].u_z;
      acadoVariables.od[ od_uminmax_offset + 3 + i*nod ] = max_controls_[i].u_yaw;
    }
  }
  catch(int i){
   std::cout<<"Max Controls Array size not valid ( "<<max_controls_.size()<<" )"<<std::endl;
  }
};//}}}

void AcadoSolver::setWeights(const State &state_weights_){//{{{Static weights
	 	acadoVariables.W[ (y_states_offset + 0 )*(ny+1) ] = state_weights_.x;//x_r
	 	acadoVariables.W[ (y_states_offset + 1 )*(ny+1) ] = state_weights_.y;//y_r
		acadoVariables.W[ (y_states_offset + 2 )*(ny+1) ] = state_weights_.z;//z_r
		acadoVariables.W[ (y_states_offset + 3 )*(ny+1) ] = state_weights_.yaw;//yaw_r
		acadoVariables.W[ (y_states_offset + 4 )*(ny+1) ] = state_weights_.v_x;//vf_r
		acadoVariables.W[ (y_states_offset + 5 )*(ny+1) ] = state_weights_.v_y;//vs_r
		acadoVariables.W[ (y_states_offset + 6 )*(ny+1) ] = state_weights_.v_z;//vu_r
		acadoVariables.W[ (y_states_offset + 7 )*(ny+1) ] = state_weights_.v_yaw;//vh_r
		acadoVariables.W[ (y_states_offset + 8 )*(ny+1) ] = state_weights_.a_x;//vf_r
		acadoVariables.W[ (y_states_offset + 9 )*(ny+1) ] = state_weights_.a_y;//vs_r
		acadoVariables.W[ (y_states_offset + 10)*(ny+1) ] = state_weights_.a_z;//vu_r
		acadoVariables.W[ (y_states_offset + 11)*(ny+1) ] = state_weights_.a_yaw;//vh_r
  //for (unsigned i = 0; i < n; ++i)
	//{
	// 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 0 )*(ny+1) ] = state_weights_.x;//x_r
	// 	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 1 )*(ny+1) ] = state_weights_.y;//y_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 2 )*(ny+1) ] = state_weights_.z;//z_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 3 )*(ny+1) ] = state_weights_.yaw;//yaw_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 4 )*(ny+1) ] = state_weights_.v_x;//vf_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 5 )*(ny+1) ] = state_weights_.v_y;//vs_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 6 )*(ny+1) ] = state_weights_.v_z;//vu_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 7 )*(ny+1) ] = state_weights_.v_yaw;//vh_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 8 )*(ny+1) ] = state_weights_.a_x;//vf_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 9 )*(ny+1) ] = state_weights_.a_y;//vs_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 10)*(ny+1) ] = state_weights_.a_z;//vu_r
	//	acadoVariables.W[ i*(ny*ny) + (y_states_offset + 11)*(ny+1) ] = state_weights_.a_yaw;//vh_r
	//}

	acadoVariables.WN[ (y_states_offset + 0 )*(nyn+1) ] = state_weights_.x;//x_r
	acadoVariables.WN[ (y_states_offset + 1 )*(nyn+1) ] = state_weights_.y;//y_r
	acadoVariables.WN[ (y_states_offset + 2 )*(nyn+1) ] = state_weights_.z;//z_r
	acadoVariables.WN[ (y_states_offset + 3 )*(nyn+1) ] = state_weights_.yaw;//yaw_r
	acadoVariables.WN[ (y_states_offset + 4 )*(nyn+1) ] = state_weights_.v_x;//vf_r
	acadoVariables.WN[ (y_states_offset + 5 )*(nyn+1) ] = state_weights_.v_y;//vs_r
	acadoVariables.WN[ (y_states_offset + 6 )*(nyn+1) ] = state_weights_.v_z;//vu_r
	acadoVariables.WN[ (y_states_offset + 7 )*(nyn+1) ] = state_weights_.v_yaw;//vh_r

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

	acadoVariables.WN[ (y_states_offset + 0 )*(nyn+1) ] = state_weights_[n].x;//x_r
	acadoVariables.WN[ (y_states_offset + 1 )*(nyn+1) ] = state_weights_[n].y;//y_r
	acadoVariables.WN[ (y_states_offset + 2 )*(nyn+1) ] = state_weights_[n].z;//z_r
	acadoVariables.WN[ (y_states_offset + 3 )*(nyn+1) ] = state_weights_[n].yaw;//yaw_r
	acadoVariables.WN[ (y_states_offset + 4 )*(nyn+1) ] = state_weights_[n].v_x;//vf_r
	acadoVariables.WN[ (y_states_offset + 5 )*(nyn+1) ] = state_weights_[n].v_y;//vs_r
	acadoVariables.WN[ (y_states_offset + 6 )*(nyn+1) ] = state_weights_[n].v_z;//vu_r
	acadoVariables.WN[ (y_states_offset + 7 )*(nyn+1) ] = state_weights_[n].v_yaw;//vh_r
	acadoVariables.WN[ (y_states_offset + 8 )*(nyn+1) ] = state_weights_[n].a_x;//vf_r
	acadoVariables.WN[ (y_states_offset + 9 )*(nyn+1) ] = state_weights_[n].a_y;//vs_r
	acadoVariables.WN[ (y_states_offset + 10)*(nyn+1) ] = state_weights_[n].a_z;//vu_r
	acadoVariables.WN[ (y_states_offset + 11)*(nyn+1) ] = state_weights_[n].a_yaw;//vh_r

};//}}}

void AcadoSolver::setWeights(const Controls &controls_weights_){//{{{ 
	 	acadoVariables.W[ (y_controls_offset + 0)*(ny+1) ] = controls_weights_.u_x;//x_r
	 	acadoVariables.W[ (y_controls_offset + 1)*(ny+1) ] = controls_weights_.u_y;//y_r
		acadoVariables.W[ (y_controls_offset + 2)*(ny+1) ] = controls_weights_.u_z;//z_r
		acadoVariables.W[ (y_controls_offset + 3)*(ny+1) ] = controls_weights_.u_yaw;//yaw_r
//  for (unsigned i = 0; i < n; ++i)
//	{
//	 	acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 0)*(ny+1) ] = controls_weights_.u_x;//x_r
//	 	acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 1)*(ny+1) ] = controls_weights_.u_y;//y_r
//		acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 2)*(ny+1) ] = controls_weights_.u_z;//z_r
//		acadoVariables.W[ i*(ny*ny) + (y_controls_offset + 3)*(ny+1) ] = controls_weights_.u_yaw;//yaw_r
//	}
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
  try{
    if( controls_.size() < n ) throw 1;
    for( unsigned i = 0; i < n; i++ ){
      controls_[i].u_x   = acadoVariables.u[ 0 + i*nu ];
      controls_[i].u_y   = acadoVariables.u[ 1 + i*nu ];
      controls_[i].u_z   = acadoVariables.u[ 2 + i*nu ];
      controls_[i].u_yaw = acadoVariables.u[ 3 + i*nu ];
    }
  }
  catch(int i){
   std::cout<<"Controls Array size not valid ( "<<controls_.size()<<" )"<<std::endl;
  }
};//}}}

void AcadoSolver::getTrajectory(StateArray  &states_)const{//{{{
  try{
    if( states_.size() < n + 1 ) throw 1;
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
  }
  catch(int i){
   std::cout<<"States Array size not valid ( "<<states_.size()<<" )"<<std::endl;
  }
};//}}}

void AcadoSolver::runTests(){//{{{ Tests
  std::cout<<"Checking the weighting matrices...\n";
  for(unsigned i =0; i<ny;i++){
    if( acadoVariables.W[i*(ny +1)] <= 0 ){
        std::cout<<"Warning! In W, non-positive value detected in the diagonal! (i,j) = ("<<i<<","<<i<<")"<<std::endl;
    }
    for(unsigned j = 0; j<ny;j++){
       if(acadoVariables.W[i*ny +j] > 0 && i!=j)
        std::cout<<"Warning! In W, positive value detected outside the diagonal! (i,j) = ("<<i<<","<<j<<")"<<std::endl;
    }
  }
  for(unsigned i =0; i<nyn;i++){
    if( acadoVariables.WN[i*(nyn +1)] <= 0 ){
        std::cout<<"Warning! In WN, non-positive value detected in the diagonal! (i,j) = ("<<i<<","<<i<<")"<<std::endl;
    }
    for(unsigned j = 0; j<nyn;j++){
       if(acadoVariables.WN[i*nyn +j] > 0 && i!=j)
        std::cout<<"Warning! In WN, positive value detected outside the diagonal! (i,j) = ("<<i<<","<<j<<")"<<std::endl;
    }
  }
}//}}}
