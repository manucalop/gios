#ifndef DRONE_MPC_ACADO_H_
#define DRONE_MPC_ACADO_H_
#include <iostream>
#include "drone_mpc_solver.h"
//#include "acado_common.h"
#include "acado_auxiliary_functions.h"


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
    
    unsigned u_num_controls;
    unsigned u_num_minmax;
    unsigned u_num_eobs;
    unsigned u_num_pobs;
    
    unsigned od_num_model_p;
    unsigned od_num_minmax;
    unsigned od_num_uminmax;
    unsigned od_num_eobs;
    unsigned od_num_pobs;
    unsigned od_num_obs_slacks;

    unsigned y_num_states;
    unsigned y_num_controls;
    unsigned y_num_minmax;
    unsigned y_num_eobs;
    unsigned y_num_pobs;


    //Set the order of the parameters in ACADOvariables.u[] << controls << minmax_slacks << obstacles_slacks <<...
    unsigned u_controls_offset;
    unsigned u_minmax_offset;
    unsigned u_eobs_offset;
    unsigned u_pobs_offset;
    
    //Set the order of the parameters in ACADOvariables.od[] << model << minmax << uminmax << eobs << pobs << ...
    unsigned od_model_offset;
    unsigned od_minmax_offset;
    unsigned od_uminmax_offset;
    unsigned od_slacks_offset;
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
    explicit AcadoSolver();
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
    void solve() override;
    void simulate() override;
  public:
    void getControls(Controls      &controls_)const override;
    void getControls(ControlsArray &controls_)const override;
    void getTrajectory(StateArray  &states_)const override;
  public:
    void runTests();
  
};/*}}}*/

#endif
