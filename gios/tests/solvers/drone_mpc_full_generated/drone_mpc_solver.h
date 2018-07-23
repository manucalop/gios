#ifndef DRONE_MPC_SOLVER_H_
#define DRONE_MPC_SOLVER_H_
#include <iostream>
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

#endif
