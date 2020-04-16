#include "drone_mpc_solver.h"
#include "drone_mpc_acado.h"

int main(){

  Solver* solver = new AcadoSolver;

  unsigned num_iters = 10;
  
  Solver::State initial_reference = {};
  initial_reference.x = 10;
  initial_reference.y = 0;
  initial_reference.z = 0;
  initial_reference.yaw = 0;
  solver->setReference(initial_reference);
  Solver::StateArray states(ACADO_N + 1);
  Solver::ControlsArray controls(ACADO_N);
  states[0].x = 0;
  states[0].y = 0;
  states[0].z = 0;
  states[0].yaw = 0;
  Solver::Controls controls_limits;
  controls_limits.u_x = 100;
  controls_limits.u_y = 100;
  controls_limits.u_z = 100;
  controls_limits.u_yaw = 100;
  solver->setBoundsMax(controls_limits);
  Solver::EllipsoidalObstacle initial_eobstacle;
  initial_eobstacle.id = 0;
  initial_eobstacle.ellipsoid.x = 2.0;
  initial_eobstacle.ellipsoid.y = 0.0;
  initial_eobstacle.ellipsoid.z = -100.0;
  initial_eobstacle.ellipsoid.r_x = 1/1;
  initial_eobstacle.ellipsoid.r_y = 1/1;
  initial_eobstacle.ellipsoid.r_z = 1/1;
  initial_eobstacle.ellipsoid.rotation_matrix[0][0] = 1;
  initial_eobstacle.ellipsoid.rotation_matrix[0][1] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[0][2] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[1][0] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[1][1] = 1;
  initial_eobstacle.ellipsoid.rotation_matrix[1][2] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[2][0] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[2][1] = 0;
  initial_eobstacle.ellipsoid.rotation_matrix[2][2] = 1;
  initial_eobstacle.s_k = 0.2;
  initial_eobstacle.s_w = 1;
  solver->setObstacle(initial_eobstacle);

  Solver::PlanarObstacle initial_pobstacle = {};
  initial_pobstacle.id = 0;
  initial_pobstacle.plane.x = 0;
  initial_pobstacle.plane.y = 0;
  initial_pobstacle.plane.z = -100;
  initial_pobstacle.plane.a = 0;
  initial_pobstacle.plane.b = 0;
  initial_pobstacle.plane.c = 1;
  initial_pobstacle.s_k = 1;
  initial_pobstacle.s_w = 1;
  solver->setObstacle(initial_pobstacle);
  
  solver->setState(states[0]);
  solver->simulate();
  
//  solver->solve();
//  acado_printDifferentialVariables();
//  acado_printControlVariables();
//  return 0;
	acado_timer t;
  for( unsigned i =0; i<num_iters; i++){
    solver->setState(states[0]);
	  acado_tic( &t );
    solver->solve();
	  real_t te = acado_toc( &t );
    solver->getControls(controls);//Get and apply controls
    solver->getTrajectory(states);//Get the predicted trajectory
    printf("x:  %f\t y:  %f\t z:  %f\t  yaw:  %f\n",states[0].x, states[0].y, states[0].z, states[0].yaw);
    printf("vx:  %f\t vy:  %f\t vz:  %f\t  vyaw:  %f\n",states[0].v_x, states[0].v_y, states[0].v_z, states[0].v_yaw);
    printf("ux: %f\t uy: %f\t uz: %f\t  uyaw: %f\t t: %f ms\n",controls[0].u_x, controls[0].u_y, controls[0].u_z, controls[0].u_yaw, te*1e3);
    states[0] = states[1];
    controls[0] = controls[1];
    solver->simulate();
  }
 // acado_printDifferentialVariables();
 // acado_printControlVariables();
  delete[] solver;
  return 0;
}
