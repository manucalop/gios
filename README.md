# GIOS

  GIOS is a Generalistic Interface for Optimization Solvers. Is written in C++ and allows you to interact with your solver's generated code using your own data structures and a standardized interface. With GIOS, you can select the solver to use and switch between them without changing your code. Is written in C++ and, since it links the data without storing it, is very lightweight and suitable for embedded applications like Model Predictive Control (MPC). 
  
## Usage

### Simple example

```cpp
  gios::AcadoSolver my_solver; //Choose one solver (ACADO in this case)
  gios::VariablePtr my_var; //Create your variable
  unsigned step = 0, pos = 3; //Step and position where you want to link it
  my_solver.linkState(my_var, step, pos); //Now your variable points to the solver state data.
  *my_var = 4; //Change it

  my_solver.solve();
```

