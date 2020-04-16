# GIOS

  GIOS is a Generalistic Interface for Optimization Solvers. Is written in C++ and allows you to interact with your solver's generated code using your own data structures and a standardized interface. With GIOS, you can select the solver to use and switch between them without changing your code. Is written in C++ and, since it links the data without storing it, is very lightweight and suitable for embedded applications like Model Predictive Control (MPC). 
  
## Basic Usage

### gios::Variable

One to one mapping to solver data.

```cpp
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<double> x(solver.get())
  step = 0; pos = 0;
  x.linkState(step, pos);
  x.set(4.0);
```

### gios::VariableArray

Array for the same variable along the horizon.

```cpp
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<double> x(solver.get(), solver.getN()+1)
  pos = 0;
  x.linkState(pos);
  x.set(4.0);
```

### gios::Struct

Struct mapping to solver data.

```cpp
  struct MyState{
    double x, y, z, yaw;
  };
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<MyState, double, &TestStruct::x,
                                &TestStruct::y,
                                &TestStruct::z,
                                &TestStruct::yaw > my_state (solver.get());
  step = 0; pos = 0;
  my_state.linkState(step,pos);
  MyState another_state={0.0, 0.0, 1.5, 1.57};
  my_state.set(another_state);
```

### gios::StructArray

Linking Struct array to solver data

```cpp
  struct MyState{
    double x, y, z, yaw;
  };
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<MyState, double, &TestStruct::x,
                                &TestStruct::y,
                                &TestStruct::z,
                                &TestStruct::yaw > my_state (solver.get(), solver.getN());
  pos = 0;
  my_state.linkState(pos);
  MyState another_state={0.0, 0.0, 1.5, 1.57};
  my_state.set(another_state);
```

### gios::NestedStruct
Linking nested struct (compatible with ROS msgs)

Globally, you can declare your state
```cpp
  using MyState = gios::NestedStruct<geometry_msgs::Pose, double,
                  gios::Member<&geometry_msgs::Pose::position,    &geometry_msgs::Point::x>,
                  gios::Member<&geometry_msgs::Pose::position,    &geometry_msgs::Point::y>,
                  gios::Member<&geometry_msgs::Pose::position,    &geometry_msgs::Point::z>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::w>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::x>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::y>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::z>>;
```

```cpp
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  Mystate my_state(solver.get());
  step = 0; pos = 0;
  my_state.linkState(step,pos);
  geometry_msgs:: Pose my_pose;
  my_state.set(my_pose);
```

### gios::NestedStructArray
Linking nested struct array (compatible with ROS msgs)

Globally, you can declare your state
```cpp
  using MyState = gios::NestedStruct<geometry_msgs::Pose, double,
                  gios::Member<&geometry_msgs::Pose::position,    &geometry_msgs::Point::x>,
                  gios::Member<&geometry_msgs::Pose::position,    &geometry_msgs::Point::y>,
                  gios::Member<&geometry_msgs::Pose::position,    &geometry_msgs::Point::z>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::w>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::x>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::y>,
                  gios::Member<&geometry_msgs::Pose::orientation, &geometry_msgs::Quaternion::z>>;
```

```cpp
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  Mystate my_state(solver.get(), solver.getN());
  step = 0; pos = 0;
  my_state.linkState(pos);
  geometry_msgs:: Pose my_pose;
  my_state.set(my_pose);
```

