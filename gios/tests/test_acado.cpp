#include "gios/acado.h"
#include "gios/gios.h"
#include <gtest/gtest.h>
#include <vector>

///* Test Problem Definition {{{*/

struct TestStruct{
  double x, y, z, r;
};

struct Child{
  double a;
};
struct Parent{
  double x, y, z, r;
  Child child;
};

namespace gios{

/* TestStruct Definition {{{*/

template<> void allocate<TestStruct>(std::vector<VariablePtr> & vec_){/*{{{*/
  TestStruct var_;
  vec_.push_back(&var_.x);
  vec_.push_back(&var_.y);
  vec_.push_back(&var_.z);
  vec_.push_back(&var_.r);
}/*}}}*/

template<> void do_set<TestStruct>(TestStruct const& var_, std::vector<VariablePtr> const& vec_){/*{{{*/
  unsigned pos = 0;
  *vec_[pos++] = var_.x;
  *vec_[pos++] = var_.y;
  *vec_[pos++] = var_.z;
  *vec_[pos++] = var_.r;
}/*}}}*/

template<> TestStruct do_get<TestStruct>(std::vector<VariablePtr> const& vec_){/*{{{*/
  unsigned pos = 0;
  TestStruct var_;
  var_.x = *vec_[pos++];
  var_.y = *vec_[pos++];
  var_.z = *vec_[pos++];
  var_.r = *vec_[pos++];
  return var_;
}/*}}}*/

template<> Variable<TestStruct>::Variable( Solver * const solver_):/*{{{*/
  solver(solver_)
{
  TestStruct var_;
  var.push_back(&var_.x);
  var.push_back(&var_.y);
  var.push_back(&var_.z);
  var.push_back(&var_.r);
}/*}}}*/
  
template<> TestStruct Variable<TestStruct>::get() const{/*{{{*/
  unsigned pos = 0;
  TestStruct var_;
  var_.x = *var[pos++];
  var_.y = *var[pos++];
  var_.z = *var[pos++];
  var_.r = *var[pos++];
  return var_;
};/*}}}*/

template<> void Variable<TestStruct>::set(TestStruct const& var_){/*{{{*/
  unsigned pos = 0;
  *var[pos++] = var_.x;
  *var[pos++] = var_.y;
  *var[pos++] = var_.z;
  *var[pos++] = var_.r;
}/*}}}*/

/*}}}*/

}

/*}}}*/

/* AcadoSolver Tests{{{*/

TEST(AcadoSolver, linkState){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned i = 0; i< ACADO_N + 1; i++){
    for(unsigned j = 0; j < ACADO_NX; j++){
      solver->linkState(x, i, j);
      EXPECT_EQ(x, &acadoVariables.x[ACADO_NX*i + j]); 
    }
  }
}//}}}

TEST(AcadoSolver, linkFeedbackState){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned j = 0; j < ACADO_NX; j++){
    solver->linkFeedbackState(x, j);
    EXPECT_EQ(x, &acadoVariables.x0[j]); 
  }
}//}}}

TEST(AcadoSolver, linkControl){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned i = 0; i< ACADO_N; i++){
    for(unsigned j = 0; j < ACADO_NU; j++){
      solver->linkControl(x, i, j);
      EXPECT_EQ(x, &acadoVariables.u[ACADO_NU*i + j]); 
    }
  }
}//}}}

TEST(AcadoSolver, linkReference){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned i = 0; i< ACADO_N; i++){
    for(unsigned j = 0; j < ACADO_NY; j++){
      solver->linkReference(x, i, j);
      EXPECT_EQ(x, &acadoVariables.y[ACADO_NY*i + j]); 
    }
  }
}//}}}

TEST(AcadoSolver, linkWeight){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned j = 0; j < ACADO_NY; j++){
    solver->linkWeight(x, 0, j);
    EXPECT_EQ(x, &acadoVariables.W[ACADO_NY*ACADO_NY*0 + j*(ACADO_NY + 1)]); 
  }
}//}}}

TEST(AcadoSolver, linkParameter){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned i = 0; i< ACADO_N + 1; i++){
    for(unsigned j = 0; j < ACADO_NOD; j++){
      solver->linkParameter(x, i, j);
      EXPECT_EQ(x, &acadoVariables.od[ACADO_NOD*i + j]); 
    }
  }
}//}}}

TEST(AcadoSolver, linkEndReference){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned j = 0; j < ACADO_NY; j++){
    solver->linkEndReference(x, j);
    EXPECT_EQ(x, &acadoVariables.yN[j]); 
  }
}//}}}

TEST(AcadoSolver, linkEndWeight){/*{{{*/
  gios::VariablePtr x;
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  for(unsigned j = 0; j < ACADO_NYN; j++){
    solver->linkEndWeight(x, j);
    EXPECT_EQ(x, &acadoVariables.WN[(ACADO_NYN + 1)*j]); 
  }
}/*}}}*/
/*}}}*/

/* Variable Test{{{*/

/* State Tests{{{*/

TEST(State, linkState){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      solver->linkState(x, step, pos);
      EXPECT_EQ(state[pos], x); 
    }
  }
}//}}}

TEST(State, linkReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      solver->linkReference(x, step, pos);
      EXPECT_EQ(state[pos], x); 
    }
  }
}//}}}

TEST(State, linkWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkWeight(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      solver->linkWeight(x, step, pos);
      EXPECT_EQ(state[pos], x); 
    }
  }
}//}}}

TEST(State, linkFeedback){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkFeedback(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkFeedbackState(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(State, linkEndReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkEndReference(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkEndReference(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(State, linkEndWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkEndWeight(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkEndWeight(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

/*}}}*/

/* Control Tests{{{*/

TEST(Control, linkControl){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Control<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    control.linkControl(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      solver->linkControl(x, step, pos);
      EXPECT_EQ(control[pos], x); 
    }
  }
}//}}}

TEST(Control, linkReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Control<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkReference(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      solver->linkReference(x, step, pos);
      EXPECT_EQ(control[pos], x); 
    }
  }
}//}}}

TEST(Control, linkWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Control<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkWeight(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      solver->linkWeight(x, step, pos);
      EXPECT_EQ(control[pos], x); 
    }
  }
}//}}}

/*}}}*/

/* Parameter Tests{{{*/

TEST(Parameter, linkParameter){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Parameter<TestStruct> parameter (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    parameter.linkParameter(step, pos);
    for(pos = base_pos; pos < parameter.size(); pos++){//Var
      solver->linkParameter(x, step, pos);
      EXPECT_EQ(parameter[pos], x); 
    }
  }
}//}}}

/*}}}*/

/* Variable Tests{{{*/

TEST(Variable, construct){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  EXPECT_EQ(state.size(), 4); 
}//}}}

TEST(Variable, linkState){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      solver->linkState(x, step, pos);
      EXPECT_EQ(state[pos], x); 
    }
  }
}//}}}

TEST(Variable, linkControl){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkControl(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      solver->linkControl(x, step, pos);
      EXPECT_EQ(control[pos], x); 
    }
  }
}//}}}

TEST(Variable, linkParameter){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> parameter (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    parameter.linkParameter(step, pos);
    for(pos = base_pos; pos < parameter.size(); pos++){//Var
      solver->linkParameter(x, step, pos);
      EXPECT_EQ(parameter[pos], x); 
    }
  }
}//}}}

TEST(Variable, linkReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      solver->linkReference(x, step, pos);
      EXPECT_EQ(state[pos], x); 
    }
  }
}//}}}

TEST(Variable, linkWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  state.linkWeight(0, pos);//Constant weighting matrix
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkWeight(x, 0, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Variable, linkFeedback){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkFeedback(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkFeedbackState(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Variable, linkEndReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkEndReference(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkEndReference(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Variable, linkEndWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkEndWeight(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkEndWeight(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Variable, set){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  srand (time(NULL));
  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    state.set(foo);
    pos = base_pos;
    solver->linkState(x, step, pos);
    EXPECT_EQ(foo.x, *(x + 0)); 
    EXPECT_EQ(foo.y, *(x + 1)); 
    EXPECT_EQ(foo.z, *(x + 2)); 
    EXPECT_EQ(foo.r, *(x + 3)); 
  }
}//}}}

TEST(Variable, get){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Variable<TestStruct> state (solver.get());
  srand (time(NULL));
  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    state.set(foo);
    EXPECT_EQ(foo.x, state.get().x); 
    EXPECT_EQ(foo.y, state.get().y); 
    EXPECT_EQ(foo.z, state.get().z); 
    EXPECT_EQ(foo.r, state.get().r); 
  }
}//}}}

/*}}}*/

/*}}}*/

/* VariableArray Tests{{{*/

/* StateArray Tests{{{*/

TEST(StateArray, linkState){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StateArray<TestStruct> states (solver.get());
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  states.linkState(pos);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
}//}}}

TEST(StateArray, linkReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StateArray<TestStruct> states (solver.get());
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  unsigned step;
  states.linkReference(pos);
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
  state.linkEndReference(base_pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

TEST(StateArray, linkWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StateArray<TestStruct> states (solver.get());
  gios::State<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  unsigned step;
  states.linkWeight(pos);
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkWeight(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
  state.linkEndWeight(base_pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

/*}}}*/

/* ControlArray Tests{{{*/

TEST(ControlArray, linkControl){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::ControlArray<TestStruct> controls (solver.get());
  gios::Control<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  controls.linkControl(pos);
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkControl(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      EXPECT_EQ(controls[step][pos],control[pos]);
    }
  }
}//}}}

TEST(ControlArray, linkReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::ControlArray<TestStruct> controls (solver.get());
  gios::Control<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  unsigned step;
  controls.linkReference(pos);
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkReference(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      EXPECT_EQ(controls[step][pos],control[pos]);
    }
  }
}//}}}

TEST(ControlArray, linkWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::ControlArray<TestStruct> controls (solver.get());
  gios::Control<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  unsigned step;
  controls.linkWeight(pos);
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkWeight(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      EXPECT_EQ(controls[step][pos],control[pos]);
    }
  }
}//}}}

/*}}}*/

/* ParameterArray Tests{{{*/

TEST(ParameterArray, linkParameter){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::ParameterArray<TestStruct> parameters (solver.get());
  gios::Parameter<TestStruct> parameter (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  parameters.linkParameter(pos);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    parameter.linkParameter(step, pos);
    for(pos = base_pos; pos < parameter.size(); pos++){//Var
      EXPECT_EQ(parameters[step][pos],parameter[pos]);
    }
  }
}//}}}

/*}}}*/

/* VariableArray Tests{{{*/

TEST(VariableArray, construct){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> state (solver.get(), solver->getN() + 1);
  EXPECT_EQ(state.size(), solver->getN() + 1); 
}//}}}

TEST(VariableArray, linkState){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN() + 1);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  const unsigned base_pos = 0;
  unsigned pos = base_pos;
  states.linkState(pos);
  for(unsigned step = 0; step <  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
}//}}}

TEST(VariableArray, linkReferenceN1){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN() + 1);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkReference(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
  pos = base_pos;
  state.linkEndReference(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

TEST(VariableArray, linkReferenceN){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN());
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkReference(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
}//}}}

TEST(VariableArray, linkWeight2){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), 2);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkWeight(pos); pos = base_pos;
  state.linkWeight(0, pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
  step++;
  pos = base_pos;
  state.linkEndWeight(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

TEST(VariableArray, linkWeightN1){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN() + 1);
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkWeight(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkWeight(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
  pos = base_pos;
  state.linkEndWeight(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

TEST(VariableArray, linkWeightN){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN());
  gios::Variable<TestStruct> state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkWeight(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkWeight(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
}//}}}

TEST(VariableArray, linkControl){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> controls (solver.get(), solver->getN());
  gios::Variable<TestStruct> control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  controls.linkControl(pos);
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkControl(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      EXPECT_EQ(controls[step][pos],control[pos]);
    }
  }
}//}}}

TEST(VariableArray, linkParameter){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> parameters (solver.get(), solver->getN() + 1);
  gios::Variable<TestStruct> parameter (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  parameters.linkParameter(pos);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    parameter.linkParameter(step, pos);
    for(pos = base_pos; pos < parameter.size(); pos++){//Var
      EXPECT_EQ(parameters[step][pos],parameter[pos]);
    }
  }
}//}}}

TEST(VariableArray, set){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN() + 1);
  gios::Variable<TestStruct> state (solver.get());
  srand (time(NULL));
  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = 0;
  states.linkState(pos);
  states.set(foo);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    EXPECT_EQ(states[step].get().x, foo.x); 
    EXPECT_EQ(states[step].get().y, foo.y); 
    EXPECT_EQ(states[step].get().z, foo.z); 
    EXPECT_EQ(states[step].get().r, foo.r); 
  }
}//}}}

TEST(VariableArray, setArray){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN() + 1);
  gios::Variable<TestStruct> state (solver.get());
  std::vector<TestStruct> foo_array(solver->getN() +1);
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = 0;
  states.linkState(pos);
  srand (time(NULL));
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    foo_array[step].x = (double)rand();
    foo_array[step].y = (double)rand();
    foo_array[step].z = (double)rand();
    foo_array[step].r = (double)rand();
  }
  states.set(foo_array);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    EXPECT_EQ(states[step].get().x, foo_array[step].x); 
    EXPECT_EQ(states[step].get().y, foo_array[step].y); 
    EXPECT_EQ(states[step].get().z, foo_array[step].z); 
    EXPECT_EQ(states[step].get().r, foo_array[step].r); 
  }
}//}}}

TEST(VariableArray, get){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::VariableArray<TestStruct> states (solver.get(), solver->getN() + 1);
  gios::Variable<TestStruct> state (solver.get());
  std::vector<TestStruct> foo_array(solver->getN() +1);
  gios::Parameters p;
  solver->getParameters(p);
  unsigned base_pos = 0;
  unsigned pos = 0;
  srand (time(NULL));
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    foo_array[step].x = (double)rand();
    foo_array[step].y = (double)rand();
    foo_array[step].z = (double)rand();
    foo_array[step].r = (double)rand();
  }
  states.linkState(pos);
  states.set(foo_array);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    EXPECT_EQ(states.get()[step].x, state.get().x); 
    EXPECT_EQ(states.get()[step].y, state.get().y); 
    EXPECT_EQ(states.get()[step].z, state.get().z); 
    EXPECT_EQ(states.get()[step].r, state.get().r); 
  }
}//}}}

/*}}}*/

/*}}}*/

/* Struct Variadic{{{*/

TEST(Struct, construct){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  EXPECT_EQ(state.size(), 4); 
}//}}}

TEST(Struct, linkState){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      solver->linkState(x, step, pos);
      EXPECT_EQ(state[pos], x); 
    }
  }
}//}}}

TEST(Struct, linkControl){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkControl(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      solver->linkControl(x, step, pos);
      EXPECT_EQ(control[pos], x); 
    }
  }
}//}}}

TEST(Struct, linkParameter){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > parameter (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    parameter.linkParameter(step, pos);
    for(pos = base_pos; pos < parameter.size(); pos++){//Var
      solver->linkParameter(x, step, pos);
      EXPECT_EQ(parameter[pos], x); 
    }
  }
}//}}}

TEST(Struct, linkReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      solver->linkReference(x, step, pos);
      EXPECT_EQ(state[pos], x); 
    }
  }
}//}}}

TEST(Struct, linkWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  state.linkWeight(0, pos);//Constant weighting matrix
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkWeight(x, 0, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Struct, linkFeedback){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkFeedback(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkFeedbackState(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Struct, linkEndReference){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkEndReference(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkEndReference(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Struct, linkEndWeight){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  pos = base_pos;
  state.linkEndWeight(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    solver->linkEndWeight(x, pos);
    EXPECT_EQ(state[pos], x); 
  }
}//}}}

TEST(Struct, set){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  srand (time(NULL));
  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    state.set(foo);
    pos = base_pos;
    solver->linkState(x, step, pos);
    EXPECT_EQ(foo.x, *(x + 0)); 
    EXPECT_EQ(foo.y, *(x + 1)); 
    EXPECT_EQ(foo.z, *(x + 2)); 
    EXPECT_EQ(foo.r, *(x + 3)); 
  }
}//}}}

TEST(Struct, get){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  srand (time(NULL));
  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos;
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    state.set(foo);
    EXPECT_EQ(foo.x, state.get().x); 
    EXPECT_EQ(foo.y, state.get().y); 
    EXPECT_EQ(foo.z, state.get().z); 
    EXPECT_EQ(foo.r, state.get().r); 
  }
}//}}}

/*}}}*/

/* StructArray Tests{{{*/

TEST(StructArray, construct){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  EXPECT_EQ(states.size(), solver->getN() + 1); 
}//}}}

TEST(StructArray, linkState){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  const unsigned base_pos = 0;
  unsigned pos = base_pos;
  states.linkState(pos);
  for(unsigned step = 0; step <  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
}//}}}

TEST(StructArray, linkReferenceN1){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkReference(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
  pos = base_pos;
  state.linkEndReference(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

TEST(StructArray, linkReferenceN){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkReference(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkReference(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
}//}}}

TEST(StructArray, linkWeight2){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), 2);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkWeight(pos); pos = base_pos;
  state.linkWeight(0, pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
  step++;
  pos = base_pos;
  state.linkEndWeight(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

TEST(StructArray, linkWeightN1){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkWeight(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkWeight(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
  pos = base_pos;
  state.linkEndWeight(pos);
  for(pos = base_pos; pos < state.size(); pos++){//Var
    EXPECT_EQ(states[step][pos],state[pos]);
  }
}//}}}

TEST(StructArray, linkWeightN){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  const unsigned base_pos = 0;
  unsigned step = 0, pos = base_pos;
  states.linkWeight(pos); pos = base_pos;
  for(step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    state.linkWeight(step, pos);
    for(pos = base_pos; pos < state.size(); pos++){//Var
      EXPECT_EQ(states[step][pos],state[pos]);
    }
  }
}//}}}

TEST(StructArray, linkControl){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > controls (solver.get(), solver->getN());
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > control (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  controls.linkControl(pos);
  for(unsigned step = 0; step<  p.n; step++){//Step
    pos = base_pos;
    control.linkControl(step, pos);
    for(pos = base_pos; pos < control.size(); pos++){//Var
      EXPECT_EQ(controls[step][pos],control[pos]);
    }
  }
}//}}}

TEST(StructArray, linkParameter){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > parameters (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > parameter (solver.get());
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = base_pos;
  parameters.linkParameter(pos);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    parameter.linkParameter(step, pos);
    for(pos = base_pos; pos < parameter.size(); pos++){//Var
      EXPECT_EQ(parameters[step][pos],parameter[pos]);
    }
  }
}//}}}

TEST(StructArray, set){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  srand (time(NULL));
  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = 0;
  states.linkState(pos);
  states.set(foo);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    EXPECT_EQ(states[step].get().x, foo.x); 
    EXPECT_EQ(states[step].get().y, foo.y); 
    EXPECT_EQ(states[step].get().z, foo.z); 
    EXPECT_EQ(states[step].get().r, foo.r); 
  }
}//}}}

TEST(StructArray, setArray){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  std::vector<TestStruct> foo_array(solver->getN() +1);
  gios::Parameters p;
  solver->getParameters(p);
  gios::VariablePtr x;
  unsigned base_pos = 0;
  unsigned pos = 0;
  states.linkState(pos);
  srand (time(NULL));
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    foo_array[step].x = (double)rand();
    foo_array[step].y = (double)rand();
    foo_array[step].z = (double)rand();
    foo_array[step].r = (double)rand();
  }
  states.set(foo_array);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    EXPECT_EQ(states[step].get().x, foo_array[step].x); 
    EXPECT_EQ(states[step].get().y, foo_array[step].y); 
    EXPECT_EQ(states[step].get().z, foo_array[step].z); 
    EXPECT_EQ(states[step].get().r, foo_array[step].r); 
  }
}//}}}

TEST(StructArray, get){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::StructArray<TestStruct, double, &TestStruct::x, 
                              					&TestStruct::y, 
                              					&TestStruct::z,
					                              &TestStruct::r
                 > states (solver.get(), solver->getN() + 1);
  gios::Struct<TestStruct, double, &TestStruct::x, 
                              &TestStruct::y, 
                              &TestStruct::z,
                              &TestStruct::r
                 > state (solver.get());
  std::vector<TestStruct> foo_array(solver->getN() +1);
  gios::Parameters p;
  solver->getParameters(p);
  unsigned base_pos = 0;
  unsigned pos = 0;
  srand (time(NULL));
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    foo_array[step].x = (double)rand();
    foo_array[step].y = (double)rand();
    foo_array[step].z = (double)rand();
    foo_array[step].r = (double)rand();
  }
  states.linkState(pos);
  states.set(foo_array);
  for(unsigned step = 0; step<  p.n + 1; step++){//Step
    pos = base_pos;
    state.linkState(step, pos);
    EXPECT_EQ(states.get()[step].x, state.get().x); 
    EXPECT_EQ(states.get()[step].y, state.get().y); 
    EXPECT_EQ(states.get()[step].z, state.get().z); 
    EXPECT_EQ(states.get()[step].r, state.get().r); 
  }
}//}}}

/*}}}*/

/* Struct Nested{{{*/

TEST(StructNested, construct){/*{{{*/
  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
  gios::Struct<Parent, double, &Parent::x, 
                              &Parent::y, 
                              &Parent::z,
                              &Parent::r,
                              &Parent::child::a
                 > state (solver.get());
  EXPECT_EQ(state.size(), 5); 
}//}}}
//
//TEST(Struct, linkState){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  for(unsigned step = 0; step<  p.n + 1; step++){//Step
//    pos = base_pos;
//    state.linkState(step, pos);
//    for(pos = base_pos; pos < state.size(); pos++){//Var
//      solver->linkState(x, step, pos);
//      EXPECT_EQ(state[pos], x); 
//    }
//  }
//}//}}}
//
//TEST(Struct, linkControl){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > control (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  for(unsigned step = 0; step<  p.n; step++){//Step
//    pos = base_pos;
//    control.linkControl(step, pos);
//    for(pos = base_pos; pos < control.size(); pos++){//Var
//      solver->linkControl(x, step, pos);
//      EXPECT_EQ(control[pos], x); 
//    }
//  }
//}//}}}
//
//TEST(Struct, linkParameter){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > parameter (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  for(unsigned step = 0; step<  p.n + 1; step++){//Step
//    pos = base_pos;
//    parameter.linkParameter(step, pos);
//    for(pos = base_pos; pos < parameter.size(); pos++){//Var
//      solver->linkParameter(x, step, pos);
//      EXPECT_EQ(parameter[pos], x); 
//    }
//  }
//}//}}}
//
//TEST(Struct, linkReference){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  for(unsigned step = 0; step<  p.n; step++){//Step
//    pos = base_pos;
//    state.linkReference(step, pos);
//    for(pos = base_pos; pos < state.size(); pos++){//Var
//      solver->linkReference(x, step, pos);
//      EXPECT_EQ(state[pos], x); 
//    }
//  }
//}//}}}
//
//TEST(Struct, linkWeight){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos = base_pos;
//  state.linkWeight(0, pos);//Constant weighting matrix
//  for(pos = base_pos; pos < state.size(); pos++){//Var
//    solver->linkWeight(x, 0, pos);
//    EXPECT_EQ(state[pos], x); 
//  }
//}//}}}
//
//TEST(Struct, linkFeedback){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  pos = base_pos;
//  state.linkFeedback(pos);
//  for(pos = base_pos; pos < state.size(); pos++){//Var
//    solver->linkFeedbackState(x, pos);
//    EXPECT_EQ(state[pos], x); 
//  }
//}//}}}
//
//TEST(Struct, linkEndReference){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  pos = base_pos;
//  state.linkEndReference(pos);
//  for(pos = base_pos; pos < state.size(); pos++){//Var
//    solver->linkEndReference(x, pos);
//    EXPECT_EQ(state[pos], x); 
//  }
//}//}}}
//
//TEST(Struct, linkEndWeight){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  pos = base_pos;
//  state.linkEndWeight(pos);
//  for(pos = base_pos; pos < state.size(); pos++){//Var
//    solver->linkEndWeight(x, pos);
//    EXPECT_EQ(state[pos], x); 
//  }
//}//}}}
//
//TEST(Struct, set){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  srand (time(NULL));
//  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  for(unsigned step = 0; step<  p.n + 1; step++){//Step
//    pos = base_pos;
//    state.linkState(step, pos);
//    state.set(foo);
//    pos = base_pos;
//    solver->linkState(x, step, pos);
//    EXPECT_EQ(foo.x, *(x + 0)); 
//    EXPECT_EQ(foo.y, *(x + 1)); 
//    EXPECT_EQ(foo.z, *(x + 2)); 
//    EXPECT_EQ(foo.r, *(x + 3)); 
//  }
//}//}}}
//
//TEST(Struct, get){/*{{{*/
//  std::unique_ptr<gios::Solver> solver( new gios::AcadoSolver);
//  gios::Struct<TestStruct, double, &TestStruct::x, 
//                              &TestStruct::y, 
//                              &TestStruct::z,
//                              &TestStruct::r
//                 > state (solver.get());
//  srand (time(NULL));
//  TestStruct foo {(double)rand(), (double)rand(), (double)rand(), (double)rand()};
//  gios::Parameters p;
//  solver->getParameters(p);
//  gios::VariablePtr x;
//  unsigned base_pos = 0;
//  unsigned pos;
//  for(unsigned step = 0; step<  p.n + 1; step++){//Step
//    pos = base_pos;
//    state.linkState(step, pos);
//    state.set(foo);
//    EXPECT_EQ(foo.x, state.get().x); 
//    EXPECT_EQ(foo.y, state.get().y); 
//    EXPECT_EQ(foo.z, state.get().z); 
//    EXPECT_EQ(foo.r, state.get().r); 
//  }
//}//}}}
//
/*}}}*/

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  std::cerr << "[          ] HEY!!!"<< std::endl;
  return RUN_ALL_TESTS();
}

