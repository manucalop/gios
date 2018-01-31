#ifndef GIOS_COMMON_H_
#define GIOS_COMMON_H_

#include <vector>
#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>
namespace gios{

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

class Solver{/*{{{*/
  public:
    explicit Solver();
    virtual ~Solver();
    virtual void linkState(         VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkControl(       VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkReference(     VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkWeight(        VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkParameter(     VariablePtr &var, const unsigned &step, const unsigned &pos) = 0;

    virtual void linkFeedbackState( VariablePtr &var, const unsigned &pos) = 0;
    virtual void linkEndReference(  VariablePtr &var, const unsigned &pos) = 0;
    virtual void linkEndWeight(     VariablePtr &var, const unsigned &pos) = 0;

    virtual void getParameters(Parameters &p) = 0;
    virtual unsigned getN() = 0;

    virtual void solve()=0;
    virtual void simulate()=0;
};/*}}}*/

/* Index {{{*/

template<class T>
class Index{/*{{{*/
  private:
    std::vector<T*> index;
  public:
    explicit Index();
    virtual ~Index(){};
    void add(T &data_);
    unsigned size() {return index.size();};
    T*& operator[] (unsigned x) {return index[x];};
};/*}}}*/

template<class T>
Index<T>::Index(){/*{{{*/
}/*}}}*/

template<class T>
void Index<T>::add(T &data_){/*{{{*/
  index.push_back(&data_); 
};/*}}}*/

/*}}}*/

/* Templated Classes {{{*/

//TODO: Parametrize getters and setters based on structs
//        gios::State<StructType>(solver)
//      Parameters given by the user (in order)
//        Struct.x
//        Struct.y
//        Struct.z 
//        ...
//      Initialize state vector
//        state = std::vector<VariablePtr>(num_parameters)
//      Generated getter:
//        Struct.x = *state[0];
//        Struct.y = *state[1];
//        Struct.z = *state[2];
//        ...
//      Generated setter:
//        *state[0] = Struct.x;
//        *state[1] = Struct.y;
//        *state[2] = Struct.z;
//        ...
//        
//      Check: Parameter pack, Metaprogramming
//        
//      Until then, the user has to inherit the 
//      classes and implement the getters and setters.
//      

/* State {{{*/
template<class T>    
class State{/*{{{*/
    Solver * const solver;
    std::vector<VariablePtr> state;
  public:
    explicit State(Solver * const solver_);
    ~State();
    T get() const;
    void set(T const& state_);
    void linkState(        const unsigned step, unsigned &pos);
    void linkReference(    const unsigned step, unsigned &pos);
    void linkWeight(       const unsigned step, unsigned &pos);
    void linkParameter(    const unsigned step, unsigned &pos);

    void linkFeedback(     unsigned &pos);
    void linkEndReference( unsigned &pos);
    void linkEndWeight(    unsigned &pos);
};/*}}}*/

template<class T>    
State<T>::State( Solver * const solver_):/*{{{*/
  solver(solver_)
{
    //T data;
    //state = std::vector<VariablePtr>(data.size());
}/*}}}*/

template<class T>    
State<T>::~State(){/*{{{*/
}/*}}}*/
//  
//template<class T>
//T const& State<T>::get() const{/*{{{*/
//    T data;
//    for(unsigned i = 0; i < data.size(); i++){
//      *data[i] = *state[i];
//    }
//  return data;
//};/*}}}*/
//
//template<class T>
//void State<T>::set(T const& state_){/*{{{*/
//  for(unsigned i = 0; i < state.size(); i++){
//    *state[i] = *state_[i];
//  }
//}/*}}}*/

template<class T>    
void State<T>::linkState(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < state.size(); i++){
    solver->linkState(state[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkReference(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < state.size(); i++){
    solver->linkReference(state[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkWeight(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < state.size(); i++){
    solver->linkWeight(state[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkParameter(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < state.size(); i++){
    solver->linkParameter(state[i], step, pos++);
  }
}/*}}}*/
 
template<class T>    
void State<T>::linkFeedback(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < state.size(); i++){
    solver->linkFeedbackState(state[i], pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkEndReference(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < state.size(); i++){
    solver->linkEndReference(state[i], pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkEndWeight(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < state.size(); i++){
    solver->linkEndWeight(state[i], pos++);
  }
}/*}}}*/

/*}}}*/
  
/* Control {{{*/

template<class T>
class Control{/*{{{*/
  Solver * const solver;
  std::vector<VariablePtr> control;
  public:
  explicit Control( Solver * const solver_);
  T get() const;
  void set(T const& control_);
  public:
  void linkControl(   const unsigned step, unsigned &pos);
  void linkReference( const unsigned step, unsigned &pos);
  void linkWeight(    const unsigned step, unsigned &pos);
  void linkParameter( const unsigned step, unsigned &pos);
};/*}}}*/

template<class T>
Control<T>::Control( Solver * const solver_):/*{{{*/
  solver(solver_){
}/*}}}*/
//  
//template<class T>
//T const& Control<T>::get() const{/*{{{*/
//    T data;
//    for(unsigned i = 0; i < data.size(); i++){
//      *data[i] = *control[i];
//    }
//  return data;
//};/*}}}*/
//
//template<class T>
//void Control<T>::set(T const& control_){/*{{{*/
//  for(unsigned i = 0; i < control.size(); i++){
//    *control[i] = *control_[i];
//  }
//}/*}}}*/
//
template<class T>
void Control<T>::linkControl(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < control.size(); i++){
    solver->linkControl(control[i], step, pos++);
  }
}/*}}}*/

template<class T>
void Control<T>::linkReference(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < control.size(); i++){
    solver->linkReference(control[i], step, pos++);
  }
}/*}}}*/

template<class T>
void Control<T>::linkWeight(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < control.size(); i++){
    solver->linkWeight(control[i], step, pos++);
  }
}/*}}}*/

template<class T>
void Control<T>::linkParameter(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < control.size(); i++){
    solver->linkParameter(control[i], step, pos++);
  }
}/*}}}*/

///*}}}*/

/* Parameter {{{*/

template<class T>
class Parameter{/*{{{*/
  Solver * const solver;
  std::vector<VariablePtr> parameter;
  public:
  explicit Parameter( Solver * const solver_);
  T get() const;
  void set(T const& parameter_);
  public:
  void linkParameter(   const unsigned step, unsigned &pos);
};/*}}}*/

template<class T>
Parameter<T>::Parameter( Solver * const solver_):/*{{{*/
  solver(solver_){
}/*}}}*/
//  
//template<class T>
//T const& Parameter<T>::get() const{/*{{{*/
//    T data;
//    for(unsigned i = 0; i < data.size(); i++){
//      *data[i] = *parameter[i];
//    }
//  return data;
//};/*}}}*/
//
//template<class T>
//void Parameter<T>::set(T const& parameter_){/*{{{*/
//  for(unsigned i = 0; i < parameter.size(); i++){
//    *parameter[i] = *parameter_[i];
//  }
//}/*}}}*/
//
template<class T>
void Parameter<T>::linkParameter(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < parameter.size(); i++){
    solver->linkParameter(parameter[i], step, pos++);
  }
}/*}}}*/

///*}}}*/


/* StateArray {{{*/

template<class T>
class StateArray{/*{{{*/
  public://Provisional
  std::vector<State<T>> var;
  public:
  explicit StateArray(Solver * const solver_);
  std::vector<T> const& get() const;
  void set(std::vector<T> const& var_);
  void set(T const& var_);
  void linkState(unsigned &pos);
  void linkReference(unsigned &pos);
  void linkWeight(unsigned &pos);
  void linkParameter(unsigned &pos);
};/*}}}*/

template<class T>
StateArray<T>::StateArray(Solver * const solver_):/*{{{*/
  var(solver_->getN() + 1, State<T>(solver_))
{
}/*}}}*/

template<class T>
std::vector<T> const& StateArray<T>::get() const{ /*{{{*/
  std::vector<T> a(var.size);
  for(unsigned i=0; i < var.size(); i++){
    a[i] = var[i].get();
  };
  return a; 
}/*}}}*/

template<class T>
void StateArray<T>::set(std::vector<T> const& var_){/*{{{*/
  for(unsigned i=0; i < var.size(); i++){
    var[i].set(var_[i]);
  }
}/*}}}*/

template<class T>
void StateArray<T>::set(T const& var_){/*{{{*/
  for(unsigned i=0; i < var.size(); i++){
    var[i].set(var_);
  }
}/*}}}*/

template<class T>
void StateArray<T>::linkState(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step < var.size(); step++){
    pos = initial_pos;
    var[step].linkState(step, pos);
  }
}/*}}}*/

template<class T>
void StateArray<T>::linkReference(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step < var.size() - 1; step++){
    pos = initial_pos;
    var[step].linkReference(step, pos);
  }
  var.back().linkEndReference(pos);
}/*}}}*/

template<class T>
void StateArray<T>::linkWeight(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step < var.size() - 1; step++){
    pos = initial_pos;
    var[step].linkWeight(step, pos);
  }
  var.back().linkEndWeight(initial_pos);
}/*}}}*/

template<class T>
void StateArray<T>::linkParameter(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step < var.size(); step++){
    pos = initial_pos;
    var[step].linkParameter(step, pos);
  }
}/*}}}*/

/*}}}*/

/* ControlArray {{{*/

template<class T>
class ControlArray{/*{{{*/
  public://Provisional
  std::vector<Control<T>> var;
  public:
  ControlArray(Solver * const solver_);
  std::vector<T> const& get() const;
  void set(std::vector<T> const& var_);
  void set(T const& var_);
  void linkControl(unsigned &pos);
  void linkReference(unsigned &pos);
  void linkWeight(unsigned &pos);
  void linkParameter(unsigned &pos);
};/*}}}*/

template<class T>
ControlArray<T>::ControlArray(Solver * const solver_):/*{{{*/
  var(solver_->getN(), Control<T>(solver_))
{
}/*}}}*/

template<class T>
std::vector<T> const& ControlArray<T>::get() const{ /*{{{*/
  std::vector<T> a(var.size);
  for(unsigned i=0; i < var.size(); i++){
    a[i] = var[i].get();
  }
  return a; 
}/*}}}*/

template<class T>
void ControlArray<T>::set(std::vector<T> const&var_){/*{{{*/
  for(unsigned i=0; i<var.size(); i++){
    var[i].set(var_[i]);
  }
}/*}}}*/

template<class T>
void ControlArray<T>::set(T const&var_){/*{{{*/
  for(unsigned i=0; i<var.size(); i++){
    var[i].set(var);
  }
}/*}}}*/

template<class T>
void ControlArray<T>::linkControl(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step<var.size(); step++){
    pos = initial_pos;
    var[step].linkControl(step, pos);
  }
}/*}}}*/

template<class T>
void ControlArray<T>::linkReference(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step<var.size(); step++){
    pos = initial_pos;
    var[step].linkReference(step, pos);
  }
}/*}}}*/

template<class T>
void ControlArray<T>::linkWeight(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step<var.size(); step++){
    pos = initial_pos;
    var[step].linkWeight(step, pos);
  }
}/*}}}*/

///*}}}*/

/* ParameterArray {{{*/

template<class T>
class ParameterArray{/*{{{*/
  public://Provisional
  std::vector<Parameter<T>> var;
  public:
  ParameterArray(Solver * const solver_);
  std::vector<T> const& get() const;
  void set(std::vector<T> const& var_);
  void set(T const& var_);
  void linkParameter(unsigned &pos);
};/*}}}*/

template<class T>
ParameterArray<T>::ParameterArray(Solver * const solver_):/*{{{*/
  var(solver_->getN() + 1, Parameter<T>(solver_)){
}/*}}}*/

template<class T>
std::vector<T> const& ParameterArray<T>::get() const{ /*{{{*/
  std::vector<T> a(var.size);
  for(unsigned i=0; i < var.size(); i++){
    a[i] = var[i].get();
  }
  return a; 
  };/*}}}*/

template<class T>
void ParameterArray<T>::set(std::vector<T> const& var_){/*{{{*/
  for(unsigned i=0; i<var.size(); i++){
    var[i].set(var_[i]);
  }
}/*}}}*/

template<class T>
void ParameterArray<T>::set(T const& var_){/*{{{*/
  for(unsigned i=0; i<var.size(); i++){
    var[i].set(var_);
  }
}/*}}}*/

template<class T>
void ParameterArray<T>::linkParameter(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step<var.size(); step++){
    pos = initial_pos;
    var[step].linkParameter(step, pos);
  }
}/*}}}*/

/*}}}*/

/*}}}*/

}

#endif

