#ifndef GIOS_H_
#define GIOS_H_

#include <algorithm>
#include <numeric>

#include <vector>
#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>

namespace gios{

// Typedefs {{{

typedef double * VariablePtr;

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
    virtual ~Solver(){};
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
    virtual double getKKTTolerance() = 0;

    virtual void solve()=0;
    virtual void simulate()=0;
    virtual void reset()=0;
};/*}}}*/

template<typename U = double>

class SolverT{/*{{{*/
  public:
    virtual ~SolverT(){};
    virtual void linkState(         U* &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkControl(       U* &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkReference(     U* &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkWeight(        U* &var, const unsigned &step, const unsigned &pos) = 0;
    virtual void linkParameter(     U* &var, const unsigned &step, const unsigned &pos) = 0;

    virtual void linkFeedbackState( U* &var, const unsigned &pos) = 0;
    virtual void linkEndReference(  U* &var, const unsigned &pos) = 0;
    virtual void linkEndWeight(     U* &var, const unsigned &pos) = 0;

    virtual void getParameters(Parameters &p) = 0;
    virtual unsigned getN() = 0;

    virtual void solve()=0;
    virtual void simulate()=0;
    virtual void reset()=0;
};/*}}}*/

template<class T> T do_get(std::vector<VariablePtr> const& vec_);
template<class T> void do_set(T const& var_, std::vector<VariablePtr> const& vec_);
template<class T> void allocate(std::vector<VariablePtr> & vec_);

/* Templated Classes {{{*/

//TODO: Parametrize getters and setters based on structs{{{
//      ++ Make templated static functions allocate, set and get and
//         include in State, Control and Parameter
//        gios::State<StructType>(solver)
//      Parameters given by the user (in order)
//        Struct.x
//        Struct.y
//        Struct.z 
//        ...
//      Initialize var vector
//        var = std::vector<VariablePtr>(num_parameters)
//      Generated getter:
//        Struct.x = *var[0];
//        Struct.y = *var[1];
//        Struct.z = *var[2];
//        ...
//      Generated setter:
//        *var[0] = Struct.x;
//        *var[1] = Struct.y;
//        *var[2] = Struct.z;
//        ...
//        
//      Check: Parameter pack, Metaprogramming
//        
//      Until then, the user has to inherit the 
//      classes and implement the getters and setters.
//template<class T>
//class Base{
//  protected:
//  std::vector<VariablePtr> var;
//  public:
//    Base();
//    virtual ~Base();
//    void allocate();
//    void set(T const& data_);
//    T get() const;
//};
//      }}}

/* Variable {{{*/

template<class T>    
class Variable{/*{{{*/
    Solver * const solver;
    std::vector<VariablePtr> var;
  public:
    explicit Variable(Solver * const solver_);
    T get() const;
    void set(T const& var_);
    VariablePtr& operator[] (unsigned x) { return var[x]; };
    unsigned size() const                { return var.size(); };
    void linkState(        const unsigned step, unsigned &pos);
    void linkControl(      const unsigned step, unsigned &pos);
    void linkReference(    const unsigned step, unsigned &pos);
    void linkWeight(       const unsigned step, unsigned &pos);
    void linkParameter(    const unsigned step, unsigned &pos);

    void linkFeedback(     unsigned &pos);
    void linkEndReference( unsigned &pos);
    void linkEndWeight(    unsigned &pos);
};/*}}}*/


//TODO
//template<class T>    
//Variable<T>::Variable( Solver * const solver_):/*{{{*/
//  solver(solver_)
//{
//  allocate<T>(var);
//}/*}}}*/
//  
//template<class T>
//T Variable<T>::get() const{/*{{{*/
//  return do_get<T>(var);
//};/*}}}*/
//
//template<class T>
//void Variable<T>::set(T const& var_){/*{{{*/
//  do_set<T>(var_, var);
//}/*}}}*/
//
template<class T>    
void Variable<T>::linkState(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkState(var[i], step, pos++);
  }
}/*}}}*/

template<class T>
void Variable<T>::linkControl(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkControl(var[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void Variable<T>::linkReference(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkReference(var[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void Variable<T>::linkWeight(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkWeight(var[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void Variable<T>::linkFeedback(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkFeedbackState(var[i], pos++);
  }
}/*}}}*/

template<class T>    
void Variable<T>::linkEndReference(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkEndReference(var[i], pos++);
  }
}/*}}}*/

template<class T>    
void Variable<T>::linkEndWeight(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkEndWeight(var[i], pos++);
  }
}/*}}}*/

template<class T>
void Variable<T>::linkParameter(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkParameter(var[i], step, pos++);
  }
}/*}}}*/

// Implementation for double

template<> Variable<double>::Variable(Solver * const solver_):/*{{{*/
  solver(solver_),
  var(1)
{
}/*}}}*/

template<> void Variable<double>::set(double const& var_){/*{{{*/
  *var[0] = var_;
}/*}}}*/

template<> double Variable<double>::get() const{/*{{{*/
  return *var[0];
}/*}}}*/

/*}}}*/

/* State {{{*/
template<class T>    
class State{/*{{{*/
    Solver * const solver;
    std::vector<VariablePtr> var;
  public:
    explicit State(Solver * const solver_);
    T get() const;
    void set(T const& var_);
    VariablePtr& operator[] (unsigned x) { return var[x]; };
    unsigned size() const{ return var.size(); };
    void linkState(        const unsigned step, unsigned &pos);
    void linkReference(    const unsigned step, unsigned &pos);
    void linkWeight(       const unsigned step, unsigned &pos);

    void linkFeedback(     unsigned &pos);
    void linkEndReference( unsigned &pos);
    void linkEndWeight(    unsigned &pos);
};/*}}}*/

template<class T>    
State<T>::State( Solver * const solver_):/*{{{*/
  solver(solver_)
{
  allocate<T>(var);
}/*}}}*/
  
template<class T>
T State<T>::get() const{/*{{{*/
  return do_get<T>(var);
};/*}}}*/

template<class T>
void State<T>::set(T const& var_){/*{{{*/
  do_set<T>(var_, var);
}/*}}}*/

template<class T>    
void State<T>::linkState(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkState(var[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkReference(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkReference(var[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkWeight(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkWeight(var[i], step, pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkFeedback(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkFeedbackState(var[i], pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkEndReference(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkEndReference(var[i], pos++);
  }
}/*}}}*/

template<class T>    
void State<T>::linkEndWeight(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkEndWeight(var[i], pos++);
  }
}/*}}}*/

/*}}}*/
  
/* Control {{{*/

template<class T>
class Control{/*{{{*/
  Solver * const solver;
  std::vector<VariablePtr> var;
  public:
    explicit Control( Solver * const solver_);
    T get() const;
    void set(T const& var_);
    VariablePtr& operator[] (unsigned x) { return var[x]; };
    unsigned size() const{ return var.size(); };
    void linkControl(   const unsigned step, unsigned &pos);
    void linkReference( const unsigned step, unsigned &pos);
    void linkWeight(    const unsigned step, unsigned &pos);

};/*}}}*/

template<class T>
Control<T>::Control( Solver * const solver_):/*{{{*/
  solver(solver_){
  allocate<T>(var);
}/*}}}*/
  
template<class T>
T Control<T>::get() const{/*{{{*/
  return do_get<T>(var);
};/*}}}*/

template<class T>
void Control<T>::set(T const& var_){/*{{{*/
  do_set<T>(var_, var);
}/*}}}*/

template<class T>
void Control<T>::linkControl(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkControl(var[i], step, pos++);
  }
}/*}}}*/

template<class T>
void Control<T>::linkReference(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkReference(var[i], step, pos++);
  }
}/*}}}*/

template<class T>
void Control<T>::linkWeight(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkWeight(var[i], step, pos++);
  }
}/*}}}*/

/*}}}*/

/* Parameter {{{*/

template<class T>
class Parameter{/*{{{*/
  Solver * const solver;
  std::vector<VariablePtr> var;
  public:
    explicit Parameter( Solver * const solver_);
    T get() const;
    void set(T const& var_);
    VariablePtr& operator[] (unsigned x) { return var[x]; };
    unsigned size() const{ return var.size(); };
    void linkParameter(   const unsigned step, unsigned &pos);
};/*}}}*/

template<class T>
Parameter<T>::Parameter( Solver * const solver_):/*{{{*/
  solver(solver_){
    allocate<T>(var);
}/*}}}*/
  
template<class T>
T Parameter<T>::get() const{/*{{{*/
  return do_get<T>(var);
};/*}}}*/

template<class T>
void Parameter<T>::set(T const& var_){/*{{{*/
  do_set<T>(var_,var);
}/*}}}*/

template<class T>
void Parameter<T>::linkParameter(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkParameter(var[i], step, pos++);
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
    std::vector<T> get() const;
    void set(std::vector<T> const& var_);
    void set(T const& var_);
    State<T>& operator[] (unsigned x) { return var[x]; };
    unsigned size() const{ return var.size(); };
    void linkState(unsigned &pos);
    void linkReference(unsigned &pos);
    void linkWeight(unsigned &pos);
};/*}}}*/

template<class T>
StateArray<T>::StateArray(Solver * const solver_):/*{{{*/
  var(solver_->getN() + 1, State<T>(solver_))
{
}/*}}}*/

template<class T>
std::vector<T> StateArray<T>::get() const{ /*{{{*/
  std::vector<T> a(var.size());
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
  pos = initial_pos;
  var.back().linkEndReference(pos);
}/*}}}*/

template<class T>
void StateArray<T>::linkWeight(unsigned &pos){/*{{{*/
  unsigned initial_pos = pos;
  for(unsigned step=0; step < var.size() - 1; step++){
    pos = initial_pos;
    var[step].linkWeight(step, pos);
  }
  pos = initial_pos;
  var.back().linkEndWeight(pos);
}/*}}}*/

/*}}}*/

/* ControlArray {{{*/

template<class T>
class ControlArray{/*{{{*/
  public://Provisional
  std::vector<Control<T>> var;
  public:
    ControlArray(Solver * const solver_);
    std::vector<T> get() const;
    void set(std::vector<T> const& var_);
    void set(T const& var_);
    Control<T>& operator[] (unsigned x) { return var[x]; };
    unsigned size() const{ return var.size(); };
    void linkControl(unsigned &pos);
    void linkReference(unsigned &pos);
    void linkWeight(unsigned &pos);
};/*}}}*/

template<class T>
ControlArray<T>::ControlArray(Solver * const solver_):/*{{{*/
  var(solver_->getN(), Control<T>(solver_))
{
}/*}}}*/

template<class T>
std::vector<T> ControlArray<T>::get() const{ /*{{{*/
  std::vector<T> a(var.size());
  for(unsigned i=0; i < var.size(); i++){
    a[i] = var[i].get();
  }
  return a; 
}/*}}}*/

template<class T>
void ControlArray<T>::set(std::vector<T> const& var_){/*{{{*/
  for(unsigned i=0; i<var.size(); i++){
    var[i].set(var_[i]);
  }
}/*}}}*/

template<class T>
void ControlArray<T>::set(T const& var_){/*{{{*/
  for(unsigned i=0; i < var.size(); i++){
    var[i].set(var_);
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
  std::vector<Parameter<T>> var;
  public:
    ParameterArray(Solver * const solver_);
    std::vector<T> get() const;
    void set(std::vector<T> const& var_);
    void set(T const& var_);
    Parameter<T>& operator[] (unsigned x) { return var[x]; };
    unsigned size() const{ return var.size(); };
    void linkParameter(unsigned &pos);
};/*}}}*/

template<class T>
ParameterArray<T>::ParameterArray(Solver * const solver_):/*{{{*/
  var(solver_->getN() + 1, Parameter<T>(solver_)){
}/*}}}*/

template<class T>
std::vector<T> ParameterArray<T>::get() const{ /*{{{*/
  std::vector<T> a(var.size());
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

/* VariableArray {{{*/

template<class T>
class VariableArray{/*{{{*/
  Solver * const solver;
  std::vector<Variable<T>> var;
  public:
    explicit VariableArray(Solver * const solver_, unsigned n);
    Variable<T>& operator[] (unsigned x) { return var[x]; };
    Variable<T>& back()                  { return var.back(); };
    unsigned size() const                { return var.size(); };
    std::vector<T> get() const;
    void set(std::vector<T> const& var_);
    void set(T const& var_);
    void linkState(unsigned &pos);
    void linkControl(unsigned &pos);
    void linkReference(unsigned &pos);
    void linkWeight(unsigned &pos);
    void linkParameter(unsigned &pos);
    bool checkVector(unsigned n);
};/*}}}*/

template<class T>
VariableArray<T>::VariableArray(Solver * const solver_, unsigned n):/*{{{*/
  solver(solver_),
  var(n, Variable<T>(solver_))
{
}/*}}}*/

template<class T>
bool VariableArray<T>::checkVector(unsigned n){/*{{{*/
  return var.size() == n;
}/*}}}*/

template<class T>
std::vector<T> VariableArray<T>::get() const{ /*{{{*/
  std::vector<T> a(var.size());
  for(unsigned i=0; i < var.size(); i++){
    a[i] = var[i].get();
  };
  return a; 
}/*}}}*/

template<class T>
void VariableArray<T>::set(std::vector<T> const& var_){/*{{{*/
  for(unsigned i=0; i < var.size(); i++){
    var[i].set(var_[i]);
  }
}/*}}}*/

template<class T>
void VariableArray<T>::set(T const& var_){/*{{{*/
  for(unsigned i=0; i < var.size(); i++){
    var[i].set(var_);
  }
}/*}}}*/

template<class T>
void VariableArray<T>::linkState(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() +1)){
    std::cout<<"linkState: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN() + 1; step++){
    pos = initial_pos;
    var[step].linkState(step, pos);
  }
}/*}}}*/

template<class T>
void VariableArray<T>::linkReference(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() +1) && !checkVector(solver->getN())){
    std::cout<<"linkReference: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN(); step++){
    pos = initial_pos;
    var[step].linkReference(step, pos);
  }
  if( !checkVector(solver->getN()) ){//The user wants to include EndReference in the same array
    pos = initial_pos;
    var.back().linkEndReference(pos);
  }
}/*}}}*/

template<class T>
void VariableArray<T>::linkWeight(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() +1) && !checkVector(solver->getN()) && !checkVector(2)){
    std::cout<<"linkWeight: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  unsigned step;
  for(step=0; step < var.size() - 1; step++){
    pos = initial_pos;
    var[step].linkWeight(step, pos);
  }
  if( checkVector(solver->getN()) ){
    pos = initial_pos;
    var.back().linkWeight(step, pos);
  }
  else{//The user wants to include EndWeights in the same array (2 steps for fixed weights)
    pos = initial_pos;
    var.back().linkEndWeight(pos);
  }

}/*}}}*/

template<class T>
void VariableArray<T>::linkControl(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN())){
    std::cout<<"linkControl: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN(); step++){
    pos = initial_pos;
    var[step].linkControl(step, pos);
  }
}/*}}}*/

template<class T>
void VariableArray<T>::linkParameter(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() + 1)){
    std::cout<<"linkParameter: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN() + 1; step++){
    pos = initial_pos;
    var[step].linkParameter(step, pos);
  }
}/*}}}*/

/*}}}*/

/*}}}*/

/* Variadic Templates {{{*/

/* Struct {{{*/
template <typename T, typename U, U T::* ... Ts>
class Struct{ /*{{{*/
   Solver * const solver;
   std::vector<U *> var;
  public:
    explicit Struct(Solver * const solver_);
    void set(T const& var_);
    T get() const;
    U* & operator[] (unsigned x) { return var[x]; };
    unsigned size() const        { return var.size(); };
    void linkState(        const unsigned step, unsigned &pos);
    void linkControl(      const unsigned step, unsigned &pos);
    void linkReference(    const unsigned step, unsigned &pos);
    void linkWeight(       const unsigned step, unsigned &pos);
    void linkParameter(    const unsigned step, unsigned &pos);

    void linkFeedback(     unsigned &pos);
    void linkEndReference( unsigned &pos);
    void linkEndWeight(    unsigned &pos);
};/*}}}*/

template <typename T, typename U, U T::* ... Ts>
Struct<T, U, Ts ... >::Struct(Solver * const solver_):/*{{{*/
  solver(solver_),
  var(sizeof...(Ts))
{
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void Struct<T, U, Ts ...>::set(T const& var_){/*{{{*/
  unsigned i = 0;
  for ( auto&& d : {Ts ...} ){
    *var[i++] = var_.*d;
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
T Struct<T, U, Ts ...>::get() const{/*{{{*/
  T var_;
  unsigned i = 0;
  for ( auto&& d : {Ts ...} ){
    var_.*d = *var[i++];
  }
  return var_;
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>    
void Struct<T, U, Ts ...>::linkState(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkState(var[i], step, pos++);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void Struct<T, U, Ts ...>::linkControl(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkControl(var[i], step, pos++);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>    
void Struct<T, U, Ts ...>::linkReference(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkReference(var[i], step, pos++);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>    
void Struct<T, U, Ts ...>::linkWeight(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkWeight(var[i], step, pos++);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>    
void Struct<T, U, Ts ...>::linkFeedback(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkFeedbackState(var[i], pos++);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>    
void Struct<T, U, Ts ...>::linkEndReference(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkEndReference(var[i], pos++);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>    
void Struct<T, U, Ts ...>::linkEndWeight(unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkEndWeight(var[i], pos++);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void Struct<T, U, Ts ...>::linkParameter(const unsigned step, unsigned &pos){/*{{{*/
  for(unsigned i = 0; i < var.size(); i++){
    solver->linkParameter(var[i], step, pos++);
  }
}/*}}}*/

/*}}}*/

/* StructArray {{{*/

template <typename T, typename U, U T::* ... Ts>
class StructArray{/*{{{*/
  Solver * const solver;
  std::vector<Struct<T, U, Ts ...>> var;
  public:
    explicit StructArray(Solver * const solver_, unsigned n);
    Struct<T, U, Ts ...>& operator[] (unsigned x) { return var[x]; };
    Struct<T, U, Ts ...>& back()                  { return var.back(); };
    unsigned size() const                { return var.size(); };
    std::vector<T> get() const;
    void set(std::vector<T> const& var_);
    void set(T const& var_);
    void linkState(unsigned &pos);
    void linkControl(unsigned &pos);
    void linkReference(unsigned &pos);
    void linkWeight(unsigned &pos);
    void linkParameter(unsigned &pos);
    bool checkVector(unsigned n);
};/*}}}*/

template <typename T, typename U, U T::* ... Ts>
StructArray<T,U,Ts ...>::StructArray(Solver * const solver_, unsigned n):/*{{{*/
  solver(solver_),
  var(n, Struct<T,U,Ts ...>(solver_))
{
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
bool StructArray<T,U,Ts...>::checkVector(unsigned n){/*{{{*/
  return var.size() == n;
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
std::vector<T> StructArray<T,U,Ts...>::get() const{ /*{{{*/
  std::vector<T> a(var.size());
  for(unsigned i=0; i < var.size(); i++){
    a[i] = var[i].get();
  };
  return a; 
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void StructArray<T,U,Ts...>::set(std::vector<T> const& var_){/*{{{*/
  for(unsigned i=0; i < var.size(); i++){
    var[i].set(var_[i]);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void StructArray<T,U,Ts...>::set(T const& var_){/*{{{*/
  for(unsigned i=0; i < var.size(); i++){
    var[i].set(var_);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void StructArray<T,U,Ts...>::linkState(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() +1)){
    std::cout<<"linkState: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN() + 1; step++){
    pos = initial_pos;
    var[step].linkState(step, pos);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void StructArray<T,U,Ts...>::linkReference(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() +1) && !checkVector(solver->getN())){
    std::cout<<"linkReference: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN(); step++){
    pos = initial_pos;
    var[step].linkReference(step, pos);
  }
  if( !checkVector(solver->getN()) ){//The user wants to include EndReference in the same array
    pos = initial_pos;
    var.back().linkEndReference(pos);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void StructArray<T,U,Ts...>::linkWeight(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() +1) && !checkVector(solver->getN()) && !checkVector(2)){
    std::cout<<"linkWeight: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  unsigned step;
  for(step=0; step < var.size() - 1; step++){
    pos = initial_pos;
    var[step].linkWeight(step, pos);
  }
  if( checkVector(solver->getN()) ){
    pos = initial_pos;
    var.back().linkWeight(step, pos);
  }
  else{//The user wants to include EndWeights in the same array (2 steps for fixed weights)
    pos = initial_pos;
    var.back().linkEndWeight(pos);
  }

}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void StructArray<T,U,Ts...>::linkControl(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN())){
    std::cout<<"linkControl: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN(); step++){
    pos = initial_pos;
    var[step].linkControl(step, pos);
  }
}/*}}}*/

template <typename T, typename U, U T::* ... Ts>
void StructArray<T,U,Ts...>::linkParameter(unsigned &pos){/*{{{*/
  if (!checkVector(solver->getN() + 1)){
    std::cout<<"linkParameter: Error in vector dimension"<<std::endl;
    exit(1);
  }
  unsigned initial_pos = pos;
  for(unsigned step=0; step < solver->getN() + 1; step++){
    pos = initial_pos;
    var[step].linkParameter(step, pos);
  }
}/*}}}*/

/*}}}*/

/*}}}*/
//
///* Variadic Templates 2 {{{*/
//
///* Member Definition{{{*/
//
//template <typename...> struct Accessor;
//
//template <typename T, typename C, T (C::*m)>
//struct Accessor<std::integral_constant<T (C::*), m>>/*{{{*/
//{
//    const T& get(const C& c) { return c.*m; }
//    T& get(C& c) { return c.*m; }
//};/*}}}*/
//
//template <typename T, typename C, T (C::*m), typename ...Ts>
//struct Accessor<std::integral_constant<T (C::*), m>, Ts...>/*{{{*/
//{
//    auto get(const C& c) -> decltype(Accessor<Ts...>().get(c.*m))
//    { return Accessor<Ts...>().get(c.*m); }
//
//    auto get(C& c) -> decltype(Accessor<Ts...>().get(c.*m))
//    { return Accessor<Ts...>().get(c.*m); }
//};/*}}}*/
//
//template <auto ...ms>
//using Member = Accessor<std::integral_constant<decltype(ms), ms>...>;
//
///*}}}*/
//
///* NestedStruct {{{*/
//
//template <typename T, typename U, typename ...Ts>
//class NestedStruct{/*{{{*/
//  Solver * const solver;
//  std::vector<U *> var{sizeof...(Ts)};
//
//  template <std::size_t ... Is>
//  T get(std::index_sequence<Is...>) const
//  {
//      T res;
//      ((Ts{}.get(res) = *var[Is]), ...); // Fold expression C++17
//      return res;
//  }
//  template <std::size_t ... Is>
//  void set(std::index_sequence<Is...>, T const& t)
//  {
//      ((*var[Is] = Ts{}.get(t)), ...); // Fold expression C++17
//  }
//
// public:
//  NestedStruct(Solver * const solver_);
//
//  T get() const { return get(std::index_sequence_for<Ts...>()); }
//  void set(const T& t) { return set(std::index_sequence_for<Ts...>(), t); }
//  U* & operator[] (unsigned x) { return var[x]; };
//  unsigned size() const        { return var.size(); };
//  void linkState(        const unsigned step, unsigned &pos);
//  void linkControl(      const unsigned step, unsigned &pos);
//  void linkReference(    const unsigned step, unsigned &pos);
//  void linkWeight(       const unsigned step, unsigned &pos);
//  void linkParameter(    const unsigned step, unsigned &pos);
//
//  void linkFeedback(     unsigned &pos);
//  void linkEndReference( unsigned &pos);
//  void linkEndWeight(    unsigned &pos);
//};/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//NestedStruct<T, U, Ts ... >::NestedStruct(Solver * const solver_):/*{{{*/
//  solver(solver_)
//{
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ... >::linkState(const unsigned step, unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkState(var[i], step, pos++);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ...>::linkControl(const unsigned step, unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkControl(var[i], step, pos++);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ...>::linkReference(const unsigned step, unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkReference(var[i], step, pos++);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ...>::linkWeight(const unsigned step, unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkWeight(var[i], step, pos++);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ...>::linkFeedback(unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkFeedbackState(var[i], pos++);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ...>::linkEndReference(unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkEndReference(var[i], pos++);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ...>::linkEndWeight(unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkEndWeight(var[i], pos++);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStruct<T, U, Ts ...>::linkParameter(const unsigned step, unsigned &pos){/*{{{*/
//  for(unsigned i = 0; i < var.size(); i++){
//    solver->linkParameter(var[i], step, pos++);
//  }
//}/*}}}*/
//
///*}}}*/
//
///* NestedStructArray {{{*/
//
//template <typename T, typename U, typename ...Ts>
//class NestedStructArray{/*{{{*/
//  Solver * const solver;
//  std::vector<NestedStruct<T, U, Ts ...>> var;
//  public:
//    explicit NestedStructArray(Solver * const solver_, unsigned n);
//    NestedStruct<T, U, Ts ...>& operator[] (unsigned x) { return var[x]; };
//    NestedStruct<T, U, Ts ...>& back()                  { return var.back(); };
//    unsigned size() const                { return var.size(); };
//    std::vector<T> get() const;
//    void set(std::vector<T> const& var_);
//    void set(T const& var_);
//    void linkState(unsigned &pos);
//    void linkControl(unsigned &pos);
//    void linkReference(unsigned &pos);
//    void linkWeight(unsigned &pos);
//    void linkParameter(unsigned &pos);
//    bool checkVector(unsigned n);
//};/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//NestedStructArray<T,U,Ts ...>::NestedStructArray(Solver * const solver_, unsigned n):/*{{{*/
//  solver(solver_),
//  var(n, NestedStruct<T,U,Ts...>(solver_))
//{
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//bool NestedStructArray<T,U,Ts...>::checkVector(unsigned n){/*{{{*/
//  return var.size() == n;
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//std::vector<T> NestedStructArray<T,U,Ts...>::get() const{ /*{{{*/
//  std::vector<T> a(var.size());
//  for(unsigned i=0; i < var.size(); i++){
//    a[i] = var[i].get();
//  };
//  return a; 
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStructArray<T,U,Ts...>::set(std::vector<T> const& var_){/*{{{*/
//  for(unsigned i=0; i < var.size(); i++){
//    var[i].set(var_[i]);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStructArray<T,U,Ts...>::set(T const& var_){/*{{{*/
//  for(unsigned i=0; i < var.size(); i++){
//    var[i].set(var_);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStructArray<T,U,Ts...>::linkState(unsigned &pos){/*{{{*/
//  if (!checkVector(solver->getN() +1)){
//    std::cout<<"linkState: Error in vector dimension"<<std::endl;
//    exit(1);
//  }
//  unsigned initial_pos = pos;
//  for(unsigned step=0; step < solver->getN() + 1; step++){
//    pos = initial_pos;
//    var[step].linkState(step, pos);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStructArray<T,U,Ts...>::linkReference(unsigned &pos){/*{{{*/
//  if (!checkVector(solver->getN() +1) && !checkVector(solver->getN())){
//    std::cout<<"linkReference: Error in vector dimension"<<std::endl;
//    exit(1);
//  }
//  unsigned initial_pos = pos;
//  for(unsigned step=0; step < solver->getN(); step++){
//    pos = initial_pos;
//    var[step].linkReference(step, pos);
//  }
//  if( !checkVector(solver->getN()) ){//The user wants to include EndReference in the same array
//    pos = initial_pos;
//    var.back().linkEndReference(pos);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStructArray<T,U,Ts...>::linkWeight(unsigned &pos){/*{{{*/
//  if (!checkVector(solver->getN() +1) && !checkVector(solver->getN()) && !checkVector(2)){
//    std::cout<<"linkWeight: Error in vector dimension"<<std::endl;
//    exit(1);
//  }
//  unsigned initial_pos = pos;
//  unsigned step;
//  for(step=0; step < var.size() - 1; step++){
//    pos = initial_pos;
//    var[step].linkWeight(step, pos);
//  }
//  if( checkVector(solver->getN()) ){
//    pos = initial_pos;
//    var.back().linkWeight(step, pos);
//  }
//  else{//The user wants to include EndWeights in the same array (2 steps for fixed weights)
//    pos = initial_pos;
//    var.back().linkEndWeight(pos);
//  }
//
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStructArray<T,U,Ts...>::linkControl(unsigned &pos){/*{{{*/
//  if (!checkVector(solver->getN())){
//    std::cout<<"linkControl: Error in vector dimension"<<std::endl;
//    exit(1);
//  }
//  unsigned initial_pos = pos;
//  for(unsigned step=0; step < solver->getN(); step++){
//    pos = initial_pos;
//    var[step].linkControl(step, pos);
//  }
//}/*}}}*/
//
//template <typename T, typename U, typename ...Ts>
//void NestedStructArray<T,U,Ts...>::linkParameter(unsigned &pos){/*{{{*/
//  if (!checkVector(solver->getN() + 1)){
//    std::cout<<"linkParameter: Error in vector dimension"<<std::endl;
//    exit(1);
//  }
//  unsigned initial_pos = pos;
//  for(unsigned step=0; step < solver->getN() + 1; step++){
//    pos = initial_pos;
//    var[step].linkParameter(step, pos);
//  }
//}/*}}}*/
//
///*}}}*/
//
///*}}}*/
//
}
#endif

