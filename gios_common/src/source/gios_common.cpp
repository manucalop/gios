#include "gios_common/gios_common.h"
  
gios::Solver::Solver( const Parameters &p)://{{{
  x ( p.n + 1, VariablePtrVector(p.nx, nullptr) ),
  u ( p.n    , VariablePtrVector(p.nu, nullptr) ),
  r ( p.n    , VariablePtrVector(p.nr, nullptr) ),
  w ( p.n    , VariablePtrVector(p.nr, nullptr) ),
  p ( p.n + 1, VariablePtrVector(p.np, nullptr) ),
  xf( p.nx,  nullptr),
  rn( p.nrn, nullptr),
  wn( p.nrn, nullptr){
  std::cout<<"Solver Created"<<std::endl;
  }//}}}

gios::Solver::~Solver(){//{{{
  std::cout<<"Solver Deleted"<<std::endl;
};//}}}

void gios::Solver::setX( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  x[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void gios::Solver::setXF(const VariablePtr &var, const unsigned &pos){//{{{
  xf[pos] = var;                                                              
}//}}}                                                                              
                                                                                    
void gios::Solver::setU( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  u[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void gios::Solver::setR( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  r[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void gios::Solver::setW( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  w[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void gios::Solver::setP( const VariablePtr &var, const unsigned &step, const unsigned &pos){//{{{
  p[step][pos] = var;                                                               
}//}}}                                                                              
                                                                                    
void gios::Solver::setRN(const VariablePtr &var, const unsigned &pos){//{{{
  rn[pos] = var;                                                                    
}//}}}                                                                              
                                                                                    
void gios::Solver::setWN(const VariablePtr &var, const unsigned &pos){//{{{
  wn[pos] = var;
}//}}}

void gios::Solver::linkState(VariablePtr &var, const unsigned &step, const unsigned &pos) const{//{{{
    var = x[step][pos];
}//}}}

void gios::Solver::linkFeedbackState(VariablePtr &var, const unsigned &pos) const{//{{{
    var = xf[pos];
}//}}}

void gios::Solver::linkControl(VariablePtr &var, const unsigned &pos, const unsigned &step) const{//{{{
    var = u[step][pos];
}//}}}

void gios::Solver::linkReference(VariablePtr &var, const unsigned &pos, const unsigned &step) const{//{{{
    var = r[step][pos];
}//}}}

void gios::Solver::linkWeight(VariablePtr &var,const unsigned &step, const unsigned &pos) const{//{{{
    var = w[step][pos];
}//}}}

void gios::Solver::linkParameter(VariablePtr &var,const unsigned &step, const unsigned &pos) const{//{{{
    var = p[step][pos];
}//}}}

void gios::Solver::linkEndReference(VariablePtr &var, const unsigned &pos) const{//{{{
    var = rn[pos];
}//}}}

void gios::Solver::linkEndWeight(VariablePtr &var, const unsigned &pos) const{//{{{
    var = wn[pos];
}//}}}

void gios::Solver::getParameters(Parameters &params) const{//{{{
params.n    = u.size();  
params.nx   = x[0].size();  //Number of differential states 
params.nu   = u[0].size();  //Number of controls
params.np   = p[0].size();  //Number of parameters
params.nr   = r[0].size();  //Number of references
params.nrn  = rn.size(); //Number of references at end
}//}}}

void gios::Solver::test_init(){//{{{
  bool ok = true;
  std::cout<<"Testing Solver initialization ...\n";
  for(unsigned i = 0; i < x.size(); i++)
    for(unsigned j = 0;j < x[0].size(); j++)
      if (x[i][j] == nullptr){
        ok = false;
        std::cout<<"Found nullptr in x["<<i<<"]["<<j<<"]"<<std::endl;
      }
  
  for(unsigned i = 0; i < u.size(); i++)
    for(unsigned j = 0;j < u[0].size(); j++)
      if (u[i][j] == nullptr){
        std::cout<<"Found nullptr in u["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }


  for(unsigned i = 0; i < r.size(); i++)
    for(unsigned j = 0;j < r[0].size(); j++)
      if (r[i][j] == nullptr){
        std::cout<<"Found nullptr in r["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }

  for(unsigned i = 0; i < w.size(); i++)
    for(unsigned j = 0;j < w[0].size(); j++)
      if (w[i][j] == nullptr){
        std::cout<<"Found nullptr in w["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < p.size(); i++)
    for(unsigned j = 0;j < p[0].size(); j++)
      if (p[i][j] == nullptr){
        std::cout<<"Found nullptr in p["<<i<<"]["<<j<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < xf.size(); i++)
      if (xf[i] == nullptr){
        std::cout<<"Found nullptr in xf["<<i<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < rn.size(); i++)
      if (rn[i] == nullptr){
        std::cout<<"Found nullptr in rn["<<i<<"]"<<std::endl;
        ok = false;
      }
  
  for(unsigned i = 0; i < wn.size(); i++)
      if (wn[i] == nullptr){
        std::cout<<"Found nullptr in wn["<<i<<"]"<<std::endl;
        ok = false;
      }
  if (ok == true)
    std::cout<<"Initialization tested successfully"<<std::endl;
}//}}}

