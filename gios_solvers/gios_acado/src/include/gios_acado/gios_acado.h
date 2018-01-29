#ifndef GIOS_ACADO_H_
#define GIOS_ACADO_H_

#include "gios_common/gios_common.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

namespace gios{

  class AcadoSolver : public Solver{//{{{
    public:
      AcadoSolver();
      ~AcadoSolver();
    public:
      void linkState(         gios::VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkControl(       gios::VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkReference(     gios::VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkWeight(        gios::VariablePtr &var, const unsigned &step, const unsigned &pos) override;
      void linkParameter(     gios::VariablePtr &var, const unsigned &step, const unsigned &pos) override;

      void linkFeedbackState( gios::VariablePtr &var, const unsigned &pos) override;
      void linkEndReference(  gios::VariablePtr &var, const unsigned &pos) override;
      void linkEndWeight(     gios::VariablePtr &var, const unsigned &pos) override;

      void getParameters(gios::Parameters &p) override;
      unsigned getN() override;

      void solve() override;
      void simulate() override;
  };//}}}

}

#endif
