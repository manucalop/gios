#ifndef GIOS_ACADO_H_
#define GIOS_ACADO_H_

#include "gios_common/gios_common.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

namespace gios_acado{

  class AcadoSolver : public gios::Solver{//{{{
    private:
      void init();
    public:
      AcadoSolver();
      ~AcadoSolver();
    public:
      void solve() override;
      void simulate() override;
  };//}}}

}

#endif
