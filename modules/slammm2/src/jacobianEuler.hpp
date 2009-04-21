/* $Id: jacobianEuler.hpp 1727 2006-05-23 09:07:57Z tlemaire $ */

#ifndef VME_JACOBIAN_EULER_HPP
#define VME_JACOBIAN_EULER_HPP

#include "jmath/jblas.hpp"

namespace jafar {
  namespace slammm {

    /** \ingroup vme
     *  @{
     */

    /// Jacobian computation for Euler representation
    void gJacTEuler(jblas::vec const& p, jblas::vec const& pp, jblas::vec const& T, jblas::sym_mat& J);

    /// Jacobian computation for Euler representation
    void gJacpEuler(jblas::vec const& p, jblas::vec const& pp, jblas::vec const& T, jblas::mat& J);

    /// Jacobian computation for Euler representation
    void gJacppEuler(jblas::vec const& p, jblas::vec const& pp, jblas::vec const& T, jblas::mat& J);

    /** @} */

  }
}

#endif // VME_JACOBIAN_EULER_HPP
