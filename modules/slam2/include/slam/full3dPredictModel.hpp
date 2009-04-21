/* $Id$ */

#ifndef SLAM_FULL3D_PREDICT_MODEL_HPP
#define SLAM_FULL3D_PREDICT_MODEL_HPP

#include "jmath/jblas.hpp"

#include "geom/t3dEuler.hpp"

#include "filter/predictModel.hpp"

namespace jafar {
  namespace slam {

    /** Full 3d predict model. This model takes a full 3d
     * transformation as input command.
     *
     * Don't forget to set the covariance matrix using for
     * instance setUCov.
     * 
     * \ingroup slam
     */
    class Full3dPredictModel :
      public jafar::filter::JacobianBlockCommandPredictModel {

    private:

      /// prediction frame to reference frame
      geom::T3D* m_predToRef;
      /// reference frame to prediction frame
      geom::T3D* m_refToPred;

    public:

      Full3dPredictModel();
      ~Full3dPredictModel();

      jafar::geom::T3D const& predToRef() const {return *m_predToRef;}
      jafar::geom::T3D const& refToPred() const {return *m_refToPred;}

      void setPredToRef(jafar::geom::T3DEuler const& predToRef);

      void predict(jblas::vec_range& x_r, jblas::vec const& u);

    }; // class Full3dPredictModel

  } // namespace slam
} // namespace jafar

#endif // SLAM_FULL3D_PREDICT_MODEL_HPP
