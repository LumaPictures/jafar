/* $Id$ */

#ifndef SLAM_ODO_3D_PREDICT_MODEL_HPP
#define SLAM_ODO_3D_PREDICT_MODEL_HPP

#include "jmath/jblas.hpp"

#include "filter/predictModel.hpp"

namespace jafar {
  namespace slam {

    /** Odometry noise model.
     *
     * \ingroup slam
     */
    class OdoNoiseModel {

    public:
	  
			/// see kvv_ in OdoNoiseModel::set(double,double,double,double)
      double kvv;
			/// see kvw_ in OdoNoiseModel::set(double,double,double,double)
      double kvw;
			/// see kwv_ in OdoNoiseModel::set(double,double,double,double)
      double kwv;
			/// see kww_ in OdoNoiseModel::set(double,double,double,double)
      double kww;

      /**
       * Set the standard deviation of the robot displacement.
       * @param kvv_ error on the axial displacement, when the robot moves on a straight line (m/sqrt(m))
       *             for a value of kvv = 0.05, the error is of 5cm when the
       *             robots move of 1 meter (and of 10cm when it moves of 4 meters)
       * @param kvw_ error on the axial displacement, when the robot turns on itself (m/sqrt(rad))
       * @param kwv_ error on the angle, when the robot moves on a straight line (rad/sqrt(m))
       * @param kww_ error on the angle, when the robot turns on itself (rad/sqrt(rad))
       */
      void set(double kvv_, double kvw_, double kwv_, double kww_) {
	kvv = kvv_;
	kvw = kvw_;
	kwv = kwv_;
	kww = kww_;
      }

	

    /**
    Computes odometry covariance according to the noise model
    @param[in] vw odometry (forward speed and yaw-rotation speed)
    @param[out] vwCov odometry covariance
    */
      void computeCov(jblas::vec const& vw,
		      jblas::sym_mat& vwCov) const
      {
	JFR_PRECOND(vw.size() == 2,
		    "OdoNoiseModel::computeOdoCov: odo must be 2-dimension");
	JFR_PRECOND(vwCov.size1() == 2 && vwCov.size2() == 2,
		    "OdoNoiseModel::computeOdoCov: odoCov must be a 2x2 matrix");

	vwCov.clear();
	vwCov(0,0) = pow(kvv,2)*fabs(vw(0)) + pow(kvw, 2)*fabs(vw(1));
	vwCov(1,1) = pow(kwv,2)*fabs(vw(0)) + pow(kww, 2)*fabs(vw(1));
      }

    };

	
    /** 3d odometry predict model.
     *
     * \ingroup slam
     */
    class Odo3dPredictModel :
      public jafar::filter::JacobianBlockCommandPredictModel {
      
    public:

      /// 2D odometry error model
      OdoNoiseModel odoNoiseModel;
      
      double kzs, kprs;

      /// transformation robot to ref (only the yaw)
      double robotToRef_yaw;

      Odo3dPredictModel();
      ~Odo3dPredictModel() {}

      /**
       * Set the standard deviation of the robot displacement.
       * @param kvv_ see OdoNoiseModel::set(double,double,double,double)
       * @param kvw_ see OdoNoiseModel::set(double,double,double,double)
       * @param kwv_ see OdoNoiseModel::set(double,double,double,double)
       * @param kww_ see OdoNoiseModel::set(double,double,double,double)
       * @param kzs_ dynamic model, coefficient of dynamic noise on the vertical displacement, when the robot moves on a straight line (m/sqrt(m))
       * @param kprs_ dynamic model, coefficient of dynamic noise on pitch and roll, when the robot moves on a straight line (rad/sqrt(m))
       */
      void setModel(double kvv_, double kvw_, double kwv_, double kww_, double kzv_, double kprs_);

      void computeUCov(jblas::vec const& u) {
	odoNoiseModel.computeCov(u, m_uCov);
      }

      void predict(jblas::vec_range& x_r, jblas::vec const& u);

    }; // class Odo3dPredictModel
	
  } // namespace slam
} // namespace jafar

#endif // SLAM_ODO_3D_PREDICT_MODEL_HPP
