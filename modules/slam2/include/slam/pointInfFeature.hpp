/* $Id$ */

#ifndef SLAM_POINT_INF_FEATURE_HPP
#define SLAM_POINT_INF_FEATURE_HPP

#include "jmath/jblas.hpp"

#include "camera/cameraBarreto.hpp"

#include "slam/feature.hpp"
#include "slam/featureModel.hpp"
#include "slam/bearingOnlyFeature.hpp"

namespace jafar {
  namespace slam {

    /** Sizeless 3D point at infinity model.
     *
     * \ingroup slam
     */
    class PointInfFeatureModel : public FeatureModel {

    public:

      PointInfFeatureModel();
      ~PointInfFeatureModel();

      void toFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);

      void toFrameJac(const jblas::vec& frame_, const jblas::vec& x_);

      void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);
      void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes);
      
      void fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_);
      void computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac );

    };

    /** Point at infinity bearing-only observe model.
     *
     * \ingroup slam
     */
    class BearingPointInfFeatureObserveModel : public InfFeatureObserveModel {
      
    protected:

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

//       jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
// 
//       void inverseObservationInSensorFrameJac(jblas::vec const& z_);

       jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);

      double computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, Observation const& obsRef) const;

    public:

      BearingPointInfFeatureObserveModel(PointInfFeatureModel& featureModel_);
      ~BearingPointInfFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_BEARING;} 

      jblas::vec const& computeInnovation(jblas::vec const& z_,
					  jblas::vec const& zPred_);     

    }; // calss BearingPointInfFeatureObserveModel

    /** Point at infinity observed in a panoramique image.
     * 
     * \ingroup slam
     */
    class OmniImagePointInfFeatureObserveModel : public InfFeatureObserveModel {

    protected:

      jafar::camera::CameraParabolicBarreto camera;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

//       jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
// 
//       void inverseObservationInSensorFrameJac(jblas::vec const& z_);

      jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);

      double computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, Observation const& obsRef) const;

    public:

      OmniImagePointInfFeatureObserveModel(PointInfFeatureModel& featureModel_,
					   jafar::camera::CameraParabolicBarreto const& camera_);
      ~OmniImagePointInfFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_OMNIIMAGE;}

    };

  }
}

#endif // SLAM_POINT_INF_FEATURE_HPP
