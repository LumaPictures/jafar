/* $Id$ */

#ifndef SLAM_POINT_FEATURE_HPP
#define SLAM_POINT_FEATURE_HPP

#include "jmath/jblas.hpp"

#include "camera/cameraPinhole.hpp"
#include "camera/cameraBarreto.hpp"

#include "slam/eulerTools.hpp"
#include "slam/feature.hpp"
#include "slam/featureModel.hpp"
#include "slam/bearingOnlyFeature.hpp"

namespace jafar {
  namespace slam {

    /** Sizeless 3D point model.
     *
     * \ingroup slam
     */
    class PointFeatureModel : public FeatureModel {

    public:

      PointFeatureModel();
      ~PointFeatureModel();

      void toFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);

      void toFrameJac(const jblas::vec& frame_, const jblas::vec& x_);

      void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);
      void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes);
      
      void fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_);
      void computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac  );

    };

    /** sizeless point cartesian observe model.
     *
     * \ingroup slam
     */
    class CartesianPointFeatureObserveModel : public FeatureObserveModel {

    public:

      CartesianPointFeatureObserveModel(PointFeatureModel& featureModel_);

      ~CartesianPointFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_CARTESIAN;}

    protected:

      jblas::vec const& predictObservationInSensorFrame(jblas::vec const& feature_);

      void predictObservationInSensorFrameJac(jblas::vec const& feature_);

//       jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
// 
//       void inverseObservationInSensorFrameJac(jblas::vec const& z_);

      jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);

    }; // class CartesianPointFeatureObserveModel

    /** sizeless point polar observe model.
     *
     * \ingroup slam
     */
    class PolarPointFeatureObserveModel : public FeatureObserveModel {

    public:

      PolarPointFeatureObserveModel(PointFeatureModel& featureModel_);

      ~PolarPointFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_POLAR;}

      void setNoiseValues(double rhoUnitStdDev_, double thetaStdDev_, double phiStdDev_);

      void computeSensorR(Observation const& obs);

    protected:

      /// standard deviation on rho, per meter
      double rhoUnitStdDev;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

      jblas::vec const& computeInnovation(jblas::vec const& z_,
					  jblas::vec const& zPred_);

//       jblas::vec inverseObservationInSensorFrame(const jblas::vec& z_);
// 
//       void inverseObservationInSensorFrameJac(const jblas::vec& z_);

      jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);


    }; // class PolarPointFeatureObserveModel

    /** sizeless point bearing-only observe model.
     *
     * \ingroup slam
     */
    class BearingPointFeatureObserveModel : public BearingOnlyFeatureObserveModel {

    public:

      /// sigma/mean ratio
      double alpha;
      /// geometric serie base
      double beta;

      BearingPointFeatureObserveModel(PointFeatureModel& featureModel_, double alpha_ = 0.2, double kSigma_ = 1.0);

      ~BearingPointFeatureObserveModel();

      jblas::vec const& computeInnovation(jblas::vec const& z_,
					  jblas::vec const& zPred_);

      Observation::ObservationType typeObs() const {return Observation::POINT_BEARING;}

      virtual void initStateInSensorFrame(InitFeature& feature_,
					  Observation const& obs_,
					  double dMin_, double dMax_);

    protected:

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

      bool doUpdateInitStateInSensorFrame(jblas::vec const& closestMemberState, Observation const& obsRef);

    }; // class BearingPointFeatureObserveModel

    /** sizeless point from perspective image observe model.
     *
     * \ingroup slam
     */
    class ImagePointFeatureObserveModel : public BearingOnlyFeatureObserveModel {

    protected:

      jafar::camera::CameraPinhole camera;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

      double computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, Observation const& obsRef) const;

    public:

      /// sigma/mean ratio
      double alpha;
      /// geometric serie base
      double beta;

      ImagePointFeatureObserveModel(PointFeatureModel& featureModel_,
				    jafar::camera::CameraPinhole const& camera_,
				    double alpha_ = 0.2, double kSigma_ = 1.0);

      ~ImagePointFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_IMAGE;}

      void initStateInSensorFrame(InitFeature& feature_,
				  Observation const& obs_,
                                  double dMin_, double dMax_);

    }; // class ImagePointFeatureObserveModel

    /** sizeless point from stereo images observe model.
     *
     * \ingroup slam
     */
    class StereoImagePointFeatureObserveModel : public FeatureObserveModel {

    protected:

      jafar::camera::StereoBench stereoBench;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

//      jblas::vec inverseObservationInSensorFrame(const jblas::vec& z_);
//
//      void inverseObservationInSensorFrameJac(const jblas::vec& z_);

      jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);


    public:

      StereoImagePointFeatureObserveModel(PointFeatureModel& featureModel_,
					  jafar::camera::StereoBench const& stereoBench_);

      ~StereoImagePointFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_STEREOIMAGE;}

    }; // class StereoImagePointFeatureObserveModel

     /** sizeless point from omnidirectional image observe model.
     *
     * \ingroup slam
     */
    class OmniImagePointFeatureObserveModel : public BearingOnlyFeatureObserveModel {

    protected:

      jafar::camera::CameraParabolicBarreto camera;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

      double computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, Observation const& obsRef) const;

    public:

      /// sigma/mean ratio
      double alpha;
      /// geometric serie base
      double beta;

      OmniImagePointFeatureObserveModel(PointFeatureModel& featureModel_,
					jafar::camera::CameraParabolicBarreto const& camera_);

      ~OmniImagePointFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_OMNIIMAGE;}

      void setBoParam(double alpha_ = 0.15, double kSigma_ = 1.0);

      void initStateInSensorFrame(InitFeature& feature_,
				  Observation const& obs_,
                                  double dMin_, double dMax_);

    }; // class OmniImagePointFeatureObserveModel



  } // namespace slam
} // namespace jafar


#endif // SLAM_POINT_FEATURE_HPP
