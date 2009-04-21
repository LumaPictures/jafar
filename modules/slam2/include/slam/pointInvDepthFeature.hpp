/* $Id$ */

#ifndef SLAM_POINT_INV_DEPTH_FEATURE_HPP
#define SLAM_POINT_INV_DEPTH_FEATURE_HPP

#include "jmath/jblas.hpp"

#include "camera/cameraBarreto.hpp"

#include "slam/feature.hpp"
#include "slam/featureModel.hpp"

namespace jafar {
  namespace slam {

    /** Sizeless 3D point using the inverse-depth parametrisation.
     * The first three parameters are the position of the camera.
     * The next three parameters is the direction of the point from
     * the camera. The last parameter is the inverse of the distance.
     * \ingroup slam
     */
    class PointInvDepthFeatureModel : public FeatureModel {
    public:

      jblas::mat J3dPoint;

      PointInvDepthFeatureModel();
      ~PointInvDepthFeatureModel();


      void toFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);

      void toFrameJac(const jblas::vec& frame_, const jblas::vec& x_);

      void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);
      void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes);
      
      void fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_);

      void compute3dPoint(jblas::vec const& x, jblas::vec& pt);
      void compute3dPointJac(jblas::vec const& x);
      void computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_ , jblas::mat& jac );

    }; // class PointInvDepthFeatureModel

    /** Sizeless 3D point using the inverse-depth parametrisation,
     * bearings-only sensor.
     * 
     * \ingroup slam
     */
    class BearingPointInvDepthFeatureObserveModel : public FeatureObserveModel {

    protected:

			/// see id0 in BearingPointInvDepthFeatureObserveModel::setup(double,double)
      double m_id0;
			/// see sigmaId0 in BearingPointInvDepthFeatureObserveModel::setup(double,double)
      double m_sigmaId0;

      PointInvDepthFeatureModel& pointInvDepthFeatureModel;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

      jblas::vec const& computeInnovation(jblas::vec const& z_,
					  jblas::vec const& zPred_);

//       jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
// 
//       void inverseObservationInSensorFrameJac(jblas::vec const& z_);

      jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);


    public:

      BearingPointInvDepthFeatureObserveModel(PointInvDepthFeatureModel& featureModel_);
      ~BearingPointInvDepthFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_BEARING;}

	  void postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jframe);

			/** setup noise parameters
			@param id0 initial inverse depth
			@param sigmaId0 standard deviation on initial inverse depth
			*/
      void setup(double id0 = 0.5, double sigmaId0 = 0.25);

    }; // class BearingPointInvDepthFeatureObserveModel


    /** Sizeless 3D point using the inverse-depth parametrisation
     * observed in a panoramic image.
     * 
     * \ingroup slam
     */
    class OmniImagePointInvDepthFeatureObserveModel : public FeatureObserveModel {

    protected:

			/// see id0 in OmniImagePointInvDepthFeatureObserveModel::setup(double,double)
      double m_id0;
			/// see sigmaId0 in OmniImagePointInvDepthFeatureObserveModel::setup(double,double)
      double m_sigmaId0;

      jafar::camera::CameraParabolicBarreto camera;

      PointInvDepthFeatureModel& pointInvDepthFeatureModel;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

//       jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
// 
//       void inverseObservationInSensorFrameJac(jblas::vec const& z_);

      jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);

    public:

      OmniImagePointInvDepthFeatureObserveModel(PointInvDepthFeatureModel& featureModel_,
						jafar::camera::CameraParabolicBarreto const& camera_);
      ~OmniImagePointInvDepthFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_OMNIIMAGE;}

	  void postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jframe);

			/** setup noise parameters 
			@param id0 initial inverse depth
			@param sigmaId0 standard deviation on initial inverse depth
			*/
      void setup(double id0 = 0.5, double sigmaId0 = 0.5);

    }; // class OmniImagePointInvDepthFeatureObserveModel

 /** Sizeless 3D point using the inverse-depth parametrisation
     * observed in a perspective image.
     * 
     * \ingroup slam
     */
    class ImagePointInvDepthFeatureObserveModel : public FeatureObserveModel {

    protected:

			/// see id0 in ImagePointInvDepthFeatureObserveModel::setup(double,double)
      double m_id0;
			/// see sigmaId0 in ImagePointInvDepthFeatureObserveModel::setup(double,double)
      double m_sigmaId0;

      jafar::camera::CameraPinhole camera;

      PointInvDepthFeatureModel& pointInvDepthFeatureModel;

      jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

      void predictObservationInSensorFrameJac(const jblas::vec& feature_);

//       jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
// 
//       void inverseObservationInSensorFrameJac(jblas::vec const& z_);

      jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

      void inverseObservationInSensorFrameJac(Observation const& obs_);

    public:

      ImagePointInvDepthFeatureObserveModel(PointInvDepthFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_);

      ~ImagePointInvDepthFeatureObserveModel();

      Observation::ObservationType typeObs() const {return Observation::POINT_IMAGE;}

      //void initStateInSensorFrame(InitFeature& feature_,
	//			  Observation const& obs_,
        //                          double dMin_, double dMax_);
	  void postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jfeature);

			/** setup noise parameters
			@param id0 initial inverse depth
			@param sigmaId0 standard deviation on initial inverse depth
			*/
      void setup(double id0 = 0.5, double sigmaId0 = 0.5);


    }; // class ImagePointInvDepthFeatureObserveModel

  }}

#endif // SLAM_POINT_INV_DEPTH_FEATURE_HPP
