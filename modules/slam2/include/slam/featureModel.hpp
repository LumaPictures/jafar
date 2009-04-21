/* $Id$ */

#ifndef SLAM_FEATURE_MODEL_HPP
#define SLAM_FEATURE_MODEL_HPP

#include <map>

#include "jmath/jblas.hpp"
#include "jmath/gaussianVector.hpp"

#include "geom/t3dEuler.hpp"

#include "filter/observeModel.hpp"

#include "slam/observation.hpp"

namespace jafar {
  namespace slam {

    // forward declaration
    class BaseFeature;

    /** This class defines a generic feature model.
     *
     * \ingroup slam
     */
    class FeatureModel {
      std::size_t m_sizeMergeState;
    public:

      /** Jacobian of toFrame() and fromFrame() with respect to feature
       * state.
       *
       * \warning do not forget to call toframJac() or fromFrameJac()
       * before using Jx
       */
      jblas::mat Jx;

      /** Jacobian of toframe() and fromFrame() with respect to
       * transformation 3D.
       *
       * \warning do not forget to call toframJac() or fromFrameJac()
       * before using Jframe
       */
      jblas::mat Jframe;

      FeatureModel(std::size_t sizeRobotPose, std::size_t sizeState, std::size_t sizeMergeState) :
          m_sizeMergeState(sizeMergeState),
		  Jx(sizeState, sizeState),
		  Jframe(sizeState, sizeRobotPose)
          {}

      virtual ~FeatureModel() {}

      std::size_t sizeState() {return Jx.size1();}
      std::size_t sizeRobotPose() {return Jframe.size2();}
      std::size_t sizeMergeState() { return m_sizeMergeState; }

      /** Computes the feature state \a x in \a frame_ , the result is
       * stored in \a xRes.
       */
      virtual void toFrame(const jblas::vec& frame_, const jblas::vec& x, jblas::vec& xRes) = 0;

      /** Computes the jacobian of toFrame().
       */
      virtual void toFrameJac(jblas::vec const& frame_, jblas::vec const& x_) = 0;

      /** Computes the feature state \a x expressed in \a frame_ in
       *  the frame of reference, the result is stored in \a xRes.
       */
      virtual void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes) = 0;
      /// \sa fromFrame(jblas::vec const&, jblas::vec const&, jblas::vec&)
      virtual void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes) = 0;

      /** Computes the jacobian of fromFrame().
       */
      virtual void fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_) = 0;

 //     /** Function to apply contraints over features (this function is only implemented 
 //       for segment features)
 //       */
 //     virtual void fixFeature(jblas::vec_range& L) {};

      virtual void computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac ) = 0;
    };


    /** The root class for all feature models. It adds to the
     * generic BlockObserveModel specific things related to
     * slam. Especially it takes into account all the frame composition
     * induced by robot pose and sensor pose relative pose on robot.
     *
     * \ingroup slam
     */
    class BaseFeatureObserveModel : public jafar::filter::BlockObserveModel {

    public:

      FeatureModel& featureModel;

	  /// Constructor for systems with equal init and update observe models
      BaseFeatureObserveModel(FeatureModel& featureModel, std::size_t sizeObs_);

    /// Constructor for systems with differing init and update observe models
    BaseFeatureObserveModel(FeatureModel& featureModel_, std::size_t sizeObs_, std::size_t sizeObsInit_);
    /// Constructor for systems with differing init and update observe models
    BaseFeatureObserveModel(FeatureModel& featureModel_, std::size_t sizeObs_, std::size_t sizeObsInit_, std::size_t sizeInnovation_, std::size_t sizePrediction);

      virtual ~BaseFeatureObserveModel();

      std::size_t sizeRobotPose() const {return sizeState1();};
      std::size_t sizeState() const {return sizeState2();};

      virtual Observation::ObservationType typeObs() const = 0;
	  
	  /// Observation covariances matrix for initialization purposes.
	  /// The init observation is the observation used for initialization purposes. It normally coincides with the observation noise, but not necessarily> it depends on the landmark type and the chosen parametrizations of the measurement space at initialization (feature detection) and update (feature matching) times.
	  jblas::sym_mat RInit;

      jafar::geom::T3D const& robotToSensor() const {return *m_robotToSensor;}

      /** Set the position of the sensor within the robot frame.
       */
      void setRobotToSensor(jafar::geom::T3DEuler const& robotToSensor_);
	  /// Get observation noise covariances matrix
	  virtual jblas::sym_mat const& getR() const {
		  if (robotToSensor().hasCov())
			  return featureR;
		  else
			  return R;
      }

	  
	  /// Get size of RInit
	  std::size_t sizeObsInit(){
		  return p_sizeObsInit;
	  }
	  
	  /// Get init observation noise covariances matrix. 
	  /// The init observation is the observation used for initialization purposes. It normally coincides with the observation noise, but not necessarily> it depends on the landmark type and the chosen parametrizations of the measurement space at initialization (feature detection) and update (feature matching) times.
      virtual jblas::sym_mat const& getRInit() const {
		  return RInit;
	  }
	  
      /** Compute observation noise covariance. It takes into account
       * robotToSensor uncertainty.
       */
      virtual void computeR(Observation const& obs);
	  
	  /// Compute observation noise covariance for init purposes.
	  /// It should consider uncertainty in robotToSensor() <- TODO: not implemented.
	  virtual void computeRInit(Observation const& obs);
	  

      /** Compute observation noise from sensor. A default
       * implementation is provided, it does nothing (constant model).
       */
      virtual void computeSensorR(Observation const& obs) {}
	  
	  
	  /// Compute sensor observation noise for initialization purposes. 
	  virtual void computeSensorRInit(Observation const& obs) {
		  computeSensorR(obs);
		  RInit = R;
	  } 

	  
      /** Predict observation from robot and feature state. Some work
        * is done considering robot and sensor frames, and observe
        * model is delegated until call to
        * predictObservationInSensorFrame().
        */
      virtual jblas::vec const& predictObservation(const jblas::vec_range& robotPose_,
						   const jblas::vec_range& feature_);

      /** Computes jacobian of predictObservation(). Same
       *  consideration as for predictObservation() and call to
       *  predictObservationInSensorFrameJac().
       */
      virtual void predictObservationJac(const jblas::vec_range& robotPose_,
					 const jblas::vec_range& feature_);

      /** Init internal state. It is a set of parameters, describing
       *  the feature, but not integrated to the SLAM stochastic map.
       */
      virtual void initInternalState(BaseFeature& feature, 
				     jafar::geom::T3DEuler const& robotPose_, 
				     Observation const& obs_) const {}

      /** Update internal state. 
       * \sa initInternalState()
       */
      virtual void updateInternalState(BaseFeature& feature,
				       jafar::geom::T3DEuler const& robotPose_,
				       Observation const& obs_) const {}

      /** Mean and covariance of the expected measurement
        */
      jafar::jmath::GaussianVector zPred;


    protected:
      /**
       * Compute prediction observation jacobian
       */
		  void computePredictObservationJac(const jblas::vec_range& robotPose_,
           const jblas::vec_range& feature_, jblas::mat& J1, jblas::mat& J2);
      /// jacobian of predictObservationInSensorFrame() function
      jblas::mat JobsSensor;

      /** Position of the sensor within the robot frame.
       */
      jafar::geom::T3D* m_robotToSensor;

      /// jacobian of predictObservation() with respect to robotToSensor
      jblas::mat JobsRobotToSensor;

      /** Observe model noise covariance. It includes observation
       * noise plus robotToSensor transformation uncertainty.
       */
      jblas::sym_mat featureR;

	  /// Size of obsevations for initialization purposes
	  std::size_t p_sizeObsInit;
	  
      /** Prediction of the observation, the feature is expressed in
       *  the sensor frame \a featureInSensorFrame_.
       */
      virtual jblas::vec const& predictObservationInSensorFrame(jblas::vec const& featureInSensorFrame_) = 0;

      /** Jacobian of predictObservationInSensorFrame().
       */
      virtual void predictObservationInSensorFrameJac(jblas::vec const& featureInSensorFrame_) = 0;

    }; // class BaseFeatureObserveModel
    
    /** Fully observable feature observe model. It takes into accout
     * sensor frame in inverse observation model.
     *
     * \ingroup slam
     */
    class FeatureObserveModel : public BaseFeatureObserveModel {
      
    public:

      /** Jacobian of the inverse observe function
       *
       * \warning do not forget to call inverseObservationJacobian()
       * before using JinvObs
       */
      jblas::mat JinvObs;

		FeatureObserveModel(FeatureModel& model, std::size_t sizeObs_);
		
    FeatureObserveModel(FeatureModel& model, std::size_t sizeObs_, std::size_t sizeObsInit_);
    FeatureObserveModel(FeatureModel& model, std::size_t sizeObs_, std::size_t sizeObsInit_, std::size_t sizeInnovation_, std::size_t sizePrediction_);
		
      virtual ~FeatureObserveModel();

      /** Takes into accout sensor frame and delegate initialisation
       *  in sensor frame to inverseObservationInSensorFrame()
       */
//       jblas::vec inverseObservation(const jblas::vec& z_);
//       
//       void inverseObservationJac(const jblas::vec& z_);

      jblas::vec inverseObservation(Observation const& obs_);
      
      void inverseObservationJac(Observation const& obs_);

	  virtual void postInitCovariance(jblas::sym_mat_range & P,jblas::mat const& Jfeature) {};


    protected:

//      /** inverse observation function in sensor frame, to be defined by user.
//       */
//      virtual jblas::vec inverseObservationInSensorFrame(const jblas::vec& z_) = 0;
//
//      /// jacobian of inverseObservationInSensorFrame()
//      virtual void inverseObservationInSensorFrameJac(const jblas::vec& z_) = 0;
      
      /** inverse observation function in sensor frame, to be defined by user.
       */
      virtual jblas::vec inverseObservationInSensorFrame(Observation const& obs_) = 0;

      /// jacobian of inverseObservationInSensorFrame()
      virtual void inverseObservationInSensorFrameJac(Observation const& obs_) = 0;

    }; // class FeatureObserveModel

  } // namespace slam
} // namespace jafar


#endif // SLAM_FEATURE_MODEL_HPP
