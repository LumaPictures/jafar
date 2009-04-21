/* $Id$ */

#ifndef SLAM_SEGMENT_FEATURE_HPP
#define SLAM_SEGMENT_FEATURE_HPP

#include "boost/tuple/tuple.hpp"

#include "jmath/jblas.hpp"
#include "jmath/ublasExtra.hpp"

#include "geom/t3d.hpp"

#include "filter/constraintModel.hpp"

#include "filter/predictModel.hpp"

#include "slam/eulerTools.hpp"
#include "slam/observation.hpp"
#include "slam/bearingOnlyFeature.hpp"
#include "camera/cameraPinhole.hpp"

namespace jafar {
namespace slam {

	class BearingOnlySlam;

	/** Observation of a segment.    
	*
	* \ingroup slam 
	*/
	class SegmentObservation : public Observation {
	
	public:

	/// first extremity in the image plane
	jblas::vec2 ext1;
	/// second extremity in the image plane
	jblas::vec2 ext2;

		/// Constructor.
		/// Constructs an empty observation from a robot
		///@param _robotId robot identifier
	SegmentObservation(ObservationType type_, unsigned int _robotId = 0) :
		Observation(type_, _robotId),
		ext1(), ext2()
	{
        JFR_ASSERT( type_ == SEGMENT_IMAGE or type_ == SEGMENTID_IMAGE or type == SEGMENTID_EXT_IMAGE or type == SEGMENT_STEREOIMAGE, "Not a segment type");
    }
	
	/// Copy constructor
	/// Constructs an observation from a given observation
	///@param obs The observation to copy
	SegmentObservation(SegmentObservation const& obs) :
		Observation(obs),
		ext1(obs.ext1),
		ext2(obs.ext2)
	{}

	/// Destructor
	virtual ~SegmentObservation() {}

	/// Sets the extremities to the observation
	///@param ext1_ the first endpoint
	///@param ext2_ the second endpoint
	void setExtremities(jblas::vec2 const& ext1_, jblas::vec2 const& ext2_) {
		ext1.assign(ext1_);
		ext2.assign(ext2_);
	}

	}; // SegmentObservation

    std::ostream& operator <<(std::ostream& s, const jafar::slam::SegmentObservation& o_);

    /** Observation of a segment.    
    *
    * \ingroup slam 
    */
    class StereoSegmentObservation : public Observation {
    
    public:

    /// first extremity in the image plane with disparity
    jblas::vec3 ext1;
    /// second extremity in the image plane with disparity
    jblas::vec3 ext2;

        /// Constructor.
        /// Constructs an empty observation from a robot
        ///@param _robotId robot identifier
    StereoSegmentObservation(unsigned int _robotId = 0) :
        Observation(SEGMENT_STEREOIMAGE, _robotId),
        ext1(), ext2()
    {}
    
    /// Copy constructor
    /// Constructs an observation from a given observation
    ///@param obs The observation to copy
    StereoSegmentObservation(StereoSegmentObservation const& obs) :
        Observation(obs),
        ext1(obs.ext1),
        ext2(obs.ext2)
    {}

    /// Destructor
    virtual ~StereoSegmentObservation() {}

    /// Sets the extremities to the observation
    ///@param ext1_ the first endpoint
    ///@param ext2_ the second endpoint
    void setExtremities(jblas::vec3 const& ext1_, jblas::vec3 const& ext2_) {
        ext1.assign(ext1_);
        ext2.assign(ext2_);
    }

    }; // StereoSegmentObservation


    std::ostream& operator <<(std::ostream& s, const jafar::slam::StereoSegmentObservation& o_);

	/// Director vector unit constraint.
	class SegmentUConstraint : public jafar::filter::JacobianStrongConstraintModel {

	public:

	SegmentUConstraint(double alpha, int maxNbTimesApplied) :
	JacobianStrongConstraintModel(6, 1.0, alpha, maxNbTimesApplied)
	{
	// some terms are always zero
	J.clear();
	}

	~SegmentUConstraint() {}

	double estimateValue(jblas::vec_range const& x0) const {
	using namespace ublas;
	return norm_2(project(x0, range(3,6)));
	}

	void estimateValueJac(jblas::vec_range const& x0) {
	using namespace ublas;
	jblas::mat_range J_r(J, range(0,1), range(3,6));
	jmath::ublasExtra::norm_2Jac<3>(project(x0, range(3,6)), J_r);
	}

	};

	class SegmentPluckerConstraint : public jafar::filter::JacobianStrongConstraintModel {

	public:

	SegmentPluckerConstraint(double alpha, int maxNbTimesApplied) :
	JacobianStrongConstraintModel(6, 0.0, alpha, maxNbTimesApplied) {}

	~SegmentPluckerConstraint() {}

	double estimateValue(jblas::vec_range const& x0) const {
	using namespace ublas;
	p_estimatedValue = inner_prod(project(x0, range(0,3)), project(x0, range(3,6)));
	return p_estimatedValue;
	}

	void estimateValueJac(jblas::vec_range const& x0) {
	using namespace ublas;
	jmath::ublasExtra::inner_prodJac<3>(project(x0, range(0,3)), project(x0, range(3,6)), J);
	}

//       double weakeningVariance() {
// // 	p_R = p_R0;
// 	return p_R;
//       }

	};

    
    /**Segment Feature constriant model.
     *
     * \ingroup slam
     */
    class SegmentFeatureContraintModel :
      public jafar::filter::JacobianBlockPredictModel {
      
    public:

      SegmentFeatureContraintModel();
      ~SegmentFeatureContraintModel() {}

      void fixFeature(jblas::vec_range& L);
      void fixFeatureJac(jblas::vec_range& L, jblas::mat& JfL);

      void predict(jblas::vec_range& x_r);

    }; // class SegmentFeatureContraintModel 


	/** A segment feature, with its extremities. The extremities are
	* not represented in a stochastic way, they are expressed in the
	* 1-dimenssion frame defined by the segment representation (s,u).
	*
	* \ingroup slam
	*/
	class SegmentFeature : public BaseFeature {

	public:

	std::string debugObs;

	/// coordinate of the first extremity
	double s1;
	/// coordinate of the second extremity
	double s2;

	/// flag to enlarge extremites 
	bool ENLARGE;

//       /// to keep extremities information, used for the display
//       SegmentObservation referenceObservation;

	SegmentFeature(unsigned int id, FeatureModel& model, std::size_t sizeObs, Observation::ObservationType typeObs_);
	virtual ~SegmentFeature();

	jblas::vec_range const& u() const {return *p_u;}
	jblas::vec_range& u() {return *p_u;}
	jblas::vec_range const& n() const {return *p_n;}
	jblas::vec_range& n() {return *p_n;}

	void setFlagEnlarge() {ENLARGE=true;}

	virtual void setState(jblas::vec& x_, jblas::sym_mat& P_);

	jblas::vec3 getExt1() const;
	jblas::sym_mat getExt1Cov() const;
	jblas::vec3 getExt2() const;
	jblas::sym_mat getExt2Cov() const;
    jblas::mat getExt1Jac() const;
    jblas::mat getExt2Jac() const;
//       void applyConstraints();

	static void setConstraintAlpha(double alpha) {p_constraintAlpha = alpha;}
	static double getConstraintAlpha() {return p_constraintAlpha;}
	static void setConstraintMaxNbTimesApplied(int maxNbTimesApplied) {p_constraintMaxNbTimesApplied = maxNbTimesApplied;}
	static int getConstraintMaxNbTimesApplied() {return p_constraintMaxNbTimesApplied;}
    static void setFlagPlucklerConstraint(bool pluckerConstraint) {ENABLE_PLUCKER_CONSTRAINT=pluckerConstraint;}
	
	/// Transformation form Homogeneous Plucker to Euclidean Plucker
	//jblas::vec homogenousToEuclideanPlucker(jblas::vec const& hPlucker);

	protected:

	virtual void writeLogHeader(jafar::kernel::DataLogger& log) const;
	virtual void writeLogData(jafar::kernel::DataLogger& log) const;

	private:

	/** n=h.|n| where h is distance to origin, and n is the normal
	*  to the plane containing the line and the origin
	*/
	jblas::vec_range* p_n;

	/// direction of the the segment (unit vector).
	jblas::vec_range* p_u;      

	  /// Get 3D endpoint given abscissa.
	void computeExt(double s_, jblas::vec3& ext) const;

          /// The Jacobian of computeExt() wrt the Plucker line.
	  void computeExtJac(double s_, jblas::mat& J) const;

	  /// Get 3D endpoint covariance given abscissa.
	void computeExtCov(double s_, jblas::sym_mat& extCov) const;


	static double p_constraintAlpha;
	static int p_constraintMaxNbTimesApplied;
    static bool ENABLE_PLUCKER_CONSTRAINT;

	}; // SegmentFeature


	std::ostream& operator <<(std::ostream& s, const jafar::slam::SegmentFeature& f);

	/** segment model.
	*
	* \ingroup slam
	*/
	class SegmentFeatureModel : public FeatureModel {

	public:

	SegmentFeatureModel();
	~SegmentFeatureModel();


	void toFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes);
	void toFrameJac(const jblas::vec& frame_, const jblas::vec& x_);      

	void fromFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec& xRes);
	void fromFrame(const jblas::vec& frame_, const jblas::vec& x_, jblas::vec_range& xRes);
	void fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_);
    //void fixFeature(jblas::vec& L);
    void computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac  );
	
	}; // class SegmentFeatureModel
	
	/** segment extracted from an image observe model.
	*
	* \ingroup slam
	*/
	class ImageEuclideanPluckerFeatureObserveModel : public BearingOnlyFeatureObserveModel {

	protected:

	jafar::camera::CameraPinhole camera;
	jblas::mat33 Pl;
	jblas::mat33 Plinv;

	void pluckerLineInit(jblas::vec const& z_, double d_, double phi_, double sg_, jblas::vec& pluckerLine) const;

	void pluckerLineInitJac(jblas::vec const& z_, double d_, double phi_, double sg_, jblas::mat& J) const;

	jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

	void predictObservationInSensorFrameJac(const jblas::vec& feature_);

	jblas::vec const& predictObservationInImageFrame(jblas::vec3 const& l_);
	void predictObservationInImageFrameJac(jblas::vec3 const& l_, 
						jblas::mat& Jl) const;

	boost::tuple<double,double> computeExtremitiesAbscissa(SegmentFeature const& segFeature_, 
								geom::T3DEuler const& robotPose_,
								SegmentObservation const& obs_) const;

	void segmentExtremitiesPluckerLines(SegmentObservation const& segObs_, 
					jblas::vec& lineExt1Sensor, 
					jblas::vec& lineExt2Sensor) const;

	static boost::tuple<double, double, double> pluckerLinesDistance(jblas::vec const& line1_, 
									jblas::vec const& line2_);

	/// covariance on pixel coordinate (u,v)
	jblas::sym_mat p_pixCov;
	
	
	/// Compute sensor covariance
	virtual void computeSensorR(Observation const& obs);
	
	/// non-constant covariance on pixel coordinate (u,v)
	// virtual void computeSensorRVariable(Observation const& obs);

	/// \todo real implementation
	double computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, Observation const& obsRef) const {return 1.0;}

	public:
	
	/** gaussian sum distribution over depth
	* geometric serie sigma/mean ratio
	*/
	double depthAlpha;
	/** gaussian sum distribution over depth
	* geometric serie base
	*/
	double depthBeta;

	/** gaussian sum over direction, phi min
	*/
	double phiMin;
	/** gaussian sum over direction, phi max
	*/
	double phiMax;

	/** gaussian sum over direction, number of gaussians
	*/
	double phiSigma;

	/** gaussian sum over direction, kSigma factor
	*/
	double phiKSigma;


	ImageEuclideanPluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, 
					jafar::camera::CameraPinhole const& camera_,
					double depthAlpha_, double depthKSigma_,
					double phiMin_, double phiMax_,
					double phiSigma_,
					double phiKSigma_);

	~ImageEuclideanPluckerFeatureObserveModel();

	Observation::ObservationType typeObs() const {return Observation::SEGMENT_IMAGE;}

	void setPixCov(jblas::vec const& pixCov_, double stabilizingFactor = 1.0);

	void initStateInSensorFrame(InitFeature& feature_,
				Observation const& obs_,
								double dMin_, double dMax_);

	/** Initialize the extremities of the segment.
	*/
	void initInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;

	/** Update extremities of the segment.
	*/
	void updateInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;

	jblas::vec const& computeInnovation(jblas::vec const& z_,
					jblas::vec const& zPred_);

	/// Predict the extremes of the observed line, with their covariance in the image plane
	///@param pose the robot pose
	///@param poseCov the robot pose covariance matrix
	///@param ext on of the extremities in 3D space
	///@param extCov the covariances matrix of the extreme in 3D space
	///@param zPredExt the predicted extreme in the 2D image
	///@param zPredExtCov the covariances matrix of the predicted extreme
	void predictExtObs(jblas::vec_range const& pose, jblas::sym_mat_range const& poseCov,
			jblas::vec3 const& ext, jblas::sym_mat const& extCov,
			jblas::vec& zPredExt, jblas::sym_mat& zPredExtCov);
    /** Predict the extreme and Jacobians
     * @param pose the robot pose
     * @param Ext one of the extremities in 3D space
     * @param eInImage the predicted extreme in the 2D image
     * @param JeR and  @param JeExt Jacobians of the prediction with respect to pose and extremity*/
    void predictExtObsJac(jblas::vec_range const& pose, jblas::vec3 const Ext, jblas::vec& eInImage,  jblas::mat& JeR, jblas::mat& JeExt);
	/// Compute two lines from the optical center through both segment extremities
	///@param ext1 the first extreme of the detected segment
	///@param ext2 the second extreme
	static jblas::vec2 extToLine(jblas::vec const& ext1,jblas::vec const& ext2);
	
	/// The Jacobians of \a extToLine() wrt the extremes
	///@param ext1 the first extreme of the detected segment
	///@param ext2 the second extreme
	///@param J The returned Jacobian
	static void extToLineJac(jblas::vec const& ext1,jblas::vec const& ext2, jblas::mat& J);

	friend std::string boSegFeatureInitState(InitFeature const& f_, BearingOnlySlam& slam_);

	}; // class ImageEuclideanPluckerFeatureObserveModel
	
	
			/** Sizeless 3D segment using the homogenous coordinates parametrisation
	* observed in a perspective image.
	* 
	* \ingroup slam
		*/
		
		
	/// Class ImagePluckerFeatureObserveModel
		
	class ImagePluckerFeatureObserveModel : public FeatureObserveModel {
			
		protected:
			
			SegmentFeatureModel& segmentFeatureModel;
			
			// Perspective camera
			jafar::camera::CameraPinhole camera;
			// Plucker camera matrix
			jblas::mat33 Pl;
			// Plucker camera inverse matrix
			jblas::mat33 Plinv;
			
			/// initial plane base vector
			jblas::vec2 beta;
			/// covariances matrix of initial plane base vector
			jblas::mat22 covar_beta;
			
			/// Jacobian of b wrt beta
			jblas::mat JbBeta;
			
		
			/** homogeneous to Image 2D line representation conversion
			*/
			virtual jblas::vec const& homogeneousToImageModel(jblas::vec3 const& l_) = 0;
			
			/** homogeneous to Image 2D line representation -- Jacobians
			*/
			virtual void homogeneousToImageModelJac(jblas::vec3 const& l_, 
					jblas::mat& Jl) const = 0;
			
			
			/** Image representation to homogeneous transform
			*@param rt_ input 2D line representation -- e.g. rt = (rho,theta)'
			*@return The homogeneous line in P3 -- l = (a,b,c)'
			*/
			virtual jblas::vec3 imageModelToHomogeneous(jblas::vec const& rt_) = 0;
			
			/** Rho-theta to homogeneous transform -- Jacobian
				*@param rt_ input  2D line representation
				*@param HMrt output transformation Jacobian
				*/
			virtual void  imageModelToHomogeneousJac(jblas::vec const& rt_, jblas::mat& HMrt) = 0;
			
			/** Rho-theta to homogeneous transform -- Jacobian
				*@param rt_ input  2D line representation
				*@param l   output homogeneous line representation
				*@param HMrt output transformation Jacobian
				*/
			virtual void  imageModelToHomogeneousJac(jblas::vec const& rt_, jblas::vec3& l, jblas::mat& HMrt) = 0;
			
			
			/// Plane sub-vector a to plane base vector beta
			jblas::vec3 planeBaseToDirVector(jblas::vec3 const& an_, jblas::vec2 const& beta_);
			
			/// Plane sub-vector a to plane base vector -- Jacobians
			void planeBaseToDirVectorJac(jblas::vec_range const& an_, jblas::vec2 const&  beta_, jblas::vec_range& b, jblas::mat33& B_a);
			
			
			boost::tuple<double,double> computeExtremitiesAbscissa(SegmentFeature const& segFeature_, 
					geom::T3DEuler const& robotPose_,
					SegmentObservation const& obs_) const;
			
			void segmentExtremitiesPluckerLines(SegmentObservation const& segObs_, 
							jblas::vec& lineExt1Sensor, 
							jblas::vec& lineExt2Sensor) const;
			
			static boost::tuple<double, double, double> pluckerLinesDistance(jblas::vec const& line1_, 
							jblas::vec const& line2_);
			
			/// covariance on pixel coordinate (u,v)
			jblas::sym_mat p_pixCov;


			
		public:
				
			ImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_);
			~ImagePluckerFeatureObserveModel();
			
			Observation::ObservationType typeObs() const {return Observation::SEGMENT_IMAGE;}
			
			void setPixCov(jblas::vec const& pixCov_, double stabilizingFactor = 1.0);
			
						
			/** Init extremities of the segment.
			*/
			void initInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;
			
			/** Update extremities of the segment.
			*/
			void updateInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;
			
			/** Predict the extremes of the observed line, with their covariance in the image plane
			 * @param pose the robot pose
			 * @param poseCov the robot pose covariance matrix
			 * @param ext one of the extremities in 3D space
			 * @param extCov the covariances matrix of the extreme in 3D space
			 * @param zPredExt the predicted extreme in the 2D image
			 * @param zPredExtCov the covariances matrix of the predicted extreme
			 *  CAUTION: THIS IMPLEMENTATION IGNORES THE CROSS-VARIANCES ROBOT/FEATURE
			 *  AND LEADS TO TOO BIG ELLIPSES IN THE IMAGE.
			 */ 
			void predictExtObs(jblas::vec_range const& pose, jblas::sym_mat_range const& poseCov,
					jblas::vec3 const& ext, jblas::sym_mat const& extCov,
					jblas::vec& zPredExt, jblas::sym_mat& zPredExtCov);
			/** Predict the extreme and Jacobians
 			 * @param pose the robot pose
 			 * @param Ext one of the extremities in 3D space
 			 * @param eInImage the predicted extreme in the 2D image
             * @param JeR and  @param JeExt Jacobians of the prediction with respect to pose and extremity*/
            void predictExtObsJac(jblas::vec_range const& pose, jblas::vec3 const Ext, jblas::vec& eInImage,  jblas::mat& JeR, jblas::mat& JeExt);

			
			/// Assign covariances of the unobserved part
			///@param P covariance of the landmark to initialize in global frame
			///@param Jx Jacobian of the feature in global frame wrt the feat. in robot frame
			void postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jx);
			
			/// Set up initial parameters
			///@param beta0 The first component of the lines director vector for initialization
			///@param beta1 the second component
			///@param sigma_beta The standard deviation of \a beta0 and \a beta1
			void setup(double beta0 = 0, double beta1 = 0, double sigma_beta = 0.5);
			
			
	}; // class ImagePluckerFeatureObserveModel


	
	
        /**Observation Model using Rho Theta parametrisation.
        *
        * \ingroup slam
        */
        class RhoThetaImagePluckerFeatureObserveModel :
        public ImagePluckerFeatureObserveModel {
    
        protected:

        /// Project line into camera and convert to 2D representation
        jblas::vec const& predictObservationInSensorFrame( jblas::vec const& feature_);

        /// Project line into camera and convert to 2D representation -- Jacobians
        void predictObservationInSensorFrameJac( jblas::vec const& feature_);

			
        /** homogeneous to rho-theta conversion
        */
        virtual jblas::vec const& homogeneousToImageModel(jblas::vec3 const& l_);
        
        /** homogeneous to rho-theta conversion -- Jacobians
        */
        virtual void homogeneousToImageModelJac(jblas::vec3 const& l_, 
                jblas::mat& Jl) const;
        
        
        /** Rho-theta to homogeneous transform
        *@param rt_ input rho-theta 2D line representation -- rt = (rho,theta)'
        *@return The homogeneous line in P3 -- l = (a,b,c)'
        */
        virtual jblas::vec3 imageModelToHomogeneous(jblas::vec const& rt_);
        
        /** Rho-theta to homogeneous transform -- Jacobian
			*@param rt_ input rho-theta 2D line representation
			*@param HMrt output transformation Jacobian
			*/
        virtual void  imageModelToHomogeneousJac(jblas::vec const& rt_, jblas::vec3& l, jblas::mat& HMrt);
		
        /** Rho-theta to homogeneous transform -- Jacobian
			*@param rt_ input rho-theta 2D line representation
			*@param l   output homogeneous line representation
			*@param HMrt output transformation Jacobian
			*/
        virtual void  imageModelToHomogeneousJac(jblas::vec const& rt_, jblas::mat& HMrt);
		
//        /** Retro-project Plucker line
//        */
//        virtual jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
//
//        /** Retro-project Plucker line -- output and Jacobian
//        */
//        virtual void inverseObservationInSensorFrameJac(jblas::vec const& z_);

        /** Retro-project Plucker line
        */
        virtual jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

        /** Retro-project Plucker line -- output and Jacobian
        */
        virtual void inverseObservationInSensorFrameJac(Observation const& obs_);
    
        public:
    
        RhoThetaImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_);
        ~RhoThetaImagePluckerFeatureObserveModel();
    
        /// Innovation
        virtual jblas::vec const& computeInnovation(jblas::vec const& z_, jblas::vec const& zPred_);

        /// Compute two lines from the optical center through both segment extremities
        ///@param ext1 the first extreme of the detected segment
        ///@param ext2 the second extreme
        static jblas::vec2 extToImageModel(jblas::vec const& ext1,jblas::vec const& ext2);

        /// The Jacobians of \a extToLine() wrt the extremes
        ///@param ext1 the first extreme of the detected segment
        ///@param ext2 the second extreme
        ///@param J The returned Jacobian
        static void extToImageModelJac(jblas::vec const& ext1, jblas::vec const& ext2, jblas::mat& J);

		
		/// Build covariances matrix of observation
		///@param obs observation as a couple of endpoints
		virtual void computeSensorR(Observation const& obs);
		

        }; // class RhoThetaImagePluckerFeatureObserveModel

		
		
        /**Observation Model using Scaled Homogeneous parametrisation.
			*
			* \ingroup slam
			*/
        class ScaledHomogeneousImagePluckerFeatureObserveModel :
			public ImagePluckerFeatureObserveModel {
				
protected:
				
				/// Project line into camera and convert to Scaled Homogeneous representation
				jblas::vec const& predictObservationInSensorFrame( jblas::vec const& feature_);
				
				/// Project line into camera and convert to Scaled Homogeneous representation -- Jacobians
				void predictObservationInSensorFrameJac( jblas::vec const& feature_);
				
				
				/** homogeneous to Scaled Homogeneous conversion
					*/
				virtual jblas::vec const& homogeneousToImageModel(jblas::vec3 const& l_);
				
				/** homogeneous to Scaled Homogeneous conversion -- Jacobians
					*/
				virtual void homogeneousToImageModelJac(jblas::vec3 const& l_, 
														jblas::mat& Jl) const;
				
				
				/** Scaled Homogeneous to homogeneous transform
					*@param sh_ input Scaled Homogeneous 2D line representation -- rt = (rho,theta)'
					*@return The homogeneous line in P3 -- l = (a,b,c)'
					*/
				virtual jblas::vec3 imageModelToHomogeneous(jblas::vec const& sh_);
				
				
				/** Scaled Homogeneous to homogeneous transform -- Jacobian
					*@param sh_ input Scaled Homogeneous 2D line representation
					*@param HM_sh output transformation Jacobian
					*/
				virtual void  imageModelToHomogeneousJac(jblas::vec const& sh_, 
														 jblas::vec3& l, 
														 jblas::mat& HM_sh);
				
				
				/** Scaled Homogeneous to homogeneous transform -- Jacobian
					*@param sh_ input Scaled Homogeneous 2D line representation
					*@param l   output homogeneous line representation
					*@param HM_sh output transformation Jacobian
					*/
				virtual void  imageModelToHomogeneousJac(jblas::vec const& sh_,
														 jblas::mat& HM_sh);
				
				
//				/** Retro-project Plucker line
//					*/
//				virtual jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
//				
//				
//				/** Retro-project Plucker line -- output and Jacobian
//					*/
//				virtual void inverseObservationInSensorFrameJac(jblas::vec const& z_);

                /** Retro-project Plucker line
                    */
                virtual jblas::vec inverseObservationInSensorFrame(Observation const& obs_);


                /** Retro-project Plucker line -- output and Jacobian
                    */
                virtual void inverseObservationInSensorFrameJac(Observation const& obs_);
				
public:
					
				/// Constructor
				ScaledHomogeneousImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_);
				
				/// Destructor
				~ScaledHomogeneousImagePluckerFeatureObserveModel();
				
				/// Innovation
				virtual jblas::vec const& computeInnovation(jblas::vec const& z_, 
															jblas::vec const& zPred_);
				
				/// Build covariances matrix of observation
				///@param obs observation as a couple of endpoints
				virtual void computeSensorR(Observation const& obs);
				

				/// Compute two lines from the optical center through both segment extremities
				///@param ext1 the first extreme of the detected segment
				///@param ext2 the second extreme
				static jblas::vec3 extToImageModel(jblas::vec const& ext1,
												   jblas::vec const& ext2);
				
				
				/// Compute two lines from the optical center through both segment extremities
				///@param ext1 the first extreme of the detected segment
				///@param ext2 the second extreme
				///@param ref the reference homogeneous line to which the new line's norm is matched
				static jblas::vec3 extToImageModel(jblas::vec const& ext1,
												   jblas::vec const& ext2,
												   jblas::vec3 const& ref);
				

				/// The Jacobian of \a extToLine() wrt the extremes
				///@param ext1 the first extreme of the detected segment
				///@param ext2 the second extreme
				///@param H_e The returned Jacobian
				static void extToImageModelJac(jblas::vec const& ext1,
											   jblas::vec const& ext2, 
											   jblas::mat& H_e);
				
				/// The Jacobian of \a extToLine() wrt the extremes
				///@param ext1 the first extreme of the detected segment
				///@param ext2 the second extreme
				///@param ref the reference homogeneous line to which the new line's norm is matched
				///@param HM_e The returned Jacobian
				static void extToImageModelJac(jblas::vec const& ext1,
											   jblas::vec const& ext2,
											   jblas::vec3 const& ref,
											   jblas::mat& HM_e);
				
				
				
			}; // class HomogeneousImagePluckerFeatureObserveModel
		

        /**Observation Model using a Stereo head, two uvd points for init, two rho-thetas for updates.
			*
			* \ingroup slam
			*/
		class StereoImagePluckerFeatureObserveModel : public FeatureObserveModel {

    protected:

            SegmentFeatureModel& segmentFeatureModel;
            // stereo bench 
            jafar::camera::StereoBench stereoBench;
            // Plucker camera matrix
            jblas::mat33 Pl;
            // Plucker camera inverse matrix
            jblas::mat33 Plinv;

            /// Plane sub-vector a to plane base vector beta
            jblas::vec3 planeBaseToDirVector(jblas::vec3 const& an_, jblas::vec2 const& beta_);
            
            /// Plane sub-vector a to plane base vector -- Jacobians
            void planeBaseToDirVectorJac(jblas::vec_range const& an_, jblas::vec2 const&  beta_, jblas::vec_range& b, jblas::mat33& B_a);
            
            
            boost::tuple<double,double> computeExtremitiesAbscissa(SegmentFeature const& segFeature_, 
                    geom::T3DEuler const& robotPose_,
                    StereoSegmentObservation const& obs_) const;
            
            void segmentExtremitiesPluckerLines(StereoSegmentObservation const& segObs_, 
                            jblas::vec& lineExt1Sensor, 
                            jblas::vec& lineExt2Sensor) const;
            
            static boost::tuple<double, double, double> pluckerLinesDistance(jblas::vec const& line1_, 
                            jblas::vec const& line2_);
            
            /// covariance on pixel coordinate (u,v)
            jblas::sym_mat p_pixCov;
			
            /// covariance on pixel disparity
            double p_dispVar;
			
            jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

            void predictObservationInSensorFrameJac(const jblas::vec& feature_);

//             jblas::vec inverseObservationInSensorFrame(const jblas::vec& z_);
// 
//              void inverseObservationInSensorFrameJac(const jblas::vec& z_);
			
			void computeSensorRInit(Observation const& obs);

            jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

             void inverseObservationInSensorFrameJac(Observation const& obs_);

    public: 
            StereoImagePluckerFeatureObserveModel(SegmentFeatureModel& featureModel_, jafar::camera::StereoBench const& stereoBench_);

            ~StereoImagePluckerFeatureObserveModel();

			/// Get observation type
            Observation::ObservationType typeObs() const {return Observation::SEGMENT_STEREOIMAGE;}

			/// Set pixel covariances matrix
            void setPixCov(jblas::vec const& pixCov_, double stabilizingFactor = 1.0);
			
			/// Set pixel disparity variance
			void setDispVar(double dispVar, double stabilizingFactor = 1.0);

            
                        
            /** Init extremities of the segment.
            */
            void initInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;
            
            /** Update extremities of the segment.
            */
            void updateInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;
            
            /** Predict the extremes of the observed line, with their covariance in the image plane
             * @param pose the robot pose
             * @param poseCov the robot pose covariance matrix
             * @param ext one of the extremities in 3D space
             * @param extCov the covariances matrix of the extreme in 3D space
             * @param zPredExt the predicted extreme in the 2D image
             * @param zPredExtCov the covariances matrix of the predicted extreme
             *  CAUTION: THIS IMPLEMENTATION IGNORES THE CROSS-VARIANCES ROBOT/FEATURE
             *  AND LEADS TO TOO BIG ELLIPSES IN THE IMAGE.
             */ 
            void predictExtObs(jblas::vec_range const& pose, jblas::sym_mat_range const& poseCov,
                    jblas::vec3 const& ext, jblas::sym_mat const& extCov,
                    jblas::vec& zPredExt, jblas::sym_mat& zPredExtCov);
            /** Predict the extreme and Jacobians
             * @param pose the robot pose
             * @param Ext one of the extremities in 3D space
             * @param eInImage the predicted extreme in the 2D image
             * @param JeR and  @param JeExt Jacobians of the prediction with respect to pose and extremity*/
            void predictExtObsJac(jblas::vec_range const& pose, jblas::vec3 const Ext, jblas::vec& eInImage,  jblas::mat& JeR, jblas::mat& JeExt);

            /// Innovation
            jblas::vec const& computeInnovation(jblas::vec const& z_, 
                                                            jblas::vec const& zPred_);
                
            /// Build covariances matrix of observation
            ///@param obs observation as a couple of stereo endpoints
			/// This function computes the covariances matrix of the measurement in the space defined for the Kalman updates, which is a couple of rho-theta lines, the stereo-rho-theta line.
            void computeSensorR(Observation const& obs);
            
            /// Endpoints to Rho-Theta transform
            ///@param ext1 the first extremity of the detected segment
            ///@param ext2 the second extremity
            jblas::vec2  extToRhoTheta(jblas::vec const& ext1,jblas::vec const& ext2);
			
			
            /// Endpoints to Rho-Theta transform
            ///@param ext1 the first extremity of the detected segment
            ///@param ext2 the second extremity
			///@param J the Jacobian wrt (ext,ext2)
            void extToRhoThetaJac(jblas::vec const& ext1,jblas::vec const& ext2, jblas::mat& J);


    }; // class StereoImagePointFeatureObserveModel
	
		
} // namespace slam
} // namespace jafar


#endif // SLAM_SEGMENT_FEATURE_HPP
