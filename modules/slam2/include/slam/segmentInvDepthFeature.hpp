    /* $Id: segInvDepthFeature.hpp 3374 2008-12-01 18:17:38Z tvidal $ */

#ifndef SLAM_SEGMENT_INV_DEPTH_FEATURE_HPP
#define SLAM_SEGMENT_INV_DEPTH_FEATURE_HPP

#include "jmath/jblas.hpp"

#include "camera/cameraBarreto.hpp"

#include "slam/feature.hpp"
#include "slam/featureModel.hpp"
#include "slam/segmentFeature.hpp"

    namespace jafar {
  namespace slam {
    
    class SegInvDepthFeatureModel;

     /** A segment inverse depth feature, with its extremities.
      * The extremities are not represented in a stochastic way, 
      * they are expressed as the inverse-depth endpoint
      * (note that are not neccesarly the supporting points of the line.
     * \ingroup slam
         */
     
    class SegmentIDFeature : public BaseFeature {

      public:

        std::string debugObs;

	/// coordinate of the first extremity
        double s1;
	/// coordinate of the second extremity
        double s2;

	/// flag to enlarge extremites 
        bool ENLARGE;

        SegmentIDFeature(unsigned int id, FeatureModel& model, std::size_t sizeObs, Observation::ObservationType typeObs_);
        virtual ~SegmentIDFeature();

        SegInvDepthFeatureModel* segidmodel;

        void setFlagEnlarge() {ENLARGE=true;}

        virtual void setState(jblas::vec& x_, jblas::sym_mat& P_);

        jblas::vec3 getExt1() const;
        jblas::sym_mat getExt1Cov() const;
        jblas::vec3 getExt2() const;
        jblas::sym_mat getExt2Cov() const;
        jblas::mat getExt1Jac() const;
        jblas::mat getExt2Jac() const;

      protected:

        virtual void writeLogHeader(jafar::kernel::DataLogger& log) const;
        virtual void writeLogData(jafar::kernel::DataLogger& log) const;

      private:

	  /// Get 3D endpoint given abscissa.
        void computeExt(double s_, jblas::vec3& ext) const;

          /// The Jacobian of computeExt() wrt the Plucker line.
        void computeExtJac(double s_, jblas::mat& J) const;

	  /// Get 3D endpoint covariance given abscissa.
        void computeExtCov(double s_, jblas::sym_mat& extCov) const;

    }; // SegmentIDFeature


    std::ostream& operator <<(std::ostream& s, const jafar::slam::SegmentIDFeature& f);

    /** 3D segments using 2 points inverse-depth parametrisation.
     * The first three parameters are the position of the camera.
     * The next three parameters is the direction of the point from
     * the camera and last parameter is the inverse of the distance.
     * This last 4 parameter for each point (In total 11).
     * \ingroup slam
     */
    class SegInvDepthFeatureModel : public FeatureModel {
      public:

        jblas::mat J3dPoints;

        SegInvDepthFeatureModel();
        ~SegInvDepthFeatureModel();


        void toFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);

        void toFrameJac(const jblas::vec& frame_, const jblas::vec& x_);

        void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);
        void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes);
      
        void fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_);

        void compute3dPoints(jblas::vec const& x, jblas::vec& p, jblas::vec& q);
        void compute3dPointsJac(jblas::vec const& x);
        void compute3dPointsJac(jblas::vec const& x, jblas::vec& p, jblas::vec& q, jblas::mat& Pt_id);
        //void compute3dLine(jblas::vec const& x, jblas::vec& pt1, jblas::vec& pt2);
        //void compute3dLineJac(jblas::vec const& x, jblas::vec& pt1, jblas::vec& pt2);
        void computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_ , jblas::mat& jac );

    }; // class SegInvDepthFeatureModel


 /** Sizeless 3D point using the inverse-depth parametrisation
     * observed in a perspective image.
     * 
     * \ingroup slam
  */
    class ImageSegInvDepthFeatureObserveModel : public FeatureObserveModel {

      protected:

      /// initial inverse depth
        double m_id0;
      /// standard deviation on initial inverse depth
        double m_sigmaId0;

        jafar::camera::CameraPinhole camera;

        SegInvDepthFeatureModel& segInvDepthFeatureModel;

        jblas::vec const& predictObservationInSensorFrame(const jblas::vec& feature_);

        void predictObservationInSensorFrameJac(const jblas::vec& feature_);

//       jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
        // 
//       void inverseObservationInSensorFrameJac(jblas::vec const& z_);

        jblas::vec inverseObservationInSensorFrame(Observation const& obs_);

        void inverseObservationInSensorFrameJac(Observation const& obs_);
        
        /// covariance on pixel coordinate (u,v)
        jblas::sym_mat p_pixCov;

      public:

        ImageSegInvDepthFeatureObserveModel(SegInvDepthFeatureModel& featureModel_, jafar::camera::CameraPinhole const& camera_);

        ~ImageSegInvDepthFeatureObserveModel();

        Observation::ObservationType typeObs() const {return Observation::SEGMENTID_IMAGE;}
			
        void setPixCov(jblas::vec const& pixCov_, double stabilizingFactor = 1.0);

      //void initStateInSensorFrame(InitFeature& feature_,
	//			  Observation const& obs_,
        //                          double dMin_, double dMax_);
        
        /** Init extremities of the segment.
         */
        void initInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;
			
	/** Update extremities of the segment.
        */
        void updateInternalState(BaseFeature& feature, jafar::geom::T3DEuler const& robotPose_, Observation const& obs_) const;
        
        void postInitCovariance(jblas::sym_mat_range & P, jblas::mat const& Jfeature);
        
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

        void setup(double id0 = 0.5, double sigmaId0 = 0.5);


    }; // class ImageSegInvDepthFeatureObserveModel

  }}

#endif // SLAM_SEG_INV_DEPTH_FEATURE_HPP
