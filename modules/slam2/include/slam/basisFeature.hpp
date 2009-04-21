#ifndef _SLAM_BASIS_FEATURES_HPP_
#define _SLAM_BASIS_FEATURES_HPP_

#include "slam/featureModel.hpp"

namespace jafar {
  namespace slam {
    /**
     * A basis features includes the X,Y,Z coordinates of the origin, plus the raw, yaw and pitch
     * angles.
     * \ingroup slam
     */
    class BasisFeatureModel : public FeatureModel {
      public:
        BasisFeatureModel();
        void toFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);
  
        void toFrameJac(const jblas::vec& frame_, const jblas::vec& x_);
  
        void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes);
        void fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes);
        
        void fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_);
        void computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac  );
    };
  
  
    class BasisFeatureObserveModel : public FeatureObserveModel {
      public:
        BasisFeatureObserveModel(BasisFeatureModel& featureModel_);
        ~BasisFeatureObserveModel();
  
        Observation::ObservationType typeObs() const {return Observation::BASIS;}
      protected:
        jblas::vec const& predictObservationInSensorFrame(jblas::vec const& feature_);
        void predictObservationInSensorFrameJac(jblas::vec const& feature_);
  
//         jblas::vec inverseObservationInSensorFrame(jblas::vec const& z_);
//         void inverseObservationInSensorFrameJac(jblas::vec const& z_);

        jblas::vec inverseObservationInSensorFrame(Observation const& obs_);
        void inverseObservationInSensorFrameJac(Observation const& obs_);

    }; // class BasisFeatureObserveModel

  }
}

#endif
