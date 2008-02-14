# ifndef CONST_VELOCITY_KF_H
# define CONST_VELOCITY_KF_H

#include <iostream>

namespace jafar{
  namespace lines{
    
    //! Kalman filter for an one dimensional constant velocity model 
    /**
    This class provides a Kalman filter for an one dimensional constant velocity model. Assumes constante time intervals.
    @ingroup lines
    */
    class ConstVelocityKF{
    public:
      ConstVelocityKF();
      
      /**
      Initialices the Kalman filter with initPos and initVar.
      */
      void initKF(double initPos, double initVar);
      
      /**
      Updates the Kalman Filter and updates the prediction for the next step (see getPred())
      @param y is the new measurement for the position
      @param sigmaY is the variance of the measurement
      @param sigmaV is the process noise and weights the old measurements
      */
      void updateKF(double y, double sigmaY, double sigmaV);
      
      /**
      Provides a state prediction with its covariance.
      */
      void getPred(double& pos, double& c11, double& c12, double& c22){
        pos = x1Pred;
        c11 = cov11Pred;
        c12 = cov12Pred;
        c22 = cov22Pred;
      }
      
      /**
      Provides a state prediction.
       */
      double getPosPred(){return x1Pred;}
      /**
      Provides the variance of the state prediction.
       */
      double getPosVar(){return cov11;}
      
      /**
      The posPeriod is an opportunity to bound the state values. They state space is assumed to be priodic (as it is the case for angles)
      */
      void setPosPeriod(double min, double max){
        minPeriod = min;
        maxPeriod = max;
        periodicFlag = 1;
      }
      
    private:
      double x1;    //!> position
      double x2;    //!> velocity
    
      double cov11; //!> variance position
      double cov12; //!> correlation
      double cov22; //!> variance velocity
    
      double x1Pred;    //!> position prediction
      double x2Pred;    //!> velocity prediction
      
      double cov11Pred; //!> variance of prediction of position
      double cov12Pred; //!> correlation of prediction 
      double cov22Pred; //!> variance of prediction of velocity 
      
      double minPeriod;   //!> Lower bound for state (priodic!)
      double maxPeriod;   //!> Upper bound for state (priodic!)
      bool periodicFlag;  //!> Indicates if the state space is priodic
    };
  } // namespace lines
} // namespace jafar
# endif
