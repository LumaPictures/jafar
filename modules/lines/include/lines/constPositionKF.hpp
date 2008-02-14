# ifndef CONST_POSITION_KF_H
# define CONST_POSITION_KF_H

#include <iostream>
namespace jafar{
  namespace lines{
    //! Kalman filter for an one dimensional constant position model 
    /**
    This class provides a Kalman filter for an one dimensional constant position model. It's a simple one dimensional Kalman filter.
    @ingroup lines
     */
    class ConstPositionKF{
    public:
      ConstPositionKF();
      
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
      Provides a state prediction with its variance.
       */
      void getPred(double& pos, double& c11){
        pos = x1Pred;
        c11 = cov11Pred;
      }
      
      /**
      Provides a state prediction.
       */
      double getPosPred(){return x1Pred;}
      
      /**
      Provides the variance of the state prediction.
       */
      double getPosVar(){return cov11;}
      
    private:
      double x1;    //!> position
    
      double cov11; //!> variance position
    
    
      double x1Pred;    //!> position prediction
      
      double cov11Pred; //!> variance of predicted position
    
      
    };
  } // namespace lines
} // namespace jafar
# endif
