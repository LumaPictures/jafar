# ifndef LS_PREDICTOR_2D 
# define LS_PREDICTOR_2D

# include <cmath>
# include <iostream> 
#include "lines/lsMisc.hpp"
#include "lines/constVelocityKF.hpp"
#include "lines/constPositionKF.hpp"

namespace jafar{
  namespace lines{
    
    //! Enumeration for the kind of motion modell for the prediction
    /** 
    This enumeration indicates the type of the modell for the motion prediction. All modells use Kalman filtering for prediction.
    
    MIDPOINT_X_Y  Constant velocity modell, Prediction of the midpoint and the orientation of the line. Fails if the length of the line is not extracted stable.
    
    POLAR_COORD  Constant velocity modell. Prediction with polar coordinates. Unfortunatly the uncertainty of polar coordinates depends on the position of the line in the image and thus it behaves sometimes strange. 
    
    ONLY_PERP_MOTION Constant position modell applyed on the projected midpoint difference and difference of orientation of consecutive measurements. For prediction these differences are  added to the last prior position of the line. Works well if there are no false matchings. False matchings destroy the performance.
    
    PERP_MIDPOINT_PROJ Constant velocity modell. The line position is represented with midpoint coordinates and orientation. These values are estimated with independent Kalman filters. The prediction of the line segment is the projection of the prior endpoints on the predicted line. This modell is recommended.
    
    @ingroup lines
    */
    enum PredictionModel{
      MIDPOINT_X_Y,
      POLAR_COORD,
      ONLY_PERP_MOTION,
      PERP_MIDPOINT_PROJ
    };
      
    
    //! Class for prediction of the movement of a line in a sequenze of images
    /**
    This Class provides containers and functions to predict the new position of a line in an image after it was tracked over prior images.
    @ingroup lines
    */
    class LsPredictor2D{
    public:
      LsPredictor2D();
      
      //! Initializes the Kalman filters for prediction
      /**
      Initializes the Kalman filters for prediction. The recommended prediction model is PERP_MIDPOINT_PROJ. 
      Depending on the prediction model the parameters that are really necessary changes.
      
      MIDPOINT_X_Y
      Requires  mx, my, phi
      
      POLAR_COORD,
      Requires rho, theta
      
      ONLY_PERP_MOTION,
      Requires mx my phi
      
      PERP_MIDPOINT_PROJ
      Requires mx, my, phi
      */
      void initPredictor(double mx, double my, double phi, double rho, double theta, PredictionModel model=PERP_MIDPOINT_PROJ);
      
      //! Updates the predition model
      /**
      Updates the Kalman filters for prediction. The variances for measurements and process noise are hard encoded. 
      Depending on the prediction model the parameters that are really necessary changes.
      
      MIDPOINT_X_Y
      Requires  mxNew, myNew, phiNew
      
      POLAR_COORD
      Requires rhoNew, thetaNew
      
      ONLY_PERP_MOTION
      Requires mxOld myOld  phiOld mxNew myNew phiNew
      
      PERP_MIDPOINT_PROJ
      Requires mxNew, myNew, phiNew
      */
      /*
      TODO Change hard encoding of variances
      */
      void updatePredictor(double mxOld, double myOld, double phiOld, double mxNew, double myNew, double phiNew, double rhoNew, double thetaNew);
    
      //! Get the prediction for new endpoints
      /**
      This function provides the prediction of a line in endpoint representation. The motion prediction is only reliable if the line was tracked over prior states and if the mortion did not change to much. A constant velocity model is used. 
      */
      double getPredictionEP(double x1Old, double y1Old, double x2Old, double y2Old, double& x1Pred, double& y1Pred, double& x2Pred, double& y2Pred, double l=0);
      
      //! Get the benchmark value
      /**
      This function provides a value that indicates if we can trust in the prediction modell. That test is not well realised for all prediction models. Should be improved.
      */
      /*
      TODO Improve the benchmark
      */
      double benchmarkTest(){return benchmark;} 
      
    private:
      PredictionModel predMod; //!< Type of motion model
      uint nUpdates;      //!< Number of updates
      double cDist;       //!< Distance of last prediction to measurement
      
      double benchmark;     //!< Indicator for the reliability of the prediction, small value is good

      ConstVelocityKF xKF;    //!<Constant velocity modell for x coordinate of midpoint
      ConstVelocityKF yKF;    //!<Constant velocity modell for y coordinate of midpoint
      ConstVelocityKF phiKF;  //!<Constant velocity modell for direction of line
      
      ConstVelocityKF rhoKF;    //!<Constant velocity modell for polar coordinate rho
      ConstVelocityKF thetaKF;  //!<Constant velocity modell for polar coordinate theta
      
      ConstPositionKF dKF;      //!<Constant position modell for perpendicular difference of lines
      ConstPositionKF alphaKF;  //!<Constant velocity modell for angular difference
      
      ConstVelocityKF xPKF;     //!<Constant velocity modell for x coordinate of midpoint, updated with perpendicular motion
      ConstVelocityKF yPKF;     //!<Constant velocity modell for y coordinate of midpoint, updated with perpendicular motion
      ConstVelocityKF phiPKF;   //!<Constant velocity modell for direction of line for perpendicular update
      
    };
  } // namespace lines
} // namespace jafar

# endif
