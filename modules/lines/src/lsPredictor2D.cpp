#include "lines/lsPredictor2D.hpp"

using namespace std;
using namespace jafar;
using namespace lines;

LsPredictor2D::LsPredictor2D(){
  nUpdates = 0;
  cDist = 0;
  benchmark = 100;
}

void LsPredictor2D::initPredictor(double mx, double my, double phi, double rho, double theta, PredictionModel model){
  nUpdates = 0;
  predMod = model;
  
  switch(model){
    case MIDPOINT_X_Y:
      xKF.initKF(mx, 100);
      yKF.initKF(my, 100);
      phiKF.initKF(phi, 100); 
      break;
    case POLAR_COORD:
      rhoKF.initKF(rho, 100);
      thetaKF.initKF(theta, 100);
      break;
    case ONLY_PERP_MOTION:
      dKF.initKF(0, 100);
      alphaKF.initKF(0, 100);
      break;
    case PERP_MIDPOINT_PROJ:
      xPKF.initKF(mx, 100);
      yPKF.initKF(my, 100);
      phiPKF.initKF(phi, 100);  
      phiPKF.setPosPeriod(-M_PI/2, M_PI/2 );
      break;
  }
  
  benchmark = 100;
  cDist = 100;
}


void LsPredictor2D::updatePredictor(double mxOld, double myOld, double phiOld, double mxNew, double myNew, double phiNew, double rhoNew, double thetaNew){
  
  double alpha, d, cosP, sinP, xPNew, yPNew;
  CvPoint a,b,c;
  
  switch(predMod){
    case MIDPOINT_X_Y:
      ///////// update midpoint ////////////////
      
      xKF.updateKF(mxNew, 1, 10);
      yKF.updateKF(myNew, 1, 10);
      phiKF.updateKF(phiNew, 0.05, 0.3);
      break;
      
    case POLAR_COORD:
      ///////// update polar coordinates /////////////
      
      rhoKF.updateKF(rhoNew, 1, 4);
      thetaKF.updateKF(thetaNew, 0.01, 0.1);
      break;
      
    case ONLY_PERP_MOTION:
      ///////// udate perpendicular midpoint motion ////////////
      
      alpha = phiNew-phiOld;
      if(alpha<-M_PI) alpha += 2*M_PI;
      if(alpha>M_PI) alpha -= 2*M_PI;

      // perpendicular motion
      a.x = (int) mxNew;
      a.y = (int) myNew;
      b.x = (int) (mxNew+sin(phiNew));
      b.y = (int) (myNew+cos(phiNew));
      c.x = (int) mxOld;
      c.y = (int) myOld;
      d = distPointVector(a,b,c) / cos(alpha);
    
      dKF.updateKF(d, 1, 3);
      alphaKF.updateKF(alpha, 0.05, 0.1);
      break;
      
    case PERP_MIDPOINT_PROJ:
      ////////// update PKF (perp. motion with global coordinates ///////////
      
      cosP = cos(phiNew);
      sinP = sin(phiNew);
      
      // project predicted midpoint on the new line, that's the new measurement
      d = (xPKF.getPosPred()-mxNew)*cosP + (yPKF.getPosPred()-myNew)*sinP;
      xPNew = mxNew + d*cosP;
      yPNew = myNew + d*sinP;
      
      // correction distance is projection distance
      a.x = (int) mxNew;
      a.y = (int) myNew;
      b.x = (int) (mxNew + 10000*sinP); // to reduce rounding error
      b.y = (int) (myNew + 10000*cosP); // to reduce rounding error
      c.x = (int) xPNew;
      c.y = (int) yPNew;
      cDist = distPointVector(a,b,c);
      
      // benchmark equals cDist
      benchmark = cDist;
      
      xPKF.updateKF(xPNew, 1, 5);
      yPKF.updateKF(yPNew, 1, 5);
      phiPKF.updateKF(phiNew, 0.01, 0.1);
      break;
  }
  
  
  nUpdates++;
  
}

double LsPredictor2D::getPredictionEP(double x1Old, double y1Old, double x2Old, double y2Old, double& x1Pred, double& y1Pred, double& x2Pred, double& y2Pred, double l){
  

  if(predMod == MIDPOINT_X_Y){
    ///////////////////// midpoint motion model /////////////////////

    double dxT, dyT, length;
    if(l==0){
      dxT = x2Old-x1Old;
      dyT = y2Old-y1Old;
      // length
      length = sqrt(dxT*dxT + dyT*dyT);
    }
    else{
      length = l;
    }
    
    // calculate midpoint representation
    double mx = (x1Old+x2Old)/2;
    double my = (y1Old+y2Old)/2;
    //double phi = atan2(dyT,dxT);
    
    // get predicted midpoint representation
    double xPred, yPred, phiPred;
    
    xPred = xKF.getPosPred();
    yPred = yKF.getPosPred();
    phiPred = phiKF.getPosPred();
    
    // calculate endpoint coordinates of prediction
    double cosPhiPred = cos(phiPred);
    double sinPhiPred = sin(phiPred);
    x1Pred = xPred - cosPhiPred*length/2;
    y1Pred = yPred - sinPhiPred*length/2;
    x2Pred = xPred + cosPhiPred*length/2;
    y2Pred = yPred + sinPhiPred*length/2;
    
    // if there is a useful prediction, return the distance of old midpoint to predicted line
    if(nUpdates>0){
      CvPoint ep1, ep2, mp;
      ep1.x = (int) x1Pred;
      ep1.y = (int) y1Pred;
      ep2.x = (int) x2Pred;
      ep2.y = (int) y2Pred;
      mp.x = (int) mx;
      mp.y = (int) my;
      double range = distPointVector(ep1, ep2, mp);
      
      return range;
    }
    // else indictate that the result is not a useful prediction by returning a negative value
    else{
      return -1;
    }
  }
  else if(predMod == POLAR_COORD){
    ////////////////// polar coordinate motion model /////////////////
    
    // get predicted polar coordinates
    double rhoPred = rhoKF.getPosPred();
    double thetaPred = thetaKF.getPosPred();
    
    // project old endpoints on the new line
   double cosT = cos(thetaPred);
   double sinT = sin(thetaPred);
    
   
   x1Pred = rhoPred*cosT - sinT*(-x1Old*sinT + y1Old*cosT);
   y1Pred = rhoPred*sinT + cosT*(-x1Old*sinT + y1Old*cosT);
   x2Pred = rhoPred*cosT - sinT*(-x2Old*sinT + y2Old*cosT);
   y2Pred = rhoPred*sinT + cosT*(-x2Old*sinT + y2Old*cosT);
    

   if(rhoKF.getPosVar() > 15) return 15;
   else return  rhoKF.getPosVar();
  }
  else if(predMod == ONLY_PERP_MOTION){
    //////////////// perpendicular midpoint motion ////////////////
    
    // get predicted d, alpha
    double d = dKF.getPosPred();
    double alpha = alphaKF.getPosPred();
    alpha = 0;
    
    double dxT = x2Old-x1Old;
    double dyT = y2Old-y1Old;

    // calculate midpoint representation
    double mx = (x1Old+x2Old)/2;
    double my = (y1Old+y2Old)/2;
    double phi = atan(dyT/dxT);

    // shift the line perpendicular by d and rotate around midpoint by alpha  
    double mxPred = mx + sin(phi)*d;
    double myPred = my + cos(phi)*d;
    cout << "d="<<d<<endl;
    
    double phiPred = phi+alpha;
    
    double cosP = cos(phiPred);
    double sinP = sin(phiPred);
    
    // project endpoints on that line
    double tmp = (x1Old-mxPred)*cosP + (y1Old-myPred)*sinP;
    x1Pred = mxPred + tmp*cosP;
    y1Pred = myPred + tmp*sinP;
    
    
    
    
    
    tmp = (x2Old-mxPred)*cosP + (y2Old-myPred)*sinP;
    x2Pred = mxPred + tmp*cosP;
    y2Pred = myPred + tmp*sinP;
  }
  else if(predMod == PERP_MIDPOINT_PROJ){
    //////////// perpendicular midpoint motion global coordinates //////////////
    
    // get prediction
    double mxPred = xPKF.getPosPred();
    double myPred = yPKF.getPosPred();
    double phiPred = phiPKF.getPosPred();
    
   
    // project old endpoints on that line
    double cosP = cos(phiPred);
    double sinP = sin(phiPred);
    
    // project endpoints on that line
    double tmp = (x1Old-mxPred)*cosP + (y1Old-myPred)*sinP;
    x1Pred = mxPred + tmp*cosP;
    y1Pred = myPred + tmp*sinP;
    
    tmp = (x2Old-mxPred)*cosP + (y2Old-myPred)*sinP;
    x2Pred = mxPred + tmp*cosP;
    y2Pred = myPred + tmp*sinP;
    
    return cDist;
  }
  
  return 100;
}
