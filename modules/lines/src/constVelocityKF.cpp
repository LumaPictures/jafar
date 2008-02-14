# include "lines/constVelocityKF.hpp"

using namespace std;
using namespace jafar;
using namespace lines;

ConstVelocityKF::ConstVelocityKF(){
  x1 = 0;    // position
  x2 = 0;    // velocity

  cov11 = 0; // variance position
  cov12 = 0; // correlation
  cov22 = 0; // variance velocity
  
  x1Pred = 0;    // position prediction
  x2Pred = 0;    // velocity prediction

  cov11Pred = 0; // variance position prediction
  cov12Pred = 0; // correlation prediction
  cov22Pred = 0; // variance velocity prediction
  
  periodicFlag = 0;

}

void ConstVelocityKF::initKF(double initPos, double initVar){
  x1 = initPos;
  x2 = 0;
  
  // cov = F * initCov * F'
  cov11 = 2*initVar;
  cov12 = initVar;
  cov22 = initVar;
  
  // predict position
  x1Pred = x1 + x2;
  x2Pred = x2;
  
  // predict covariance
  cov11Pred = cov11;
  cov12Pred = cov12;
  cov22Pred = cov22;

}

void ConstVelocityKF::updateKF(double y, double sigmaY, double sigmaV){
  // predict position
  x1Pred = x1 + x2;
  x2Pred = x2;
  // predict covariance
  cov11Pred = cov11 + 2*cov12 + cov22 + sigmaV/4;
  cov12Pred = cov12 + cov22 + sigmaV/2;
  cov22Pred = cov22 + sigmaV;
  
  double K1 = cov11Pred/(cov11Pred + sigmaY);
  double K2 = cov12Pred/(cov11Pred + sigmaY);
  
  
  x1 = x1Pred + K1*(y - x1Pred);
  x2 = x2Pred + K2*(y - x1Pred);
  if(periodicFlag){
    if(x1 > maxPeriod){
      x1 -= (maxPeriod-minPeriod);
    }
    else if(x1 < minPeriod){
      x1 += (maxPeriod-minPeriod);
    }
  }
  
  cov11 = cov11Pred - K1*cov11Pred;
  cov12 = cov12Pred - K1*cov12Pred;
  cov22 = cov22Pred - K2*cov12Pred;

  x1Pred = x1 + x2;
  x2Pred = x2;
  
  if(periodicFlag){
    if(x1Pred > maxPeriod){
      x1Pred -= (maxPeriod-minPeriod);
    }
    else if(x1Pred < minPeriod){
      x1Pred += (maxPeriod-minPeriod);
    }
  }
}


