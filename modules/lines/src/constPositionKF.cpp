# include "lines/constPositionKF.hpp"

using namespace std;
using namespace jafar;
using namespace lines;

ConstPositionKF::ConstPositionKF(){
  x1 = 0;    // position

  cov11 = 0; // variance position
  
  x1Pred = 0;    // position prediction

  cov11Pred = 0; // variance position prediction


}

void ConstPositionKF::initKF(double initPos, double initVar){
  x1 = initPos;
  
  // cov = F * initCov * F'
  cov11 = initVar;
  
  // predict position
  x1Pred = x1;
  
  // predict covariance
  cov11Pred = cov11;
  

}

void ConstPositionKF::updateKF(double y, double sigmaY, double sigmaV){
  // predict position
  x1Pred = x1;
  // predict covariance
  cov11Pred = cov11 + sigmaV;
  
  double K1 = cov11Pred/(cov11Pred + sigmaY);
  
  x1 = x1Pred + K1*(y - x1Pred);
  
  cov11 = cov11Pred - K1*cov11Pred;

  x1Pred = x1;
}


