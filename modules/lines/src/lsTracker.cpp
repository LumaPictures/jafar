#include "lines/lsTracker.hpp"

using namespace std;
using namespace jafar;
using namespace image;
using namespace lines;

LsTracker::LsTracker(){
  
  // create gradient images with arbitrary size, size is updated automatically if neccessary
  gradX = new Image(320,240,IPL_DEPTH_16S,JfrImage_CS_GRAY);
  gradY = new Image(320,240,IPL_DEPTH_16S,JfrImage_CS_GRAY);
  cannyIm = new Image(320,240,IPL_DEPTH_8U,JfrImage_CS_GRAY);
  
  // start ID
  nextId = 1;
  
  minLength = 25;
  
  // initialize the parameters of lsExtractor
  lsExtractor.setCannyPara(150, 75, 3);
}


LsTracker::~LsTracker(){
  cout << "startDestTracker\n";
  // release memory of gradient images
  // TODO check if some further delete operations are needed 
  IplImage* tmp = *gradX;
  cvReleaseImage(&tmp);   // release gradX
  tmp = *gradY;
  cvReleaseImage(&tmp);   // release gradY
  tmp = *cannyIm;
  cvReleaseImage(&tmp);   // release cannyIm
  cout << "endDestTracker\n";

  // release line segment set
  //delete newLS;
}

/**
 * Proceed one tracking step. The new line segments are stored in newLS and the matching for all old line segments (_oldLS) to new line segments is stored in match.
 * 
 * @param image greyvalue image to proceed in this step
 * @param oldLS pointer to the set of line segments of the last step
 * @param newLS pointer to the storage for the new line segments created in this step
 * @param match storage for the matching, assigns old lines new lines
 * @param colorImage (optional) image to draw lines during the tracking (for debugging)
 */
void LsTracker::procTracking(Image* image, LineSegmentSet* oldLS, LineSegmentSet* newLS, MatchingSet* match, TrackScheme scheme, Image* colorImage ){
  // resize gradient images if neccessary
  if(gradX->width()!=image->width() || gradX->height()!=image->height()){
    IplImage* tmp = *gradX;
    cvReleaseImage(&tmp);
    tmp = *gradY;
    cvReleaseImage(&tmp);
    tmp = *cannyIm;
    cvReleaseImage(&tmp);
    gradX = new Image(image->width(),image->height(),IPL_DEPTH_16S,JfrImage_CS_GRAY);
    gradY = new Image(image->width(),image->height(),IPL_DEPTH_16S,JfrImage_CS_GRAY);
    cannyIm = new Image(image->width(),image->height(),IPL_DEPTH_8U,JfrImage_CS_GRAY);

  }
  
  // calculate gradient images
  calcXYGradientImages(image, gradX, gradY, 1);
  double t1 = startTimeMeasure();

  // extract lines (global extraction) and store in newLS
  newLS->clear();
  lsExtractor.extractLineSegments(image, cannyIm, *newLS, minLength);
  newLS->orientLines(image);
  
  /*
  double t1 = startTimeMeasure();
  newLS->calcHistogramDescriptor(image);
  cout << "time to extract "<<newLS->lineSegments.size()<<" Histograms: "<<stopTimeMeasure(t1)<<endl;
  
  
  double t = startTimeMeasure();
  // compare all histograms
  for(uint i=1; i< newLS->lineSegments.size(); i++){
  for(uint j=1; j< newLS->lineSegments.size(); j++){
      
  double d1 = cvCompareHist(newLS->lineSegments[i].histogramB->histo, newLS->lineSegments[j].histogramB->histo, CV_COMP_BHATTACHARYYA);
      
  double d2 = cvCompareHist(newLS->lineSegments[i].histogramD->histo, newLS->lineSegments[j].histogramD->histo, CV_COMP_BHATTACHARYYA);
      
  cout << "comparison of "<<i<<" to "<<j<< " B="<<d1<< " D="<<d2 <<endl;
      //newLS->lineSegments[i].histogramD->print();
      //newLS->lineSegments[i].histogramB->print();

}
}
  cout << "time for "<<newLS->lineSegments.size()*newLS->lineSegments.size()<<" comparisons: "<<stopTimeMeasure(t1)<<endl;
  */
  // apply tracking scheme
  if(scheme == GLOB_LOC_SEARCH) trackingScheme4(*oldLS, *newLS, image, gradX, gradY, *match, colorImage);
  else if(scheme == GLOB_MATCHING) trackingScheme6(*oldLS, *newLS, image, gradX, gradY, *match, colorImage);
  
  cout << "time for trackingScheme: "<<stopTimeMeasure(t1)<<endl;

}

void LsTracker::trackingScheme6(LineSegmentSet& lsA, LineSegmentSet& lsB, Image* image, Image* gradX, Image* gradY, MatchingSet& match, Image* colorImage){
  
  ///////// parameters /////////
  double threshA = 0.3;
  double threshM = 20;
  double threshH = 0.7;
  
  //////// values for statistic ///////
  uint nPassA=0;
  uint nPassM=0;
  uint nPassH=0;
  
  //////////////////////////////////////////////////////////
  ///////// calculate descriptor for all new lines /////////
  //////////////////////////////////////////////////////////
  
  lsB.calcHistogramDescriptor(image);
  
  // normalize the descriptors
  for(uint i=0; i<lsB.lineSegments.size(); i++){
    cvNormalizeHist(lsB.lineSegments[i].histogramB->histo, 1);
    cvNormalizeHist(lsB.lineSegments[i].histogramD->histo, 1);
  }

  /////////////////////////////////////////////////////////////////
  // compare all old lines with new lines with similiar position //
  /////////////////////////////////////////////////////////////////
  
  for(uint i=0; i<lsB.lineSegments.size(); i++){
    // reference for shorter writing
    LineSegment& lineB = lsB.lineSegments[i];
    double minDistH = threshH;
    uint minIdx = 0;
    
    for(uint j=0; j<lsA.lineSegments.size(); j++){
      // reference for shorter writing
      LineSegment& lineA=lsA.lineSegments[j];
      
      // compare angles
      double diffA = abs(lineA.getOAlpha() - lineB.getOAlpha());
      if(diffA>M_PI) diffA -= M_PI;
      
      if(diffA < threshA){
        nPassA++;
        
        // check projection distances of midpoints
        CvPoint aPt1, aPt2, aPtM, bPt1, bPt2, bPtM;
        aPt1.x = (int) lineA.getX1();
        aPt1.y = (int) lineA.getY1();
        aPt2.x = (int) lineA.getX2();
        aPt2.y = (int) lineA.getY2();
        aPtM.x = (int) lineA.getMx();
        aPtM.y = (int) lineA.getMy();
        
        bPt1.x = (int) lineB.getX1();
        bPt1.y = (int) lineB.getY1();
        bPt2.x = (int) lineB.getX2();
        bPt2.y = (int) lineB.getY2();
        bPtM.x = (int) lineB.getMx();
        bPtM.y = (int) lineB.getMy();
        
        double diffM = (distPointVector(aPt1, aPt2, bPtM) + distPointVector(bPt1, bPt2, aPtM)) / 2;
        
        if(diffM < threshM){
          nPassM++;
          
           // compare histograms
          double diffHB = cvCompareHist(lineA.histogramB->histo,lineB.histogramB->histo, CV_COMP_BHATTACHARYYA);
          double diffHD = cvCompareHist(lineA.histogramD->histo,lineB.histogramD->histo, CV_COMP_BHATTACHARYYA);
         
          if(diffHB < threshH && diffHD < threshH){
            nPassH++;
            double diffH = (diffHB + diffHD) / 2;
            if(diffH < minDistH){
              minDistH = diffH;
              minIdx = j;
            }
          } // thresH
          
        } // threshM
      } // threshA
    } // for j
    
    // check if line was mached
    if(minDistH < threshH){
      // update line with color and id
      lineB.color = lsA.lineSegments[minIdx].color;
      lineB.id = lsA.lineSegments[minIdx].id;
    }
    else{
      lineB.id = nextId;
      nextId++;
    }
  } // for i
  cout << "nPassA="<<nPassA<<" nPassM="<<nPassM<<" nPassH="<<nPassH<<endl;
}


/**
 * This function provides a tracking approach, gradX and gradY have to be gradient images 
 * lsA is old set, lsB new  set
 * Here the two sets of line segments are clustered before processing
 * Extends new set with fitted lines of old set and assigns an old line to each of the extended new lines set
 * All lines should be oriented.
 *
 * @param lsA reference to storage of lines of last tracking step
 * @param lsB reference to storage for the new lines
 * @param image greyvalue image to proceed in current step
 * @param gradX gradient image with gradient in x direction (e.g. with Sobel)
 * @param gradY gradient image with gradient in y direction (e.g. with Sobel)
 * @param match storage for the matching, assign old lines new lines
 * @param colorImage (optional) image to draw line segments during the tracking (for debugging)
 */
void LsTracker::trackingScheme5(LineSegmentSet& lsA, LineSegmentSet& lsB, Image* image, Image* gradX, Image* gradY, MatchingSet& match, Image* colorImage){

  
  ///////////////////////////////////////////////////////////////////////////
  /////////// cluster lines with respect to directed orientation ////////////
  ///////////////////////////////////////////////////////////////////////////
  cout << "Start clustering\n";
  int nCluster = 30;    // should be even 
  int nCluster05 = nCluster/2;
  int key;
  
  cout << "\t...cluster old lines ("<<lsA.lineSegments.size()<<")"<<endl;
  // old lines
  vector< vector<int> > clusterOld(nCluster);
  for(uint i=0; i<lsA.lineSegments.size(); i++){
    key = (int)((lsA.lineSegments[i].getOAlpha() / M_PI) * (nCluster05-1)) + nCluster05;
    //cout << "i="<<i<<" oAlpha="<<lsA.lineSegments[i].getOAlpha() << " key=" << key << endl;
    clusterOld[key].push_back(i);
  }
  
  cout << "\t...cluster new lines ("<<lsB.lineSegments.size()<<")"<<endl;
  // new lines
  vector< vector<int> > clusterNew(nCluster);
  for(uint i=0; i<lsB.lineSegments.size(); i++){
    key = (int)((lsB.lineSegments[i].getOAlpha() / M_PI) * (nCluster05-1)) + nCluster05;
    //cout << "i="<<i<<" oAlpha="<<lsB.lineSegments[i].getOAlpha() << " key=" << key << endl;
    clusterNew[key].push_back(i);
    
    
    CvPoint p1, p2;
    p1.x = (int) lsB.lineSegments[i].getX1();
    p1.y = (int) lsB.lineSegments[i].getY1();
    p2.x = (int) lsB.lineSegments[i].getX2();
    p2.y = (int) lsB.lineSegments[i].getY2();
    cvLine(*colorImage, p1,p2, CV_RGB(0, 255, 255),1);

  }
  
  /////////////////////////////////////////////////////////////////////////
  ////////// fit old lines and add results to newCluster //////////////////
  /////////////////////////////////////////////////////////////////////////
  cout << "Start fitting\n";
  match.clear();
  LineSegmentSet fitSet;
  int idx;
  for(int i=0; i<nCluster; i++){
    for(uint j=0; j<clusterOld[i].size(); j++){
      //cout << "enter loop"<<endl;
      idx = clusterOld[i][j];
      lsA.lineSegments[idx].fitLineOrientation(fitSet, gradX, gradY, 20, 10, M_PI*0.1, 10, 10, 0);
      //cout << "fitting size="<<fitSet.lineSegments.size()<<endl;
      
      //cout << "add best line"<<endl;
      // add best line to lsB (set of new lines) and insert its index in clusterNew
      if(fitSet.lineSegments.size()>0){
        //drawLineSegments(colorImage, fitSet, CV_RGB(255, 0, 0),1);
        // orient new lines
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
          fitSet.lineSegments[k].orientLine(image);
        }
        
        for(uint fitId=0; fitId<fitSet.lineSegments.size(); fitId++){
          CvPoint p1, p2;
          p1.x = (int) fitSet.lineSegments[fitId].getX1();
          p1.y = (int) fitSet.lineSegments[fitId].getY1();
          p2.x = (int) fitSet.lineSegments[fitId].getX2();
          p2.y = (int) fitSet.lineSegments[fitId].getY2();
          cvLine(*colorImage, p1,p2, CV_RGB(255, 255, 0),1);
        }
        
        // try to merge the new lines
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
          for(uint l=k+1; l<fitSet.lineSegments.size(); l++){
            int mergeResult = LineSegment::tryMergeLinesChiSq( fitSet.lineSegments[l], fitSet.lineSegments[k], 30);
            // if merged and result is one of the old lines (stored in fitSet.lineSegments[l])
            if(mergeResult==1){
              fitSet.lineSegments[k].setInvalid();
              break;
            }
            // if merged and result is a new line (stored in fitSet.lineSegments[l])
            if(mergeResult==3){
              fitSet.lineSegments[k].setInvalid();
              fitSet.lineSegments[l].orientLine(image);
              break;
            }
          }
        }
        
        // take longest line
        double maxL=0;
        int maxIdxFit=0;
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
          if(fitSet.lineSegments[k].isValid()){
            if(fitSet.lineSegments[k].getEucLength() > maxL){
              maxL = fitSet.lineSegments[k].getEucLength();
              maxIdxFit = k;
            }
          }
        }
        // add longest line
        lsB.addLine(&fitSet.lineSegments[maxIdxFit]);
        clusterNew[i].push_back( lsB.lineSegments.size()-1 );
        
        // save origin of this line
        lsB.lineSegments[ lsB.lineSegments.size()-1 ].param1 = clusterOld[i][j];
        
        /*
        // find best matching
        // TODO special matching necessary because the lines are very similiar
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
        if(fitSet.lineSegments[k].isValid()){
        curDist = LineSegment::mahaDistLines(fitSet.lineSegments[k], lsA.lineSegments[ clusterOld[i][j] ], varEP, varDir, varGrey, varVGrey);
        if(curDist<min){
        min=curDist;
        minIdx = k;
      }
      }
      }
        // add best matched line
        lsB.addLine(&fitSet.lineSegments[minIdx]);
        clusterNew[i].push_back( lsB.lineSegments.size()-1 );
        */
      }
      
      /*
      // add resulting lines to lsB (set of new lines) and insert whose index in clusterNew
      for(uint k=0; k<fitSet.lineSegments.size(); k++){
        // orient new lines
      fitSet.lineSegments[k].orientLine(originImage);
      lsB.addLine(&fitSet.lineSegments[k]);
      clusterNew[i].push_back( lsB.lineSegments.size()-1 );
    }
      */
      
      fitSet.clear();
      //cout << "leave loop"<<endl;
    }
  }
  
  //////////////////////////////////////////////////////////////
  ///////// try to merge all lines in each clusterNew //////////
  //////////////////////////////////////////////////////////////
  
  //TODO try to merge with lines of neighboured clusters
  
  cout << "Start merging\n";
  
  int nMerge;
  bool merged=0;
  for(uint i=0; i<clusterNew.size(); i++){
    for(uint j=0; j<clusterNew[i].size(); j++){
      merged = 0;
      for(uint k=j+1; k<clusterNew[i].size(); k++){
        // try to merge j and k, if merged, result is in k
        CvPoint kPt1, kPt2, jPt1, jPt2;
        kPt1.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX1();
        kPt1.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY1();
        kPt2.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX2();
        kPt2.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY2();
        
        jPt1.x = (int) lsB.lineSegments[ clusterNew[i][j] ].getX1();
        jPt1.y = (int) lsB.lineSegments[ clusterNew[i][j] ].getY1();
        jPt2.x = (int) lsB.lineSegments[ clusterNew[i][j] ].getX2();
        jPt2.y = (int) lsB.lineSegments[ clusterNew[i][j] ].getY2();
        
        double pD1 = lsB.lineSegments[ clusterNew[i][k] ].polarD;
        double pD2 = lsB.lineSegments[ clusterNew[i][j] ].polarD;
        double pA1 = lsB.lineSegments[ clusterNew[i][k] ].polarAlpha;
        double pA2 = lsB.lineSegments[ clusterNew[i][j] ].polarAlpha;
        
        nMerge = LineSegment::tryMergeLinesChiSq( lsB.lineSegments[ clusterNew[i][k] ], lsB.lineSegments[ clusterNew[i][j] ] );
        // if j could be merged into another line (nMerge==1 || nMerge==3)
        if(nMerge==1){
          merged = 1;
          CvPoint kNew1, kNew2;
          kNew1.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX1();
          kNew1.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY1();
          kNew2.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX2();
          kNew2.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY2();
           
          double dist1 = distPointVector(kNew1, kNew2, kPt1) + distPointVector(kNew1, kNew2, kPt2) + distPointVector(kNew1, kNew2, jPt1) + distPointVector(kNew1, kNew2, jPt2);
           
           
          CvScalar color = CV_RGB(rand()%255, rand()%255, rand()%255);
          if(dist1>20){
            cvLine(*colorImage, kPt1, kPt2, color, 2);
            cvLine(*colorImage, jPt1, jPt2, color, 2);
            cout << endl << "pD1=" << pD1 << " pd2=" << pD2 << endl << "pA1=" << pA1 << " pA2=" << pA2 << endl << endl;
            return;
          }
           
           //cout << "dist="<<dist1<<endl;
           
        }
        else if(nMerge==3){
          merged = 1;
          lsB.lineSegments[ clusterNew[i][k] ].orientLine(image);
          
          CvPoint kNew1, kNew2;
          kNew1.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX1();
          kNew1.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY1();
          kNew2.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX2();
          kNew2.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY2();
           
          double dist1 = distPointVector(kNew1, kNew2, kPt1) + distPointVector(kNew1, kNew2, kPt2) + distPointVector(kNew1, kNew2, jPt1) + distPointVector(kNew1, kNew2, jPt2);
           
          
          CvScalar color = CV_RGB(rand()%255, rand()%255, rand()%255);
          if(dist1>20){
            cvLine(*colorImage, kPt1, kPt2, color, 2);
            cvLine(*colorImage, jPt1, jPt2, color, 2);
            cout << endl << "pD1=" << pD1 << " pd2=" << pD2 << endl << "pA1=" << pA1 << " pA2=" << pA2 << endl << endl;
            return;
          }
          
          
          //cout << "dist="<<dist1<<endl;
        }
      } // for k
      
      // if j could be merged into another line at least one time, then erase it
      if(merged){
        clusterNew[i][j] = -1;
      }
    } // for j
  } // for i
  
  //////////////////////////////////////////////////////////////////////
  ///////// try to match old lines with updated new line set  //////////
  //////////////////////////////////////////////////////////////////////
  cout << "Start matching\n";
  
  
  LineSegmentSet updatedB;
  
  match.clear();

  
  // go through each clusterNew and search for matching in corresponding clusterOld (or small set of clusterOld)
  for(int i=0; i<nCluster; i++){
    // for each old line in this cluster
    //cout << "size["<<i<<"]="<<clusterNew[i].size()<<endl;
    for(uint j=0; j<clusterNew[i].size(); j++){
      // if not erased (set to -1) in merging step
      if(clusterNew[i][j]>=0){
        // if it is one of the fitted lines, save the corresponding matching
        if(lsB.lineSegments[ clusterNew[i][j] ].param1 >= 0){
          // update color of new line with color of matching
          lsB.lineSegments[ clusterNew[i][j] ].color = lsA.lineSegments[ lsB.lineSegments[ clusterNew[i][j] ].param1 ].color;
          // save line in updatedB (to overcome the gaps in lsB occured through merging)
          updatedB.addLine( &lsB.lineSegments[ clusterNew[i][j] ] );
          // save matching of old line to new line
          match.addMatching( lsB.lineSegments[ clusterNew[i][j] ].param1, updatedB.lineSegments.size()-1 );
        }
        else{
          // if there is no matching line, just save line in updatedB
          updatedB.addLine( &lsB.lineSegments[ clusterNew[i][j] ]);
        }
        
        
      } // end if not erased
      else{
        cout << "...erased"<<endl;
      }
    } // for j
    
    
    
  } // for i
  cout << "Start clone\n";
  LineSegmentSet::clone(updatedB, lsB);

  //match.print();
}


/**
 * This function provides a tracking approach, gradX and gradY have to be gradient images 
 * lsA is old set, lsB new  set
 * Extends new set with fitted lines of old set and assigns an old line to each of the extended new lines set
 * All lines should be oriented.
 *
 * @param lsA reference to storage of lines of last tracking step
 * @param lsB reference to storage for the new lines
 * @param image greyvalue image to proceed in current step
 * @param gradX gradient image with gradient in x direction (e.g. with Sobel)
 * @param gradY gradient image with gradient in y direction (e.g. with Sobel)
 * @param match storage for the matching, assign old lines new lines
 * @param colorImage (optional) image to draw line segments during the tracking (for debugging)
 */
void LsTracker::trackingScheme4(LineSegmentSet& lsA, LineSegmentSet& lsB, Image* image, Image* gradX, Image* gradY, MatchingSet& match, Image* colorImage){
  
  // flags
  bool drawFlag = 0;  // enables drawing on colorImage
  bool predFlag = 1;
  bool logFlag = 0;
  
  double threshAvLR = 40;
  /////////////////////////////////////////////////////////////////////////
  ////////// fit old lines and add results to newCluster //////////////////
  /////////////////////////////////////////////////////////////////////////
  cout << "Start fitting\n";
  LineSegmentSet fitSet;
  LineSegment predLine;
  double pX1, pY1, pX2, pY2;
  
  double predRange;
  
  if(logFlag){
    for(uint i=0; i<lsB.lineSegments.size();i++){
      cout << "lsB["<<i<<"]: id="<<lsB.lineSegments[i].id <<" param1="<<lsB.lineSegments[i].param1<<" sizeOfParamaters="<<lsB.lineSegments[i].parameters.size()<<endl;
    }
    for(uint i=0; i<lsA.lineSegments.size();i++){
      cout << "lsA["<<i<<"]: id="<<lsA.lineSegments[i].id <<" param1="<<lsB.lineSegments[i].param1<<" sizeOfParamaters="<<lsB.lineSegments[i].parameters.size()<<endl;
    }
  }
  
  for(uint i=0; i<lsA.lineSegments.size(); i++){
    if(predFlag){

      // get prediction
      if(logFlag)cout << "============================================\n";
      
      //if there is a prediction from anywhere (e.g. SLAM) use it, otherwise use the lsPredictor prediction
      if(lsA.lineSegments[i].predFlag){
        cout << "use prediction"<<endl;
        predRange = lsA.lineSegments[i].getEPPredictionByRhoThetaPrediction(lsA.lineSegments[i].getX1(), lsA.lineSegments[i].getY1(), lsA.lineSegments[i].getX2(), lsA.lineSegments[i].getY2(), pX1, pY1, pX2, pY2);
      }
      else{
        predRange = lsA.lineSegments[i].predictor.getPredictionEP(lsA.lineSegments[i].getX1(), lsA.lineSegments[i].getY1(), lsA.lineSegments[i].getX2(), lsA.lineSegments[i].getY2(), pX1, pY1, pX2, pY2);
      }
      
      if(logFlag)cout << "predRange="<<predRange<<endl;
      
      // a range < 0 indicates a not reliable prediction
      if(predRange < 0) predRange = 15;
      else if(predRange<3) predRange = 3;
      else if(predRange>15) predRange = 15;
      
      
      // create predicted line
      predLine.setPoints(pX1, pY1, pX2, pY2);
      //predLine.orientLine(image);
      CvPoint pp1, pp2;
      pp1.x = (int) pX1;
      pp1.y = (int) pY1;
      pp2.x = (int) pX2;
      pp2.y = (int) pY2;
      //cvLine(*colorImage, pp1, pp2, CV_RGB(255,0,0), 2);
      cvLine(*colorImage, pp1, pp2,lsA.lineSegments[i].color , 2);
      pp1.x = (int) lsA.lineSegments[i].getX1();
      pp1.y = (int) lsA.lineSegments[i].getY1();
      pp2.x = (int) lsA.lineSegments[i].getX2();
      pp2.y = (int) lsA.lineSegments[i].getY2();
      //cvLine(*colorImage, pp1, pp2, CV_RGB(255,255,0), 2);
  
      predLine.fitLineOrientation(fitSet, gradX, gradY, 8, (int)predRange, M_PI*0.1, 6, 10, 0);
    }
    else{
        
      lsA.lineSegments[i].fitLineOrientation(fitSet, gradX, gradY, 8, 15, M_PI*0.1, 6, 10, 0);
      
    }

    if(fitSet.lineSegments.size()>0){

      // orient new lines
      for(uint k=0; k<fitSet.lineSegments.size(); k++){
        fitSet.lineSegments[k].orientLine(image);
      }
      if(drawFlag){
        // draw all lines of fitSet
        for(uint fitId=0; fitId<fitSet.lineSegments.size(); fitId++){
          CvPoint p1, p2;
          p1.x = (int) fitSet.lineSegments[fitId].getX1();
          p1.y = (int) fitSet.lineSegments[fitId].getY1();
          p2.x = (int) fitSet.lineSegments[fitId].getX2();
          p2.y = (int) fitSet.lineSegments[fitId].getY2();
          cvLine(*colorImage, p1,p2, CV_RGB(255, 0, 255),3);
        }
      }
      
        // try to merge the new lines
      for(uint k=0; k<fitSet.lineSegments.size(); k++){
        for(uint l=k+1; l<fitSet.lineSegments.size(); l++){
          int mergeResult = LineSegment::tryMergeLinesChiSq( fitSet.lineSegments[l], fitSet.lineSegments[k], 50);

            // if merged and the result is one of the old lines (stored in fitSet.lineSegments[l])
          if(mergeResult==1){
            fitSet.lineSegments[k].setInvalid();
            break;
          }
            // if merged and result is a new line (stored in fitSet.lineSegments[l])
          if(mergeResult==3){
            fitSet.lineSegments[k].setInvalid();
            fitSet.lineSegments[l].orientLine(image);
            break;
          }
        }
      }
      
      // take longest line
      double maxL=0;
      int maxIdxFit=-1;
      for(uint k=0; k<fitSet.lineSegments.size(); k++){
        if(fitSet.lineSegments[k].isValid()){
          if(fitSet.lineSegments[k].getEucLength() > maxL){
            maxL = fitSet.lineSegments[k].getEucLength();
            maxIdxFit = k;
          }
        }
      }
      
      /*
      // draw longest line
      CvPoint p1,p2;
      p1.x = fitSet.lineSegments[maxIdxFit].getX1();
      p1.y = fitSet.lineSegments[maxIdxFit].getY1();
      p2.x = fitSet.lineSegments[maxIdxFit].getX2();
      p2.y = fitSet.lineSegments[maxIdxFit].getY2();
      cvLine(*colorImage, p1, p2, CV_RGB(255,255,0), 2);
      */
      if(drawFlag){
        // draw longest line
        CvPoint p1,p2;
        p1.x = (int) fitSet.lineSegments[maxIdxFit].getX1();
        p1.y = (int) fitSet.lineSegments[maxIdxFit].getY1();
        p2.x = (int) fitSet.lineSegments[maxIdxFit].getX2();
        p2.y = (int) fitSet.lineSegments[maxIdxFit].getY2();
        cvLine(*colorImage, p1, p2, CV_RGB(255,255,0), 3);
        
        // draw all the other lines of fitSet
        for(uint fitId=0; fitId<fitSet.lineSegments.size(); fitId++){
          CvPoint p1, p2;
          p1.x = (int) fitSet.lineSegments[fitId].getX1();
          p1.y = (int) fitSet.lineSegments[fitId].getY1();
          p2.x = (int) fitSet.lineSegments[fitId].getX2();
          p2.y = (int) fitSet.lineSegments[fitId].getY2();
          cvLine(*colorImage, p1,p2, CV_RGB(0, 255, 255),1);
        }
      }
      
      // add longest line
      lsB.addLine(&fitSet.lineSegments[maxIdxFit]); // insert line
      lsB.lineSegments[(int)lsB.lineSegments.size()-1].param1 = i;    // set param1
      lsB.lineSegments[(int)lsB.lineSegments.size()-1].parameters.push_back(i); // update parameters
      //lsB.lineSegments[(int)lsB.lineSegments.size()-1].id = lsA.lineSegments[i].id; // copy id

      if(logFlag) cout << "oldLs["<<lsA.lineSegments[i].id<<"] was fitted to lsB["<< (int)lsB.lineSegments.size()-1<<"]"<<endl;
    } // fitset not empty
    else{
      /////////// line was not fitted ///////////////
      if(logFlag) cout << "oldLs["<<lsA.lineSegments[i].id<<"] was not fitted ("<<predLine.getX1()<<" "<<predLine.getY1()<<" "<<predLine.getX2()<<" "<<predLine.getY2()<<")"<<endl;

      // add to not yet matched set
      //
      //
      //
      //////////////////////////////
    }
    fitSet.clear();
  } // for i

  ///////////////////////////////////////////
  ///////// try to merge new lines //////////
  ///////////////////////////////////////////
  
  cout << "Start merging\n";
  
  int paramI, paramJ, idI, idJ;
  double lI, lJ;
  vector<int> ants;
  for(uint i=0; i<lsB.lineSegments.size(); i++){
    if(lsB.lineSegments[i].isValid()){

      paramI = lsB.lineSegments[i].param1;
      lI = lsB.lineSegments[i].getEucLength();
      if(paramI>=0) idI = lsA.lineSegments[paramI].id;
      else idI = -1;
      
      for(uint j=i+1; j<lsB.lineSegments.size(); j++){
        if(lsB.lineSegments[j].isValid()){

          // store vector of antecessors
          ants.clear();
          for(uint pIdxI=0; pIdxI<lsB.lineSegments[i].parameters.size(); pIdxI++){
            ants.push_back(lsB.lineSegments[i].parameters[pIdxI]);
          }
          for(uint pIdxJ=0; pIdxJ<lsB.lineSegments[j].parameters.size(); pIdxJ++){
            ants.push_back(lsB.lineSegments[j].parameters[pIdxJ]);
          }
          
          paramJ = lsB.lineSegments[j].param1;
          lJ = lsB.lineSegments[j].getEucLength();
          if(paramJ>=0) idJ = lsA.lineSegments[paramJ].id;
          else idJ = -1;
          
          int mergeResult = LineSegment::tryMergeLinesChiSq( lsB.lineSegments[j], lsB.lineSegments[i] );
          
          
          // if result of merging is an new line, orient it and erase line i
          if( mergeResult==3 ){
            lsB.lineSegments[j].orientLine(image);
            lsB.lineSegments[i].setInvalid();
            // copy vector of antecessors in new line
            lsB.lineSegments[j].parameters.clear();
            for(uint idxA=0; idxA<ants.size(); idxA++){
              lsB.lineSegments[j].parameters.push_back(ants[idxA]);
            }
                              
            
            if(paramI>=0 && paramJ<0)
              lsB.lineSegments[j].param1 = paramI;
            else if(paramI<0 && paramJ>=0)
              lsB.lineSegments[j].param1 = paramJ;
            else{
              // decision by global id
              if(idI<idJ){
                lsB.lineSegments[j].param1 = paramI;
              }
              else{
                lsB.lineSegments[j].param1 = paramJ;
              }
              
              // decision by length
              // if(lI>lJ) lsB.lineSegments[j].param1 = paramI;
              // else lsB.lineSegments[j].param1 = paramJ;
            }
            if(logFlag) cout << "lsB["<<i<<"] and ["<<j<<"] were merged to lsB["<<j<<endl; 
            break;
          }
          // if result of merging is one of the old lines, erase the other
          else if( mergeResult==1 ){
            lsB.lineSegments[i].setInvalid();
            
            // copy vector of antecessors in new line
            lsB.lineSegments[j].parameters.clear();
            for(uint idxA=0; idxA<ants.size(); idxA++){
              lsB.lineSegments[j].parameters.push_back(ants[idxA]);
            }
            
            
            if(paramI>=0 && paramJ<0)
              lsB.lineSegments[j].param1 = paramI;
            else if(paramI<0 && paramJ>=0)
              lsB.lineSegments[j].param1 = paramJ;
            else{
              // decision by global id
              if(idI<idJ){
                lsB.lineSegments[j].param1 = paramI;
              }
              else{
                lsB.lineSegments[j].param1 = paramJ;
              }
              
              // decision by length
              // if(lI>lJ) lsB.lineSegments[j].param1 = paramI;
              // else lsB.lineSegments[j].param1 = paramJ;
            }
            
            if(logFlag){ 
              cout << "lsB["<<i<<"] and ["<<j<<"] were merged to lsB["<<j<<"], its validFlag is  "<<lsB.lineSegments[j].isValid()<<", its param1 is "<<lsB.lineSegments[j].param1<<", its parameters are";
              for(uint pI=0; pI< lsB.lineSegments[j].parameters.size(); pI++){
                cout << " "<<lsB.lineSegments[j].parameters[pI];
              }
              cout << endl;
            }
            break;
          }
            
        } // if lsB[j] is valid
      }
    } // if lsB[i] is valid
  }

  //////////////////////////////////////////////////////////////////////
  ///////// try to match old lines with updated new line set  //////////
  //////////////////////////////////////////////////////////////////////
  cout << "Start matching\n";
  
  match.clear();
  
  // take all lines that could not be matched
  
  // validate the matching 
  for(uint i=0; i<lsB.lineSegments.size(); i++){
    if(lsB.lineSegments[i].isValid()){
      if(lsB.lineSegments[i].parameters.size() > 0){
        // have a look at all antecessor lines, if no antecessor pass the similarity test, set the line delete the all antecessors
        double similarityState=0;
        double distMin=10000;
        for(uint j=0; j<lsB.lineSegments[i].parameters.size();j++){
          // process validation test
          double distGrey = LineSegment::compareAvLR(lsA.lineSegments[ lsB.lineSegments[i].parameters[j] ], lsB.lineSegments[i]);
          if(distGrey < threshAvLR){
            similarityState=1;
          }
          if(distGrey < distMin) distMin=distGrey;
  
        }
        if(!similarityState){
          lsB.lineSegments[i].parameters.clear();
          lsB.lineSegments[i].param1 = -1;
          if(logFlag) cout << "lsB["<<i<<"] failed similarity test, minDist="<<distMin<<endl; 
        }
      }
      // if no antecessors
      else{
        if(logFlag) cout << "lsB["<<i<<"] has no fitting antecessor"<<endl; 
      }
    }
  }
  
  lsB.eraseInvalid();

  for(uint i=0; i<lsB.lineSegments.size(); i++){
    if(lsB.lineSegments[i].isValid()){
      
      if(logFlag) cout << "ls has been ";

      // if line is not matched, asign new id, otherwise update 
      if(lsB.lineSegments[i].parameters.size()==0){
        if(logFlag) cout << "...nothing...";
        lsB.lineSegments[i].id = nextId;
        nextId++;
      }
      else{
        for(uint j=0; j<lsB.lineSegments[i].parameters.size();j++){

          if(logFlag){
            double distGrey = LineSegment::compareAvLR(lsA.lineSegments[ lsB.lineSegments[i].parameters[j] ], lsB.lineSegments[i]);

            cout << "["<< lsA.lineSegments[ lsB.lineSegments[i].parameters[j] ].id <<"], distAVLR="<<distGrey<<" ";
          }
          
          match.addMatching(lsB.lineSegments[i].parameters[j], i);
          
          if(lsB.lineSegments[i].parameters[j] == lsB.lineSegments[i].param1){
            // references for shorter writing 
            LineSegment &lsOld = lsA.lineSegments[ lsB.lineSegments[i].param1 ];
            LineSegment &lsNew = lsB.lineSegments[i];
        
            if(predFlag){
              lsB.lineSegments[i].predictor = lsA.lineSegments[ lsB.lineSegments[i].param1 ].predictor;
              lsB.lineSegments[i].predictor.updatePredictor(lsOld.getMx(), lsOld.getMy(), lsOld.getAlpha(), lsNew.getMx(), lsNew.getMy(), lsNew.getAlpha(), lsNew.polarD, lsNew.polarAlpha);
            }
            
            lsB.lineSegments[i].color = lsA.lineSegments[ lsB.lineSegments[i].param1 ].color;
            lsB.lineSegments[i].id = lsA.lineSegments[ lsB.lineSegments[i].param1 ].id;
            
            //cout << "param1="<<lsB.lineSegments[i].param1<<endl;
          }
            
        } // for j
      }
      if(logFlag) cout << " and is now ["<<lsB.lineSegments[i].id<<"]"<< endl;
    }
    
  }

  lsB.clearParameters();
  
}


/**
 * This function provides a tracking approach, gradX and gradY have to be gradient images 
 * lsA is old set, lsB new  set
 * Here the two sets of line segments are clustered before processing
 * Extends new set with fitted lines of old set and assigns an old line to each of the extended new lines set
 * All lines should be oriented.
 *
 * @param lsA reference to storage of lines of last tracking step
 * @param lsB reference to storage for the new lines
 * @param image greyvalue image to proceed in current step
 * @param gradX gradient image with gradient in x direction (e.g. with Sobel)
 * @param gradY gradient image with gradient in y direction (e.g. with Sobel)
 * @param match storage for the matching, assign old lines new lines
 * @param colorImage (optional) image to draw line segments during the tracking (for debugging)
 */
void LsTracker::trackingScheme3(LineSegmentSet& lsA, LineSegmentSet& lsB, Image* image, Image* gradX, Image* gradY, MatchingSet& match, Image* colorImage){
  
  // matching parameters
  double varEP = 40;
  double varDir = 0.01;
  double varGrey = 3;
  double varVGrey = 400;
  
  ///////////////////////////////////////////////////////////////////////////
  /////////// cluster lines with respect to directed orientation ////////////
  ///////////////////////////////////////////////////////////////////////////
  cout << "Start clustering\n";
  int nCluster = 30;    // should be even 
  int nCluster05 = nCluster/2;
  int key;
  
  cout << "\t...cluster old lines ("<<lsA.lineSegments.size()<<")"<<endl;
  // old lines
  vector< vector<int> > clusterOld(nCluster);
  for(uint i=0; i<lsA.lineSegments.size(); i++){
    key = (int)((lsA.lineSegments[i].getOAlpha() / M_PI) * (nCluster05-1)) + nCluster05;
    //cout << "i="<<i<<" oAlpha="<<lsA.lineSegments[i].getOAlpha() << " key=" << key << endl;
    clusterOld[key].push_back(i);
  }
  
  cout << "\t...cluster new lines ("<<lsB.lineSegments.size()<<")"<<endl;
  // new lines
  vector< vector<int> > clusterNew(nCluster);
  for(uint i=0; i<lsB.lineSegments.size(); i++){
    key = (int)((lsB.lineSegments[i].getOAlpha() / M_PI) * (nCluster05-1)) + nCluster05;
    //cout << "i="<<i<<" oAlpha="<<lsB.lineSegments[i].getOAlpha() << " key=" << key << endl;
    clusterNew[key].push_back(i);
  }
  
  /////////////////////////////////////////////////////////////////////////
  ////////// fit old lines and add results to newCluster //////////////////
  /////////////////////////////////////////////////////////////////////////
  cout << "Start fitting\n";
  LineSegmentSet fitSet;
  int idx;
  for(int i=0; i<nCluster; i++){
    for(uint j=0; j<clusterOld[i].size(); j++){
      //cout << "enter loop"<<endl;
      idx = clusterOld[i][j];
      lsA.lineSegments[idx].fitLineOrientation(fitSet, gradX, gradY, 20, 10, M_PI*0.1, 15, 10, 0);
      //cout << "fitting size="<<fitSet.lineSegments.size()<<endl;
      
      //cout << "add best line"<<endl;
      // add best line to lsB (set of new lines) and insert its index in clusterNew
      if(fitSet.lineSegments.size()>0){
        //drawLineSegments(colorImage, fitSet, CV_RGB(255, 0, 0),1);
        // orient new lines
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
          fitSet.lineSegments[k].orientLine(image);
        }
        
        // try to merge the new lines
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
          for(uint l=k+1; l<fitSet.lineSegments.size(); l++){
            int mergeResult = LineSegment::tryMergeLinesChiSq( fitSet.lineSegments[l], fitSet.lineSegments[k], 10);
            // if merged and result is one of the old lines (stored in fitSet.lineSegments[l])
            if(mergeResult==1){
              fitSet.lineSegments[k].setInvalid();
              break;
            }
            // if merged and result is a new line (stored in fitSet.lineSegments[l])
            if(mergeResult==3){
              fitSet.lineSegments[k].setInvalid();
              fitSet.lineSegments[l].orientLine(image);
              break;
            }
          }
        }
        
        // take longest line
        double maxL=0;
        int maxIdxFit=0;
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
          if(fitSet.lineSegments[k].isValid()){
            if(fitSet.lineSegments[k].getEucLength() > maxL){
              maxL = fitSet.lineSegments[k].getEucLength();
              maxIdxFit = k;
            }
          }
        }
        // add longest line
        lsB.addLine(&fitSet.lineSegments[maxIdxFit]);
        clusterNew[i].push_back( lsB.lineSegments.size()-1 );
        
        /*
        // find best matching
        // TODO special matching necessary because the lines are very similiar
        for(uint k=0; k<fitSet.lineSegments.size(); k++){
        if(fitSet.lineSegments[k].isValid()){
        curDist = LineSegment::mahaDistLines(fitSet.lineSegments[k], lsA.lineSegments[ clusterOld[i][j] ], varEP, varDir, varGrey, varVGrey);
        if(curDist<min){
        min=curDist;
        minIdx = k;
      }
      }
      }
        // add best matched line
        lsB.addLine(&fitSet.lineSegments[minIdx]);
        clusterNew[i].push_back( lsB.lineSegments.size()-1 );
        */
      }
      
      /*
      // add resulting lines to lsB (set of new lines) and insert whose index in clusterNew
      for(uint k=0; k<fitSet.lineSegments.size(); k++){
        // orient new lines
      fitSet.lineSegments[k].orientLine(originImage);
      lsB.addLine(&fitSet.lineSegments[k]);
      clusterNew[i].push_back( lsB.lineSegments.size()-1 );
    }
      */
      
      fitSet.clear();
      //cout << "leave loop"<<endl;
    }
  }
  
  //////////////////////////////////////////////////////////////
  ///////// try to merge all lines in each clusterNew //////////
  //////////////////////////////////////////////////////////////
  
  //TODO try to merge with lines of neighboured clusters
  
  cout << "Start merging\n";
  
  int nMerge;
  bool merged=0;
  for(uint i=0; i<clusterNew.size(); i++){
    for(uint j=0; j<clusterNew[i].size(); j++){
      merged = 0;
      for(uint k=j+1; k<clusterNew[i].size(); k++){
        // try to merge j and k, if merged, result is in k
        CvPoint kPt1, kPt2, jPt1, jPt2;
        kPt1.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX1();
        kPt1.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY1();
        kPt2.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX2();
        kPt2.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY2();
        
        jPt1.x = (int) lsB.lineSegments[ clusterNew[i][j] ].getX1();
        jPt1.y = (int) lsB.lineSegments[ clusterNew[i][j] ].getY1();
        jPt2.x = (int) lsB.lineSegments[ clusterNew[i][j] ].getX2();
        jPt2.y = (int) lsB.lineSegments[ clusterNew[i][j] ].getY2();
        
        double pD1 = lsB.lineSegments[ clusterNew[i][k] ].polarD;
        double pD2 = lsB.lineSegments[ clusterNew[i][j] ].polarD;
        double pA1 = lsB.lineSegments[ clusterNew[i][k] ].polarAlpha;
        double pA2 = lsB.lineSegments[ clusterNew[i][j] ].polarAlpha;
        
        nMerge = LineSegment::tryMergeLinesChiSq( lsB.lineSegments[ clusterNew[i][k] ], lsB.lineSegments[ clusterNew[i][j] ] );
        // if j could be merged into another line (nMerge==1 || nMerge==3)
        if(nMerge==1){
          merged = 1;
          CvPoint kNew1, kNew2;
          kNew1.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX1();
          kNew1.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY1();
          kNew2.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX2();
          kNew2.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY2();
           
          double dist1 = distPointVector(kNew1, kNew2, kPt1) + distPointVector(kNew1, kNew2, kPt2) + distPointVector(kNew1, kNew2, jPt1) + distPointVector(kNew1, kNew2, jPt2);
           
           
          CvScalar color = CV_RGB(rand()%255, rand()%255, rand()%255);
          if(dist1>20){
            cvLine(*colorImage, kPt1, kPt2, color, 2);
            cvLine(*colorImage, jPt1, jPt2, color, 2);
            cout << endl << "pD1=" << pD1 << " pd2=" << pD2 << endl << "pA1=" << pA1 << " pA2=" << pA2 << endl << endl;
            return;
          }
           
           //cout << "dist="<<dist1<<endl;
           
        }
        else if(nMerge==3){
          merged = 1;
          lsB.lineSegments[ clusterNew[i][k] ].orientLine(image);
          
          CvPoint kNew1, kNew2;
          kNew1.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX1();
          kNew1.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY1();
          kNew2.x = (int) lsB.lineSegments[ clusterNew[i][k] ].getX2();
          kNew2.y = (int) lsB.lineSegments[ clusterNew[i][k] ].getY2();
           
          double dist1 = distPointVector(kNew1, kNew2, kPt1) + distPointVector(kNew1, kNew2, kPt2) + distPointVector(kNew1, kNew2, jPt1) + distPointVector(kNew1, kNew2, jPt2);
           
          
          CvScalar color = CV_RGB(rand()%255, rand()%255, rand()%255);
          if(dist1>20){
            cvLine(*colorImage, kPt1, kPt2, color, 2);
            cvLine(*colorImage, jPt1, jPt2, color, 2);
            cout << endl << "pD1=" << pD1 << " pd2=" << pD2 << endl << "pA1=" << pA1 << " pA2=" << pA2 << endl << endl;
            return;
          }
          
          
          //cout << "dist="<<dist1<<endl;
        }
      } // for k
      
      // if j could be merged into another line at least one time, then erase it
      if(merged){
        clusterNew[i][j] = -1;
      }
    } // for j
  } // for i
  
  //////////////////////////////////////////////////////////////////////
  ///////// try to match old lines with updated new line set  //////////
  //////////////////////////////////////////////////////////////////////
  cout << "Start matching\n";
  
  
  LineSegmentSet updatedB;
  double minDist, curDist;
  int minIdx;
  
  match.clear();

  
  // go through each clusterNew and search for matching in corresponding clusterOld (or small set of clusterOld)
  for(int i=0; i<nCluster; i++){
    // for each old line in this cluster
    //cout << "size["<<i<<"]="<<clusterNew[i].size()<<endl;
    for(uint j=0; j<clusterNew[i].size(); j++){
      // if not erased (set to -1) in merging step
      if(clusterNew[i][j]>=0){
        minDist = 500;
        minIdx = -1;
        for(uint k=0; k<clusterOld[i].size(); k++){
          // calculate Mahalanobis distance
          curDist = LineSegment::mahaDistLines(lsA.lineSegments[ clusterOld[i][k] ], lsB.lineSegments[ clusterNew[i][j] ], varEP, varDir, varGrey, varVGrey);
          //cout << " dist j="<<j<<" k="<<k<<" new="<<clusterNew[i][j]<<" old="<<clusterOld[i][k]<<endl;
          if(curDist < minDist){
            minDist = curDist;
            minIdx = k;
          }
          
        } // for k
        
        if(minIdx >= 0){
          //cout << "matched, dist="<< minDist << endl;
          // update color of new line with color of best matching
          lsB.lineSegments[ clusterNew[i][j] ].color = lsA.lineSegments[ clusterOld[i][minIdx] ].color;
          // save line in updatedB (to overcome the gaps in lsB occured through merging)
          updatedB.addLine( &lsB.lineSegments[ clusterNew[i][j] ] );
          // save matching of old line to new line
          match.addMatching( clusterOld[i][minIdx], updatedB.lineSegments.size()-1 );
        }
        else{
          // if there is no matching line, just save line in updatedB
          updatedB.addLine( &lsB.lineSegments[ clusterNew[i][j] ]);
        }
        
      } // if not erased
      else{
        cout << "...erased"<<endl;
      }
    } // for j
    
    
    
  } // for i
  cout << "Start clone\n";
  LineSegmentSet::clone(updatedB, lsB);

  //match.print();
}


/**
 * This function provides a simple tracking, gradX and gradY have to be gradient images 
 * lsA is old set, lsB new  set
 */
void LsTracker::trackingScheme2(LineSegmentSet& lsA, LineSegmentSet& lsB, Image* image, Image* gradX, Image* gradY, MatchingSet& match){
  
  // parameters
  double varEP = 40;
  double varDir = 0.01;
  double varGrey = 3;
  double varVGrey = 400;
  
  match.clear();
  
  double minDist, curDist;
  int minIdx;
  // have a look at all possible matchings
  for(uint i=0; i<lsA.lineSegments.size(); i++){
    minDist = 500;
    minIdx = -1;
    for(uint j=0; j<lsB.lineSegments.size(); j++){
      // calculate Mahalanobis distance of both lines
      //cout << "dist [" << i <<"]->[" <<j <<"]:"<<endl;
      curDist = LineSegment::mahaDistLines(lsA.lineSegments[i], lsB.lineSegments[j], varEP, varDir, varGrey, varVGrey);
      //cout << "\tdist=" << curDist << endl;
      if(curDist < minDist){
        minDist = curDist;
        minIdx = j;
      }
    }
    
    if(minIdx >= 0){
      cout << "matched, dist="<< minDist << endl;
      match.addMatching(i, minIdx);
      lsB.lineSegments[minIdx].color=lsA.lineSegments[i].color;
      //lsB.lineSegments[minIdx].color=CV_RGB(255,0,0);

    }
    else{
      cout << "not matched" << endl;
      LineSegmentSet fitSet;
      lsA.lineSegments[i].fitLineOrientation(fitSet, gradX, gradY, 5, 15, M_PI*0.1, 8, 10, 0);
      fitSet.orientLines(image);
      
      for(uint k=0; k<fitSet.lineSegments.size(); k++){
        curDist = LineSegment::mahaDistLines(lsA.lineSegments[i], fitSet.lineSegments[k], varEP, varDir, varGrey, varVGrey);
        if(curDist < minDist){
          minDist = curDist;
          minIdx = k;
        }
      }
      if(minIdx >= 0){
        cout << "\t fit matched, dist="<< minDist << endl;
        match.addMatching(i, minIdx);
        
        fitSet.lineSegments[minIdx].color=lsA.lineSegments[i].color;
        //fitSet.lineSegments[minIdx].color=CV_RGB(0,0,255);
        
        lsB.addLine(&(fitSet.lineSegments[minIdx]));
        //lsB.lineSegments[lsB.lineSegments.size()-1].color=lsA.lineSegments[i].color;
      }
    }
  }
  
  match.print();
}


/**
 * This function provides a simple tracking, _image have to be a gradient image 
 *
 */
void LsTracker::trackingScheme1(LineSegmentSet& lsA, LineSegmentSet& lsB, Image* image, MatchingSet& match){
  match.clear();
  LineSegmentSet tmpLSS;
  LineSegmentSet::clone(lsB, tmpLSS);
  
  //tmpLSS.fitLines(image, 1, 40, 15, colorImage);
  
  
  vector<CvPoint> startPts;
  vector<CvPoint> endPts;
  
  for(uint i=0; i<tmpLSS.lineSegments.size(); i++){
    tmpLSS.lineSegments[i].fitLine(image, 1, 20, 20, image);
  }
  //drawLineSegments(colorImage, tmpLSS, CV_RGB(0, 255,0) ,1);
  
  for(uint i=0; i<lsA.lineSegments.size(); i++){
    int minLS = -1;
    double min = 10;
    
    int trackedFlag=0;
    
    for(uint j=0; j<lsB.lineSegments.size(); j++){
      // distance of middle points
      double distM = sqrt((lsA.lineSegments[i].getMx() - lsB.lineSegments[j].getMx())*(lsA.lineSegments[i].getMx() - lsB.lineSegments[j].getMx()) + (lsA.lineSegments[i].getMy() - lsB.lineSegments[j].getMy())*(lsA.lineSegments[i].getMy() - lsB.lineSegments[j].getMy()));
      
      // distance of projection
      CvPoint p1, p2, p3, p4;
      p1.x = (int)lsA.lineSegments[i].getX1();
      p1.y = (int)lsA.lineSegments[i].getY1();
      p2.x = (int)lsA.lineSegments[i].getX2();
      p2.y = (int)lsA.lineSegments[i].getY2();
      p3.x = (int)lsB.lineSegments[j].getX1();
      p3.y = (int)lsB.lineSegments[j].getY1();
      p4.x = (int)lsB.lineSegments[j].getX2();
      p4.y = (int)lsB.lineSegments[j].getY2();
      double dist1=distPointVector(p1, p2, p3);
      double dist2=distPointVector(p1, p2, p4);
          
      double dist = (dist1 < dist2) ? dist1 : dist2;
      
      if( dist < min && distM < min){
        trackedFlag=1;
        lsA.lineSegments[i].color = lsB.lineSegments[minLS].color;
        match.addMatching(i,j);
      }
    }
    
    // if not tracked, try again on fitted lines
    if(!trackedFlag){
      cout << "fitting neccessary\n";
           
      for(uint j=0; j<tmpLSS.lineSegments.size(); j++){
      // distance of middle points
        double distM = sqrt((lsA.lineSegments[i].getMx() - tmpLSS.lineSegments[j].getMx())*(lsA.lineSegments[i].getMx() - tmpLSS.lineSegments[j].getMx()) + (lsA.lineSegments[i].getMy() - tmpLSS.lineSegments[j].getMy())*(lsA.lineSegments[i].getMy() - tmpLSS.lineSegments[j].getMy()));
      
      // distance of projection
        CvPoint p1, p2, p3, p4;
        p1.x = (int)lsA.lineSegments[i].getX1();
        p1.y = (int)lsA.lineSegments[i].getY1();
        p2.x = (int)lsA.lineSegments[i].getX2();
        p2.y = (int)lsA.lineSegments[i].getY2();
        p3.x = (int)lsB.lineSegments[j].getX1();
        p3.y = (int)lsB.lineSegments[j].getY1();
        p4.x = (int)lsB.lineSegments[j].getX2();
        p4.y = (int)lsB.lineSegments[j].getY2();
        double dist1=distPointVector(p1, p2, p3);
        double dist2=distPointVector(p1, p2, p4);
          
        double dist = (dist1 < dist2) ? dist1 : dist2;
      
        if( dist < min && distM < min){
          trackedFlag=1;
          lsA.lineSegments[i].color = tmpLSS.lineSegments[minLS].color;
          match.addMatching(i,j);
        }
      } // for j
    } // if not tracked yet

  } // for i
  match.print();
}
