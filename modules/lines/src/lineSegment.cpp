#include "lines/lineSegment.hpp"

using namespace jafar;
using namespace image;
using namespace std;
using namespace lines;


LineSegment::LineSegment(){
  x1=y1=x2=y2=mx=my=slope=alpha=oAlpha=eucLength=polarD=polarAlpha=covDD=covAA=covAD=0;
  covXX=covYY=covXY=covAlpha=covXA=covYA=0;
  length=0;
  param1 = -1;
  avL=avR=varL=varR=0;
  validFlag=1;
  color = CV_RGB( rand()%255, rand()%255, rand()%255);
  numberOfNeighbours=0;
  
  id = -1;
  
  histogramB = new Histogram;
  histogramD = new Histogram;
  
  // no neighboured lines
  parallels = 0;
 
  predFlag = 0;
}


LineSegment::LineSegment(int _numberOfNeighbours){
  //cout << "in ls constr mit para, numberOfNeighbours: " << numberOfNeighbours << endl;
  x1=y1=x2=y2=mx=my=slope=alpha=oAlpha=eucLength=polarD=polarAlpha=covDD=covAA=covAD=0;
  length=0;
  avL=avR=varL=varR=0;
  validFlag=1;
  color = CV_RGB( rand()%255, rand()%255, rand()%255);
  
  id = -1;
  
  numberOfNeighbours = _numberOfNeighbours;
  histogramB = new Histogram;
  histogramD = new Histogram;


  if( numberOfNeighbours){
    // neighboured lines
    parallels = new LineSegment*[numberOfNeighbours];
    for(int i=0; i<numberOfNeighbours; i++){
      parallels[i] = new LineSegment;
    }
  }
}



LineSegment::LineSegment(const LineSegment& ls){
  //cout <<"in ls copy const, numberOfNeighbours: " << ls.numberOfNeighbours << endl;
  numberOfNeighbours = ls.numberOfNeighbours;

  color = ls.color;
  parallels = 0;
  histogramB = new Histogram(*(ls.histogramB));
  histogramD = new Histogram(*(ls.histogramD));

  avL = ls.avL;
  avR = ls.avR;
  varL = ls.varL;
  varR = ls.varR;
  eucLength = ls.eucLength;
  length = ls.length;
  alpha = ls.alpha;
  oAlpha = ls.oAlpha;
  polarD = ls.polarD;
  polarAlpha = ls.polarAlpha;
  covDD = ls.covDD;
  covAA = ls.covAA;
  covAD = ls.covAD;
  
  covXX = ls.covXX;
  covXY = ls.covXY;
  covYY = ls.covYY;
  covAlpha = ls.covAlpha;
  covXA = ls.covXA;
  covYA = ls.covYA;
  
  covXX_M = ls.covXX_M;
  covXY_M = ls.covXY_M;
  covYY_M = ls.covYY_M;
  covXA_M = ls.covXA_M;
  covYA_M = ls.covYA_M;
  
  u = ls.u;
  v = ls.v;
  
  rhoPred = ls.rhoPred;     
  thetaPred = ls.thetaPred;   
  covRRpred = ls.covRRpred;
  covRTpred = ls.covRTpred;
  covTTpred = ls.covTTpred;
  predFlag = ls.predFlag;
  
  x1 = ls.x1;
  y1 = ls.y1;
  x2 = ls.x2;
  y2 = ls.y2;
  
  mx = ls.mx;
  my = ls.my;
  
  id = ls.id;
  
  param1 = ls.param1;
  parameters.clear();
  for(uint i=0; i<ls.parameters.size(); i++){
    parameters.push_back(ls.parameters[i]);
  }
  
  predictor = ls.predictor;
  
  validFlag = ls.validFlag;
  
  greyscale.clear();
  gradientscale.clear();
  laplacescale.clear();
  
  for(uint i=0; i<ls.greyscale.size(); i++){
    greyscale.push_back(ls.greyscale[i]);
  }
  for(uint i=0; i<ls.gradientscale.size(); i++){
    gradientscale.push_back(ls.gradientscale[i]);
  }
  for(uint i=0; i<ls.laplacescale.size(); i++){
    laplacescale.push_back(ls.laplacescale[i]);
  }
  
  greyspace.clear();
  for(uint i=0; i<ls.greyspace.size(); i++){
    greyspace.push_back(ls.greyspace[i]);
  }
  greyspaceL.clear();
  for(uint i=0; i<ls.greyspaceL.size(); i++){
    greyspaceL.push_back(ls.greyspaceL[i]);
  }
  greyspaceR.clear();
  for(uint i=0; i<ls.greyspaceR.size(); i++){
    greyspaceR.push_back(ls.greyspaceR[i]);
  }
  
  contourPoints.clear();
  for(uint i=0; i<ls.contourPoints.size(); i++){
    vector<CvPoint> tmp;
    for(uint j=0; j<ls.contourPoints[i].size(); j++){
      tmp.push_back(ls.contourPoints[i][j]);
    }
    contourPoints.push_back(tmp);
  }
  
  if( numberOfNeighbours ){
    // neighboured lines
    parallels = new LineSegment*[numberOfNeighbours];
    for(int i=0; i<numberOfNeighbours; i++){
      parallels[i] = new LineSegment;
      
      for(uint j=0; j<ls.parallels[i]->greyscale.size(); j++){
        parallels[i]->greyscale.push_back(ls.parallels[i]->greyscale[j]);
      }
      for(uint j=0; j<ls.parallels[i]->gradientscale.size(); j++){
        parallels[i]->gradientscale.push_back(ls.parallels[i]->gradientscale[j]);
      }
      for(uint j=0; j<ls.parallels[i]->laplacescale.size(); j++){
        parallels[i]->laplacescale.push_back(ls.parallels[i]->laplacescale[j]);
      }
    }
  }
  //setPoints(ls.x1, ls.y1, ls.x2, ls.y2);

}


LineSegment::~LineSegment(){
  // delete parallels
  if(parallels){
    for(int i=0; i<numberOfNeighbours; i++){
      delete parallels[i];
    }
    delete parallels;
  }
  if(histogramB) delete histogramB;
  if(histogramD) delete histogramD;
}


LineSegment &LineSegment::operator=(const LineSegment &ls){
  if (this != &ls){
    
    if(parallels){
      for(int i=0; i<numberOfNeighbours; i++){
        delete parallels[i];
      }
      delete parallels;
    }
    
    numberOfNeighbours = ls.numberOfNeighbours;
    color = ls.color;
    avL = ls.avL;
    avR = ls.avR;
    varL = ls.varL;
    varR = ls.varR;
    eucLength = ls.eucLength;
    length = ls.length;
    alpha = ls.alpha;
    oAlpha = ls.oAlpha;
    polarD = ls.polarD;
    polarAlpha = ls.polarAlpha;
    
    covDD = ls.covDD;
    covAA = ls.covAA;
    covAD = ls.covAD;
    
    covXX = ls.covXX;
    covXY = ls.covXY;
    covYY = ls.covYY;
    covAlpha = ls.covAlpha;
    covXA = ls.covXA;
    covYA = ls.covYA;
    
    covXX_M = ls.covXX_M;
    covXY_M = ls.covXY_M;
    covYY_M = ls.covYY_M;
    covXA_M = ls.covXA_M;
    covYA_M = ls.covYA_M;
    
    u = ls.u;
    v = ls.v;
    
    x1 = ls.x1;
    y1 = ls.y1;
    x2 = ls.x2;
    y2 = ls.y2;
    
    mx = ls.mx;
    my = ls.my;
    
    id = ls.id;
    
    param1 = ls.param1;
    parameters.clear();
    for(uint i=0; i<ls.parameters.size(); i++){
      parameters.push_back(ls.parameters[i]);
    }
    
    validFlag = ls.validFlag;
    
    predictor = ls.predictor;
    
    parallels = 0;  
    if(histogramB) delete histogramB;
    histogramB = new Histogram(*(ls.histogramB));
    if(histogramD) delete histogramD;
    histogramD = new Histogram(*(ls.histogramD));
    
    greyscale.clear();
    gradientscale.clear();
    laplacescale.clear();
    
    for(uint i=0; i<ls.greyscale.size(); i++){
      greyscale.push_back(ls.greyscale[i]);
    }
    for(uint i=0; i<ls.gradientscale.size(); i++){
      gradientscale.push_back(ls.gradientscale[i]);
    }
    for(uint i=0; i<ls.laplacescale.size(); i++){
      laplacescale.push_back(ls.laplacescale[i]);
    }
    
    greyspace.clear();
    for(uint i=0; i<ls.greyspace.size(); i++){
      greyspace.push_back(ls.greyspace[i]);
    }
    greyspaceL.clear();
    for(uint i=0; i<ls.greyspaceL.size(); i++){
      greyspaceL.push_back(ls.greyspaceL[i]);
    }
    greyspaceR.clear();
    for(uint i=0; i<ls.greyspaceR.size(); i++){
      greyspaceR.push_back(ls.greyspaceR[i]);
    }
    
    contourPoints.clear();
    for(uint i=0; i<ls.contourPoints.size(); i++){
      vector<CvPoint> tmp;
      for(uint j=0; j<ls.contourPoints[i].size(); j++){
        tmp.push_back(ls.contourPoints[i][j]);
      }
      contourPoints.push_back(tmp);
    }
    
    if( numberOfNeighbours ){
      // neighboured lines
      parallels = new LineSegment*[numberOfNeighbours];
      for(int i=0; i<numberOfNeighbours; i++){
        parallels[i] = new LineSegment;
        
        for(uint j=0; j<ls.parallels[i]->greyscale.size(); j++){
          parallels[i]->greyscale.push_back(ls.parallels[i]->greyscale[j]);
        }
        for(uint j=0; j<ls.parallels[i]->gradientscale.size(); j++){
          parallels[i]->gradientscale.push_back(ls.parallels[i]->gradientscale[j]);
        }
        for(uint j=0; j<ls.parallels[i]->laplacescale.size(); j++){
          parallels[i]->laplacescale.push_back(ls.parallels[i]->laplacescale[j]);
        }
      }
    }
    //setPoints(ls.x1, ls.y1, ls.x2, ls.y2);
  }
  return *this;

}


double LineSegment::assignContourPts(Image* image){
  //////// clear old contourPoints /////////
  for(uint i=0; i<contourPoints.size(); i++){
    for(uint j=0; j<contourPoints[i].size(); j++){
      contourPoints[i].clear();
    }
  }
  contourPoints.clear();
  
  
  /////// assign new contour points /////////
  CvLineIterator lineIt;
  CvPoint p1,p2;

  p1.x = (int)x1;
  p1.y = (int)y1;
  p2.x = (int)x2;
  p2.y = (int)y2;
        
  int l = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
      
  // count pixels on line, that are next to a contour pixel
  int count=0;
  
  // for each pixel on the line
  for(int j=0; j<l;j++){
    // calc point coordinates
    int off, x, y;
    IplImage* tmp = *image;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
            
    //cout << "\tpoint: x="<<x << " y=" << y << endl;
    
    // if line pixel is a contour pixel, use this, else insert all contour pixels around it ( euclidean dist=1)
    vector<CvPoint> tmpContourPts;
    CvPoint tmpPt;
            
    tmpPt.x=x;
    tmpPt.y=y;
    CvScalar s;
    s=cvGet2D(*image,tmpPt.y,tmpPt.x);
    if(s.val[0] >0){
      tmpContourPts.push_back(tmpPt);
      //cout << "\t\tcontour point: x="<<tmpPt.x << " y=" << tmpPt.y << endl;
    }
    else{
      // check for all neighboured pixels if they are on a contour and insert in contourPoints accordingly
      tmpPt.x=x+1;
      tmpPt.y=y;
      if( tmpPt.x>=0 && tmpPt.x<image->width() && tmpPt.y>=0 && tmpPt.y<image->height()){
        CvScalar s;
        s=cvGet2D(*image,tmpPt.y,tmpPt.x);
        if(s.val[0] >0){
          tmpContourPts.push_back(tmpPt);
          //cout << "\t\tcontour point: x="<<tmpPt.x << " y=" << tmpPt.y << endl;
        }
      }
      tmpPt.x=x-1;
      tmpPt.y=y;
      if( tmpPt.x>=0 && tmpPt.x<image->width() && tmpPt.y>=0 && tmpPt.y<image->height()){
        CvScalar s;
        s=cvGet2D(*image,tmpPt.y,tmpPt.x);
        if(s.val[0] >0){
          tmpContourPts.push_back(tmpPt);
          //cout << "\t\tcontour point: x="<<tmpPt.x << " y=" << tmpPt.y << endl;
        }
      }
      tmpPt.x=x;
      tmpPt.y=y+1;
      if( tmpPt.x>=0 && tmpPt.x<image->width() && tmpPt.y>=0 && tmpPt.y<image->height()){
        CvScalar s;
        s=cvGet2D(*image,tmpPt.y,tmpPt.x);
        if(s.val[0] >0){
          tmpContourPts.push_back(tmpPt);
          //cout << "\t\tcontour point: x="<<tmpPt.x << " y=" << tmpPt.y << endl;
        }
      }
      tmpPt.x=x;
      tmpPt.y=y-1;
      if( tmpPt.x>=0 && tmpPt.x<image->width() && tmpPt.y>=0 && tmpPt.y<image->height()){
        CvScalar s;
        s=cvGet2D(*image,tmpPt.y,tmpPt.x);
        if(s.val[0] >0){
          tmpContourPts.push_back(tmpPt);
          //cout << "\t\tcontour point: x="<<tmpPt.x << " y=" << tmpPt.y << endl;
        }
      }
    }
        
    // if there is no contur point around this point, insert the point itself (hapens, but very seldom, only possible for special contours and often for the strange connecting lines)
    if(tmpContourPts.size()==0){
      tmpPt.x=x;
      tmpPt.y=y;
      tmpContourPts.push_back(tmpPt);
    }
    else{
      count++;
    }
            
    contourPoints.push_back(tmpContourPts);
    CV_NEXT_LINE_POINT(lineIt);
  } // for i (each pixel of line)

  return (double) count / l;
}



bool LineSegment::fitLine(Image* image, int gradientimage, int nPoints, int pDist, Image* colorImage){
  
  //cout << "line points(): x1="<<x1<<" y1="<<y1<<" x2="<<x2<<" y2="<<y2<<endl; 
  CvSize size;
  size.width = image->width();
  size.height = image->height();
  
  // calculate gradient image if neccessary
  if(!gradientimage){
    calcGradientImage(image);
  }
  
  vector<CvPoint> points;
  
  int dx = (int) (x1-x2);
  int dy = (int) (y1-y2);
  if (abs(dx) < pDist || abs(dy) < pDist) { 
    dx *= 2; 
    dy *= 2;
  }
  //cout << "dx="<<dx << " dy="<<dy << endl;

  
  
  CvLineIterator lineIt;
  CvPoint p1,p2;
  p1.x = (int) x1;
  p1.y = (int) y1;
  p2.x = (int) x2;
  p2.y = (int) y2;
  
  int l = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
  double dist = (double)l / (nPoints-1);
  double state = 0;
  //cout << "l: " << l << " dist: " << dist<< endl;
  int count=0;
  for( int i = 0; i < nPoints; i++ ){
    // calc point coordinates
    int off, x, y;
    IplImage* tmp = *image;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
    
    if(x<0 || x>=image->width() || y<0 || y>=image->height()) break;
    
    // origin point
    CvPoint p0;
    p0.x = x;
    p0.y = y;
    //cout << "origin point: x="<<x << " y=" << y << endl;
    
    // coordinates of perpendicular ending points
    //cout << "dx="<<dx << " dy="<<dy << endl;
    p1.x = x-dy;
    p1.y = y+dx;
    p2.x = x+dy;
    p2.y = y-dx;
    
    //cout << "pPoints(befor): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
    cvClipLine(size, &p1, &p2);
    //cout << "pPoints(after): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
    
    int max = 0, maxX = x, maxY = y;
    // go from origin to p1
    CvLineIterator linePara;
    int lP = cvInitLineIterator( *image, p0, p1, &linePara, 8, 0);
    CV_NEXT_LINE_POINT(linePara);
    for( int j=0; j < pDist && j < (lP-1); j++){
      if(linePara.ptr[0] > max){
        max = linePara.ptr[0];
         // calc point coordinates
        int offset;
        IplImage* tmp = *image;
        offset = linePara.ptr - (uchar*)(tmp->imageData);
        maxY = offset/tmp->widthStep;
        maxX = (offset - maxY*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      }
      CV_NEXT_LINE_POINT(linePara);
    }

    // store resulting point
    CvPoint result;
    result.x = maxX;
    result.y = maxY;
    points.push_back(result);
    
    // go from origin to p2
    max = 0, maxX = x, maxY = y;

    lP = cvInitLineIterator( *image, p0, p2, &linePara, 8, 0);
    CV_NEXT_LINE_POINT(linePara);
    for( int j=0; j < pDist && j < (lP-1); j++){
      if(linePara.ptr[0] > max){
        max = linePara.ptr[0];
        // calc point coordinates
        int offset;
        IplImage* tmp = *image;
        offset = linePara.ptr - (uchar*)(tmp->imageData);
        maxY = offset/tmp->widthStep;
        maxX = (offset - maxY*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      }
      CV_NEXT_LINE_POINT(linePara);
    }
    
    // store resulting point
    result.x = maxX;
    result.y = maxY;
    points.push_back(result);
    //cout << "fit point: x=" << maxX << " y=" << maxY << endl; 
    // go to next point
    for( double j=count; j<(state+dist); j++){
      CV_NEXT_LINE_POINT(lineIt);
      count++;
    }
    state += dist;
    //cout << "state: "<<state << "  count: "<<count<<endl;
  }
  //cout << "count: " << count << endl;
  
  if(colorImage){
    CvScalar color=CV_RGB(rand()%255, rand()%255, rand()%255);
    cout << "rand: "<<rand()%255 << endl;
    for(uint i=0; i< points.size(); i++){
      //if(i!=0) cvLine( *colorImage,points[i], points[i-1], CV_RGB(0,255,255), 1, 8, 0);
      CvPoint tmpPoint;
      tmpPoint.x=points[i].x;
      tmpPoint.y=points[i].y;
      cvLine( *colorImage,points[i], tmpPoint, color, 1, 8, 0);
    }
  }
  
  /////// RANSAC ////////
  double threshD = 4;
  double d = 0;
  double maxD = nPoints/2;
  int bestA=0, bestB=0;
  
  for(int i=0; i<nPoints*2; i++){
    d=0;
    int indA = rand()%points.size();
    int indB = rand()%points.size();
    
    int startPoint = -1;
    int endPoint = -1;
    
    for(uint j=0; j<points.size(); j++){
      if( distPointVector(points[indA], points[indB],points[j]) < threshD ){ 
        d++;
        if( distPointVector(points[indA], points[indB],points[j]) <= 1){
          if(startPoint==-1) startPoint = j;
          endPoint = j;
        }
      }
    }
    if( d>maxD ){
      maxD=d;
      bestA=startPoint;
      bestB=endPoint;
    }
    //cout << "counter: " << d << endl;
  }
  //cvLine( *colorImage,points[bestA], points[bestB], CV_RGB(255,0,255), 3, 8, 0);
  if(maxD>nPoints/2){
    setPoints(points[bestA].x,points[bestA].y,points[bestB].x,points[bestB].y);
  }
  return 1;
}


int LineSegment::fitLineCandidates(Image* image, int gradientimage,vector<CvPoint>& startPts, vector<CvPoint>& endPts, int nPoints, int pDist, int nIter, Image* colorImage){
  // dummy section to prevent warnings in gcc
  if((startPts.size() || endPts.size()) && nIter) {
    startPts.clear();
    endPts.clear();
  }
  
  //cout << "line points(): x1="<<x1<<" y1="<<y1<<" x2="<<x2<<" y2="<<y2<<endl; 
  CvSize size;
  size.width = image->width();
  size.height = image->height();
  
  // calculate gradient image if neccessary
  if(!gradientimage){
    calcGradientImage(image);
  }
  
  vector<CvPoint> points;
  
  int dx = (int) (x1-x2);
  int dy = (int) (y1-y2);
  if (abs(dx) < pDist || abs(dy) < pDist) { 
    dx *= 2; 
    dy *= 2;
  }
  //cout << "dx="<<dx << " dy="<<dy << endl;

  
  
  CvLineIterator lineIt;
  CvPoint p1,p2;
  p1.x = (int) x1;
  p1.y = (int) y1;
  p2.x = (int) x2;
  p2.y = (int) y2;
  
  int l = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
  double dist = (double)l / (nPoints-1);
  double state = 0;
  //cout << "l: " << l << " dist: " << dist<< endl;
  int count=0;
  for( int i = 0; i < nPoints; i++ ){
    // calc point coordinates
    int off, x, y;
    IplImage* tmp = *image;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
    
    if(x<0 || x>=image->width() || y<0 || y>=image->height()) break;
    
    // origin point
    CvPoint p0;
    p0.x = x;
    p0.y = y;
    //cout << "origin point: x="<<x << " y=" << y << endl;
    
    // coordinates of perpendicular ending points
    //cout << "dx="<<dx << " dy="<<dy << endl;
    p1.x = x-dy;
    p1.y = y+dx;
    p2.x = x+dy;
    p2.y = y-dx;
    
    //cout << "pPoints(befor): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
    cvClipLine(size, &p1, &p2);
    //cout << "pPoints(after): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
    
    // go from origin to p1
    CvLineIterator linePara;
    int lP = cvInitLineIterator( *image, p0, p1, &linePara, 8, 0);
    
    // secondLast stores the gradient 2 pixels ago, last stores gradient of last point, lastX and lastY store its coordinates
    int secondLast=0, last = linePara.ptr[0], lastX = x, lastY = y;
    int incFlag;
    
    CV_NEXT_LINE_POINT(linePara);
    for( int j=0; j < pDist && j < (lP-1); j++){
      if(last>secondLast) incFlag=1;
      else incFlag=0;
      
      // if last is local maximum (greater gradient than second last and than current pixel) insert in list
      //if(last>=thresh && last>(secondLast+gradDist) && last>(linePara.ptr[0]+gradDist)){
      if(incFlag && last>=linePara.ptr[0]){
        // store resulting point
        CvPoint result;
        result.x = lastX;
        result.y = lastY;
        points.push_back(result);
      }
      
      // calc point coordinates
      int offset;
      IplImage* tmp = *image;
      offset = linePara.ptr - (uchar*)(tmp->imageData);
      lastY = offset/tmp->widthStep;
      lastX = (offset - lastY*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      
      secondLast=last;
      last=linePara.ptr[0];
      
      CV_NEXT_LINE_POINT(linePara);
    }

    
    
    // go from origin to p2

    lP = cvInitLineIterator( *image, p0, p2, &linePara, 8, 0);
    // secondLast stores the gradient 2 pixels ago, last stores gradient of last point, lastX and lastY store its coordinates
    secondLast=0, last = linePara.ptr[0], lastX = x, lastY = y;

    CV_NEXT_LINE_POINT(linePara);
    for( int j=0; j < pDist && j < (lP-1); j++){
      if(last>secondLast) incFlag=1;
      else incFlag=0;
      // if last is local maximum (greater gradient than second last and than current pixel) insert in list
      //if(last>=thresh && last>(secondLast+gradDist) && last>(linePara.ptr[0]+gradDist)){
      if(incFlag && last>=linePara.ptr[0]){
        // store resulting point
        CvPoint result;
        result.x = lastX;
        result.y = lastY;
        points.push_back(result);
      }
      
      // calc point coordinates
      int offset;
      IplImage* tmp = *image;
      offset = linePara.ptr - (uchar*)(tmp->imageData);
      lastY = offset/tmp->widthStep;
      lastX = (offset - lastY*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      
      secondLast=last;
      last=linePara.ptr[0];
      
      CV_NEXT_LINE_POINT(linePara);
    }
    //cout << "fit point: x=" << maxX << " y=" << maxY << endl; 
    // go to next point
    for( double j=count; j<(state+dist); j++){
      CV_NEXT_LINE_POINT(lineIt);
      count++;
    }
    state += dist;
    //cout << "state: "<<state << "  count: "<<count<<endl;
  }
  //cout << "count: " << count << endl;
  
 
  
  /////// RANSAC ////////
  double threshD = 0.9;
  double d = 0;
  double maxD = nPoints*0.8;
  int bestA=0, bestB=0;
  
  CvPoint oriSt, oriEnd;
  oriSt.x = (int) x1;
  oriSt.y = (int) y1;
  oriEnd.x = (int) x2;
  oriEnd.y = (int) y2;
  double distOri=distPtPt(oriSt, oriEnd);
  
  for(uint i=0; i<points.size(); i++){
    d=0;
    int indA = rand()%points.size();
    int indB = rand()%points.size();
    
    //if( distPtPt(points[indA], points[indB])>20 && abs(signedDistPointVector(oriSt, oriEnd, points[indA])+signedDistPointVector(oriSt, oriEnd, points[indB]))<5) { 
      
      
    int startPoint = -1;
    int endPoint = -1;
      
    for(uint j=0; j<points.size(); j++){
      if( distPointVector(points[indA], points[indB],points[j]) <= threshD ){ 
        d++;
        if( distPointVector(points[indA], points[indB],points[j]) <= 1){
          if(startPoint==-1) startPoint = j;
          endPoint = j;
        }
      }
    }
      
    double distPts=distPtPt(points[startPoint], points[endPoint]);

    double mult=distPts/distOri;
    if( d>maxD*( mult>1 ? mult : 1) ){
        //maxD=d;
      bestA=startPoint;
      bestB=endPoint;
        //cvLine( *colorImage,points[bestA], points[bestB], CV_RGB(255,0,255), 1, 8, 0);
    }
      //cout << "counter: " << d << endl;
    //}
  }
  if(colorImage){
    CvScalar color=CV_RGB(rand()%255, rand()%255, rand()%255);
    cout << "rand: "<<rand()%255 << endl;
    for(uint i=0; i< points.size(); i++){
      //if(i!=0) cvLine( *colorImage,points[i], points[i-1], CV_RGB(0,255,255), 1, 8, 0);
      CvPoint tmpPoint;
      tmpPoint.x=points[i].x;
      tmpPoint.y=points[i].y;
      cvLine( *colorImage,points[i], tmpPoint, color, 1, 8, 0);
    }
  }
  //cvLine( *colorImage,points[bestA], points[bestB], CV_RGB(255,0,255), 3, 8, 0);
  if(maxD>nPoints/2){
    setPoints(points[bestA].x,points[bestA].y,points[bestB].x,points[bestB].y);
  }
  return 1; 
}


bool LineSegment::fitLineOrientation(LineSegmentSet& lsSet, Image* gradX, Image* gradY, int nPoints, int pDist, double aDist, double minLength, double extend, Image* colorImage){
  // RANSAC                               1
  // LINEGROW with probabilistic merging  2
  int mode = 2;
  
  vector<CvPoint> points;
  vector<double> pointGrads;
  
  CvPoint p1,p2;
  int dx = (int) (x1-x2);
  int dy = (int) (y1-y2);
  
  // extend the line in both directions for better fitting capability if the line is shifted in this direction
  if(extend){
    double norm = sqrt(dx*dx+dy*dy);
    double extendDx = extend * dx/norm;
    double extendDy = extend * dy/norm;
    p1.x = (int) (x1 + extendDx);
    p1.y = (int) (y1 + extendDy);
    p2.x = (int) (x2 - extendDx);
    p2.y = (int) (y2 - extendDy);
  }
  else{
    p1.x = (int) x1;
    p1.y = (int) y1;
    p2.x = (int) x2;
    p2.y = (int) y2;
  }
  
  // search gradient maxima perpendicular to line
  searchGradMax(p1, p2, points, pointGrads, gradX, gradY, nPoints, pDist);

  vector<LineSegment> lines;
  vector< vector<int> > bitlines;

  switch(mode){
    case 1:
      interpolateLineRansac(lines, points, pointGrads, gradX, gradY, nPoints, nPoints*20, 0.3, minLength, colorImage);
      break;
    case 2:
      // works only with up to 32 search points in total (parameter<=16)!!!
      interpolateLineGrowline(lines, points, pointGrads, gradX, gradY, nPoints, minLength, aDist, colorImage);
      break;
  }
  
  // create LineSegmentSet from lines
  for(uint i=0; i<lines.size(); i++){
    if(lines[i].getX1() != 0 || lines[i].getY1() != 0 || lines[i].getX2() != 0 || lines[i].getY2() != 0){
      lsSet.addLine(&(lines[i]));
      CvScalar color = CV_RGB(0,255,255);
      if(colorImage) lines[i].drawLine(colorImage, color,1);
    }
  }

  // draw all points
  if(colorImage){
    CvScalar color=CV_RGB(rand()%255, rand()%255, rand()%255);
    for(uint i=0; i< points.size(); i++){
      cvLine( *colorImage,points[i], points[i], color, 3, 8, 0);
    }
  }
  
  return 1;
}



void LineSegment::searchGradMax(CvPoint p1, CvPoint p2, vector<CvPoint>& points, vector<double>& pointGrads, Image* gradX, Image* gradY, int nPoints, double pDist){

  // values to calculate perpendicular ending points
  int dx = (int) (x1-x2);
  int dy = (int) (y1-y2);
  if (abs(dx) < pDist || abs(dy) < pDist) { 
    dx *= 2; 
    dy *= 2;
  }
  // gradient orientation of the current line 
  // an other way: vector of gradient
  double lineGradDirX = -dy;
  double lineGradDirY = dx;
  
  // normalise gradient vector of the line
  double gradNorm = sqrt(lineGradDirX*lineGradDirX + lineGradDirY*lineGradDirY);
  lineGradDirX /= gradNorm;
  lineGradDirY /= gradNorm;
  
  // size of the image, gradX and gradY should have the same size, gradX is taken as reference
  CvSize size;
  size.width = gradX->width();
  size.height = gradX->height();
  
  // init line iterator on gradX after clipping (extended) ending points of the line at image borders
  if(!cvClipLine(size, &p1, &p2)) return;
  CvLineIterator lineIt;
  int l = cvInitLineIterator( *gradX, p1, p2, &lineIt, 8, 0);

  // neccessary values to distribute the origin points proper to the discret image pixels
  double dist = (double)l / (nPoints-1);  // distance to go up to next origin point on the line (not integer!)
  double state = 0;                       // distance we should have traversed on the line
  int count=0;                            // allready traversed distance on the line
  
  // prepare fast access to pixels of gradY
  //int step = (*gradY)->widthStep/sizeof(uchar);
  //uchar* gradY_data = (uchar *)img->imageData;
  //data[i*step+j] = 111;
  CvScalar s;
  
  
  // determine nPoint-times an origin point and search perpendicular for gradient maximas in both directions
  for( int i = 0; i < nPoints; i++ ){
    // calc point coordinates of current origin point
    int off, x, y;
    IplImage* tmp = *gradX;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar)*2 /* size of pixel */);
    
    // in some cases the last point is no longer in the image, just scip these case
    if(x<0 || x>=gradX->width() || y<0 || y>=gradX->height()) break;
    
    // origin point
    CvPoint p0;
    p0.x = x;
    p0.y = y;

    // coordinates of perpendicular ending points
    p1.x = x-dy;
    p1.y = y+dx;
    p2.x = x+dy;
    p2.y = y-dx;
    
    // clip perpendicular line at image boarders, if it's completly outside, skip
    //cout << "pPoints(befor): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
    if(!cvClipLine(size, &p1, &p2)){
      break;
    }
    //cout << "pPoints(after): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
    
    int offset, tmpX, tmpY;   // variables to calculate coordinates from iterator

    int maxX = x, maxY = y;  // variables for coordinates of maximum gradient, initializd with origin 
    double max = 0;         // variable for temporary maximum gradient on this perpendicular line
    // go from origin to p1
    CvLineIterator linePara;
    int lP = cvInitLineIterator( *gradX, p0, p1, &linePara, 8, 0);
    CV_NEXT_LINE_POINT(linePara);
    
    double gradXval, gradYval, dGrad;
    
    for( int j=0; j < pDist && j < (lP-1); j++){
      // calculate point coordinates
      offset = linePara.ptr - (uchar*)(tmp->imageData);
      tmpY = offset/tmp->widthStep;
      tmpX = (offset - tmpY*tmp->widthStep)/(sizeof(uchar)*2 /* size of pixel */);
      
      // calculate directed gradient |g| = <lineGradDir, [gradX gradY]> / |[gradX gradY]|
      s =cvGet2D(*gradX,tmpY,tmpX);
      gradXval = s.val[0];
      s=cvGet2D(*gradY,tmpY,tmpX);
      gradYval = s.val[0];
      
      dGrad = (gradXval*lineGradDirX + gradYval*lineGradDirY);
      //cout << "gradXval=" << gradXval << " gradYval=" << gradYval <<" dGrad="<<dGrad<<" lineGradDirX="<< lineGradDirX << " lineGradDirY=" << lineGradDirY <<  endl;

      if(dGrad > max){
        max = dGrad;
        maxY = tmpY;
        maxX = tmpX;
      }
      CV_NEXT_LINE_POINT(linePara);
    }

    // store resulting point
    CvPoint result;
    result.x = maxX;
    result.y = maxY;
    points.push_back(result);
    pointGrads.push_back(max);
    

    // go from origin to p2
    max = 0, maxX = x, maxY = y;
    
    lP = cvInitLineIterator( *gradX, p0, p2, &linePara, 8, 0);
    CV_NEXT_LINE_POINT(linePara);
    for( int j=0; j < pDist && j < (lP-1); j++){
      // calculate point coordinates
      offset = linePara.ptr - (uchar*)(tmp->imageData);
      tmpY = offset/tmp->widthStep;
      tmpX = (offset - tmpY*tmp->widthStep)/(sizeof(uchar)*2 /* size of pixel */);
      
      // calculate directed gradient |g| = <lineGradDir, [gradX gradY]> / |[gradX gradY]|
      s=cvGet2D(*gradX,tmpY,tmpX);
      gradXval = s.val[0];
      s=cvGet2D(*gradY,tmpY,tmpX);
      gradYval = s.val[0];
      
      dGrad = (gradXval*lineGradDirX + gradYval*lineGradDirY);
    
      if(dGrad > max){
        max = dGrad;
        maxY = tmpY;
        maxX = tmpX;
      }
      CV_NEXT_LINE_POINT(linePara);
    }
    
    // store resulting point
    result.x = maxX;
    result.y = maxY;
    points.push_back(result);
    pointGrads.push_back(max);

    // go to next point
    for( double j=count; j<(state+dist); j++){
      CV_NEXT_LINE_POINT(lineIt);
      count++;
    }
    state += dist;
  }
}


void LineSegment::interpolateLineRansac(vector<LineSegment> lines, vector<CvPoint>& points, vector<double>& pointGrads, Image* gradX, Image* gradY, int nPoints, int nIt, double minInlierRate, double minLength, Image* colorImage){
  
  //vector<LineSegment> lines;
  //vector< vector<int> > bitlines;
  
  /////// RANSAC ////////
  double threshD = 0.9;
  double d = 0;
  double maxD = nPoints * minInlierRate;
  int bestA=0, bestB=0;
  
  for(int i=0; i<nIt; i++){
    d=0;
    int indA = rand()%points.size();
    int indB = rand()%points.size();
    
    int startPoint = -1;
    int endPoint = -1;
    
    for(uint j=0; j<points.size(); j++){
      if( distPointVector(points[indA], points[indB],points[j]) < threshD ){ 
        d++;
        if( distPointVector(points[indA], points[indB],points[j]) <= 1){
          if(startPoint==-1) startPoint = j;
          endPoint = j;
        }
      }
    }
    if( d>=maxD ){
      
      double avEPGrad = (pointGrads[startPoint]+pointGrads[endPoint]) / 2;
      if( abs(validLine(gradX, gradY, points[startPoint], points[endPoint], avEPGrad*0.75,0.2))>avEPGrad*0.75){
        LineSegment newLine;
        newLine.setPoints(points[startPoint].x,points[startPoint].y,points[endPoint].x,points[endPoint].y);
        lines.push_back(newLine);
        bestA=startPoint;
        bestB=endPoint;
      }
    }
    //cout << "counter: " << d << endl;
  }
  
  // draw lines on colorImage
  if(colorImage){
    for(uint i=0; i<lines.size(); i++){
      lines[i].drawLine(colorImage, CV_RGB(0,255,255),1);
    }
  }
  
  // if line contains enough points, set it as new line
  if(maxD>nPoints/2){
    setPoints(points[bestA].x,points[bestA].y,points[bestB].x,points[bestB].y);
  }
  
  
}


void LineSegment::interpolateLineGrowline(vector<LineSegment>& lines, vector<CvPoint>& points, vector<double>& pointGrads, Image* gradX, Image* gradY, int nPoints, double minLength, double aDist, Image* colorImage){

  double minGrad = 25;
  double minLength2 = minLength*minLength;    // square, to avoid squareroots later
  
// GROWLINE
  double threshD = 0.25;         // maximal distance of point projection to belong to a line
  
  // size of the image, gradX and gradY should have the same size, gradX is taken as reference
  CvSize size;
  size.width = gradX->width();
  size.height = gradX->height();
  
    // slope of origin line
  int dx = (int) (x2-x1);
  int dy = (int) (y2-y1);
  double alphaOri = atan2(dy,dx);
    
    //int startPoint, endPoint;
  double alphaTmp, diff, otherDiff, dist;
        

    // container that holds a integer for each line that indicates bitwise for each point if it is contained in this line (bit i of line l is set if point i belongs to line l)
  uint pointLines[32];
    
    // container for lines themselves, each line is vector of points
  uint linePoints[32];
    
    // initialise
  for(uint i=0; i<32; i++){
    pointLines[i] = 0;
    linePoints[i] = 0;
  }
    
    // number of lines
  unsigned char lineCount=0;
    
  uint bitRep, newLinePoints;
  bool collinearFlag=0;
    
    // storage for polar coordinates of the line
  double tmpPolarAlpha, tmpPolarD, cosTmpPolarAlpha, sinTmpPolarAlpha, m;
  CvPoint cosOrigin;
  cosOrigin.x = 0;
  cosOrigin.y = 0;
    
    // have a look at all small lines (each point connected to next neighbour on same side and on the other)
  for(int i=0; i<(int)points.size()-2; i++){
    if(pointGrads[i]>=minGrad){
      int sideOffset = 1 - i%2;
      for (int j = i+sideOffset+1; j<=i+sideOffset+2 && j< (int)points.size(); j++){
        if(pointGrads[j]>=minGrad){
          collinearFlag=0;
            // distance of orientations has to be smaller/equal aDist
          dx = points[j].x-points[i].x;
          dy = points[j].y-points[i].y;
          alphaTmp = atan2(dy, dx);       // MAYBE POSSIBLE TO ELIMINATE (INVERSION?)
          diff = abs(alphaOri-alphaTmp);
          otherDiff = 2*M_PI-diff;
          dist = (diff < otherDiff) ? diff : otherDiff; 
            
          if( dist <= aDist){
              // polar coordinates for faster point distance calculation
              // (calculation of polar angle is complicate)
              
              // alphaTmp should be in range [-pi/2, pi/2]
            if(alphaTmp<-M_PI/2) alphaTmp += M_PI;
            else if(alphaTmp>M_PI/2) alphaTmp -= M_PI;
              
            if(points[i].x==points[j].x){
              if(points[i].x >= 0) 
                tmpPolarAlpha = 0;
              else 
                tmpPolarAlpha = M_PI;
            }
            else if(points[i].y==points[j].y){
              if(points[i].y >= 0) 
                tmpPolarAlpha = M_PI/2;
              else 
                tmpPolarAlpha = -M_PI/2;
            }
            else{
              m = (double)(points[i].y-points[j].y)/(points[i].x-points[j].x);
              if( points[i].y - m*points[i].x < 0) tmpPolarAlpha = alphaTmp - M_PI/2;
              else  tmpPolarAlpha = alphaTmp + M_PI/2;
            }
              // cos und sin for faster computations later
            cosTmpPolarAlpha = cos(tmpPolarAlpha);
            sinTmpPolarAlpha = sin(tmpPolarAlpha);
              
              //tmpPolarD = distPointVector(points[i], points[j], cosOrigin);
            tmpPolarD = points[i].x*cosTmpPolarAlpha + points[i].y*sinTmpPolarAlpha;
              
            vector<unsigned char> counters(lineCount, 0); 
            newLinePoints = 0;
            int pointCounter=0;
              // grow line
            for(uint k=0; k<points.size(); k++){
              // if point is close to line, count it as inlier
              if( abs(points[k].x*cosTmpPolarAlpha + points[k].y*sinTmpPolarAlpha - tmpPolarD) <= threshD){
                pointCounter++;
                  // point belong to line, update binary representation
                bitRep = 1 << k;
                newLinePoints = newLinePoints | bitRep;
                  // update counters that hold the number of common points with all other lines
                for(uint l=0; l<lineCount; l++){
                    // if k th point is contained in l th line too, increment (bitrepresentation!)
                  if(pointLines[l] & bitRep){
                    counters[l]++;
                    if(counters[l]>=2){
                      collinearFlag = 1;
                        // change loop values to leave loops
                      l=lineCount;
                      k=points.size();
                    }
                  } 
                } // for l
              } // if dist <= threshD
            } // for k
              
              //cout << "POINTCOUNTER=" << pointCounter << endl;
              
            if(!collinearFlag){
              pointLines[lineCount] = newLinePoints;
              lineCount++;
            }
              
          } // if dist <= aDist
        } // if pointGrads[j] >= minGrad
      } // for j
    }// if pointGrads[i] >= minGrad
  } // for i
    
    // total least square (= generalised least square = orthogonal regression) 
  for(uint i=0; i<lineCount; i++){
      // calculate average x and y
    double avY=0, avX=0;
    uint tmpLine = pointLines[i];
    uint bitMask=1;
    uint nLinePoints=0;
    uint idx=0;
    while(tmpLine){
      if(tmpLine & bitMask){
        avX += points[idx].x;
        avY += points[idx].y;
        nLinePoints++;
        tmpLine = tmpLine ^ bitMask; // delete this bit
          //cout <<"idx=" << idx << " x=" << points[idx].x << " y=" << points[idx].y << endl;
      }
      idx++;
        //cout << "mask=" << bitMask << endl;
      bitMask = bitMask << 1;
    }
    avX /= nLinePoints;
    avY /= nLinePoints;

      // calculate sxx, syy, sxy (elements of covariance matrix) 
    double sxx=0, syy=0, sxy=0;
    tmpLine = pointLines[i];
    bitMask=1;
    idx=0;
    nLinePoints=0;
    while(tmpLine){
      if(tmpLine & bitMask){
          //cout <<"idx="<<idx <<" distx="<<(points[idx].x-avX) << " x=" << points[idx].x << " avx=" << avX  << endl;
        sxx += (points[idx].x-avX)*(points[idx].x-avX);
        syy += (points[idx].y-avY)*(points[idx].y-avY);
        sxy += (points[idx].x-avX)*(points[idx].y-avY);
        nLinePoints++;
        tmpLine = tmpLine ^ bitMask; // delete this bit
          //cout <<"idx=" << idx << " x=" << points[idx].x << " y=" << points[idx].y << endl;
      }
      idx++;
        //cout << "mask=" << bitMask << endl;
      bitMask = bitMask << 1;
    }
    sxx /= nLinePoints;
    syy /= nLinePoints;
    sxy /= nLinePoints;
      
      // smaller eigenvalue (eigenvector to smaller eigenvalue should direct perpendicular to line)
    double lambda = 0.5 * (sxx + syy - sqrt( (sxx-syy)*(sxx-syy) + 4*sxy*sxy ) );
      
      // eigenvector belonging to smaller eigenvalue [v1 v1]
    double v1, v2;
    if(sxx == 0){
      v1=1;
      v2=0;
    }
    else if(syy == 0){
      v1=0;
      v2=1;
    }
    else if(sxy == 0){
      v1 = (sxx > syy ) ? 0 : 1;
      v2 = (sxx > syy ) ? 1 : 0;
    }
    else{
      v2 = -(sxx-lambda) / sxy;
      double norm = sqrt(v2*v2 + 1);
      v1 = 1/norm;
      v2 = v2/norm;
    }
      
      
    double polarD = v1*avX + v2*avY;
    double polarAlpha = atan2( v2, v1); // in range [-pi, pi]
      
    double sinPA = sin(polarAlpha);
    double cosPA = cos(polarAlpha);
      
      //cout << "nPoints=" << nLinePoints << endl;
      //cout << "d="<<polarD << " alpha=" << polarAlpha <<" (v=["<<v1 << " " <<v2 <<"] sxx="<<sxx<<" sxy=" << sxy << " syy="<<syy << " avX="<<avX << " avY="<<avY <<endl;
      
      
    //////////////////////////////////////////////////////////////////////////////
    ///////// project search points on line (orthogonal projection) //////////////
    //////////////////////////////////////////////////////////////////////////////

    double dProj, dxProj, dyProj;
    CvPoint newProjPt;
    vector<CvPoint> projPoints;   // vector of points projected on new lineSegment
    vector<double> projPointsGrads;  // vector of gradients of projected points
    vector<double> onlineDists;      // vector of distances of projected points to center of line 
    int pPtLiDiffX, pPtLiDiffY;
    tmpLine = pointLines[i];
    bitMask=1;
    idx=0;
      
    while(tmpLine){
      if(tmpLine & bitMask){
          //cout <<"idx="<<idx <<" distx="<<(points[idx].x-avX) << " x=" << points[idx].x << " avx=" << avX  << endl;
          
          // calculate coordinates
        dProj = abs(points[idx].x*cosPA + points[idx].y*sinPA - polarD);
        dxProj = abs(cosPA * dProj);
        dyProj = abs(sinPA * dProj);
          
        if((points[idx].x+dxProj)*cosPA + (points[idx].y+dyProj)*sinPA - polarD < 0.0001){
          newProjPt.x = points[idx].x+ (int) dxProj;
          newProjPt.y = points[idx].y+ (int) dyProj;
            //cout << "++" << endl;
        }
        else if((points[idx].x+dxProj)*cosPA + (points[idx].y-dyProj)*sinPA - polarD < 0.0001){
          newProjPt.x = points[idx].x+ (int) dxProj;
          newProjPt.y = points[idx].y- (int) dyProj;
            //cout << "+-" << endl;
        }
        else if((points[idx].x-dxProj)*cosPA + (points[idx].y+dyProj)*sinPA - polarD < 0.0001){
          newProjPt.x = points[idx].x- (int) dxProj;
          newProjPt.y = points[idx].y+ (int) dyProj;
            //cout << "-+" << endl;
        }
        else if((points[idx].x-dxProj)*cosPA + (points[idx].y-dyProj)*sinPA - polarD < 0.0001){
          newProjPt.x = points[idx].x- (int) dxProj;
          newProjPt.y = points[idx].y- (int) dyProj;
            //cout << "--" << endl;
        }

          // calculate distances of projected points to center of line
        pPtLiDiffX = newProjPt.x - (int) avX;
        pPtLiDiffY = newProjPt.y - (int) avY;
        onlineDists.push_back(sqrt(pPtLiDiffX*pPtLiDiffX + pPtLiDiffY*pPtLiDiffY));
          
          // store results
        projPoints.push_back(newProjPt);
        projPointsGrads.push_back(pointGrads[idx]);
          
        tmpLine = tmpLine ^ bitMask; // delete this bit
          //cout <<"idx=" << idx << " x=" << points[idx].x << " y=" << points[idx].y << endl;
      }
      idx++;
        //cout << "mask=" << bitMask << endl;
      bitMask = bitMask << 1;
    }
      
    /////////////////////////////////////////////////////////////////////////////////////////
    ///////// determine covariance of the line (polar: dist, alpha, alpha-dist) ////////////
    /////////////////////////////////////////////////////////////////////////////////////////

    double covDD, covAA, covAD;
    double qX = 2;  // x cov of a local gradient maximum pixel (assume: x,y uncorrelated)
    double qY = 2;  // y cov of a local gradient maximum pixel (assume: x,y uncorrelated)
      
    double pCovDD = qX*cosPA*cosPA + qY*sinPA*sinPA;  // polar distance cov of a single pixel 
      
    covDD = pCovDD / nLinePoints;   // final covariance DD
      
    covAA = 0;  // here used as temporary storage
    covAD = 0;  // here used as temporary storage, too
    for(uint j=0; j<onlineDists.size(); j++){
      covAA += onlineDists[j]*onlineDists[j];
      covAD += onlineDists[j];
    }
    covAA = pCovDD / covAA;                 // final covariance AA
    covAD = -covDD * covAA * covAD/pCovDD;  // final correlation AD
      
      //cout << "covDD="<<covDD << " covAA=" << covAA << " covAD=" << covAD << endl;
      
      
    ////////////////////////////////////////////////////////////////////////
    //////////// determine line segments on this infinite line /////////////
    ////////////////////////////////////////////////////////////////////////

    CvPoint startConnect;
    int conLength=0;
    
    if(projPoints.size()>0){
      // start first connected segment
      startConnect.x = projPoints[0].x;
      startConnect.y = projPoints[0].y;
       
      // perform hypothesis test and construct connected segments
      for(uint j=0; j<projPoints.size()-1; j++){
        double avEPGrad = (projPointsGrads[j]+projPointsGrads[j+1]) / 2;
        if(avEPGrad>100) avEPGrad=100;
        
          // test hypothesis
          //cout << "startHypo\n";
          //cvClipLine(size, &projPoints[j], &projPoints[j+1])
        if( cvClipLine(size, &projPoints[j], &projPoints[j+1]) && abs(validLine(gradX, gradY, projPoints[j], projPoints[j+1], avEPGrad/2, 0.2))>avEPGrad*0.75){
          conLength++;
          //cvLine( *colorImage, projPoints[j], projPoints[j+1], CV_RGB(255, 255, 0), 1, 8, 0);
        }
        else{
            // end old connected line
          if(conLength>0){
            if( (startConnect.x-projPoints[j].x)*(startConnect.x-projPoints[j].x) + (startConnect.y-projPoints[j].y)*(startConnect.y-projPoints[j].y) >= minLength2){
              LineSegment newLS;
              newLS.setPoints(startConnect.x, startConnect.y, projPoints[j].x, projPoints[j].y);
              lines.push_back(newLS);
            }
            conLength=0;
          }
            // start new connected line
          startConnect.x = projPoints[j+1].x;
          startConnect.y = projPoints[j+1].y;
        }
          //cout << "endHypo\n";
  
      }
        
        // end last connected line
      if(conLength>0){
        if( (startConnect.x-projPoints[projPoints.size()-1].x)*(startConnect.x-projPoints[projPoints.size()-1].x) + (startConnect.y-projPoints[projPoints.size()-1].y)*(startConnect.y-projPoints[projPoints.size()-1].y) >= minLength2){
          LineSegment newLS;
          newLS.setPoints(startConnect.x, startConnect.y, projPoints[projPoints.size()-1].x, projPoints[projPoints.size()-1].y);
          lines.push_back(newLS);
        }
      }
    } // if projPoints.size()>0
  } // for i (each line)
    
    
    // least squares
    /*
    // for each line calculate linear least square with y = mx+n 
  for(uint i=0; i<lineCount; i++){
  double avY=0, avX=0, xySum=0, xxSum=0;
        
  uint tmpLine = pointLines[i];
  uint bitMask=1;
  uint nLinePoints=0;
        
  uint idx=0;
        
        // calculate average x and y, x*x sum and x*y sum
  while(tmpLine){
  if(tmpLine & bitMask){
  avX += points[idx].x;
  avY += points[idx].y;
  xySum += points[idx].x*points[idx].y;
  xxSum += points[idx].x*points[idx].x;
  nLinePoints++;
  tmpLine = tmpLine ^ bitMask; // delete this bit
            //cout <<"idx=" << idx << " x=" << points[idx].x << " y=" << points[idx].y << endl;
}
  idx++;
          //cout << "mask=" << bitMask << endl;
  bitMask = bitMask << 1;
}
  avX /= nLinePoints;
  avY /= nLinePoints;
        
        // calculate m and n for line representation y=mx+n
        //cout << "nLinePoints=" << nLinePoints << endl;
        //cout << "xySum=" << xySum << " xxSum="<< xxSum << endl;
        //cout << "avX=" << avX << " avY="<< avY << endl;
        
  double divisor = xxSum - nLinePoints*avX*avX;
  double m = (divisor!=0) ? (xySum - nLinePoints*avX*avY)/divisor : 0;
  double n = avY - m*avX;
        
  CvPoint mn1, mn2;
  mn1.x = 0;
  mn1.y = n;
  mn2.x = 500;
  mn2.y = m*500 + n;
  cvLine( *colorImage,mn1, mn2, CV_RGB(255, 255, 0), 1, 8, 0);
        
        //cout << "line["<<i<<"] (pointsLine=" << pointLines[i]<<") : m="<<m << " n="<<n << endl;
        
}
    */
    // merging

}


void LineSegment::calcHistogramDescriptor(Image* image, int d){
  
  //////////////////////////////////////////////////////////////
  ///// create endpoints of parallel lines with distance d /////
  //////////////////////////////////////////////////////////////
  
  CvPoint lP1, lP2, rP1, rP2;
  
  if( slope<1 && slope>-1){
    // x is major, parallels have other x-values
    if(x1 < x2 ){
      lP1.x = (int) x1;
      lP1.y = (int) y1-d;
      lP2.x = (int) x2; 
      lP2.y = (int) y2-d;
      
      rP1.x = (int) x1;
      rP1.y = (int) y1+d;
      rP2.x = (int) x2; 
      rP2.y = (int) y2+d;
    }
    else{
      lP1.x = (int) x1;
      lP1.y = (int) y1+d;
      lP2.x = (int) x2; 
      lP2.y = (int) y2+d;
      
      rP1.x = (int) x1;
      rP1.y = (int) y1-d;
      rP2.x = (int) x2; 
      rP2.y = (int) y2-d;
    }
  }
  else{
    // y is major, parallels have other y-values
    if(y1 < y2 ){
      lP1.x = (int) x1+d;
      lP1.y = (int) y1;
      lP2.x = (int) x2+d; 
      lP2.y = (int) y2;
      
      rP1.x = (int) x1-d;
      rP1.y = (int) y1;
      rP2.x = (int) x2-d; 
      rP2.y = (int) y2;
    }
    else{
      lP1.x = (int) x1-d;
      lP1.y = (int) y1;
      lP2.x = (int) x2-d; 
      lP2.y = (int) y2;
      
      rP1.x = (int) x1+d;
      rP1.y = (int) y1;
      rP2.x = (int) x2+d; 
      rP2.y = (int) y2;
    }
  }
  
  /////////////////////////////////////////////////
  ////// clip parallels on image boarders /////////
  /////////////////////////////////////////////////
  
  CvSize size;
  size.width = image->width();
  size.height = image->height();
  if(!cvClipLine(size, &lP1, &lP2)){
    lP1.x = (int) x1;
    lP1.y = (int) y1;
    lP2.x = (int) x2;
    lP2.y = (int) y2;
  }
  if(!cvClipLine(size, &rP1, &rP2)){
    rP1.x = (int) x1;
    rP1.y = (int) y1;
    rP2.x = (int) x2;
    rP2.y = (int) y2;
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //// create 2 Image witch contains the values of image_ on parallels with distance d to the line  ////
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  
  CvLineIterator lineItLeft;
  int lengthL = cvInitLineIterator( *image, lP1, lP2, &lineItLeft, 8, 0);
  
  CvLineIterator lineItRight;
  int lengthR = cvInitLineIterator( *image, rP1, rP2, &lineItRight, 8, 0);
  
  
  // temporary image for pixels on parallel line
  Image* tmpImage = new Image(lengthL, 1 ,8, JfrImage_CS_GRAY);
  
  uchar* imPt = cvPtr1D(*tmpImage, 0);    // pointer to first pixel of image
  
  

  for( int j = 0; j < lengthL; j++ ){
    imPt[j] = lineItLeft.ptr[0];    // write pixel value
    CV_NEXT_LINE_POINT(lineItLeft);   // go to next pixel on line
  }

  histogramB->initHisto();
  histogramB->addImageInfo(tmpImage);
  
  

  if(lengthL != lengthR) tmpImage->resize(lengthR, 1);
  

  imPt = cvPtr1D(*tmpImage, 0);    // pointer to first pixel of image

  for( int j = 0; j < lengthR; j++ ){
    imPt[j] = lineItRight.ptr[0];    // write pixel value
    CV_NEXT_LINE_POINT(lineItRight);   // go to next pixel on line
  }
  
  histogramD->initHisto();
  histogramD->addImageInfo(tmpImage);

  // release tmpImage
  IplImage* tmp = *tmpImage;
  cvReleaseImage(&tmp);
  
}

void LineSegment::calcHistogramDescriptor(){
  
  // find brighter side
  double odd=0, even=0;
  if(numberOfNeighbours>=2){
    for(uint i=0; i<parallels[0]->greyscale.size(); i++){
      even += parallels[0]->greyscale[i];
    }
    odd /= parallels[0]->greyscale.size();
    
    for(uint i=0; i<parallels[1]->greyscale.size(); i++){
      even += parallels[1]->greyscale[i];
    }
    odd /= parallels[1]->greyscale.size();
  
    if(odd < even){
      histogramB->initHisto();
      for(int i=0; i<numberOfNeighbours; i+=2){
        histogramB->addVectorInfo(parallels[i]->greyscale);
      }
      histogramD->initHisto();
      for(int i=1; i<numberOfNeighbours; i+=2){
        histogramD->addVectorInfo(parallels[i]->greyscale);
      }
    }
    else{
      histogramB->initHisto();
      for(int i=1; i<numberOfNeighbours; i+=2){
        histogramB->addVectorInfo(parallels[i]->greyscale);
      }
      histogramD->initHisto();
      for(int i=0; i<numberOfNeighbours; i+=2){
        histogramD->addVectorInfo(parallels[i]->greyscale);
      }
    }
    cvNormalizeHist(histogramB->histo, 100);
    cvNormalizeHist(histogramD->histo, 100);
  }
}


void LineSegment::calcGreyspaceDescriptor(Image* image){
  
  // number of regarded pixels left and right from line
  int range=3; // if change this, change normWeight too!
  
  // list of weights, weight[0] for pixel under line, weight[i] for pixel with distance i
  double weight[] = {1,1,1,1,1,1,1};  // if change this, change normWeight too!
  int normWeight= 4;                  // sum of used weights ( weight[0] + sum of following weights in range)
  
  int dx = (int) (x1-x2);
  int dy = (int) (y1-y2);
  
  CvSize size;
  size.width = image->width();
  size.height = image->height();
  
  CvLineIterator lineIt;
  CvPoint p1,p2;
  
  p1.x = (int) x1;
  p1.y = (int) y1;
  p2.x = (int) x2;
  p2.y = (int) y2;
  
  
  
  int l = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
  
  // calculate stepwidth ( to solve the problem of line shortening on the pixel base under different angels), stepwidth is (number of pixels on line)/(minimal number of pixels on line) with  (minimal number of pixels on line)=(real length of line)/(maximal space of one pixel) with (maximal space of one pixel) is the length of the diagonal of a pixel (= sqrt(2))  
  double stepwidth = (double) l /  (sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) / 1.414213 );
  CvPoint start, end;
  int startedFlag = 0;
  
  int count=0;
  double state=0;
  
  for( int i = 0; count < l; i++ ){
    double keyValue=0, keyValueL=0, keyValueR=0;
    int nCPts=0;
    for(uint j=0; j<contourPoints[i].size(); j++){

      //double weight = pow(2.0, range);
      double valueL, valueR;
  
      int valid=1;
      /*
      // calc point coordinates
      int off, x, y;
      IplImage* tmp = *image;
      off = lineIt.ptr - (uchar*)(tmp->imageData);
      y = off/tmp->widthStep;
      x = (off - y*tmp->widthStep)/(sizeof(uchar));
      */
      
      // origin point is according contour point
      int x=(contourPoints[i][j]).x;
      int y=(contourPoints[i][j]).y;

      // init value with greyvalue of line point
      CvScalar s;
      s=cvGet2D(*image,y,x);
      valueL = weight[0]*s.val[0];
      valueR = weight[0]*s.val[0];
  
      //weight /= 2;
      
      //if(x<0 || x>=image->width() || y<0 || y>=image->height()) break;
      
      // origin point
      CvPoint p0;
      p0.x = x;
      p0.y = y;
      //cout << "origin point: x="<<x << " y=" << y << endl;
      
      // coordinates of perpendicular ending points
      // this claculation garantees, that p1 is on left side o line ( gone from (x1,y1) to (x2, y2)) 
      p1.x = x-dy;
      p1.y = y+dx;
      p2.x = x+dy;
      p2.y = y-dx;
      
      //cout << "pPoints(befor): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
      cvClipLine(size, &p1, &p2);
      //cout << "pPoints(after): x1="<<p1.x<<" y1="<<p1.y<<" x2="<<p2.x<<" y2="<<p2.y<<endl; 
      
      // go from origin to p1
      CvLineIterator linePara;
      int lP = cvInitLineIterator( *image, p0, p1, &linePara, 8, 0);
      CV_NEXT_LINE_POINT(linePara);
      for( int j=0; j < range; j++){
        if( j>=lP ){
          valid=0;
          break;
        }
        valueL += weight[j+1]*linePara.ptr[0];
        //weight /= 2;
        CV_NEXT_LINE_POINT(linePara);
      }
      
      // go from origin to p2
      //weight = pow(2.0, range-1);
      lP = cvInitLineIterator( *image, p0, p2, &linePara, 8, 0);
      CV_NEXT_LINE_POINT(linePara);
      for( int j=0; j < range; j++){
        if( j>=lP ){
          valid=0;
          break;
        }
        valueR += weight[j+1]*linePara.ptr[0];
        //weight /= 2;
        CV_NEXT_LINE_POINT(linePara);
      }
      
      if(valid){
        nCPts++;
        if(!startedFlag){
          start.x = x;
          start.y = y;
        }
        end.x = x;
        end.y = y;
        
        keyValueL += valueL / normWeight;
        keyValueR += valueR / normWeight;
        keyValue += (valueL+valueR) / ((2*normWeight)-weight[0]) ;
  
        //cout << "value[" << i <<"]=" << value << endl;
        //cout << "normalized value[" << i <<"]=" << value / 22  << " (greyscale: " << greyscale[i] << ")"<< endl;
        
      }
      
    }// for j (contourPoints)
    
    if(nCPts>0){
      greyspaceL.push_back( (int) (keyValueL / nCPts) );
      greyspaceR.push_back( (int) (keyValueR / nCPts) );
      greyspace.push_back( (int) (keyValue / nCPts) );
    }
    // go on the line until next multiple of stepwidth
    state += stepwidth;
    while(count < state){
      count++;
      CV_NEXT_LINE_POINT(lineIt);
    }
    //cout << "line: count="<<count << " stat=" << state <<" (stepwidth="<<stepwidth << ", length=" <<l <<") i=" << i<< endl;
    
  } // for i
  
  
  /*
  // smooth greyspaces by taking greyspace of neighbours into account
  int mult=1;
  int last=greyspace[0], lastR=greyspaceR[0], lastL=greyspaceL[0], curr, currL, currR;
  for(uint i=1; i<greyspace.size()-1; i++){
  curr=greyspace[i];
  currL=greyspaceL[i];
  currR=greyspaceR[i];
  greyspace[i] = (mult*greyspace[i]+last+greyspace[i+1])/(2+mult);
  greyspaceL[i] = (mult*greyspaceL[i]+lastL+greyspaceL[i+1])/(2+mult);
  greyspaceR[i] = (mult*greyspaceR[i]+lastR+greyspaceR[i+1])/(2+mult);
  last=curr;
  lastR=currR;
  lastL=currL;
}
  */
  
  
}


void LineSegment::setPoints(double _x1, double _y1, double _x2, double _y2, bool resetFlag){
  //cout << "in setPoints, numberOfNeighbours: " << numberOfNeighbours << endl;
  x1=_x1;
  y1=_y1;
  x2=_x2;
  y2=_y2;
  mx=(x1+x2)/2;
  my=(y1+y2)/2;
  if(x1-x2 != 0) slope=(y1-y2)/(x1-x2);
  else slope = 10000; 
   
  // calculate eucLength (euclidean distance of ending points) and alpha (angle to positiv x-axis) 
  double dx = _x2-_x1;
  double dy = _y2-_y1;
  double m = dy/dx;
  eucLength = sqrt( dx*dx + dy*dy);
  
  if(dx == 0 && dy==0){
    alpha=0;
  }
  else if(dx == 0){
    alpha = M_PI/2;
  }
  else if(dy == 0){
    alpha = 0;
  }
  else{
    alpha = (m>0)?atan(m):atan(m); //(M_PI + atan(m));
    //alpha = atan2(dy, dx);
  }
  
  // set oAlpha to a useful value, maybe it is not true, because oAlpha should respect the gradient along the line, to set it according to the gradient, use orientLine()
  oAlpha = atan2(dy,dx);
  

  //////// covariance matrix //////////////
  
  double para = 10;    // variance parallel to line
  double perp = 2;  // variance perpendicular to line
  
  double cosA = cos(alpha);
  double sinA = sin(alpha);
  double cosA2 = cosA*cosA;
  double sinA2 = sinA*sinA;
  
  double h = dx*dx + dy*dy;
  
  covXX = (para*cosA2 + perp*sinA2);
  covYY = (perp*cosA2 + para*sinA2);
  covXY = (para-perp) * sinA * cosA;
  covAlpha = 2 * perp / (eucLength*eucLength);
  covXA = (dy*covXX - covXY*dx) / h;
  covYA = (dy*covXY - covYY*dx) / h;
  
  // cov for merging (without uncertainty parallel to line)
  covXX_M = perp*sinA2;
  covYY_M = perp*cosA2;
  covXY_M = -perp * sinA * cosA;
  covXA_M = (dy*covXX_M - covXY_M*dx) / h;
  covYA_M = (dy*covXY_M - covYY_M*dx) / h;
  

  //////// initialise polar coordinates //////////
  
  // alphaTmp should be in range [-pi/2, pi/2]
  double alphaTmp;
  if(alpha<-M_PI/2) alphaTmp = alpha + M_PI;
  else if(alpha>M_PI/2) alphaTmp = alpha - M_PI;
  else alphaTmp = alpha;
  
  // polarAlpha will be in range [-pi, pi]
  if(x1 == x2){
    if(x1 >= 0) 
      polarAlpha = 0;
    else 
      polarAlpha = M_PI;
  }
  else if(y1 == y2){
    if(y1 >= 0) 
      polarAlpha = M_PI/2;
    else 
      polarAlpha = -M_PI/2;
  }
  else{
    m = (double)(y1-y2)/(x1-x2);
    if( y1 - m*x1 < 0) polarAlpha = alphaTmp - M_PI/2;
    else  polarAlpha = alphaTmp + M_PI/2;
  }
  
  double cosPA = cos(polarAlpha);
  double sinPA = sin(polarAlpha);
  //double cosPA2 = cosPA*cosPA;
  //double sinPA2 = sinPA*sinPA;
  polarD = mx*cosPA+my*sinPA;

  // init predictor
  if(resetFlag) predictor.initPredictor(mx, my, alpha, polarD, polarAlpha);
  
  // initialize parallel lines
  // therefor analyse slope, (similar to Bresenham algorithm:) if abs(slope)<1: start and end parallel
  // lines above and under the origin line, otherwise left and right of origin line
  if(numberOfNeighbours != 0){
    int key; // to respect the orientation, even numbered paralleles are left of line and odd are right
    
    if( slope<1 && slope>-1){
      
      if(x1<x2) key=1;
      else key=-1;
      
      for(int i=0; i<numberOfNeighbours; i++){
        int inc = key*((i/2)+1);
        if(i%2==0) parallels[i]->setPoints(x1, (y1-inc>=0)?(y1-inc):0, x2, (y2-inc>=0)?(y2-inc):0);
        else parallels[i]->setPoints(x1, y1+inc, x2, y2+inc);
      }
    }
    else{
      
      if(y1>y2) key=1;
      else key=-1;
      
      for(int i=0; i<numberOfNeighbours; i++){
        int inc = key* ((i/2)+1);
        if(i%2==0) parallels[i]->setPoints((x1-inc>=0)?(x1-inc):0, y1, (x2-inc>=0)?(x2-inc):0, y2);
        else parallels[i]->setPoints(x1+inc, y1, x2+inc, y2);
      }
    }
  }
}

void LineSegment::changePointOrder(){
  double tmp;
  
  tmp = x1;
  x1 = x2;
  x2 = tmp;
  
  tmp = y1;
  y1 = y2;
  y2 = tmp;
  
  oAlpha = oAlpha+M_PI;
  if(oAlpha>M_PI) oAlpha -= 2*M_PI;
  
  tmp=avL;
  avL = avR;
  avR = tmp;
  
  // change cov
  covXA = -covXA;
  covYA = -covYA;
  
  // change histograms
  Histogram* tmpH = histogramB;
  histogramB = histogramD;
  histogramD = tmpH;
}


void LineSegment::orientLine(Image* image){
  CvPoint ol1,ol2, ur1,ur2;
  int dist=2; // distance for measurement of average greylevel
  
  // initialize coordinates of parallels (over/left and under/right)  
  if( slope<1 && slope>-1){
    ol1.x = (int) x1;
    ol1.y = (int) ( (y1-dist>=0)?(y1-dist):0 );
    ol2.x = (int) x2;
    ol2.y = (int) ( (y2-dist>=0)?(y2-dist):0 );
    
    ur1.x = (int) x1;
    ur1.y = (int) (y1+dist);
    ur2.x = (int) x2;
    ur2.y = (int) (y2+dist);
  }
  else{
    ol1.x = (int) ( (x1-dist>=0)?(x1-dist):0 );
    ol1.y = (int) y1;
    ol2.x = (int) ( (x2-dist>=0)?(x2-dist):0 );
    ol2.y = (int) y2;
    
    ur1.x = (int) (x1+dist);
    ur1.y = (int) y1;
    ur2.x = (int) (x2+dist);
    ur2.y = (int) y2;
  }

  CvSize size;
  size.width = image->width();
  size.height = image->height();
  
  double avOL=0, avUR=0;
  double tmpVarOL=0, tmpVarUR=0;
  int min=255, max=0;
  // clip line, if one is completly out of the image skip orinting phase and don't change the points
  if( cvClipLine(size, &ol1, &ol2) && cvClipLine(size, &ur1, &ur2) ){
    // calculate average over/left of the line
    CvLineIterator lineIt;
    int length = cvInitLineIterator( *image, ol1, ol2, &lineIt, 8, 0);
    for( int i = 0; i < length; i++ ){
      // update average grey
      avOL += lineIt.ptr[0];

      if(lineIt.ptr[0]>max) max = lineIt.ptr[0];
      if(lineIt.ptr[0]<min) min = lineIt.ptr[0];
      
      
      CV_NEXT_LINE_POINT(lineIt);
    }
    avOL /= length;
    tmpVarOL = max-min;
    
    min=255, max=0;
    // calculate average under/right of the line
    length = cvInitLineIterator( *image, ur1, ur2, &lineIt, 8, 0);
    for( int i = 0; i < length; i++ ){
      // update average grey
      avUR += lineIt.ptr[0];
      
      if(lineIt.ptr[0]>max) max = lineIt.ptr[0];
      if(lineIt.ptr[0]<min) min = lineIt.ptr[0];
      
      CV_NEXT_LINE_POINT(lineIt);
    }  
    avUR /= length;
    tmpVarUR = max-min;;

    // change points if neccessary
    if( slope<1 && slope>-1){
      // if over the line is brighter than under
      if(avOL > avUR){
        // if order of points is wrong
        if(x2 < x1){
          changePointOrder();
        }
      }
      else{
        // if order of points is wrong
        if(x2 > x1){
          changePointOrder();
        }
      }
    }
    else{
      // if left of the line is brighter than right of line
      if(avOL > avUR){
        // if order of points is wrong
        if(y2 > y1){
          changePointOrder();
        }
      }
      else{
        // if order of points is wrong
        if(y2 < y1){
          changePointOrder();
        }
      }
    }
  } // if one parallel line is completly outside the image  
  
  ///////// calculate oriented angel (oAlpha) /////////
  double dy = y2-y1;
  double dx = x2-x1;

  if(dx == 0){
    if(y1==y2) oAlpha=0;
    else if(y1<y2) oAlpha = M_PI/2;
    else if(y1>y2) oAlpha = -M_PI/2;
  }
  else if(dy == 0){
    if(x1<x2) oAlpha = 0;
    else if(x1>x2) oAlpha = M_PI;
  }
  else{
    //alpha = (m>0)?atan(m):atan(m); //(M_PI + atan(m));
    oAlpha = atan2(dy, dx);
  }
  
  /////// update member variables for average grey levels /////////
  if(avOL > avUR){
    avL = avOL;
    avR = avUR;
    varL = tmpVarOL;
    varR = tmpVarUR;
  }
  else{
    avL = avUR;
    avR = avOL;
    varL = tmpVarUR;
    varR = tmpVarOL;
  }
}


void LineSegment::setNumberOfNeighbours(int number){
  if(numberOfNeighbours==0){
    numberOfNeighbours=number;
    parallels = new LineSegment*[numberOfNeighbours];
    for(int i=0; i<numberOfNeighbours; i++){
      parallels[i] = new LineSegment;
    }
    if( slope<1 && slope>-1){
      for(int i=0; i<numberOfNeighbours; i++){
        int inc = (i/2)+1;
        if(i%2==0) parallels[i]->setPoints(x1, (y1-inc>=0)?(y1-inc):0, x2, (y2-inc>=0)?(y2-inc):0);
        else parallels[i]->setPoints(x1, y1+inc, x2, y2+inc);
      }
    }
    else{
      for(int i=0; i<numberOfNeighbours; i++){
        int inc = (i/2)+1;
        if(i%2==0) parallels[i]->setPoints((x1-inc>=0)?(x1-inc):0, y1, (x2-inc>=0)?(x2-inc):0, y2);
        else parallels[i]->setPoints(x1+inc, y1, x2+inc, y2);
      }
    }
  }
  else if( number < numberOfNeighbours ){
    for(int i = number; i < numberOfNeighbours; i++){
      delete parallels[i];
    }
    numberOfNeighbours=number;
  }
  else if( number > numberOfNeighbours ){
    LineSegment **oldParallels = parallels;
    parallels = new LineSegment*[number];
    for(int i=0; i<numberOfNeighbours; i++){
      parallels[i] = oldParallels[i];
    }
    for(int i=numberOfNeighbours; i<number; i++){
      parallels[i] = new LineSegment();
      if( slope<1 && slope>-1){
        for(int i=0; i<numberOfNeighbours; i++){
          int inc = (i/2)+1;
          if(i%2==0) parallels[i]->setPoints(x1, (y1-inc>=0)?(y1-inc):0, x2, (y2-inc>=0)?(y2-inc):0);
          else parallels[i]->setPoints(x1, y1+inc, x2, y2+inc);
        }
      }
      else{
        for(int i=0; i<numberOfNeighbours; i++){
          int inc = (i/2)+1;
          if(i%2==0) parallels[i]->setPoints((x1-inc>=0)?(x1-inc):0, y1, (x2-inc>=0)?(x2-inc):0, y2);
          else parallels[i]->setPoints(x1+inc, y1, x2+inc, y2);
        }
      }
    }   
    numberOfNeighbours = number;
  }
}


void LineSegment::extractGreyscale(Image* image, int parallel_flag){
  //cout << "in extract greyscale\n";
  greyscale.clear();
  
  CvSize size;
  size.width = image->width();
  size.height = image->height();
  
  CvPoint p1,p2;
  p1.x = (int)x1;
  p1.y = (int)y1;
  p2.x = (int)x2;
  p2.y = (int)y2;
  cvClipLine( size, &p1, &p2 );

  CvLineIterator lineIt;
  length = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
  for( int i = 0; i < length; i++ ){
    greyscale.push_back( lineIt.ptr[0]);
    CV_NEXT_LINE_POINT(lineIt);
  }
  
  
  //cout << "start update parallels\n";
  // update parallels if desired
  if(parallel_flag){
    for( int i=0; i<numberOfNeighbours; i++){
      

      // clear old greyscale 
      parallels[i]->greyscale.clear();
      // extract ending points
      CvPoint p1,p2;
      p1.x = (int)parallels[i]->x1;
      p1.y = (int)parallels[i]->y1;
      p2.x = (int)parallels[i]->x2;
      p2.y = (int)parallels[i]->y2;
      // update ending points if they are out of image size
      
      //cout << "width: "<< image->width() << " height: "<< image->height() << endl;
      //cout << "vorher  points: p1.x: " << p1.x << " p1.y: " << p1.y << " p2.x: " << p2.x << " p2.y: " << p2.y << endl;

      // clip parallel line at image boarders
      if(!cvClipLine( size, &p1, &p2 )){
        // if completly out of image take points of neighboured parallel line or if there is no 
        // neighboured parallel line the points of the main line
        if( i>1 ){ 
          p1.x = (int)parallels[i-2]->x1;
          p1.y = (int)parallels[i-2]->y1;
          p2.x = (int)parallels[i-2]->x2;
          p2.y = (int)parallels[i-2]->y2;
        }
        else{
          p1.x = (int) x1;
          p1.y = (int) y1;
          p2.x = (int) x2;
          p2.y = (int) y2;
        }
      }
      parallels[i]->setPoints( p1.x, p1.y, p2.x, p2.y);
      // iterate over (updated) parallel line an extract greyscale
      CvLineIterator lineIt;
      //cout << "nachher points: p1.x: " << p1.x << " p1.y: " << p1.y << " p2.x: " << p2.x << " p2.y: " << p2.y << endl;
      parallels[i]->length = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
      //cout << "in extractsGreyscal with parallel_flag, parallels length:" << parallels[i]->length << endl;
      for( int j = 0; j < parallels[i]->length; j++ ){
        //cout << "in extractsGreyscal, insert grey \n" ;
        parallels[i]->greyscale.push_back( lineIt.ptr[0]);
        CV_NEXT_LINE_POINT(lineIt);
      }
    }
    
  }
  //cout << "end greyscale\n";
  // calculate histogram and print
  // histogram->initHisto();
  // histogram->addVectorInfo(greyscale);
  // histogram->print();
}


void LineSegment::extractGradientscale(Image* image, int gradientimage, int parallel_flag){
  //cout << "in extract gradientscale\n";

  gradientscale.clear();
  // if _image is an gradient image, just take the according pixel-values
  CvPoint p1,p2;
  p1.x = (int)x1;
  p1.y = (int)y1;
  p2.x = (int)x2;
  p2.y = (int)y2;
  if(gradientimage){
    CvLineIterator lineIt;
    length = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
    for( int i = 0; i < length; i++ ){
      gradientscale.push_back( lineIt.ptr[0]);
      CV_NEXT_LINE_POINT(lineIt);
    }
    // update parallels if desired
    if(parallel_flag){
      for( int i=0; i<numberOfNeighbours; i++){
      // clear old gradientscale 
        parallels[i]->gradientscale.clear();
      // extract ending points
        CvPoint p1,p2;
        p1.x = (int)parallels[i]->x1;
        p1.y = (int)parallels[i]->y1;
        p2.x = (int)parallels[i]->x2;
        p2.y = (int)parallels[i]->y2;
      // update ending points if they are out of image size
        CvSize size;
        size.width = image->width();
        size.height = image->height();
        cvClipLine( size, &p1, &p2 );
        parallels[i]->setPoints( p1.x, p1.y, p2.x, p2.y);
      // iterate over (updated) parallel line an extract greyscale
        CvLineIterator lineIt;
        parallels[i]->length = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
        for( int j = 0; j < parallels[i]->length; j++ ){
          parallels[i]->gradientscale.push_back( lineIt.ptr[0]);
          CV_NEXT_LINE_POINT(lineIt);
        }
      }
    
    }
  }
  // otherwhise calculate the gradient image first
  else{
    Image* gradientImage = new Image( image->width(), image->height(), IPL_DEPTH_16S, JfrImage_CS_GRAY);
    Image* scaleImage = new Image( image->width(), image->height(), 8, JfrImage_CS_GRAY);

    // calculate sobel
    cvSobel(*image, *gradientImage,1,1, 3);
    // scale to 8 bit
    cvConvertScaleAbs(*gradientImage, *scaleImage);

    CvLineIterator lineIt;
    length = cvInitLineIterator( *scaleImage, p1, p2, &lineIt, 8, 0);
    for( int i = 0; i < length; i++ ){
      gradientscale.push_back( lineIt.ptr[0]);
      CV_NEXT_LINE_POINT(lineIt);
    } 
     // update parallels if desired
    if(parallel_flag){
      for( int i=0; i<numberOfNeighbours; i++){
      // clear old gradientscale 
        parallels[i]->gradientscale.clear();
      // extract ending points
        CvPoint p1,p2;
        p1.x = (int)parallels[i]->x1;
        p1.y = (int)parallels[i]->y1;
        p2.x = (int)parallels[i]->x2;
        p2.y = (int)parallels[i]->y2;
      // update ending points if they are out of image size
        CvSize size;
        size.width = image->width();
        size.height = image->height();
        cvClipLine( size, &p1, &p2 );
        parallels[i]->setPoints( p1.x, p1.y, p2.x, p2.y);
      // iterate over (updated) parallel line an extract greyscale
        CvLineIterator lineIt;
        parallels[i]->length = cvInitLineIterator( *scaleImage, p1, p2, &lineIt, 8, 0);
        for( int j = 0; j < parallels[i]->length; j++ ){
          parallels[i]->gradientscale.push_back( lineIt.ptr[0]);
          CV_NEXT_LINE_POINT(lineIt);
        }
      }
    
    }
    //delete gradientImage;
    IplImage* tmp = *gradientImage;
    cvReleaseImage(&tmp);
    //delete gradientImage;
    tmp = *scaleImage;
    cvReleaseImage(&tmp);
  }
}


void LineSegment::extractLaplacescale(Image* image, int laplaceimage, int parallel_flag){
  //cout << "in extractLaplacescale\n";
  laplacescale.clear();
  // if _image is an gradient image, just take the according pixel-values
  CvPoint p1,p2;
  p1.x = (int)x1;
  p1.y = (int)y1;
  p2.x = (int)x2;
  p2.y = (int)y2;
  if(laplaceimage){
    CvLineIterator lineIt;
    length = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
    for( int i = 0; i < length; i++ ){
      laplacescale.push_back( lineIt.ptr[0]);
      CV_NEXT_LINE_POINT(lineIt);
    }
    // update parallels if desired
    if(parallel_flag){
      for( int i=0; i<numberOfNeighbours; i++){
      // clear old laplacescale 
        parallels[i]->laplacescale.clear();
      // extract ending points
        CvPoint p1,p2;
        p1.x = (int)parallels[i]->x1;
        p1.y = (int)parallels[i]->y1;
        p2.x = (int)parallels[i]->x2;
        p2.y = (int)parallels[i]->y2;
      // update ending points if they are out of image size
        CvSize size;
        size.width = image->width();
        size.height = image->height();
        cvClipLine( size, &p1, &p2 );
        parallels[i]->setPoints( p1.x, p1.y, p2.x, p2.y);
      // iterate over (updated) parallel line an extract greyscale
        CvLineIterator lineIt;
        parallels[i]->length = cvInitLineIterator( *image, p1, p2, &lineIt, 8, 0);
        for( int j = 0; j < parallels[i]->length; j++ ){
          parallels[i]->laplacescale.push_back( lineIt.ptr[0]);
          CV_NEXT_LINE_POINT(lineIt);
        }
      }
    
    }
  }
  // otherwhise calculate the gradient image first
  else{
    Image* laplaceImage = new Image( image->width(), image->height(), IPL_DEPTH_16S, JfrImage_CS_GRAY);
    Image* scaleImage = new Image( image->width(), image->height(), 8, JfrImage_CS_GRAY);

    // calculate Laplace
    cvLaplace(*image, *laplaceImage, 3);
    // scale to 8 bit
    cvConvertScaleAbs(*laplaceImage, *scaleImage);

    CvLineIterator lineIt;
    length = cvInitLineIterator( *scaleImage, p1, p2, &lineIt, 8, 0);
    for( int i = 0; i < length; i++ ){
      laplacescale.push_back( lineIt.ptr[0]);
      CV_NEXT_LINE_POINT(lineIt);
    } 
    // update parallels if desired
    if(parallel_flag){
      for( int i=0; i<numberOfNeighbours; i++){
      // clear old laplacescale 
        parallels[i]->laplacescale.clear();
      // extract ending points
        CvPoint p1,p2;
        p1.x = (int)parallels[i]->x1;
        p1.y = (int)parallels[i]->y1;
        p2.x = (int)parallels[i]->x2;
        p2.y = (int)parallels[i]->y2;
      // update ending points if they are out of image size
        CvSize size;
        size.width = image->width();
        size.height = image->height();
        cvClipLine( size, &p1, &p2 );
        parallels[i]->setPoints( p1.x, p1.y, p2.x, p2.y);
      // iterate over (updated) parallel line an extract greyscale
        CvLineIterator lineIt;
        parallels[i]->length = cvInitLineIterator( *scaleImage, p1, p2, &lineIt, 8, 0);
        for( int j = 0; j < parallels[i]->length; j++ ){
          parallels[i]->laplacescale.push_back( lineIt.ptr[0]);
          CV_NEXT_LINE_POINT(lineIt);
        }
      }
    
    }
    //delete laplaceImage;
    IplImage* tmp = *laplaceImage;
    cvReleaseImage(&tmp);
    //delete scaleImage;
    tmp = *scaleImage;
    cvReleaseImage(&tmp);
  }
}


bool LineSegment::calcEndpointsOfSupportingLine(double alpha, double y0, int height, int width, double& x1, double& y1, double& x2, double& y2){
   // special cases
  if( alpha==0 ){
    x1=0;
    y1=y0;
    x2=width;
    y2=y0;
    return 1;
  }
  if( alpha==M_PI/2 ){
    x1=y0;
    y1=0;
    x2=y0;
    y2=height;
    return 1;
  }
   
   // gradient (?)
  double m = tan(alpha);
   // number of already accepted intersection points
  int count=0;
   
  double tmp;
   // intersection at x=0
  if( y0>=0 && y0<height){
    x1=0;
    y1=y0;
    count++;
  }
   // intersection at x=width
  tmp = m*width+y0;
  if( tmp>=0 && tmp<height){
    if(count==0){
      x1=width;
      y1=tmp;
      count++;
    }
    else{
      x2=width;
      y2=tmp;
      return 1;
    }
  }
   // intersection at y=0
  tmp = -y0/m; 
  if( tmp>=0 && tmp<width ){ 
    if(count==0){
      x1=tmp;
      y1=0;
      count++;
    }
    else{
      x2=tmp;
      y2=0;
      return 1;
    }
  }
   // intersection at y=height
  tmp = (height-y0)/m;
  if( tmp>=0 && tmp<width){
    x2=tmp;
    y2=height;
    return 0;
  } 
  return 1;  
}


bool LineSegment::calcSupportingLine(double x1, double y1, double x2, double y2, double& alpha, double& y0){
  double dx = x1-x2;
  double dy = y1-y2;
  double m = dy/dx;
  
  if(dx == 0 && dy==0){
    alpha=0;
    y0=0;
    return 0;
  }
  else if(dx == 0){
    //alpha = M_PI;
    alpha = M_PI/2;
    y0 = x1;
    return 1;
  }
  else if(dy == 0){
    alpha = 0;
    y0 = y1;
    return 1;
  }
  else{
    alpha = (m>0)?atan(m):M_PI+atan(m);
    y0 = y1 - m*x1;
    
    //cout << "alpha old: " << alpha <<endl;
    if (alpha > M_PI/2) alpha -= M_PI;
    //cout << "alpha new: " << alpha <<endl;
    
    return 1;
  }
  

}

int getKey(Image* image, int x, int y, double alpha){
  int w=image->width();
  int h=image->height();
  CvScalar s, s1, s2, s3, s4, s5, s6, s7, s8;
  s=cvGet2D(*image,y,x);
  if( abs(alpha) < M_PI/4 ){
    s1=cvGet2D(*image,y,(x-1<0)?0:(x-1));
    s2=cvGet2D(*image,y,(x+1>=w)?(w-1):(x+1));
    s3=cvGet2D(*image,y,(x-2<0)?0:(x-2));
    s4=cvGet2D(*image,y,(x+2>=w)?(w-1):(x+2));
    s5=cvGet2D(*image,y,(x-3<0)?0:(x-3));
    s6=cvGet2D(*image,y,(x+3>=w)?(w-1):(x+3));
    s7=cvGet2D(*image,y,(x-4<0)?0:(x-4));
    s8=cvGet2D(*image,y,(x+4>=w)?(w-1):(x+4));
  }
  else{
    s1=cvGet2D(*image,(y-1<0)?0:(y-1),x);
    s2=cvGet2D(*image,(y+1>=h)?(h-1):(y+1),x);
    s3=cvGet2D(*image,(y-2<0)?0:(y-2),x);
    s4=cvGet2D(*image,(y+2>=h)?(h-1):(y+2),x);
    s5=cvGet2D(*image,(y-3<0)?0:(y-3),x);
    s6=cvGet2D(*image,(y+3>=h)?(h-1):(y+3),x);
    s7=cvGet2D(*image,(y-4<0)?0:(y-4),x);
    s8=cvGet2D(*image,(y+4>=h)?(h-1):(y+4),x);
  }

  //cout << "val: " << s.val[0] <<" "<< s1.val[0] <<" "<< s2.val[0] <<" "<< s3.val[0] <<" "<< s4.val[0] << endl;
  int av1 = (int) (s1.val[0] + s3.val[0])/2; // + s5.val[0] + s7.val[0]) / 4;
  int av2 = (int) (s2.val[0] + s4.val[0])/2; // + s6.val[0] + s8.val[0]) / 4;
  return abs( av1-av2 );
}


bool LineSegment::growLineParallel(Image* image, int dist, int thresh, int distThresh){
    
  double x1, y1, x2, y2;
  double alpha, y0;

  LineSegment::calcSupportingLine(getX1(), getY1(), getX2(), getY2(), alpha, y0);
  LineSegment::calcEndpointsOfSupportingLine(alpha, y0, image->height(), image->width(), x1, y1, x2, y2);
  
  CvPoint minLine, maxLine;
  CvPoint minSL, maxSL;
  
  CvScalar s;
  
  // x-value is major
  if ( abs(alpha) < M_PI/4 ){
    ///// sort ending points of line and supporting line increasingly according to x-coordinate  /////
    if(getX1() < getX2()){
      minLine.x = (int) getX1();
      minLine.y = (int) getY1();
      maxLine.x = (int) getX2();
      maxLine.y = (int) getY2();
    }
    else{
      minLine.x = (int) getX2();
      minLine.y = (int) getY2();
      maxLine.x = (int) getX1();
      maxLine.y = (int) getY1();
    }
    if(x1 < x2){
      minSL.x = (int) x1;
      minSL.y = (int) y1;
      maxSL.x = (int) x2;
      maxSL.y = (int) y2;
    }
    else{
      minSL.x = (int) x2;
      minSL.y = (int) y2;
      maxSL.x = (int) x1;
      maxSL.y = (int) y1;
    }
    
    CvSize size;
    size.width=image->width();
    size.height=image->height();
    cvClipLine(size, &minLine, &maxLine);
    cvClipLine(size, &minSL, &maxSL);
    
    ///// go from minLine to minSL /////
    CvLineIterator lineIt;
    int length = cvInitLineIterator( *image, minLine, minSL, &lineIt, 8, 0);

    // current ending point, initialized with origin ending point
    CvPoint newPoint;
    newPoint.x = minLine.x;
    newPoint.y = minLine.y;
    
    int old1, old2;
    if(minLine.y-dist >=0){
      s=cvGet2D(*image,minLine.y-dist,minLine.x);
      old1 = (int) s.val[0];
    }
    else{
      old1 = -1000;
    }
    
    if(minLine.y+dist < image->height()){
      s=cvGet2D(*image,minLine.y+dist,minLine.x);
      old2 = (int) s.val[0];
    }
    else{
      old2 = -1000;
    }
    
    for( int j = 0; j < length; j++ ){
      int goon=0;
      // calc point coordinates
      int off, x, y;
      IplImage* tmp = *image;
      off = lineIt.ptr - (uchar*)(tmp->imageData);
      y = off/tmp->widthStep;
      x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      
      int new1=-1000, new2=-1000;
      if(y-dist >= 0){
        s=cvGet2D(*image,y-dist,x);
        new1 = (int) s.val[0];
        if( abs(new1-old1) <= thresh) goon=1;
      }
      if(y+dist < image->height()){
        s=cvGet2D(*image,y+dist,x);
        new2 = (int) s.val[0];
        if( abs(new2-old2) <= thresh) goon=1;
      }
      
      if(goon && abs(new1-new2)>=distThresh){
        newPoint.x = x;
        newPoint.y = y;
      }
      else{
        break;
      }
      CV_NEXT_LINE_POINT(lineIt);
    } // for j
    
    minLine.x = newPoint.x;
    minLine.y = newPoint.y;
    
    ///// go from maxLine to maxSL /////
    length = cvInitLineIterator( *image, maxLine, maxSL, &lineIt, 8, 0);
    
    // current ending point, initialized with origin ending point
    newPoint.x = maxLine.x;
    newPoint.y = maxLine.y;
    
    if(minLine.y-dist >=0){
      s=cvGet2D(*image,minLine.y-dist,minLine.x);
      old1 = (int) s.val[0];
    }
    else{
      old1 = -1000;
    }
    
    if(minLine.y+dist < image->height()){
      s=cvGet2D(*image,minLine.y+dist,minLine.x);
      old2 = (int) s.val[0];
    }
    else{
      old2 = -1000;
    }
    
    for( int j = 0; j < length; j++ ){
      int goon=0;
      // calc point coordinates
      int off, x, y;
      IplImage* tmp = *image;
      off = lineIt.ptr - (uchar*)(tmp->imageData);
      y = off/tmp->widthStep;
      x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      
      int new1=-1000, new2=-1000;

      if(y-dist >= 0){
        s=cvGet2D(*image,y-dist,x);
        new1 = (int) s.val[0];
        if( abs(new1-old1) <= thresh) goon=1;
      }
      if(y+dist < image->height()){
        s=cvGet2D(*image,y+dist,x);
        new2 = (int) s.val[0];
        if( abs(new2-old2) <= thresh) goon=1;
      }
      
      if(goon && abs(new1-new2)>=distThresh){
        newPoint.x = x;
        newPoint.y = y;
      }
      else{
        break;
      }
      CV_NEXT_LINE_POINT(lineIt);
    } // for j
    
    maxLine.x = newPoint.x;
    maxLine.y = newPoint.y;

    setPoints(minLine.x, minLine.y, maxLine.x, maxLine.y);
    
  } 
  // y is major
  else{

    ///// sort ending points of line and supporting line increasingly according to y-coordinate  /////

    if(getY1() < getY2()){
      minLine.x = (int) getX1();
      minLine.y = (int) getY1();
      maxLine.x = (int) getX2();
      maxLine.y = (int) getY2();
    }
    else{
      minLine.x = (int) getX2();
      minLine.y = (int) getY2();
      maxLine.x = (int) getX1();
      maxLine.y = (int) getY1();
    }
    if(y1 < y2){
      minSL.x = (int) x1;
      minSL.y = (int) y1;
      maxSL.x = (int) x2;
      maxSL.y = (int) y2;
    }
    else{
      minSL.x = (int) x2;
      minSL.y = (int) y2;
      maxSL.x = (int) x1;
      maxSL.y = (int) y1;
    }
  }

  CvSize size;
  size.width=image->width();
  size.height=image->height();
  cvClipLine(size, &minLine, &maxLine);
  cvClipLine(size, &minSL, &maxSL);
    
  ///// go from minLine to minSL /////
  CvLineIterator lineIt;
  int length = cvInitLineIterator( *image, minLine, minSL, &lineIt, 8, 0);

    // current ending point, initialized with origin ending point
  CvPoint newPoint;
  newPoint.x = minLine.x;
  newPoint.y = minLine.y;
    
  int old1, old2;
  if(minLine.x-dist >=0){
    s=cvGet2D(*image,minLine.y,minLine.x-dist);
    old1 = (int) s.val[0];
  }
  else{
    old1 = -1000;
  }
    
  if(minLine.x+dist < image->width()){
    s=cvGet2D(*image,minLine.y,minLine.x+dist);
    old2 = (int) s.val[0];
  }
  else{
    old2 = -1000;
  }
    
  for( int j = 0; j < length; j++ ){
    int goon=0;
      // calc point coordinates
    int off, x, y;
    IplImage* tmp = *image;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      
    int new1=-1000, new2=-1000;

    if(x-dist >= 0){
      s=cvGet2D(*image,y, x-dist);
      new1 = (int) s.val[0];
      if( abs(new1-old1) <= thresh) goon=1;
    }
    if(x+dist < image->width()){
      s=cvGet2D(*image,y, x+dist);
      new2 = (int) s.val[0];
      if( abs(new2-old2) <= thresh) goon=1;
    }
      
    if(goon && abs(new1-new2)>=distThresh){
      newPoint.x = x;
      newPoint.y = y;
    }
    else{
      break;
    }
    CV_NEXT_LINE_POINT(lineIt);
  } // for j
    
  minLine.x = newPoint.x;
  minLine.y = newPoint.y;
    
  ///// go from maxLine to maxSL /////
  length = cvInitLineIterator( *image, maxLine, maxSL, &lineIt, 8, 0);
    
  // current ending point, initialized with origin ending point
  newPoint.x = maxLine.x;
  newPoint.y = maxLine.y;
    
  if(minLine.x-dist >=0){
    s=cvGet2D(*image,minLine.y,minLine.x-dist);
    old1 = (int) s.val[0];
  }
  else{
    old1 = -1000;
  }
    
  if(minLine.x+dist < image->width()){
    s=cvGet2D(*image,minLine.y,minLine.x+dist);
    old2 = (int) s.val[0];
  }
  else{
    old2 = -1000;
  }
    
  for( int j = 0; j < length; j++ ){
    int goon=0;
      // calc point coordinates
    int off, x, y;
    IplImage* tmp = *image;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
      
    int new1=-1000, new2=-1000;

    if(x-dist >= 0){
      s=cvGet2D(*image,y, x-dist);
      new1 = (int) s.val[0];
      if( abs(new1-old1) <= thresh) goon=1;
    }
    if(x+dist < image->width()){
      s=cvGet2D(*image,y, x+dist);
      new2 = (int) s.val[0];
      if( abs(new2-old2) <= thresh) goon=1;
    }
      
    if(goon && abs(new1-new2)>=distThresh){
      newPoint.x = x;
      newPoint.y = y;
    }
    else{
      break;
    }
    CV_NEXT_LINE_POINT(lineIt);
  } // for j
    
  maxLine.x = newPoint.x;
  maxLine.y = newPoint.y;

  setPoints(minLine.x, minLine.y, maxLine.x, maxLine.y);
  return 1;
}


bool LineSegment::growLine( Image* image, int gapThresh, int thresh, int maxOffset){

  int w=image->width();
  int h=image->height();
  
  double alpha, y0;

  double x1, y1, x2, y2;
  LineSegment::calcSupportingLine(getX1(), getY1(), getX2(), getY2(), alpha, y0);
  LineSegment::calcEndpointsOfSupportingLine(alpha, y0, image->height(), image->width(), x1, y1, x2, y2);
  
  CvPoint minLine, maxLine;
  CvPoint minSL, maxSL;
  
  // x-value is major
  if ( abs(alpha) < M_PI/4 ){
    ///// sort ending points of line and supporting line increasingly according to x-coordinate  /////

    if(getX1() < getX2()){
      minLine.x = (int) getX1();
      minLine.y = (int) getY1();
      maxLine.x = (int) getX2();
      maxLine.y = (int) getY2();
    }
    else{
      minLine.x = (int) getX2();
      minLine.y = (int) getY2();
      maxLine.x = (int) getX1();
      maxLine.y = (int) getY1();
    }
    if(x1 < x2){
      minSL.x = (int) x1;
      minSL.y = (int) y1;
      maxSL.x = (int) x2;
      maxSL.y = (int) y2;
    }
    else{
      minSL.x = (int) x2;
      minSL.y = (int) y2;
      maxSL.x = (int) x1;
      maxSL.y = (int) y1;
    }
    
    CvSize size;
    size.width=image->width();
    size.height=image->height();
    cvClipLine(size, &minLine, &maxLine);
    cvClipLine(size, &minSL, &maxSL);
  
    ///// go from minLine to minSL /////
    CvLineIterator lineIt;
    int gapCount=0;
    int length = cvInitLineIterator( *image, minLine, minSL, &lineIt, 8, 0);
    
    // current ending point, initialized with origin ending point
    CvPoint newPoint;
    newPoint.x = minLine.x;
    newPoint.y = minLine.y;
    
    for( int j = 0; j < length && gapCount<=gapThresh; j++ ){
      // calc point coordinates
      int off, x, y;
      IplImage* tmp = *image;
      off = lineIt.ptr - (uchar*)(tmp->imageData);
      y = off/tmp->widthStep;
      x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
    
      // current maxOffset (depends on length)
      int cmaxOffset = maxOffset;
      int startOffset = (y-cmaxOffset<0)?0:(y-cmaxOffset);
      int endOffset = (y+cmaxOffset>=h)?(h-1):(y+cmaxOffset);
      int maxKey = thresh;
      int maxY = 0;
      for(int k=startOffset; k<=endOffset; k++){
        
        int key = getKey(image, x,k, alpha);
        if( key>maxKey ){
          maxKey = key;
          maxY = k;
        }
      }
        
      if(maxKey > thresh){
        newPoint.x = x;
        newPoint.y = maxY;
        gapCount = 0;
      }
      else{
        gapCount++;
      }
      CV_NEXT_LINE_POINT(lineIt);
      
    } // for j
    
    minLine.x = newPoint.x;
    minLine.y = newPoint.y;
    
    ///// go from maxLine to maxSL /////
    gapCount=0;
    length = cvInitLineIterator( *image, maxLine, maxSL, &lineIt, 8, 0);
    
    // current ending point, initialized with origin ending point
    newPoint.x = maxLine.x;
    newPoint.y = maxLine.y;
    
    for( int j = 0; j < length && gapCount<=gapThresh; j++ ){
      // calc point coordinates
      int off, x, y;
      IplImage* tmp = *image;
      off = lineIt.ptr - (uchar*)(tmp->imageData);
      y = off/tmp->widthStep;
      x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
    
      // current maxOffset (depends on length)
      int cmaxOffset = maxOffset;
      int startOffset = (y-cmaxOffset<0)?0:(y-cmaxOffset);
      int endOffset = (y+cmaxOffset>=h)?(h-1):(y+cmaxOffset);
      int maxKey = thresh;
      int maxY = 0;
      for(int k=startOffset; k<=endOffset; k++){
        int key = getKey(image, x,k, alpha);
        if( key>maxKey ){
          maxKey = key;
          maxY = k;
        }
      }
        
      if(maxKey > thresh){
        newPoint.x = x;
        newPoint.y = maxY;
        gapCount = 0;
      }
      else{
        gapCount++;
      }
      CV_NEXT_LINE_POINT(lineIt);
      
    } // for j
    
    maxLine.x = newPoint.x;
    maxLine.y = newPoint.y;

    setPoints(minLine.x, minLine.y, maxLine.x, maxLine.y);
    
  } 
  // y is major
  else{
    //cout << "y is major\n";
    ///// sort ending points of line and supporting line increasingly according to y-coordinate  /////

    if(getY1() < getY2()){
      minLine.x = (int) getX1();
      minLine.y = (int) getY1();
      maxLine.x = (int) getX2();
      maxLine.y = (int) getY2();
    }
    else{
      minLine.x = (int) getX2();
      minLine.y = (int) getY2();
      maxLine.x = (int) getX1();
      maxLine.y = (int) getY1();
    }
    if(y1 < y2){
      minSL.x = (int) x1;
      minSL.y = (int) y1;
      maxSL.x = (int) x2;
      maxSL.y = (int) y2;
    }
    else{
      minSL.x = (int) x2;
      minSL.y = (int) y2;
      maxSL.x = (int) x1;
      maxSL.y = (int) y1;
    }
  }

  CvSize size;
  size.width=image->width();
  size.height=image->height();
  cvClipLine(size, &minLine, &maxLine);
  cvClipLine(size, &minSL, &maxSL);
  
  ///// go from minLine to minSL /////
  CvLineIterator lineIt;
  int gapCount=0;
  int length = cvInitLineIterator( *image, minLine, minSL, &lineIt, 8, 0);
    
    // current ending point, initialized with origin ending point
  CvPoint newPoint;
  newPoint.x = minLine.x;
  newPoint.y = minLine.y;
    
  for( int j = 0; j < length && gapCount<=gapThresh; j++ ){
      // calc point coordinates
    int off, x, y;
    IplImage* tmp = *image;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
    
      // current maxOffset (depends on length)
    int cmaxOffset = maxOffset;
    int startOffset = (x-cmaxOffset<0)?0:(x-cmaxOffset);
    int endOffset = (x+cmaxOffset>=w)?(w-1):(x+cmaxOffset);
    int maxKey = thresh;
    int maxX = 0;
    for(int k=startOffset; k<=endOffset; k++){
      int key = getKey(image, k,y, alpha);
       
      if( key>maxKey ){
        maxKey = key;
        maxX = k;
      }
    }
        
    if(maxKey > thresh){
      newPoint.x = maxX;
      newPoint.y = y;
      gapCount = 0;
    }
    else{
      gapCount++;
    }
    CV_NEXT_LINE_POINT(lineIt);
      
  } // for j
    
  minLine.x = newPoint.x;
  minLine.y = newPoint.y;
    
  ///// go from maxLine to maxSL /////
  gapCount=0;
  length = cvInitLineIterator( *image, maxLine, maxSL, &lineIt, 8, 0);
    
    // current ending point, initialized with origin ending point
  newPoint.x = maxLine.x;
  newPoint.y = maxLine.y;
    
  for( int j = 0; j < length && gapCount<=gapThresh; j++ ){
      // calc point coordinates
    int off, x, y;
    IplImage* tmp = *image;
    off = lineIt.ptr - (uchar*)(tmp->imageData);
    y = off/tmp->widthStep;
    x = (off - y*tmp->widthStep)/(sizeof(uchar) /* size of pixel */);
    
      // current maxOffset (depends on length)
    int cmaxOffset = maxOffset;
    int startOffset = (x-cmaxOffset<0)?0:(x-cmaxOffset);
    int endOffset = (x+cmaxOffset>=w)?(w-1):(x+cmaxOffset);
    int maxKey = thresh;
    int maxX = 0;
    for(int k=startOffset; k<=endOffset; k++){
      int key = getKey(image, k,y, alpha);
     
      if( key>maxKey ){
        maxKey = key;
        maxX = k;
      }
    }
        
    if(maxKey > thresh){
      newPoint.x = maxX;
      newPoint.y = y;
      gapCount = 0;
    }
    else{
      gapCount++;
    }
    CV_NEXT_LINE_POINT(lineIt);
      
  } // for j
    
  maxLine.x = newPoint.x;
  maxLine.y = newPoint.y;

  setPoints(minLine.x, minLine.y, maxLine.x, maxLine.y);
  
  return 1;
}


double LineSegment::compareGreyspace( vector<int>& greyA, vector<int>& greyB){
  int maxDist = 20;
  
  double max=0;
  int maxA=0, maxB=0;
  
  /*
  for(uint startA=0; startA<greyA.size(); startA++){
  for(uint startB=0; startB<greyB.size(); startB++){
  int indA=startA;
  int indB=startB;
  double sim=0;
  while(indA<greyA.size() && indB<greyB.size()){
  int dist = abs( greyA[indA] - greyB[indB] );
  if (dist > 10) sim -= dist;
  else sim += dist;
  indA++;
  indB++;
        //cout << "drinnen, sim=" << sim << endl;
}
  if( sim>max ){
  max = sim;
  maxA = startA;
  maxB = startB;
} 
}
}
  */
  
  // use fact, that one line should start at begining
  for(uint startA=0; startA<greyA.size(); startA++){
    uint startB=0;
    uint indA=startA;
    uint indB=startB;
    double sim=0;
    while(indA<greyA.size() && indB<greyB.size()){
      int dist = abs( greyA[indA] - greyB[indB] );
      if (dist > maxDist) sim -= dist;
      else sim += maxDist-dist;
      indA++;
      indB++;
      //cout << "drinnen, sim=" << sim << endl;
    }
    if( sim>max ){
      max = sim;
      maxA = startA;
      maxB = startB;
    } 
  }
  for(uint startB=0; startB<greyB.size(); startB++){
    uint startA=0;
    uint indA=startA;
    uint indB=startB;
    double sim=0;
    while(indA<greyA.size() && indB<greyB.size()){
      int dist = abs( greyA[indA] - greyB[indB] );
      if (dist > maxDist) sim -= dist;
      else sim += maxDist-dist;
      indA++;
      indB++;
      //cout << "drinnen, sim=" << sim << endl;
    }
    if( sim>max ){
      max = sim;
      maxA = startA;
      maxB = startB;
    } 
  }
  
  cout << "max="<<max << " startA="<<maxA << " startB="<<maxB << endl;
  return max;
  
}


double LineSegment::compareAvLR(LineSegment& lsA, LineSegment& lsB){
  double distL = abs(lsB.avL - lsA.avL);
  double distR = abs(lsB.avR - lsA.avR);
  return (distL + distR)/2;
}

void LineSegment::drawLine(Image* image, CvScalar color, int width){
  CvPoint p1,p2;
  p1.x = (int) x1;
  p1.y = (int) y1;
  p2.x = (int) x2;
  p2.y = (int) y2;
  
  cvLine( *image, p1, p2, color, width, 8, 0);
}

double LineSegment::mahaDistLines(LineSegment &lA, LineSegment &lB, double varEP, double varDir, double varGrey, double varVGrey){
  
  bool outputFlag=0;
  double tmp;
  
  double dist = 0;
  double diff;
  
  ////// distance of endpoints to the other line /////////
  CvPoint A1, A2, B1, B2;
  A1.x = (int) lA.getX1();
  A1.y = (int) lA.getY1();
  A2.x = (int) lA.getX2();
  A2.y = (int) lA.getY2();
  B1.x = (int) lB.getX1();
  B1.y = (int) lB.getY1();
  B2.x = (int) lB.getX2();
  B2.y = (int) lB.getY2();
  
  diff = distPointVector(A1, A2, B1);
  dist += diff*diff /varEP;
  
  diff = distPointVector(A1, A2, B2);
  dist += diff*diff /varEP;
  
  diff = distPointVector(B1, B2, A1);
  dist += diff*diff /varEP;
  
  diff = distPointVector(B1, B2, A2);
  dist += diff*diff /varEP;
  
  if(outputFlag){
    cout <<"\t\tdist dEP=" << dist<<endl;
    tmp=dist;
  }

  // average grey values
  diff = lA.avL - lB.avL;
  dist += diff*diff /varGrey;
  
  diff = lA.avR - lB.avR;
  dist += diff*diff /varGrey;
  
  if(outputFlag){
    cout << "\t\tdist dGrey=" << dist-tmp << " (greyOldL="<<lB.avL<<" greyNewL="<<lA.avL<<" | greyOldR="<<lB.avR<<" greyNewR="<<lA.avR<<")"<<endl;
    tmp = dist;
  }
  
  // variance of grey values parallel to the line
  diff = lA.varL - lB.varL;
  dist += diff*diff /varVGrey;

  diff = lA.varR - lB.varR;
  dist += diff*diff /varVGrey;
  
  if(outputFlag){
    cout << "\t\tdist dVGrey=" << dist-tmp << " (greyVarOldL="<<lB.varL<<" greyVarNewL="<<lA.varL<<" | greyVarOldR="<<lB.varR<<" greyVarNewR="<<lA.varR<<")"<<endl;
    tmp = dist;
  }
  
  // oriented direction of the line
  diff = abs(lA.getOAlpha() - lB.getOAlpha());
  diff = (diff < 2*M_PI-diff) ? diff : 2*M_PI-diff;
  dist += diff*diff /varDir;
  
  if(outputFlag){
    cout << "\t\tdist dDir=" << dist-tmp << " (oAlphaOld="<<lA.getOAlpha() << " oAlphaNew=" <<lB.getOAlpha()<<")"<<endl;
  }
  
  return dist;
}


int LineSegment::tryMergeLinesChiSq(LineSegment &lsA, LineSegment &lsB, double maxGap){
  //double chi2 = 2.71; // means: alpha(chi2) = 0.1 @ 1 DOF
  double chi2 = 10.83; // means: alpha(chi2) = 0.01 @ 1 DOF
  double chi2_2 = 5.99; // means: alpha(chi2) = 0.05 @ 2 DOF
  //double chi2_2 = 13.82; // means: alpha(chi2) = 0.01 @ 2 DOF
  
  
  // test if the gap is to big
  if(sizeOfGap(lsA, lsB) > maxGap) return 2;
  
  // test the angles
  /*
  double diffAlpha = lsB.getAlpha()-lsA.getAlpha();
  //double diffAlpha = lsA.getAlpha() + lsB.getAlpha();

  if (diffAlpha > M_PI/2) diffAlpha -= M_PI;
  else if (diffAlpha < -M_PI/2) diffAlpha += M_PI;
  */
  
  double diffAlpha = lsB.getOAlpha()-lsA.getOAlpha();
  if (diffAlpha > M_PI) diffAlpha -= 2*M_PI;
  else if (diffAlpha < -M_PI) diffAlpha += 2*M_PI;
  
  double mahaDist = chi2; // dafault value, so that test is failed
  double covAlpha = lsA.covAlpha+lsB.covAlpha;

  mahaDist = diffAlpha * 1/covAlpha * diffAlpha;
  
  double diffRho,covRho, covRA, mahaDist2 = chi2_2;
  double J1,J2,J3,J4,J5,J6;


  double a,b,c,d,e,f2,f,invF;
  // test distance of lines
  if(mahaDist < chi2){
    // the test is not symmetric, it is more reliable if the shorter line defines the COS
    if(lsA.eucLength < lsB.eucLength){
      
      
      
      a = lsA.getX1()-lsB.getX1();
      b = lsA.getY1()-lsB.getY1();
      c = lsB.getX2()-lsB.getX1();
      d = lsB.getY2()-lsB.getY1();
      e = a*d - b*c;
      f2 = c*c+d*d;
      f = sqrt(f2);
      invF = 1/f;
  
      J1 = d/f;
      J2 = -c/f;
      J3 = (-d*f + invF*c*e) / f2;
      J4 = (c*f + invF*d*e) / f2;
      J5 = (-b*f - invF*c*e) / f2;
      J6 = (a*f - invF*d*e) / f2;
      
      covRho = J1*J1*lsA.covXX_M + 2*J1*J2*lsA.covXY_M + J2*J2*lsA.covYY_M + J3*J3*lsB.covXX_M + 2*J3*J4*lsB.covXY_M + J4*J4*lsB.covYY_M + J5*J5*lsB.covXX_M + 2*J5*J6*lsB.covXY_M + J6*J6*lsB.covYY_M;
      
      //covRho = 4;
      
      covRA = (J3-J5)*lsB.covXA_M + (J4-J6)*lsB.covYA_M - J1*lsA.covXA_M - J2*lsA.covYA_M;
      /*
      cout << "lsA.covXA="<<lsA.covXA<< " lsA.covYA="<<lsA.covYA<< " lsB.covXA="<<lsB.covXA<< " lsB.covYA="<<lsB.covYA<<endl;
      cout << "J1="<<J1<<" J2="<<J2<<" J3="<<J3<<" J4="<<J4<<" J5="<<J5<<" J6="<<J6<<endl;
      cout << "values a="<<a<<" b="<<b<<" c="<<c<<" d="<<d<<" e="<<e<<" f="<<f<<" f2="<<f2<<" invF="<<invF<<endl;

      cout << "==========> covRA="<<covRA<<endl;
      */
      diffRho = e/f;
      
      /*
      diffAlpha = abs(diffAlpha);
      diffRho = abs(diffRho);
      covRA = -abs(covRA);
      */
      
      if( (lsA.getAlpha()<lsB.getAlpha() && e>0) || (lsA.getAlpha()>lsB.getAlpha() && e<0) ){
        //cout << "old covRA="<<covRA<<endl;
        diffAlpha =- diffAlpha;
        covRA = -(J3-J5)*lsB.covXA_M - (J4-J6)*lsB.covYA_M + J1*lsA.covXA_M + J2*lsA.covYA_M;
        //cout << "========>CHANGED"<<endl;
      }
      
      //mahaDist2 = diffRho * 1/covRho * diffRho;
      mahaDist2 = (1/(covRho*covAlpha - covRA*covRA)) * (diffRho*diffRho*covAlpha + diffAlpha*diffAlpha*covRho - 2*diffAlpha*diffRho*covRA);
      //cout << "lsA.eucLength < lsB.eucLength"<<endl;
    }
    else{
      //change order in calculation of diffAlpha
      diffAlpha = -diffAlpha;
      
      a = lsB.getX1()-lsA.getX1();
      b = lsB.getY1()-lsA.getY1();
      c = lsA.getX2()-lsA.getX1();
      d = lsA.getY2()-lsA.getY1();
      e = a*d - b*c;
      f2 = c*c+d*d;
      f = sqrt(f2);
      invF = 1/f;
  
      J1 = d/f;
      J2 = -c/f;
      J3 = (-d*f + invF*c*e) / f2;
      J4 = (c*f + invF*d*e) / f2;
      J5 = (-b*f - invF*c*e) / f2;
      J6 = (a*f - invF*d*e) / f2;
      
      covRho = J1*J1*lsB.covXX_M + 2*J1*J2*lsB.covXY_M + J2*J2*lsB.covYY_M + J3*J3*lsA.covXX_M + 2*J3*J4*lsA.covXY_M + J4*J4*lsA.covYY_M + J5*J5*lsA.covXX_M + 2*J5*J6*lsA.covXY_M + J6*J6*lsA.covYY_M;
      
      //covRho = 4;

      
      covRA = (J3-J5)*lsA.covXA_M + (J4-J6)*lsA.covYA_M + J1*lsB.covXA_M + J2*lsB.covYA_M;
       
      diffRho = e/f;
      
      if( (lsB.getAlpha()<lsA.getAlpha() && e>0) || (lsB.getAlpha()>lsA.getAlpha() && e<0) ){
        diffAlpha =- diffAlpha;
        covRA = -(J3-J5)*lsA.covXA - (J4-J6)*lsA.covYA + J1*lsB.covXA + J2*lsB.covYA;
      }
      
      //mahaDist2 = diffRho * 1/covRho * diffRho;
      mahaDist2 = (1/(covRho*covAlpha - covRA*covRA)) * (diffRho*diffRho*covAlpha + diffAlpha*diffAlpha*covRho - 2*diffAlpha*diffRho*covRA);
      //cout << "else"<<endl;

    }
  }
    
  
  /////////////////// merge ///////////////////
  if(mahaDist2 < chi2_2){
   
   
    if(abs(diffRho)>5){
     // merging test is too optimistic, just delete the shorter line
      if(lsA.getEucLength() < lsB.getEucLength()){
        lsA = lsB;
      }
      return 1;
     /*
      cout << "diffAlpha="<<diffAlpha<<endl;
      cout << "mahDist="<<mahaDist<<endl;
      cout << "                diffRho="<<diffRho<<endl;
      cout << "                mahaDist2="<<mahaDist2<<endl;
      cout << "lines"<<lsA.getX1()<<", "<<lsA.getY1()<<", "<<lsA.getX2()<<", "<<lsA.getY2()<<", "<<lsB.getX1()<<", "<<lsB.getY1()<<", "<<lsB.getX2()<<", "<<lsB.getY2()<<", "<<endl;
    
      cout << "covRho="<<covRho<<endl;
      cout << "covRA="<<covRA<<endl;
      cout << "covAlpha="<<covAlpha<<endl;
     */
    /*
      cout << "values a="<<a<<" b="<<b<<" c="<<c<<" d="<<d<<" e="<<e<<" f="<<f<<endl;
      cout << "lsA.covXA="<<lsA.covXA<< " lsA.covYA="<<lsA.covYA<< " lsB.covXA="<<lsB.covXA<< " lsB.covYA="<<lsB.covYA<<endl;
      cout << "lsA.covXX="<<lsA.covXX<<" lsA.covXY="<<lsA.covXY<<" lsA.covYY="<<lsA.covYY<<endl;    
      cout << "lsB.covXX="<<lsB.covXX<<" lsb.covXY="<<lsB.covXY<<" lsB.covYY="<<lsB.covYY<<endl;
    */
    }
    else{
    // realy merge the lines
      mergeLinesProb(lsA, lsB);
    //mergeLinesEP(lsA, lsB);
      return 3;
    
    /*
    // just ignore the shorter line
      if(lsA.getEucLength() < lsB.getEucLength()){
      lsA = lsB;
    }
      return 1;
    */
    }
  }
  else{
    return 2;
  }

}


void LineSegment::mergeLinesProb(LineSegment& lsA, LineSegment& lsB){
  
    
    // new lines centroid (weighted midpoint)
  double sumL = lsA.eucLength+lsB.eucLength;
  double gx = (lsA.eucLength * lsA.getMx() + lsB.eucLength * lsB.getMx()) / sumL;
  double gy = (lsA.eucLength * lsA.getMy() + lsB.eucLength * lsB.getMy()) / sumL;
    
    // new lines orientation
  double aaTmp=lsA.getAlpha(); // = lsA.getAlpha()>=0 ? lsA.getAlpha() : lsA.getAlpha()+M_PI;
  double baTmp=lsB.getAlpha(); // = lsB.getAlpha()>=0 ? lsB.getAlpha() : lsB.getAlpha()+M_PI;
    
  if(baTmp-aaTmp < -M_PI/2) baTmp += M_PI;
  else if(aaTmp-baTmp < -M_PI/2) aaTmp += M_PI;
    
  double gAlpha = (lsA.eucLength*aaTmp + lsB.eucLength*baTmp) / sumL;
    
    // normalized direction vector of new line
  double fx = cos(gAlpha);
  double fy = sin(gAlpha);
    
    // multipliers
  double tA1 = (lsA.getX1()-gx)*fx + (lsA.getY1()-gy)*fy;
  double tA2 = (lsA.getX2()-gx)*fx + (lsA.getY2()-gy)*fy;
  double tB1 = (lsB.getX1()-gx)*fx + (lsB.getY1()-gy)*fy;
  double tB2 = (lsB.getX2()-gx)*fx + (lsB.getY2()-gy)*fy;
    
    // chose the extremal projected endpoints
  double minTA, maxTA, minTB, maxTB, minT, maxT;
  if(tA1<tA2){
    minTA = tA1;
    maxTA = tA2;
  }
  else{
    minTA = tA2;
    maxTA = tA1;
  }
  if(tB1<tB2){
    minTB = tB1;
    maxTB = tB2;
  }
  else{
    minTB = tB2;
    maxTB = tB1;
  }
    
  minT = (minTA<minTB) ? minTA : minTB;
  maxT = (maxTA>maxTB) ? maxTA : maxTB;
    
    // update lsA
  lsA.setPoints(gx+minT*fx, gy+minT*fy, gx+maxT*fx, gy+maxT*fy, 0);
}


int LineSegment::mergeLinesEP(LineSegment& lsA, LineSegment& lsB){
  // flag to dictinguish between a clear version (=1)(for testing changes) and a slightly faster and more complex version (=0) with the very same result
  bool clearVersionFlag=0;
  
  // clear version
  if(!clearVersionFlag){
    CvPoint minA, maxA, minB, maxB;           // container for ending points
    int minKeyA, maxKeyA, minKeyB, maxKeyB;   // container for deciding values (depending on slope, they are x or y values)
      
    /////////////////// initialise containers /////////////////////
      // x is major
    if( abs(lsA.alpha) < M_PI/4){ 
        // lsA
      if( lsA.getX1() < lsA.getX2() ){
        minA.x = (int) lsA.getX1();
        minA.y = (int) lsA.getY1();
        maxA.x = (int) lsA.getX2();
        maxA.y = (int) lsA.getY2();
        minKeyA = (int) lsA.getX1();
        maxKeyA = (int) lsA.getX2();
      }
      else{
        minA.x = (int) lsA.getX2();
        minA.y = (int) lsA.getY2();
        maxA.x = (int) lsA.getX1();
        maxA.y = (int) lsA.getY1();
        minKeyA = (int) lsA.getX2();
        maxKeyA = (int) lsA.getX1();
      }
        // lsB
      if( lsB.getX1() < lsB.getX2() ){
        minB.x = (int) lsB.getX1();
        minB.y = (int) lsB.getY1();
        maxB.x = (int) lsB.getX2();
        maxB.y = (int) lsB.getY2();
        minKeyB = (int) lsB.getX1();
        maxKeyB = (int) lsB.getX2();
      }
      else{
        minB.x = (int) lsB.getX2();
        minB.y = (int) lsB.getY2();
        maxB.x = (int) lsB.getX1();
        maxB.y = (int) lsB.getY1();
        minKeyB = (int) lsB.getX2();
        maxKeyB = (int) lsB.getX1();
      }
    }
    else{
        //lsA
      if( lsA.getY1() < lsA.getY2() ){
        minA.x = (int) lsA.getX1();
        minA.y = (int) lsA.getY1();
        maxA.x = (int) lsA.getX2();
        maxA.y = (int) lsA.getY2();
        minKeyA = (int) lsA.getY1();
        maxKeyA = (int) lsA.getY2();
      }
      else{
        minA.x = (int) lsA.getX2();
        minA.y = (int) lsA.getY2();
        maxA.x = (int) lsA.getX1();
        maxA.y = (int) lsA.getY1();
        minKeyA = (int) lsA.getY2();
        maxKeyA = (int) lsA.getY1();
      }
        // lsB
      if( lsB.getY1() < lsB.getY2() ){
        minB.x = (int) lsB.getX1();
        minB.y = (int) lsB.getY1();
        maxB.x = (int) lsB.getX2();
        maxB.y = (int) lsB.getY2();
        minKeyB = (int) lsB.getY1();
        maxKeyB = (int) lsB.getY2();
      }
      else{
        minB.x = (int) lsB.getX2();
        minB.y = (int) lsB.getY2();
        maxB.x = (int) lsB.getX1();
        maxB.y = (int) lsB.getY1();
        minKeyB = (int) lsB.getY2();
        maxKeyB = (int) lsB.getY1();
      }
    }
      
      
      // line A contains line B
    if(minKeyA<=minKeyB && maxKeyA>=maxKeyB){
      return 1;
    }
      
      // line B contains line A
    if(minKeyB<=minKeyA && maxKeyB>=maxKeyA){
      lsA = lsB;
      return 1;
    }
        
      // overlap: A first part
    if(minKeyA<=minKeyB && maxKeyA<=maxKeyB && maxKeyA>=minKeyB){
      lsA.setPoints( minA.x, minA.y, maxB.x, maxB.y,0);
      return 1;
    }
      
      // overlap: B first part
    if(minKeyB<=minKeyA && maxKeyB<=maxKeyA && maxKeyB>=minKeyA){
      lsA.setPoints( minB.x, minB.y, maxA.x, maxA.y,0);
      return 1;
    }
  } // if clearVersionFlag
  
  else{
  
    // faster version
    // have a look at the ending points
    int minA, maxA, minB, maxB;
      // if x major
    if( abs(lsA.alpha) < M_PI/4){ 
      if(lsA.getX1() < lsA.getX2()){
        minA = (int) lsA.getX1();
        maxA = (int) lsA.getX2();
      }
      else{
        minA = (int) lsA.getX2();
        maxA = (int) lsA.getX1();
      }
        
      if(lsB.getX1() < lsB.getX2()){
        minB = (int) lsB.getX1();
        maxB = (int) lsB.getX2();
      }
      else{
        minA = (int) lsB.getX2();
        maxA = (int) lsB.getX1();
      }
    }
    else{
      if(lsA.getY1() < lsA.getY2()){
        minA = (int) lsA.getY1();
        maxA = (int) lsA.getY2();
      }
      else{
        minA = (int) lsA.getY2();
        maxA = (int) lsA.getY1();
      }
      if(lsB.getY1() < lsB.getY2()){
        minB = (int) lsB.getY1();
        maxB = (int) lsB.getY2();
      }
      else{
        minB = (int) lsB.getY2();
        maxB = (int) lsB.getY1();
      }
    }
      
      // if lsA contains lsB, ignore lsB
    if(minA<=minB && maxA>=maxB){
      return 1;
    }
      
      // if lsB contains lsA, ignore lsA
    if(minB<=minA && maxB>=maxA){
      lsA = lsB;
      return 1;
    }
      
      // if overlap, return long segment
    if(minA<=minB && maxB>=maxA && maxA>=minB){ 
      int x1New, y1New, x2New, y2New;
        
        // if x major
      if( abs(lsA.alpha) < M_PI/4){
        if( lsA.getX1() < lsA.getX2()){
          x1New = (int) lsA.getX1();
          y1New = (int) lsA.getY1();
        }
        else{
          x1New = (int) lsA.getX2();
          y1New = (int) lsA.getY2();
        }
        if( lsB.getX1() > lsB.getX2()){
          x2New = (int) lsB.getX1();
          y2New = (int) lsB.getY1();
        }
        else{
          x2New = (int) lsB.getX2();
          y2New = (int) lsB.getY2();
        }
      }
      else{
        if( lsA.getY1() < lsA.getY2()){
          x1New = (int) lsA.getX1();
          y1New = (int) lsA.getY1();
        }
        else{
          x1New = (int) lsA.getX2();
          y1New = (int) lsA.getY2();
        }
        if( lsB.getY1() > lsB.getY2()){
          x2New = (int) lsB.getX1();
          y2New = (int) lsB.getY1();
        }
        else{
          x2New = (int) lsB.getX2();
          y2New = (int) lsB.getY2();
        }
      }
      LineSegment newLine;
      newLine.setPoints(x1New, y1New, x2New, y2New, 0);
      lsA = newLine;
      return 3;
    }
      
    if(minB<=minA && maxA>=maxB && maxB>=minA ){
      int x1New, y1New, x2New, y2New;
        
        // if x major
      if( abs(lsA.alpha) < M_PI/4){
        if( lsA.getX1() > lsA.getX2()){
          x1New = (int) lsA.getX1();
          y1New = (int) lsA.getY1();
        }
        else{
          x1New = (int) lsA.getX2();
          y1New = (int) lsA.getY2();
        }
        if( lsB.getX1() < lsB.getX2()){
          x2New = (int) lsB.getX1();
          y2New = (int) lsB.getY1();
        }
        else{
          x2New = (int) lsB.getX2();
          y2New = (int) lsB.getY2();
        }
      }
      else{
        if( lsA.getY1() > lsA.getY2()){
          x1New = (int) lsA.getX1();
          y1New = (int) lsA.getY1();
        }
        else{
          x1New = (int) lsA.getX2();
          y1New = (int) lsA.getY2();
        }
        if( lsB.getY1() < lsB.getY2()){
          x2New = (int) lsB.getX1();
          y2New = (int) lsB.getY1();
        }
        else{
          x2New = (int) lsB.getX2();
          y2New = (int) lsB.getY2();
        }
      }
      LineSegment newLine;
      newLine.setPoints(x1New, y1New, x2New, y2New, 0);
      lsA = newLine;
      return 3;
    }
      
      // if gap, have a look at size of the gap
    if( maxA<minB || maxB<minA ){
      return 2;
    }
  } // else clearVersionFlag
  return 2;
}


double LineSegment::sizeOfGap(LineSegment& lsA, LineSegment& lsB){
  
  CvPoint minA, maxA, minB, maxB;           // container for ending points
  int minKeyA, maxKeyA, minKeyB, maxKeyB;   // container for deciding values (depending on slope, they are x or y values)
    
  /////////////////// initialise containers /////////////////////
    // x is major
  if( abs(lsA.alpha) < M_PI/4){ 
      // lsA
    if( lsA.getX1() < lsA.getX2() ){
      minA.x = (int) lsA.getX1();
      minA.y = (int) lsA.getY1();
      maxA.x = (int) lsA.getX2();
      maxA.y = (int) lsA.getY2();
      minKeyA = (int) lsA.getX1();
      maxKeyA = (int) lsA.getX2();
    }
    else{
      minA.x = (int) lsA.getX2();
      minA.y = (int) lsA.getY2();
      maxA.x = (int) lsA.getX1();
      maxA.y = (int) lsA.getY1();
      minKeyA = (int) lsA.getX2();
      maxKeyA = (int) lsA.getX1();
    }
      // lsB
    if( lsB.getX1() < lsB.getX2() ){
      minB.x = (int) lsB.getX1();
      minB.y = (int) lsB.getY1();
      maxB.x = (int) lsB.getX2();
      maxB.y = (int) lsB.getY2();
      minKeyB = (int) lsB.getX1();
      maxKeyB = (int) lsB.getX2();
    }
    else{
      minB.x = (int) lsB.getX2();
      minB.y = (int) lsB.getY2();
      maxB.x = (int) lsB.getX1();
      maxB.y = (int) lsB.getY1();
      minKeyB = (int) lsB.getX2();
      maxKeyB = (int) lsB.getX1();
    }
  }
  else{
      //lsA
    if( lsA.getY1() < lsA.getY2() ){
      minA.x = (int) lsA.getX1();
      minA.y = (int) lsA.getY1();
      maxA.x = (int) lsA.getX2();
      maxA.y = (int) lsA.getY2();
      minKeyA = (int) lsA.getY1();
      maxKeyA = (int) lsA.getY2();
    }
    else{
      minA.x = (int) lsA.getX2();
      minA.y = (int) lsA.getY2();
      maxA.x = (int) lsA.getX1();
      maxA.y = (int) lsA.getY1();
      minKeyA = (int) lsA.getY2();
      maxKeyA = (int) lsA.getY1();
    }
      // lsB
    if( lsB.getY1() < lsB.getY2() ){
      minB.x = (int) lsB.getX1();
      minB.y = (int) lsB.getY1();
      maxB.x = (int) lsB.getX2();
      maxB.y = (int) lsB.getY2();
      minKeyB = (int) lsB.getY1();
      maxKeyB = (int) lsB.getY2();
    }
    else{
      minB.x = (int) lsB.getX2();
      minB.y = (int) lsB.getY2();
      maxB.x = (int) lsB.getX1();
      maxB.y = (int) lsB.getY1();
      minKeyB = (int) lsB.getY2();
      maxKeyB = (int) lsB.getY1();
    }
  }
    
  if(minKeyA>maxKeyB) return sqrt( (minA.x-maxB.x)*(minA.x-maxB.x) + (minA.y-maxB.y)*(minA.y-maxB.y) );
  if(minKeyB>maxKeyA) return sqrt( (minB.x-maxA.x)*(minB.x-maxA.x) + (minB.y-maxA.y)*(minB.y-maxA.y) );
  return 0;

}

double LineSegment::getEPPredictionByRhoThetaPrediction(double x1Old, double y1Old, double x2Old, double y2Old, double& x1Pred, double& y1Pred, double& x2Pred, double& y2Pred){
  if(!predFlag) return 0;

  double cosT = cos(thetaPred);
  double sinT = sin(thetaPred);
    
  x1Pred = rhoPred*cosT - sinT*(-x1Old*sinT + y1Old*cosT);
  y1Pred = rhoPred*sinT + cosT*(-x1Old*sinT + y1Old*cosT);
  x2Pred = rhoPred*cosT - sinT*(-x2Old*sinT + y2Old*cosT);
  y2Pred = rhoPred*sinT + cosT*(-x2Old*sinT + y2Old*cosT);
  
  return covRRpred;
}





