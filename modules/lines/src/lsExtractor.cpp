#include "lines/lsExtractor.hpp"

using namespace std;
using namespace jafar;
using namespace image;
using namespace lines;

LsExtractor::LsExtractor(){
  // initialise parameters for Canny
  cannyLowerThresh = 130;
  cannyHigherThresh = 220;
  cannyAp = 3;
  
  // initialise parameters for Hough
  houghMethod = CV_HOUGH_STANDARD;
  houghRho = 1;
  houghTheta = CV_PI/180;
  houghThreshold = 80;
  houghParam1 = 0;
  houghParam2 = 0;
}



void LsExtractor::extractLineSegments(Image* image, Image* cannyIm, LineSegmentSet& segmentStorage, double thresh){
  procCanny(image, cannyIm);
  findLinesCalife(cannyIm,segmentStorage, thresh);
  //procCvFindContour(image,segmentStorage, thresh);
  //procHough(image,segmentStorage);
}


void LsExtractor::extractLineSegmentsSt(Image* image, Image* cannyIm, LineSegmentSet& segmentStorage, double thresh, int lowerThresh, int higherThresh, int ap){
  procCannySt(image, cannyIm, lowerThresh, higherThresh, ap);
  
  findLinesDP(cannyIm,segmentStorage, thresh);
  
  //findLinesCalife(cannyIm,segmentStorage, thresh);
  //procCvFindContour(image,segmentStorage, thresh);
  //procHough(image,segmentStorage);
}



void LsExtractor::findLinesDP(Image* image, LineSegmentSet& newSegments, double thresh ){
  
  double threshSq=thresh*thresh;
  IplImage* tmpIm = *image;
  IMCT *output = new IMCT;
  
  ///////////// extract contours //////////////////
  
  // if image height or length are not normalized (a multiple of 4) than the data refered by IplImage->imageDataOrigin is not ordered proper for use of ExtractContours1 function, therefore all the data has to be copied in a char array
  if( image->width()%4!=0 || image->height()%4!=0 ){   
    // change image format to char vector 
    char cImage[image->width()*image->height()];
    for(int i=0; i<image->height(); i++){
      for(int j=0; j<image->width(); j++){
        CvScalar s;
        s=cvGet2D(*image,i,j);
        cImage[i*image->width() + j] = (char)s.val[0];
      }
    }
    ExtractContours1(cImage, output, image->height() , image->width(), 0, 0, image->height(), image->width(), 80, 40, (int)thresh);
  }
  else{
    ExtractContours1(tmpIm->imageDataOrigin, output, image->height() , image->width(), 0, 0, image->height(), image->width(), 80, 40, (int)thresh);
  }
 
  ///////////// apply Douglas-Peucker algorithm to get lines /////////////////

  vector<CvPoint> endpoints;
  for( uint i=0; i < output->liste.size(); i++){
    endpoints.clear();
    dp( output->liste[i]->y, output->liste[i]->x, 0, output->liste[i]->npoints-1, endpoints, 1);
    
    for( uint j=0; j<endpoints.size()-1; j++){
      //cvLine( *image, endpoints[j], endpoints[j+1], CV_RGB(255,255,255), 1, 8, 0);
      
      double dx = endpoints[j].x - endpoints[j+1].x;
      double dy = endpoints[j].y - endpoints[j+1].y;
      // if long enough 
      if(dx*dx + dy*dy > threshSq ){
        LineSegment newLine;
        newLine.setPoints(endpoints[j].x, endpoints[j].y, endpoints[j+1].x, endpoints[j+1].y);
        // assign contour points to line points and use line only if there are only few points on line with no neighboured contour pixel (neccessary because of strange connecting lines of cvApproxPoly function) 
        if(newLine.assignContourPts(image)>0.9)
          newSegments.lineSegments.push_back( newLine );
      }
    }
  }
}


void LsExtractor::findLinesCalife(Image* image, LineSegmentSet& newSegments, double thresh ){
  
  double threshSq=thresh*thresh;
  IplImage* tmpIm = *image;
  IMCT *output = new IMCT;
  
  ///////////// extract contours //////////////////
  
  // if image height or length are not normalized (a multiple of 4) than the data refered by IplImage->imageDataOrigin is not ordered proper for use of ExtractContours1 function, therefore all the data has to be copied in a char array
  if( image->width()%4!=0 || image->height()%4!=0 ){   
    // change image format to char vector 
    char cImage[image->width()*image->height()];
    for(int i=0; i<image->height(); i++){
      for(int j=0; j<image->width(); j++){
        CvScalar s;
        s=cvGet2D(*image,i,j);
        cImage[i*image->width() + j] = (char)s.val[0];
      }
    }
    ExtractContours1(cImage, output, image->height() , image->width(), 0, 0, image->height(), image->width(), 80, 40, (int) thresh);
  }
  else{
    ExtractContours1(tmpIm->imageDataOrigin, output, image->height() , image->width(), 0, 0, image->height(), image->width(), 80, 40, (int) thresh);
  }
 
  //// put contours into openCV sequenz, calculate lines and store in newSegments ///////////

  CvSeq* chain = 0;  
  CvMemStorage* storageChain = cvCreateMemStorage(0);
  chain = cvCreateSeq(CV_SEQ_POLYGON, sizeof(CvContour), sizeof(CvPoint), storageChain);
  

  CvMemStorage* storageLines = cvCreateMemStorage(0);
  CvSeq* lines = 0;
  
  CvPoint tmp;
  for( uint i=0; i < output->liste.size(); i++){


    //cout << "new chain, npoints=" << output->liste[i]->npoints << endl;
    CvScalar color=CV_RGB( rand()%255, rand()%255, rand()%255);
    for( int j=0; j< output->liste[i]->npoints; j++){
      tmp.x=output->liste[i]->y[j];
      tmp.y=output->liste[i]->x[j];
      cvSeqPush(chain, &tmp);
      //cout << "point: "<<output->liste[i]->x[j] << "     " << output->liste[i]->y[j] <<endl;
      //cvLine( *colorImage, tmp, tmp, color, 1, 8, 0);

    }

   
    lines=cvApproxPoly( chain, sizeof(CvContour), storageLines, CV_POLY_APPROX_DP, 1, 0 );

    //cout << "lines total: " << lines->total << endl;
    for(int k=0; k<(lines->total)-1; k++){
      // extract endpoints from line-sequence
      CvPoint *p1, *p2;
      p1 = (CvPoint*)cvGetSeqElem(lines,k);
      p2 = (CvPoint*)cvGetSeqElem(lines,k+1);
      
      //cout << "p1: x: " << p1->x << " y: " << p1->y;
      //cout << "   p2: x: " << p2->x << " y: " << p2->y << endl;
      //cvLine( *image, *p1, *p2, CV_RGB(255,255,255), 1, 8, 0);
      double dx = p1->x - p2->x;
      double dy = p1->y - p2->y;
      // if long enough 
      if(dx*dx + dy*dy > threshSq ){
        //LineSegment newLine(10);
        LineSegment newLine;
        newLine.setPoints(p1->x, p1->y, p2->x, p2->y);
        // assign contour points to line points and use line only if there are only few points on line with no neighboured contour pixel (neccessary because of strange connecting lines of cvApproxPoly function) 
        if(newLine.assignContourPts(image)>0.9)
          newSegments.lineSegments.push_back( newLine );
      }

    }
    
    cvClearSeq(lines);
    cvClearSeq(chain);
  }
  ////////////// release memory ////////////////
  cvReleaseMemStorage(&storageLines);
  cvReleaseMemStorage(&storageChain);

}

bool LsExtractor::procCvFindContour( Image* image, LineSegmentSet& newSegments, double thresh ){
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvMemStorage* storageLines = cvCreateMemStorage(0);
  
  CvSeq* contour = 0;
  CvSeq* lines = 0;

  cvFindContours( *image, storage, &contour, sizeof(CvContour), CV_RETR_LIST,  CV_CHAIN_APPROX_NONE);

  int countAll = 0;
  int countTaken = 0;
  
  srand(time(NULL));
  //cout << "\t:::::: contour total: "<<contour->total << endl;
  
  double threshSq = thresh*thresh;
  //int dings=0;
  for( ; contour != 0; contour = contour->h_next )
  {
    
    // uncomment to draw contours on colorImage
    //CvScalar color = CV_RGB( rand()%255, rand()%255, rand()%255);
    //cvDrawContours( *colorImage, contour, color, color, -1, 1, 8 );
    
    countAll++;

    CvRect bndRect = cvBoundingRect(contour, 0);
    int key = (bndRect.width>bndRect.height) ? bndRect.width : bndRect.height;

    if (key>=thresh){
      lines=cvApproxPoly( contour, sizeof(CvContour), storageLines, CV_POLY_APPROX_DP, 3, 1 );
              
      //cout << "\t:::::: total: "<<lines->total << endl;
      
      CvPoint *p1, *p2;
      
      // lines-2 is neccessary because the last point of the polygon received by cvApproxPoly is the starting point
      for(int i=0; i<(lines->total-1); i++){
        // extract endpoints from line-sequence
        p1 = (CvPoint*)cvGetSeqElem(lines,i);
        p2 = (CvPoint*)cvGetSeqElem(lines,i+1);
        
        //cout << "p1: x: " << p1->x << " y: " << p1->y << endl;
        //cout << "p2: x: " << p2->x << " y: " << p2->y << endl;

        double dx = p1->x - p2->x;
        double dy = p1->y - p2->y;
        if(dx*dx + dy*dy > threshSq){
          countTaken++;
          LineSegment newLine(10);
          newLine.setPoints(p1->x, p1->y, p2->x, p2->y);
          newSegments.lineSegments.push_back( newLine );
        }
      }
    }
  }

  // release memory
  cvReleaseMemStorage(&storage);
  cvReleaseMemStorage(&storageLines);
  cout << "number of contours:\t" << countAll << endl << "number of taken:\t" << countTaken << endl;

  return 1;
}



bool LsExtractor::procCanny(Image* image, Image* cannyIm){
  if( image ){
    cvCanny( *image, *cannyIm, cannyLowerThresh, cannyHigherThresh, cannyAp);
    return 1;
  }
  else{
    return 0;
  }
}


bool LsExtractor::procCannySt(Image* image, Image* cannyIm, int lowerThresh, int higherThresh, int ap){
  if(image){
    cvCanny( *image, *cannyIm, lowerThresh, higherThresh, ap);
    return 1;
  }
  else{
    return 0;
  }
}


bool LsExtractor::procHough(Image* image, LineSegmentSet& newSegments){
  //cvHoughLines2( *baseImage, *baseImage);
  //cvHoughLines2
      //IplImage* dst = cvCreateImage( cvGetSize(src), 8, 1 );
      //IplImage* color_dst = cvCreateImage( cvGetSize(src), 8, 3 );
      CvMemStorage* storage = cvCreateMemStorage(0);
      CvSeq* lines = 0;
      int i;
      //cvCanny( src, dst, 50, 200, 3 );
      //cvCvtColor( dst, color_dst, CV_GRAY2BGR );

      
      /*CvSeq* cvHoughLines2( CvArr* image, void* line_storage, int method,
                            double rho, double theta, int threshold,
                            double param1=0, double param2=0 );*/
      
      
      //lines = cvHoughLines2( *baseImage, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 80, 0, 0 );
      if( houghMethod==0 || houghMethod==2){
      lines = cvHoughLines2( *image, storage, houghMethod, houghRho, houghTheta, houghThreshold, houghParam1, houghParam2 );
      
      cout << "\ti: " << lines->total << endl;
      
        CvPoint pt1, pt2;
        
        for( i = 0; i < lines->total; i++ ){
          float* line = (float*)cvGetSeqElem(lines,i);
          float rho = line[0];
          float theta = line[1];
          double a = cos(theta), b = sin(theta), c = tan(theta);
          if( fabs(b) < 0.001 )
          {
            pt1.x = pt2.x = cvRound(rho);
            pt1.y = 0;
            pt2.y = image->height();
          }
          else if( fabs(a) < 0.001 )
          {
            pt1.y = pt2.y = cvRound(rho);
            pt1.x = 0;
            pt2.x = image->width();
          }
          else
          {
            pt1.x = 0;
            pt1.y = cvRound(rho/b);
      
            if(pt1.y < 0)
            {
              pt1.x = cvRound(rho / a);
              pt1.y = 0;
            }
      
            if(pt1.y > image->height())
            {
              pt1.x = cvRound((pt1.y - image->height())*c);
              pt1.y = image->height();
            }
      
            pt2.x = image->width();
            pt2.y = cvRound(rho/b - image->width()/c);
            if(pt2.y < 0)
            {
              pt2.x = cvRound(rho/a);
              pt2.y = 0;
            }
            if(pt2.y > image->height())
            {
              pt2.x = cvRound(-1.0 * ((image->height() - rho/b) * c));
              pt2.y = image->height();
            }
          }
          
          
          
            LineSegment newLine;
            newLine.setPoints(pt1.x, pt1.y, pt2.x, pt2.y);
            newSegments.lineSegments.push_back( newLine );
          
          //cvLine( *colorImage, pt1, pt2, CV_RGB( rand()%255, rand()%255, rand()%255), 1, 8 );
        }
      } // if houghMethod == 0 || houghMethod==2
      
      
      if (houghMethod == 1){
        lines = cvHoughLines2( *image, storage, houghMethod, houghRho, houghTheta, houghThreshold, houghParam1, houghParam2 );
        cout << "\ti: " << lines->total << endl;
        for( i = 0; i < lines->total; i++ )
        {
          CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);

          LineSegment newLine(2);
          newLine.setPoints(line[0].x, line[0].y, line[1].x, line[1].y);
          newSegments.lineSegments.push_back( newLine );
          //cvLine( *colorImage, line[0], line[1], CV_RGB( rand()%255, rand()%255, rand()%255), 2, 8 );
        }
      }
  return 1;
}


int LsExtractor::dp(short *x, short *y, int startIdx, int endIdx, std::vector<CvPoint> &endpoints, double thresh){
  // parameters of line between startpoint and endpoint l = a + Lambda*b
  double ax = x[startIdx];
  double ay = y[startIdx];
  double bx = x[endIdx]-ax;
  double by = y[endIdx]-ay;
  // normalize direction vector
  double bl = sqrt(bx*bx+by*by);
  bx /= bl;
  by /= bl;
  
  // calculate point with maximal distance to the line
  double dist, maxDist=0;
  short maxIdx=0;
  for(short i=startIdx+1; i<endIdx; i++){
     dist = abs( (x[i]-ax)*by - (y[i]-ay)*bx );
     if(dist>maxDist){
       maxDist = dist;
       maxIdx = i;
     }
  }
  
  // if there the largest distance is greater thresh, split the line and apply dp on both parts
  if( maxDist>thresh){
    int nDP1 = dp(x, y, startIdx, maxIdx, endpoints, thresh);
    int nDP2 = dp(x, y, maxIdx, endIdx, endpoints, thresh);
    return nDP1+nDP2;
  }
  // else create two CvPoints as endpoints for this line and insert them in endpoints
  else{
    CvPoint ep1, ep2;
    ep1.x = x[startIdx];
    ep1.y = y[startIdx];
    ep2.x = x[endIdx];
    ep2.y = y[endIdx];
    
    endpoints.push_back(ep1);
    endpoints.push_back(ep2);
    
    return 1;
  }
  
}
