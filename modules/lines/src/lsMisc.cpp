# include "lines/lsMisc.hpp"

namespace ublas = boost::numeric::ublas;
using namespace std;
using namespace jafar;
using namespace image;
using namespace lines;

/*
ublas::matrix<double> getInvMat(ublas::matrix<double> A){
  // Invert A using LU Factorization (assume a is non-singular)
  // code taken from http://www.cs.colostate.edu/~nate/tutorial/tutorial.html#tab3
  ublas::matrix<double> A_inv(A.size1(), A.size2());
  ublas::permutation_matrix<std::size_t> pm(A.size1());
  ublas::lu_factorize(A,pm);
  A_inv.assign(ublas::identity_matrix<double>(A.size1()));
  ublas::lu_substitute(A, pm, A_inv);
  return A_inv;
}
*/  

double jafar::lines::validLine(Image* gradX, Image* gradY, CvPoint p1, CvPoint p2, double minGrad, double lowRate){
  double accumGrad=0;
  int countFalse=0;
  int countAll=0;
  
  // direction of the gradient determined by direction of line (line should be oriented)
  int dx = (int) (p1.x-p2.x);
  int dy = (int) (p1.y-p2.y);
  double lineGradDirX = -dy;
  double lineGradDirY = dx;
  // normalise gradient vector of the line
  double gradNorm = sqrt(lineGradDirX*lineGradDirX + lineGradDirY*lineGradDirY);
  lineGradDirX /= gradNorm;
  lineGradDirY /= gradNorm;
  
  ////////// go from p1 to p2 and calculate gradients /////////////
  
  // init line iterator on gradX
  CvLineIterator lineIt;
  int l = cvInitLineIterator( *gradX, p1, p2, &lineIt, 8, 0);
  
  IplImage* tmp = *gradX;
  int offset, tmpX, tmpY;   // variables to calculate coordinates from iterator
  CvScalar s;
  //cout << "lineGradDirX="<<lineGradDirX<<" lineGradDirY="<<lineGradDirY<<endl;
  for( int i = 0; i < l; i++ ){
    countAll++;
    // calculate point coordinates
    offset = lineIt.ptr - (uchar*)(tmp->imageData);
    tmpY = offset/tmp->widthStep;
    tmpX = (offset - tmpY*tmp->widthStep)/(sizeof(uchar)*2 /* size of pixel */);
    
    // calculate directed gradient |g| = <lineGradDir, [gradX gradY]> / |[gradX gradY]|
    s=cvGet2D(*gradX,tmpY,tmpX);
    double gradXval = s.val[0];
    s=cvGet2D(*gradY,tmpY,tmpX);
    double gradYval = s.val[0];
    
    double oGrad = (gradXval*lineGradDirX + gradYval*lineGradDirY);
    accumGrad += oGrad;
    if(oGrad<minGrad) countFalse++;
    //cout << "grad["<<tmpY<<"]["<<tmpX<<"]=" << gradXval*lineGradDirX + gradYval*lineGradDirY<<" (x="<<gradXval<<", y="<<gradYval<<" ";
    CV_NEXT_LINE_POINT(lineIt);
  }

  if((double)countFalse/countAll <= lowRate){
    return accumGrad/countAll;
  }
  else{
    //cout << "\tto much low gradients (grad="<<accumGrad/l<<"\n";
    return 0;
  }
}

/**
  Test if a neighbour of point with coordinates _x and _y is a non zero pixel
*/
bool jafar::lines::contourPtTest(Image* image, int x, int y){
  int tx, ty;
  // check for all neighboured pixels if they are on a contour
  for(int i=-1; i<=1;i++){
    for(int j=-1; j<=1; j++){
      // coordinates of current point to test
      tx = x+i;
      ty = y+j;
      if( tx>=0 && tx<image->width() && ty>=0 && ty<image->height()){
        CvScalar s;
        s=cvGet2D(*image,ty,tx);
        cout << "x=" <<tx << " y=" <<ty << " val="<<s.val[0] << endl;
        if(s.val[0] >0){
          return 1;
        }
      }
    } // for j
  } // for i
  return 0;
}

double jafar::lines::distPointVector(CvPoint a, CvPoint b, CvPoint c){
  int abX = b.x-a.x;
  int abY = b.y-a.y;
  int acX = c.x-a.x;
  int acY = c.y-a.y;
  double d = ((acX*abY)-(acY*abX))/sqrt(abX*abX+abY*abY);

  return abs(d);
}

double jafar::lines::signedDistPointVector(CvPoint a, CvPoint b, CvPoint c){
  int abX = b.x-a.x;
  int abY = b.y-a.y;
  int acX = c.x-a.x;
  int acY = c.y-a.y;
  double d = ((acX*abY)-(acY*abX))/sqrt(abX*abX+abY*abY);

  return d;
}

double jafar::lines::distPtPt(CvPoint a, CvPoint b){
  double t1=a.x-b.x;
  double t2=a.y-b.y;
  return sqrt(t1*t1 + t2*t2);
}


bool jafar::lines::calcGradientImage(Image* image){
  if(!image) return 0;
  
  // change uncommented part to apply sobel dx and dy too
  /*
  Image* laplace1 = new Image(image->width(),image->height() ,32,JfrImage_CS_GRAY);
  Image* laplace2 = new Image(image->width(),image->height() ,32,JfrImage_CS_GRAY);
  Image* laplace3 = new Image(image->width(),image->height() ,32,JfrImage_CS_GRAY);
  Image* laplace4 = new Image(image->width(),image->height() ,32,JfrImage_CS_GRAY);
  
  cvSobel(*image, *laplace1,0,1,3);
  cvSobel(*image, *laplace2,1,0,3);
  cvSobel(*image, *laplace3,1,1,3);
  
  cvAdd(*laplace1, *laplace2, *laplace4, 0);
  cvAdd(*laplace4, *laplace3, *laplace4, 0);
  cvConvertScaleAbs(*laplace4, *image, 0.3, 0);
  
  IplImage* tmp = *laplace1;
  cvReleaseImage(&tmp);
  tmp = *laplace2;
  cvReleaseImage(&tmp);
  tmp = *laplace3;
  cvReleaseImage(&tmp);
  tmp = *laplace4;
  cvReleaseImage(&tmp);
  return 1;
  */
  /*
  Image* laplace1 = new Image(image->width(),image->height() ,32,JfrImage_CS_GRAY);
  Image* laplace2 = new Image(image->width(),image->height() ,32,JfrImage_CS_GRAY);
  Image* laplace3 = new Image(image->width(),image->height() ,32,JfrImage_CS_GRAY);
  
  cvSobel(*image, *laplace1,0,1,3);
  cvSobel(*image, *laplace2,1,0,3);
  
  cvAdd(*laplace1, *laplace2, *laplace3, 0);
  cvConvertScaleAbs(*laplace3, *image, 0.25, 0);
  
  IplImage* tmp = *laplace1;
  cvReleaseImage(&tmp);
  tmp = *laplace2;
  cvReleaseImage(&tmp);
  tmp = *laplace3;
  cvReleaseImage(&tmp);
  return 1;
  */
  IplImage* image_sobel_x = cvCreateImage(cvGetSize(*image), IPL_DEPTH_32F, 1);
  IplImage* image_sobel_y = cvCreateImage(cvGetSize(*image), IPL_DEPTH_32F, 1);
  IplImage* image_f = cvCreateImage(cvGetSize(*image), IPL_DEPTH_32F, 1);
  cvConvert(*image, image_f);
  cvSobel(image_f, image_sobel_x, 1, 0); //x-direction
  cvSobel(image_f, image_sobel_y, 0, 1); //y-direction

  //then rescale back to 0-255 (only x done below).
  double maxVal, minVal;
  double scale, shift;
  cvMinMaxLoc( image_sobel_x, &minVal, &maxVal );
  scale = 255/(maxVal - minVal);
  shift = -minVal * scale;
  //cout << "x: shift=" << shift << " scale=" << scale << endl;
  cvAbs(image_sobel_x, image_sobel_x);
  cvConvertScale( image_sobel_x, image_sobel_x, scale, 0);
  
  cvMinMaxLoc( image_sobel_y, &minVal, &maxVal );
  scale = 255/(maxVal - minVal);
  shift = -minVal * scale;
  //cout << "y: shift=" << shift << " scale=" << scale << endl;
  cvAbs(image_sobel_y, image_sobel_y);
  cvConvertScale( image_sobel_y, image_sobel_y, scale, 0);
  
  cvAdd(image_sobel_x, image_sobel_y, image_f, 0);
  cvConvert(image_f, *image);

  cvReleaseImage(&image_sobel_x);
  cvReleaseImage(&image_sobel_y);
  cvReleaseImage(&image_f);
  return 1;
}


bool jafar::lines::calcXYGradientImages(Image* image, Image* destX, Image* destY, int signedFlag){

  cvSobel(*image, *destX, 1, 0); //x-direction
  cvSobel(*image, *destY, 0, 1); //y-direction
  if(signedFlag){
    //cvConvertScale( *destX, *destX, 0.25, 127);
    //cvConvertScale( *destY, *destY, 0.25, 127);
  }
  else{
    cvAbs(*destX, *destX);
    cvAbs(*destY, *destY);
  }
  return 1;

}


bool jafar::lines::calcOrientedGradientImage(Image* image, Image* gradAbsImage, Image* gradOriImage){
  if(!image) return 0;

  IplImage* image_sobel_x = cvCreateImage(cvGetSize(*image), IPL_DEPTH_32F, 1);
  IplImage* image_sobel_y = cvCreateImage(cvGetSize(*image), IPL_DEPTH_32F, 1);
  IplImage* image_f = cvCreateImage(cvGetSize(*image), IPL_DEPTH_32F, 1);
  cvConvert(*image, image_f);
  cvSobel(image_f, image_sobel_x, 1, 0); //x-direction
  cvSobel(image_f, image_sobel_y, 0, 1); //y-direction

  //then rescale back to 0-255 
  double maxVal, minVal;
  double scale, shift;
  cvMinMaxLoc( image_sobel_x, &minVal, &maxVal );
  scale = 255/(maxVal - minVal);
  shift = -minVal * scale;
  //cout << "x: shift=" << shift << " scale=" << scale << endl;
  cvAbs(image_sobel_x, image_sobel_x);
  cvConvertScale( image_sobel_x, image_sobel_x, scale, 0);
  
  cvMinMaxLoc( image_sobel_y, &minVal, &maxVal );
  scale = 255/(maxVal - minVal);
  shift = -minVal * scale;
  //cout << "y: shift=" << shift << " scale=" << scale << endl;
  cvAbs(image_sobel_y, image_sobel_y);
  cvConvertScale( image_sobel_y, image_sobel_y, scale, 0);
  
  cvAdd(image_sobel_x, image_sobel_y, image_f, 0);
  
  cvConvert(image_f, *gradAbsImage);

  IplImage* pImage = *image;
  
  //int height = pImage->height;
  //int width = pImage->width;
  int step = pImage->widthStep/sizeof(float);
  float * dataX = (float *)image_sobel_x->imageData;
  float * dataY = (float *)image_sobel_y->imageData;
  pImage = *gradOriImage;
  int step1 = pImage->widthStep/sizeof(uchar);
  uchar * data = (uchar *)pImage->imageData;
  float x;
  float y;
  uchar ori;
  
  //cout << "height="<<image->height()<<" width=" << image->width()<<endl;
  
  for(int i=0; i<image->height(); i++){
    for(int j=0; j<image->width(); j++){
      x = dataX[i*step+j];
      y = dataY[i*step+j];
      ori = (uchar) (255 * atan2(y,x));
      /*
      ATTENTION
      reordering is neccessary (should be done befor cvAbs(image_sobel_x)...
      */
      data[i*step1+j] = ori;
    }
  }
  cvReleaseImage(&image_sobel_x);
  cvReleaseImage(&image_sobel_y);
  cvReleaseImage(&image_f);
  return 1;
}



int jafar::lines::calcIntersec(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4, CvPoint& s){
  double x1 = p1.x;
  double y1 = p1.y;
  double x2 = p1.x - p2.x;
  double y2 = p1.y - p2.y;
  double x3 = p3.x;
  double y3 = p3.y;
  double x4 = p3.x - p4.x;
  double y4 = p3.y - p4.y;
  
  double divisor = y2*x4 - x2*y4;
  
  if(divisor == 0) return 0;
  
  double l = (x1*y4+x3*y4-y1*x4+y3*x4) / divisor;
  
  s.x = int(x1 + l*x2);
  s.y = int(y1 + l*y2);
  
  return 1;
}

double jafar::lines::startTimeMeasure(){
  timeval tim;
  gettimeofday(&tim, NULL);
  return tim.tv_sec+(tim.tv_usec/1000000.0);
}


double jafar::lines::stopTimeMeasure(double t1){
  timeval tim;
  gettimeofday(&tim, NULL);
  double t2=tim.tv_sec+(tim.tv_usec/1000000.0);
  return t2-t1;
}
    
    
