#ifndef LS_MISC_H
#define LS_MISC_H

#include <iostream>
#include <cmath> 
#include <sys/time.h>

#include <image/Image.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>

namespace jafar{
  namespace lines{
    /*
    Calculates inverse matrix using LU-factorization. 
    @ingroup lines
    
    boost::numeric::ublas::matrix<double> getInvMat(boost::numeric::ublas::matrix<double> A);
    */
    
    /*!
    Test if a neighbour of point with coordinates _x and _y is a non zero pixel.
    @ingroup lines
     */
    bool contourPtTest(jafar::image::Image* image, int x, int y);
    
    //! Calculates distance of a point to an infinite line defined by a vector @ingroup lines
    double distPointVector(CvPoint a, CvPoint b, CvPoint c);
    
    //! Calculates signed distance of a point to an infinite line defined by a vector (sign results of vector product (b-a) x c ) @ingroup lines
    double signedDistPointVector(CvPoint a, CvPoint b, CvPoint c);
    
    //! Calculates distance of two points @ingroup lines
    double distPtPt(CvPoint a, CvPoint b);
    
    //! Calculate gradient image
    /*!
    Calculate gradient image of image and store in image. Gradient image means sum of vertical and horizontal Sobel images with first derivatives.
    @ingroup lines
     */
    bool calcGradientImage(jafar::image::Image* image);
    
    
    //! Calculate gradient images in x and y direction
    /*!
    Calculates two gradient images, first with result of cvSobel in x direction and second in y direction.
    destX and destY have to be two [IPL_DEPTH_16S  JfrImage_CS_GRAY] Jafar-Images
    signedFlag indicates whether the reslut should be in range [-128..127] or [0..255]
    @ingroup lines
     */
    bool calcXYGradientImages(jafar::image::Image* image, jafar::image::Image* destX, jafar::image::Image* destY, int signedFlag);
    
    //! Calculate gradient image and image with orientation of the gradients.
    /*!
    Calculate two images, first is gradient image with absolute values, second is image with direction of gradients in each pixel. Gradients are calculated with cvSobel one time in x and one time in y direction.
    @param image the image that should be processed
    @param gradAbsImage storage for the gradients
    @param gradOriImage storage for the orientations of the gradients
    @ingroup lines
     */
    bool calcOrientedGradientImage(jafar::image::Image* image, jafar::image::Image* gradAbsImage, jafar::image::Image* gradOriImage);
    
    /*!
    Evaluates the line defined by p1 and p2 with respect to the gradients in image.
    
    Line should be oriented: p1->p2 left side is brighter (LineSegment::orientLine() could be useful)
    
    Line must be completly inside the gradient images (use cvClipLine).
    
    Returns average gradient value along the line.
    
    @param gradX gradients in x-direction
    @param gardY gradients in y-direction
    @param p1 start point of the line
    @param p2 end point of the line
    @param minGrad the minimal gradient (perpendicular to the line) of a line-pixel to be count as inlier, others are outliers
    @param lowRate Can be used to return 0 if the number of ouliers (gradient is to small) is to high (if outliers/(ouliers+inliers) > lowRate return 0) 
    @ingroup lines
     */
    double validLine(jafar::image::Image* gradX, jafar::image::Image* gradY, CvPoint p1, CvPoint p2, double minGrad=0, double lowRate=1);
    
    //! Calculate intersection point of two lines given by endpoints
    /*!
    Calculate intersection point line through p1 and p2 and line through p3 and p4 and store  intersection point.in s. Returns 1 if intersection point exists.
    @ingroup lines
     */
    int calcIntersec(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4, CvPoint& s);
    
    //! Starts a time measurement
    /*! 
    That function initializes a time measurement with use of gettimeofday() function. The return value can be used later as parameter for stopTimeMeasure() function to get the time difference in seconds.
    @ingroup lines
    */
    double startTimeMeasure();
    
    //! Evaluates time measurement
    /*!
    Can be used to measure the passed time since the startTimeMeasure() function was called. Therefore assign the return value of according startTimeMeasure() function to the parameter of this function.
    @ingroup lines
     */
    double stopTimeMeasure(double t1);
    
  } // namespace lines
} // namespace jafar


#endif
