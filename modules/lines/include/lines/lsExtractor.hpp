#ifndef LS_EXTRACTOR
#define LS_EXTRACTOR

#include <image/Image.hpp>
#include "lineSegment.hpp"
#include "lineSegmentSet.hpp"
#include "histogram.hpp"
#include "lsMisc.hpp"
#include "ctdef.hpp"

namespace jafar{
  namespace lines{
    
    //! Class with functions to extract line segments of an image
    /**
    This class provides a set of functions to extract line segments of an image. The idea is to extract all line segments contained in an image (global extraction). To re-extract a line from a prediction of that line (local extraction) have a look at LineSegment::fitLineOrientation() function and its Usage in the LsTracker class.
    
    
    To simply extract the set of lines of an image, the extractLineSegments() function can be used. It combines the extraction steps with the currently best performance.
    
    
    The functions of LsExtractor class works all with the same scheme: calculate gradient image or Canny image and find contour points by maxima suppression with hysteresis and find lines either with polygonalisation  There is a combination of the two OpenCV function for that task, but the contour representation that result by OpenCv cvFindContours runs a lot of contour points for multiple times if there are branched contours, thus the contours are very long. Therefore the well tested Calife functions for that task can be used alternatively. Some documentation of the Calife functions is contained in ctdef.hpp file. 
    The OpenCv polygonalization algorithm seems to be a little bit buggy (CVS version avaiable at January 2008), therefore a simple implementation of Douglas-Peucker Algorithm is provided.
    
    Further there is a function to apply a Hough-transformation to find lines.
    @ingroup lines
    */
    class LsExtractor{
      public:
        LsExtractor();
      
        //! Procced cvCanny on _image with member variables as Canny parameters
        /*!
         * Applies Canny edge detection on image and saves the result in cannyIm. 
         * The member variables are taken as parameters for Canny.
         */
        bool procCanny(jafar::image::Image* image, jafar::image::Image* cannyIm);
        
        //! Procced cvCanny on _image with function parameters as Canny parameters
        /*!
         * Applies Canny edge detection on image and saves the result in cannyIm.
         * The function parameters are taken as parameters for Canny
         */
        static bool procCannySt(jafar::image::Image* image, jafar::image::Image* cannyIm, int lowerThresh, int higherThresh, int ap);
        
        //! Extraction of line segments with memeber variables as parameters
        /**
         * Extracts lines from image and appends result to _segmentStorage. 
         * If only the resulting lines are desired to be contained in _segmentStorage, then _segmentStorage should be cleared before calling this function (LineSegmentSet::clear()). The _image is used as container for the Canny image, therefore it is changed within this function. As parameters for Canny the member variables are taken. 
         *
         * @param image Greyvalue image whose lines are extracted, image content is changed
         * @param cannyIm Storage for the resulting image of Canny edge detection
         * @param segmentStorage Storage for resulting line segments
         * @param thresh Minimum length of accepted lines.
         *
         */
        void extractLineSegments(jafar::image::Image* image, jafar::image::Image* cannyIm, LineSegmentSet& segmentStorage,double thresh=15);
        
        //! Extraction of line segments with function parameters as parameters
        /**
         * Extracts lines from image and appends result to _segmentStorage. 
         * If only the resulting lines are desired to be contained in _segmentStorage, then _segmentStorage should be cleared before calling this function (LineSegmentSet::clear()). The _image is used as container for the Canny image, therefore it is changed within this function. As parameters for Canny the function parameters are taken. 
         *
         * @param image Greyvalue image whose lines should be extracted, image content is changed
         * @param cannyIm Storage for the resulting image of Canny edeg detection
         * @param segmentStorage Storage for resulting line segments
         * @param thresh Minimum length of accepted lines.
         * @param lowerThresh Lower threshold for Canny edge detection
         * @param higherThresh Higher threshold for Canny edge detection
         * @param ap Aperture size for Canny edge detection
         *
         */
        static void extractLineSegmentsSt(jafar::image::Image* image, jafar::image::Image* cannyIm, LineSegmentSet& segmentStorage, double thresh=15, int lowerThresh=75, int higherThresh=150, int ap=3);
        
        //! Procced cvHoughLines2 on image and gives resulting set of lines
        /*!
         * Applies Hough on image and saves the 
         * result in image.
         *
         */
        bool procHough(jafar::image::Image* image, LineSegmentSet& newSegments);
        
        /**
         * Proceed cvFindContours and cvApproxPoly on image
         * Therefore the image has be a binary image (e.g. the result of cvCanny, 
         * have a look at the procCanny function)
         * To accept only large lineSegments a threshold is used
         *
         */
        bool procCvFindContour(jafar::image::Image* image, LineSegmentSet& newSegments, double thresh=15 );
        
        /** 
        Input image should either be canny or gradient image, attention: an image with height or width which is not a multiple of 4 slows down the function (because the whole image data must be copied)
        The content of the image is changed. All pixels, that are contained in a taken contour are set to 127. 
        The polygonal approximation is done with db() function, witch implements Douglas-Peucker algorithm.
        
        @param image Greyvalue image whose lines are extracted, image content is changed
        @param newSegments Storage for resulting line segments
        @param thresh Minimum length of accepted lines.
         */
        static void findLinesDP(jafar::image::Image* image, LineSegmentSet& newSegments, double thresh );
    
        /** 
        Input image should either be canny or gradient image, attention: an image with height or width which is not a multiple of 4 slows down the function (because the whole image data must be copied)
        The content of the image is changed. All pixels, that are contained in a taken contour are set to 127. 
         */
        static void findLinesCalife(jafar::image::Image* image, LineSegmentSet& newSegments, double thresh=15 );
        
        /**
        Recursive function for polygonal approximation of a set of points. The points are given by x and y coordinates. The used algorithm is Douglas-Peucker.
        Designed for use with ExtractContours1 function.
         */
        static int dp(short *x, short *y, int startIdx, int endIdx, std::vector<CvPoint> &endpoints, double thresh=1);
    
        void setCannyLowerThresh(int lowerThresh){ cannyLowerThresh = lowerThresh;}
        void setCannyHigherThresh(int higherThresh){ cannyHigherThresh = higherThresh;}
        void setCannyAp(int ap){ if(ap>0) cannyAp = ap; } 
        void setCannyPara(int lowerThresh, int higherThresh, int ap){
          cannyLowerThresh = lowerThresh;
          cannyHigherThresh = higherThresh;
          cannyAp = ap;
        }
        
        void setHoughMethod(int method){ houghMethod = method;} 
        void setHoughRho(double rho){ if(rho>0) houghRho = rho;}
        void setHoughTheta(double theta){ if(theta>0) houghTheta = theta;}
        void setHoughThreshold(int thresh){ houghThreshold = thresh;}
        void setHoughParam1(double param){ houghParam1 = param;}
        void setHoughParam2(double param){ houghParam2 = param;}
        void setHoughPara(int method, double rho, double theta, int thresh, int param1, int param2){
          houghMethod = method;
          if(rho>0)houghRho = rho;
          if(theta>0) houghTheta = theta;
          houghThreshold = thresh;
          houghParam1 = param1;
          houghParam2 = param2;
        }
        
      protected:
    
        // parameters for Canny
        int cannyLowerThresh;   //!< Parameter for cvCanny
        int cannyHigherThresh;  //!< Parameter for cvCanny
        int cannyAp;            //!< Parameter for cvCanny
        
        // parameters for Hough
        int houghMethod;      //!<Parameter for cvHoughLines2
        double houghRho;      //!<Parameter for cvHoughLines2
        double houghTheta;    //!<Parameter for cvHoughLines2
        int houghThreshold;   //!<Parameter for cvHoughLines2
        double houghParam1;   //!<Parameter for cvHoughLines2
        double houghParam2;   //!<Parameter for cvHoughLines2
    };
    
  } // namespace lines
} // namespace jafar

#endif
