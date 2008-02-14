#ifndef LINE_SEGMENT
#define LINE_SEGMENT

#include <cmath>
#include <vector>

#include <image/Image.hpp>

#include "lines/histogram.hpp"
#include "lines/lsMisc.hpp"
#include "lines/lineSegmentSet.hpp"
#include "lines/lsPredictor2D.hpp"


namespace jafar{
  namespace lines{
    
    class LineSegmentSet;

    //! Class for a line segment with descriptor
    /*!
    This class contains attributes (e.g. coordinates of ending points, coordinates of midpoint, slope) of a line segment. 
    
    Attention, there are two kinds of length. In the length member the number of pixels on the line is stored, the eucLength member is the euclidean distance of endpoints. There are two kinds of orientation as well. The alpha member is the orientation of the line and oAlpha member is the directed orientation from endpoint (x1, y1) to (x2, y2).
    
    The very most attibutes are set in the setPoints() function. If only the direction of the line changed, but not the orientation or the coordinates of endpoints, the changePointOrder() function can be used.
    
    In the setPoints() function the covariances for endpoints and olar coordinates are calculated. They are calculated from assumed uncertainty parallel and perpendicular to the line.  There is a special covariance for the merging process which assumes the only uncertainty perpendicular to the line.
    
    Several containers for descriptors (e.g. graylevel, gradient, LaPlace of pixels underlying the line) are provided. Functions to extract the descriptors are provided as well. 
    Furthermore there is a storrage for lines that describe the neighbouhood of the line. The number of this parallel lines is arbitrary, this feature can also be ignored.  
    
    @ingroup lines
    
     */
    class LineSegment{
      public:
        //! Constructor without generating parallel lines
        /*!
        Simple constructor. Initializes color randomly. Number of parallels is 0.  
        */
        LineSegment();
      
        //! Constructor that generates parallel lines
        /*!
        Constructor. Initializes color randomly. Number of parallels is given by parameter. Memory for that lines is allocated, but the lines are not initialized, this is done by using the LineSegment::setPoints function for this line. In this function the coordinates for the ending points of parallel lines are calculated.
        @param _numberOfNeighbours  Initialisation for number of parallel lines to describe neighbourhood of the line segment. .
        */
        LineSegment(int _numberOfNeighbours);
        
        //! Copy-constructor
        LineSegment(const LineSegment& ls);
        
        //! Destructor
        ~LineSegment();
        
        //! Provides alpha and y0 of a line given by endpoints
        /*!
        This function calculates supporting line for a line segment given by two points.
        Alpha is the angle to positiv x-axis in radian and y is y coordinate of the
        intersection point with y-axis, if alpha==pi/2 then y0 is the x-coordinate of
        the intersection point with the x-axis (because there is probably no intersection
        point with y-axis).
        (origin of the image COS is the upper left corner, x-axis is horizontally and y-axis vertically)
 
        @param x1     x Coordinate of the first point
        @param y1     y Coordinate of the first point
        @param x2     x Coordinate of the first point
        @param y2     y Coordinate of the first point
        @param alpha  Reference, storage for angel to positiv x-axis (in radian)
        @param y0     Reference, storage for y-coordinate of intersection point with y-axis
        @returns 0 if points are equal, else returns 1
         */
        static bool calcSupportingLine(double x1, double y1, double x2, double y2, double& alpha, double& y0);
        
        //! Provides cross points of a supporting line with image borders
        /*!
        This function calculates intersectionpoints of suporting line with image boarders. That means the angel to positiv x-axis and the y-value of intersection point with y-axis of a line are given and the (x,y) coordinates of the intersection points with the image boarders are provided. Therefore the height and width of the image must be given. The coordinate frame is origined in the upper left corner, x-axis is horizontally and y-axis vertically. 

        @param alpha  Angel to positiv x-axis of the line (in radian).
        @param y0     y-value of intersection point with y-axis
        @param height Height of the image for that intersection points should be calculated
        @param width  Width of the image for that intersection points should be calculated
        @param x1     Reference, storage for x coordinate of first intersection point
        @param y1     Reference, storage for y coordinate of first intersection point
        @param x2     Reference, storage for x coordinate of second intersection point
        @param y2     Reference, storage for y coordinate of second intersection point
         */
        static bool calcEndpointsOfSupportingLine(double alpha, double y0, int height, int width, double& x1, double& y1, double& x2, double& y2);
        
        //!< Calculate Mahalanobis distance of two line segments
        /*!
        Calculate Mahalanobis distance of two line segments. The components of the state vector are assumed to be uncorrelated.  Line segments have to be oriented befor using this function. (Have a look at LineSegment::orientLine())
         */
        static double mahaDistLines(LineSegment &lA, LineSegment &lB, double varEP, double varDir, double varGrey, double varVGrey);
    
        //! Extracts grey values of pixels on the line segment
        /*!
        This function extracts grey values of pixels according to the line if the line would be drawn in image.
        To determine the according pixels Bresenham-algorithm (8-neighbourhood) is used. If parallel_flag!=0 the scale is also extracted for parallels (with respect to numberOfNeighbours). In that case the ending points and the length of the parallel lines are updated with respect to _image size. (This is done by the use of LineSegment::setPoints function.) If a parallel line lies completly out of image, it gets the same point coordinates of the next neighboured parallel line (or of the line itself if there is no neighboured parallel line).
 
        @param image Typ: jafar::image::Image*. Image that's greyvalues are used.
        @param parallel_flag If !=0 the scale is also extracted for parallels (with respect to numberOfNeighbours) 
         */
        void extractGreyscale(jafar::image::Image* image, int parallel_flag=0);
        
        //! Extracts gradient values of pixels on the line segment
        /*!
        Extracts gradient values of pixels according to the line if the line would be drawn on _image.
        To determine the according pixels Bresenham-algorithm (8-neighbourhood) is used.
        This function can deal with greyscale and gradien images. The flag _gradientimage must be set accordingly. If an greyscale image is given, the gradien image is calculated, that decreases speed of this function. If this function is called for serveral lines on one image, it's better to calculate the gradient image outside this function. If parallel_flag!=0 the scale is also extracted for parallels (with respect to numberOfNeighbours). In that case the ending points and the length of the parallel lines are updated with respect to _image size. (This is done by the use of LineSegment::setPoints function.
 
        @param image Typ: jafar::image::Image*. Can be a pointer to an greyscale image or to an gradient image. If it's an pointer to an greyscale image _gradientimage must be 0, otherwise _gradienimage must be !=0.
        @param gradientimage Flag to indicate if _image is an greyscale or an gradient image. 
        @param parallel_flag If !=0 the scale is also extracted for parallels (with respect to numberOfNeighbours) 
         */
        void extractGradientscale(jafar::image::Image* image, int gradientimage, int parallel_flag=0);
        
        //! Extracts Laplace values of pixels on the line segment
        /*!
        Extracts Laplace values of pixels according to the line if the line would be drawn on _image.
        To determine the according pixels Bresenham-algorithm (8-neighbourhood) is used.
        This function can deal with greyscale and Laplace images. The flag _laplacetimage must be set accordingly. If an greyscale image is given, the Laplace image is calculated, that decreases speed of this function. If this function is called for serveral lines on one image, it's better to calculate the gradient image outside this function. If parallel_flag!=0 the scale is also extracted for parallels (with respect to numberOfNeighbours). In that case the ending points and the length of the parallel lines are updated with respect to _image size. (This is done by the use of LineSegment::setPoints function.
 
        @param image Typ: jafar::image::Image*. Can be a pointer to an greyscale image or to an laplace image. If it's an pointer to an greyscale image _laplacetimage must be 0, otherwise _laplaceimage must be !=0.
        @param laplace Flag to indicate if _image is an greyscale or an laplace image. 
        @param parallel_flag If !=0 the scale is also extracted for parallels (with respect to numberOfNeighbours) 
         */
        void extractLaplacescale(jafar::image::Image* image, int laplaceimage, int parallel_flag=0);
        
        //! Sets point coordinates and initialises parallels.
        /*!
        This function sets points according to parameters. The coordinates of midpoint, slope, eucLength (Eulidean langth) and alpha ( angle to positiv x-axis) are calculated.
        Furthermore the parallel lines (LineSegment::parallels) are initialized with their setPoints-functions. The coordinates of start and endpoint of these parallel lines are calculated accordingly to the slope of the origin line. If abs(slope)<1 then the points are located above and under the origin points, otherwise left and right.
        The parallels are ordered alterning above - under (left - right) according to their index mod 2.
        oAlpha is set value according to direction (x1,y1)->(x2,y2) without respect to the image-gradients along the line, to set it according to the gradient, use orientLine()
        The resetFlag indicates if the predictor member is reinitialized.
         */
        void setPoints(double _x1, double _y1, double _x2, double _y2, bool resetFlag=1); 
        
        //! Assignment operator
        LineSegment& operator=(const LineSegment &ls);
        
        //! Sets numberOfNeighbours and updates parallels
        /*! 
        Set numberOfNeighbours member. parallels is updated according to the relation between the old numberOfNeighbours and _number. If the new number is smaller, the appropriate  parallels are deleted. If the new number is greater, the old parallels stay and memory for the new parallels is allocated. The LineSegment::setPoints function is called for the new parallels, but the desciptors are not set!

        @param number The new numberOfNeighbours

         */
        void setNumberOfNeighbours(int number);
        
        //! Assigns contour pixels to each point of the line
        /**
        This function assigns a list of contour pixels to each pixel of the line. This is usefull for calculating more robust descriptors. Returns the proportional share of pixels, that are neighboured to a contour pixel, range: [0...1]
  
        @param image Image should be a black and white immage with white pixels for all contour points (e.g. the result of Canny edge detection).
         */
        double assignContourPts(jafar::image::Image* image);
    
        //! Changes direction of line without changing the orientation or endpoint coordinates
        /**
        Changes direction of line without changing the orientation or endpoint coordinates. The oAlpha value some covariances are updated. The histograms are swap.
        */
        void changePointOrder();
        
        //! Orient line with respect to the average greylevels on both sides of the line.
        /*!
        Calculate average greylevel on both sides of the line (on a parallel line at the distance 2) and arange ending points such that from (x1,y1) to (y2,x2) the brighter side is on the left.
        The members avL and avR are updated. The member oAlpha is updated.
        If one of the parallels used to calculate the average grey value on left and right side is completly out of image, both grey values are 0.
         */
        void orientLine(jafar::image::Image* image);
    
        //! Compute histograms of line from image
        /**
        This function computes the histograms histogramB (left side) and histogramD (right side) for that line with respect to the graylevel values in image. The histograms are calculated on parallels with distance d to the line.
        */
        void calcHistogramDescriptor(jafar::image::Image* image, int d=3);
        
         //! Compute histograms of line form parallels vector
        /**
        This function computes the histograms histogramB (left side) and histogramD (right side) for that line with respect to the graylevel values in parallels. 
         */
        void calcHistogramDescriptor();
        
        //! Compute descriptor based on contour points acording to line points
        /*!
        Compute descriptor based on contour points acording to line points. To compute descriptor based on just line points, assign each line point to itself bevor calling this function. 
         */
        void calcGreyspaceDescriptor(jafar::image::Image* image);
    
        //! Compares the greyspace of two lines
        /*!
        Compares the greyspace of two lines. Lines have to be oriented, (have a look at orientLine() function) before the greyspace is extracted with calcGreyspaceDescriptor() function.
         */
        static double compareGreyspace( std::vector<int>& greyA, std::vector<int>& greyB);
        
        //! Compares the average grey levels of two lines
        /*!
        Compares the average grey levels of two lines independently for both sides. The two results are fusioned and returned. The avL and avR values of both lines have to be set befor, therefore have a look at orientLine() function.
         */
        static double compareAvLR(LineSegment& lsA, LineSegment& lsB);
        
        //!Fit line to gradient maxima, using RANSAC. Result is a single line
        /*!
        Fit line to gradient maxima, using RANSAC.Result is a single line.  This function does not care about the orientation of the gradients. To fit a line with respect to the orientation of the gradient, use fitLineOrientation.
         */
        bool fitLine(jafar::image::Image* image, int gradientimage = 0, int nPoints=20, int pDist=25, jafar::image::Image* colorImage=0);
        
        //!Fit line to gradient maxima, using RANSAC. Result is a set of candidate lines
        /*!
        Fit line to gradient maxima, using RANSAC.
        Similar to fitLine function, but here a number of candidate lines is stored in startPts and endPts and the number of lines is returned. The origin line is not changed
         */
        int fitLineCandidates(jafar::image::Image* image, int gradientimage,std::vector<CvPoint>& startPts, std::vector<CvPoint>& endPts, int nPoints, int pDist, int nIter, jafar::image::Image* colorImage);
        
        //!Fit line to gradient maxima with respect to gradient orientation.
        /*!
        Fit line to gradient maxima. gradX and gradY are [IPL_DEPTH_16S  JfrImage_CS_GRAY] Jafar-Images with gradient in x and y direction, the gradient has to be signed. 
        The function can handle lines that are partialy out of the image. Line has to be oriented (have a look at LineSegment::orientLine())
        nPoints has to be in [1, 16]
         */
        bool fitLineOrientation(LineSegmentSet& lsSet, jafar::image::Image* gradX, jafar::image::Image* gradY, int nPoints=10, int pDist=25, double aDist=M_PI*0.5, double minLength=0, double extend=0, jafar::image::Image* colorImage=0);
      
        //! This function searches for gradient maxima perpendicular to the line between p1 and p1
        /**
        This function searches for gradient maxima perpendicular to the line between p1 and p1. At the begining the function calcultes nPoints-often a search point on the line. These search points are distributed with equal distance over the line. From these points it searches for gradient maxima in the perpendicular direction to the line. The point on each side of the line with the maximal gradient is chosen and stored in points. Its gradient is stored in pointGrads. 
        pDist is the maximum distance to the line in that is searched for maxima.
        gradX and gradY are [IPL_DEPTH_16S  JfrImage_CS_GRAY] Jafar-Images with gradient in x and y direction, the gradient has to be signed.
         */
        void searchGradMax(CvPoint p1, CvPoint p2, std::vector<CvPoint>& points, std::vector<double>& pointGrads, jafar::image::Image* gradX, jafar::image::Image* gradY, int nPoints, double pDist);
      
        //! This function uses RANSAC to interpolate a set of points by lines
        /**
        This function uses RANSAC to interpolate a set of points by lines. Therefore nIt often two points of points are chosen and the line defined by the two points is calculated. The number of inliers is counted and compared to nPoints*minInlierRate. If its is larger or equal the validLine() Test is applyd. If it is passd, the line is put into lines.
        If colorImage is set, the lines are drawn on it.
         */
        void interpolateLineRansac(std::vector<LineSegment> lines, std::vector<CvPoint>& points, std::vector<double>& pointGrads, jafar::image::Image* gradX, jafar::image::Image* gradY, int nPoints, int nIt, double minInlierRate, double minLength, jafar::image::Image* colorImage);
      
        //!This function interpolates a set of points by lines
        /**
        This function solves the same problem as the interpolateLineRansac() function but in a deterministical way.
         */
        void interpolateLineGrowline(std::vector<LineSegment>& lines, std::vector<CvPoint>& points, std::vector<double>& pointGrads, jafar::image::Image* gradX, jafar::image::Image* gradY, int nPoints, double minLength,double aDist, jafar::image::Image* colorImage);
    
        //! Merges two lines on base of Chi2-Test
        /*!
          Merges two lines on base of Chi2-Test. The return value indicates the number of results. If there is only one resulting line, it is stored in lA. If it return 3, the returned line is a new line (maybe you want to apply orientLine() on this new line) If it returns 2, both lines stay the same.
          Lines have to be oriented.
        */
        static int tryMergeLinesChiSq(LineSegment &lA, LineSegment &lB, double maxGap=0);
        
        //! Compute the "average" line of two lines
        /** 
        Find average line parameters (weighted by the length of the lines) and calculate the projection of the endpoints on this line. Use the the extremal endpoints as new endpoints.
        It should be tested before if you want the lines to be merged (tryMergeLinesChiSq() function combines these two steps)
         */
        static void mergeLinesProb(LineSegment& lsA, LineSegment& lsB);
        
        //! This function process a merging by connecting the endpoints of the line that defines the longest possible lineSegment.
        /**
        This function process a merging by connecting the endpoints of the line that defines the longest possible lineSegment.
        If it return 3, the returned line is a new line (maybe you want to apply orientLine() on this new line) If it returns 2, both lines stay the same.
        It should be tested before if you want the lines to be merged (you can modify tryMergeLinesChiSq() function to combine these two steps)
         */
        static int mergeLinesEP(LineSegment& lsA, LineSegment& lsB);
        
        //!Returns the size of the gap between two line segments
        /**
        Returns the size of the gap between two line segments. This is useful for merging. If the lines overlap, 0 is returned;
         */
        static double sizeOfGap(LineSegment& lsA, LineSegment& lsB);
        
        //! Calculates the prediction of the endpoints from the prediction in polar coordinates stored in rhoPred and thetaPred
        /**
        Calculates the prediction of the endpoints from the prediction in polar coordinates stored in rhoPred and thetaPred. The old endpoints of the line segmenent are projected onn the predicted line to get the predicted endpoints.
        */
        double getEPPredictionByRhoThetaPrediction(double x1Old, double y1Old, double x2Old, double y2Old, double& x1Pred, double& y1Pred, double& x2Pred, double& y2Pred);

        
        //!Grow line with use of the parallel lines
        /*! 
        Grow line with use of the parallel lines
        @param image Pointer to grey value image
        @param dist Distance of the parallels to origin lineSegment
        @param thresh Maximal difference between two consecutive points on parallel to increase origin lineSegment
        @param threshDist Minimal difference between left and right parallel to increase origin lineSegment
         */
        bool growLineParallel(jafar::image::Image* image, int dist=3, int thresh=5, int distThresh=20);
        
        //!Grow LineSegment based on image
        /*!
        Grow LineSegment based on image.
         */
        bool growLine( jafar::image::Image* image, int gapThresh=1, int thresh=40, int maxOffset=2);
    
        
        void drawLine(jafar::image::Image* image, CvScalar color, int width=1);
      
        double getX1(){return x1;}  //!< To get x1 (Coordinate of ending point)
        double getY1(){return y1;}  //!< To get y1 (Coordinate of ending point)
        double getX2(){return x2;}  //!< To get x2 (Coordinate of ending point)
        double getY2(){return y2;}  //!< To get y2 (Coordinate of ending point)
        double getMx(){return mx;}  //!< To get mx (Coordinate of midpoint)
        double getMy(){return my;}  //!< To get my (Coordinate of midpoint)
        double getAlpha(){return alpha;}  //!< To get alpha (Angel to positiv x-axis)
        double getOAlpha(){return oAlpha;}  //!< To get oAlpha (oriented Angel to positiv x-axis)
        double getEucLength(){return eucLength;}  //!< To get euclidean length
      
        bool isValid(){return validFlag;}   //!< To know if line is valid. validFlag is for user to indicated invalid lines.
        void setValid(){validFlag=1;}       //!< Set line valid.(Done by constructor, too) 
        void setInvalid(){validFlag=0;}     //!< Marke line as invalid. (e.g. if merged with another line)
      
        int length; //!< The number of pixels on this line given by Bresenham algorithm (8-neighbourhood)
      
        CvScalar color; //!< Color for drawing the line 
    
        std::vector<int> greyscale;      //!< Container for grey values of underlying pixels 
        std::vector<int> gradientscale;  //!< Container for gradient values of underlying pixels
        std::vector<int> laplacescale;   //!< Container for LaPlace values of underlying pixels
    
        int numberOfNeighbours;   //!< Number of neighboured parallel lines
    
        LineSegment **parallels;  //!<  Array of neighboured parallel lines
      
        jafar::lines::Histogram* histogramB;    //!< Histogram of brighter side
        jafar::lines::Histogram* histogramD;    //!< Histogram of darker side
      
      
        std::vector<int> greyspace;    //!< Descriptor of greyvalues along the line segment
        std::vector<int> greyspaceL;   //!< Descriptor of greyvalues along the line segment left side
        std::vector<int> greyspaceR;   //!< Descriptor of greyvalues along the line segment right side
      
        double avL;   //!< Average grey level on left side of the line
        double avR;   //!< Average grey level on right side of the line
        double varL;  //!< Average change of neighboured greyscales on left side of the line
        double varR;  //!< Average change of neighboured greyscales on right side of the line
    
      // values for polar coordinates and covariance matrix for polar coordinates
        double polarD;      //!< Polar coordinate distance (rho)
        double polarAlpha;  //!< Polar coordinate angle (theta) ,polarAlpha is in range [-pi, pi]
        double covDD;       //!< Covariance of polar coordinates
        double covAA;       //!< Covariance of polar coordinates
        double covAD;       //!< Covariance of polar coordinates
      
        double covXX;       //!< Element of covariance matrix of endpoints and orientation
        double covXY;       //!< Element of covariance matrix of endpoints and orientation
        double covYY;       //!< Element of covariance matrix of endpoints and orientation
        double covAlpha;    //!< Element of covariance matrix of endpoints and orientation
        double covXA;       //!< Element of covariance matrix of endpoints and orientation
        double covYA;       //!< Element of covariance matrix of endpoints and orientation
      
        double covXX_M;     //!< Element of covariance matrix for merging (no uncertainty parallel to line) 
        double covXY_M;     //!< Element of covariance matrix for merging (no uncertainty parallel to line)
        double covYY_M;     //!< Element of covariance matrix for merging (no uncertainty parallel to line)
        double covXA_M;     //!< Element of covariance matrix for merging (no uncertainty parallel to line)
        double covYA_M;     //!< Element of covariance matrix for merging (no uncertainty parallel to line)
      
        double u;           //!< x coordinate of closest point to origin on the infinite line
        double v;           //!< y coordinate of closest point to origin on the infinite line
      
        uint id;    //!< Unique identifier of the line
      
        std::vector<std::vector<CvPoint> > contourPoints;   //!< Storage for the contour points of that line
        int param1;                   //!< Special parameter, in LsTracker used as storage for longest antecessor line
        std::vector<int> parameters;  //!< set of special parameters, in LsTracker used as storage for all antecessor lines
      
        jafar::lines::LsPredictor2D predictor;  //!< Motion predcitor for the line, used for tracking in LsTracker 
      
        //! Set the prediction for that line in polar coordinates
        /**
        Set the prediction for the infinite line of that line segment in polar coordinates.
        To indicate that there is a useful prediction, the predFlag of that LineSegment is set-
        
        @param rho rho value of polar coordinates of the prediction
        @param theta theta value of polar coordinates of the predicition
        @param covRR variance of predicted rho
        @param covRT correlation of predicted rho and theta
        @param covTT variance of predicted theta
        
        */
        void setPrediction(double rho, double theta, double covRR, double covRT, double covTT){
          rhoPred = rho;
          thetaPred = theta;
          covRRpred = covRR;
          covRTpred = covRT;
          covTTpred = covTT;
          predFlag = 1;

        }
        
        double rhoPred;     //!< rho value of polar coordinates of the prediction
        double thetaPred;   //!< theta value of polar coordinates of the predicition
        double covRRpred;   //!< variance of predicted rho
        double covRTpred;   //!< correlation of predicted rho and theta
        double covTTpred;   //!< variance of predicted theta
        bool predFlag;      //!< indicates if there is a valid prediction stored in rhoPred, thetaPred 
        
        
      private:
        bool validFlag;
        double x1;      //!< Coordinate of ending point
        double y1;      //!< Coordinate of ending point
        double x2;      //!< Coordinate of ending point
        double y2;      //!< Coordinate of ending point
        double mx;      //!< Coordinate of midpoint
        double my;      //!< Coordinate of midpoint
        double slope;   //!< Slope of the line segment
        double eucLength;   //!< Euclidean distance of ending points
        double alpha;   //!< Angle to positiv x-axis, [-pi/2 , +pi/2], calculated in setPoints()
        double oAlpha;  //!< (oriented) Angle to positiv x-axis, not of the total line but of the line segment with (x1,y1) shifted to origin, [-pi, +pi], calculated in orientLine()
    };
  } // namespace lines
} // namespace jafar
#endif
