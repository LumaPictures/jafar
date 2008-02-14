#ifndef LS_TRACKER
#define LS_TRACKER

#include <vector>
#include <image/Image.hpp>
#include "lineSegmentSet.hpp"
#include "matchingSet.hpp"
#include "lsExtractor.hpp"
namespace jafar{
  namespace lines{
    
    //! Indictor for tracking scheme
    /**
    Here the different tracking schemes which can be used in procTracking() are enumerated
    
    GLOB_LOC_SEARCH
    Extract lines on the whole image and applys local search (LineSegment::fitLineOrientation()) on all old lines. (trackingScheme4)
    
    GLOB_MATCHING
    Extract lines on the whole image and try to match them with histogram descriptor (trackingScheme6)
    
    @ingroup lines
    */
    enum TrackScheme{
      GLOB_LOC_SEARCH,
      GLOB_MATCHING,
    };
    
    //! Provides tracking of line segments.
    /**
    This class provides a tracking scheme for line segments. The interface for tracking is the procTracking() function which can be applyed iteratively on a sequence of images. The user has to handle the LineSegmentSet instances. They are not stored in LsTracker. 
    
    There is no need to use the same instance of lsTracker for each iteration of tracking but it increases speed, because otherwise a lot of temporary buffers are allocated and deallocated.
    If the tarcking is not done by only one instance of LsTracker you have to take special care of the id that are given to new lines. If there is a new line in procTracking() that could not be matched to an old line, it is given an id provided by nextId member, then nextId member is increased. Thus if you use different instances of LsTracker you have to update the nextId member before calling procTracking (use setNextId()). 
    
    Because it's tracking, the movement in the consecutive images should not be too large! 
    There are some tracking schemes provided to be used in procTracking() function (see TrackScheme enumeration). The SVN version of the LsTracker-class should provide some of the currently best tracking schemes, further schemes are provided for further development as internal functions with name trackingSchemeX
    
    @ingroup lines
    */
    class LsTracker{
    public:
      LsTracker();
      ~LsTracker();
    
      //! Proceed one tracking step (also the first step with empty _lsOld)
      /**
       Proceed one tracking step. The new line segments are stored in newLS and the matching for all old line segments (_oldLS) to new line segments is stored in match.
       The matching is stored in matching and indicated by updating the id member and the color member of matched lines of newLS with the id and the color of the according line of oldLS. (Update of color member is nice for visualization of the tracking).
      
        
        @param image greyvalue image to proceed in this step
        @param oldLS pointer to the set of line segments of the last step
        @param newLS pointer to the storage for the new line segments created in this step
        @param match storage for the matching, assigns old lines new lines
        @param colorImage (optional) image to draw lines during the tracking (for debugging)
       */
      void procTracking(jafar::image::Image* image, LineSegmentSet* oldLS, LineSegmentSet* newLS, jafar::lines::MatchingSet* match, TrackScheme scheme=GLOB_LOC_SEARCH, jafar::image::Image* colorImage=0);
      
       //! Internal function for tracking. Have a look at procTracking to proceed tracking.
      /**
       * This function provides a tracking approach, gradX and gradY have to be gradient images 
       * lsA is old set, lsB new  set
       * That function uses a matching with histograms for tracking.
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
      void trackingScheme6(LineSegmentSet& lsA, LineSegmentSet& lsB, jafar::image::Image* image, jafar::image::Image* gradX, jafar::image::Image* gradY, jafar::lines::MatchingSet& match, jafar::image::Image* colorImage=0);
      
      //! Internal function for tracking. Have a look at procTracking to proceed tracking.
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
      void trackingScheme5(LineSegmentSet& lsA, LineSegmentSet& lsB, jafar::image::Image* image, jafar::image::Image* gradX, jafar::image::Image* gradY, jafar::lines::MatchingSet& match, jafar::image::Image* colorImage=0);
      
      //! Internal function for tracking. Have a look at procTracking to proceed tracking.
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
      void trackingScheme4(LineSegmentSet& lsA, LineSegmentSet& lsB, jafar::image::Image* image, jafar::image::Image* gradX, jafar::image::Image* gradY, jafar::lines::MatchingSet& match, jafar::image::Image* colorImage=0);
      
      //! Internal function for tracking. Have a look at procTracking to proceed tracking.
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
      void trackingScheme3(LineSegmentSet& lsA, LineSegmentSet& lsB, jafar::image::Image* image, jafar::image::Image* gradX, jafar::image::Image* gradY, jafar::lines::MatchingSet& match, jafar::image::Image* colorImage=0);
      
      //! Internal function for tracking. Have a look at procTracking to proceed tracking.
      /**
       * This function provides a simple tracking, gradX and gradY have to be gradient images 
       * lsA is old set, lsB new  set
       */
      void trackingScheme2(LineSegmentSet& lsA, LineSegmentSet& lsB, jafar::image::Image* image, jafar::image::Image* gradX, jafar::image::Image* gradY, jafar::lines::MatchingSet& match);
      
      //! Internal function for tracking. Have a look at procTracking to proceed tracking.
      /**
       * This function provides a simple tracking, _image has to be a gradient image 
       *
       */
      void trackingScheme1(LineSegmentSet& lsA, LineSegmentSet& lsB, jafar::image::Image* image, jafar::lines::MatchingSet& match);
    

      //! Set the parameters for Canny edge detector of lsExtractor
      /**
      Set the parameters for Canny edge detector of lsExtractor. The cvCanny() function is used in LsExtractor to find contour points. Here the parameters for tht call can be set.
      */
      void setCannyPara(int lowerThresh, int higherThresh, int ap){
        lsExtractor.setCannyPara(lowerThresh, higherThresh, ap);
      }
      
      //! Update the nextId member
      /**
      Update the nextId member. This is neccessary if there is an ow ninstance of LsTracker for each iteration of the tracking or if the identifiers are managed outside of LsTracker.
      */
      void setNextId(uint _nextId){ nextId = _nextId;}
      
      //! Set the minimal length of new lines
      /**
      Set the minimal length of new lines. Only the global extractio ntakes care of that lenth. If the line is tracked with fitLineOrientation function, it can be shorter.
      */
      void setMinLength(double _minLength){minLength = _minLength;}
      
    private:
      jafar::image::Image* gradX;           //!< Gradient image (x direction)
      jafar::image::Image* gradY;           //!< Gradient image (y direction)
      jafar::image::Image* cannyIm;         //!< Canny edge detection image 
      
      jafar::lines::LsExtractor lsExtractor;  //!< LsExtractor instance for managment of line segment extarction 
      
      double minLength;   //!< Minimal length of new lines
      
      uint nextId;  //!< New identifier for the next new line that could not be matched to a line.
      
    };
  } // namespace lines
} // namespace jafar
#endif
