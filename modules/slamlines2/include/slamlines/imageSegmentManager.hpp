/* $$ */

#ifndef SLAM_IMAGE_SEGMENT_MANAGER_HPP
#define SLAM_IMAGE_SEGMENT_MANAGER_HPP

#include <iostream>
#include <vector>
#include <map>

#include "jmath/jblas.hpp"

#include "slam/bearingOnlySlam.hpp"

#include "lines/lineSegmentSet.hpp"
#include "lines/lineSegment.hpp"
#include "lines/matchingSet.hpp"
#include "slam/observation.hpp"

namespace jafar {
  namespace slamlines {

	  /// class Image Segment
	  ///\ingroup slam
    class ImageSegment {

    public:

      static const unsigned int NO_ID = 0;

      /// id of the point, id=NO_ID means it has no id
      unsigned int id;

      /// ending points
      jblas::vec2 ext1;
      jblas::vec2 ext2;

      /// ending points covariance 
      jblas::sym_mat ext1Cov;
      jblas::sym_mat ext2Cov;


      ImageSegment(jblas::vec2 ext1_, jblas::vec2 ext2_) :
	id(NO_ID) ,
	ext1(ext1_),
	ext2(ext2_)
      {}

      ImageSegment(jblas::vec2 ext1_, jblas::vec2 ext2_, jblas::sym_mat ext1Cov_, jblas::sym_mat ext2Cov_,unsigned int id_) :
	id(id_) ,
	ext1(ext1_),
	ext2(ext2_),
	ext1Cov(ext1Cov_),
	ext2Cov(ext2Cov_)
       {}
    };

    std::ostream& operator <<(std::ostream& s, ImageSegment const& seg);

    /// a single match
    typedef std::pair<std::size_t, std::size_t> Match;

    /// vector of matches
    typedef std::vector<Match> VecMatches;

    /// a vector of image segments
    typedef std::vector<ImageSegment> VecImageSegments;

    void readImageSegments(std::string const& filename, VecImageSegments& vecImageSegments);

    void readImageSegmentsId(std::string const& filename, VecImageSegments& vecImageSegments);

    void readImageSegmentsMatches(std::string const& filename, VecMatches& matches);

    void lineSegmentSetToImageSegmentVec(jafar::lines::LineSegmentSet& lineSegmentSet, VecImageSegments& vecImageSegments);

    void matchingSetToImageSegmentsMatches(jafar::lines::MatchingSet& matchingSet, VecMatches& matches);

    unsigned int propagateId(VecMatches const& vecMatches, 
			     VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch);


	
	/// class Image Segment Manager
	///\ingroup slam
    class ImageSegmentManager {

    protected:

      unsigned int p_idFactory;
      unsigned int getNewFeatureId() {return ++p_idFactory;}
      //unsigned int setNewFeatureId(unsigned int& p_idFactory, unsigned int global_id) {p_idFactory = global_id;}

      slam::BearingOnlySlam& p_slam;

      /// minimum size in pixel
      double p_minSegmentSize;

      jblas::vec2 p_zone1;
      jblas::vec2 p_zone2;

      bool isInZone(jblas::vec2 const& pix) const {
	return pix(0) > p_zone1(0) && pix(1) > p_zone1(1) && pix(0) < p_zone2(0) && pix(1) < p_zone2(1);
      }

      void featureSelection(VecImageSegments& vecImageSegments);

      void slamProcessObservations(unsigned int currentFrameIndex, VecImageSegments& vecImageSegments) const;

    public:
      /// type of segment Observation 
      slam::Observation::ObservationType type;

          
      ImageSegmentManager(slam::BearingOnlySlam& slam, slam::Observation::ObservationType type_=slam::Observation::SEGMENT_IMAGE) : p_idFactory(0), p_slam(slam), type(type_) {}

      void setup(double minSegmentSize, double u1, double v1, double u2, double v2) {
	p_minSegmentSize = minSegmentSize;
	p_zone1(0) = u1;
	p_zone1(1) = v1;
	p_zone2(0) = u2;
	p_zone2(1) = v2;
      }

      void initFrame(unsigned int frameIndex_, VecImageSegments& vecImageSegments);
      void processFrame(unsigned int frameIndex, VecImageSegments& vecImageSegments);
     /* void ImageSegmentManager::getSegmentsPrediction(lines::LineSegmentSet& lineSegmentSet, VecImageSegments& vecImageSegments);*/
      unsigned int compareId(VecMatches const& matches, 
		 VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch, jafar::lines::LineSegmentSet& lineSegmentSet);

      unsigned int compareIdInv(VecMatches const& matches, 
		 VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch, jafar::lines::LineSegmentSet& lineSegmentSet);
      unsigned int compareIdInvAll(VecMatches const& matches, VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch, jafar::lines::LineSegmentSet& lineSegmentSet);
    }; // class ImageSegmentManager

    
	
// class ImageHomogenCoordSegmentManager
///\ingroup slam
class ImageHomogenCoordSegmentManager {

	protected:

		unsigned int p_idFactory;
		unsigned int getNewFeatureId() {return ++p_idFactory;}
//unsigned int setNewFeatureId(unsigned int& p_idFactory, unsigned int global_id) {p_idFactory = global_id;}
		slam::SlamEkf& slam;

/// minimum size in pixel
		double p_minSegmentSize;

		jblas::vec2 p_zone1;
		jblas::vec2 p_zone2;

		/// Test to know if the feature is in the region of interest
		bool isInZone(jblas::vec2 const& pix) const {
			return pix(0) > p_zone1(0) && pix(1) > p_zone1(1) && pix(0) < p_zone2(0) && pix(1) < p_zone2(1);
		}

		/// Feature selection
		void featureSelection(VecImageSegments& vecImageSegments);

		/// Proess observations
		void slamProcessObservations(unsigned int currentFrameIndex, VecImageSegments& vecImageSegments) const;

	public:
          
                /// type of segment Observation 
                slam::Observation::ObservationType type;

			/// Constructor
                ImageHomogenCoordSegmentManager(slam::SlamEkf& slam_, slam::Observation::ObservationType type_=slam::Observation::SEGMENT_IMAGE) : p_idFactory(0), slam(slam_), type(type_) {}

		/// Setup.
		/// This sets some key atributes
		///@param minSegmentSize the minimum segment length, in pixels
		///@param u1 min horizontal coordinate of the region of interest
		///@param v1 min vertical coordinate of the region of interest
		///@param u2 max horizontal coordinate of the region of interest
		///@param v2 max vertical coordinate of the region of interest
		void setup(double minSegmentSize, double u1, double v1, double u2, double v2) {
			p_minSegmentSize = minSegmentSize;
			p_zone1(0) = u1;
			p_zone1(1) = v1;
			p_zone2(0) = u2;
			p_zone2(1) = v2;
		}

		/// Initialize frame
		void initFrame(unsigned int frameIndex_, VecImageSegments& vecImageSegments);
		
		/// Process image frame
		void processFrame(unsigned int frameIndex, VecImageSegments& vecImageSegments);
		/* void ImageSegmentManager::getSegmentsPrediction(lines::LineSegmentSet& lineSegmentSet, VecImageSegments& vecImageSegments);*/
		
		
		/// Compare feature identifiers.
		/// Compares identifiers of matched features against features in the map
		unsigned int compareId(VecMatches const& matches, 
					VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch, jafar::lines::LineSegmentSet& lineSegmentSet);

		/// Compare identifiers - inverse.
		/// No idea
		unsigned int compareIdInv(VecMatches const& matches, 
					VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch, jafar::lines::LineSegmentSet& lineSegmentSet);
		
		/// Compare identifiers - inverse, all.
		/// No idea
		unsigned int compareIdInvAll(VecMatches const& matches, VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch, jafar::lines::LineSegmentSet& lineSegmentSet);
}; // class ImageHomogenCoordSegmentManager


  } // namespace slam
} // namespace jafar

#endif // SLAM_IMAGE_SEGMENT_MANAGER_HPP
