/* $Id$ */

#ifndef SLAM_MANAGER_TOOLS_HPP
#define SLAM_MANAGER_TOOLS_HPP

#include "jmath/jblas.hpp"

#include "geom/t3dEuler.hpp"

#include "slam/feature.hpp"
#include "slam/bearingOnlyFeature.hpp"
#include "slam/slamEvents.hpp"

namespace jafar {
  namespace slampt {

    /** An image point as recorded in the frame database.
     * \ingroup slampt
     */
    struct DBFrameLandmark {

      DBFrameLandmark(std::size_t hpmIndex_,
		      unsigned int id_) :
	hpmIndex(hpmIndex_), id(id_) {};

      /// index of the point in the vector returned by hpm.
      std::size_t hpmIndex;
      /// id of the landmark.
      unsigned int id;
    };

    struct LandmarkListRemovePred {
      unsigned int id;
      
      LandmarkListRemovePred(unsigned int id_) : id(id_) {}

      bool operator()(DBFrameLandmark const& landmark) {
	return landmark.id == id;
      }

    };

    /** An image frame as recorded in the frame database.
     * \ingroup slampt
     */
    struct DBFrame {

      typedef std::list<DBFrameLandmark> LandmarksList;

      /// frame index.
      unsigned int index;

      /** pose of the robot at that frame.
       * \todo move this away, it is not generic
       */
      geom::T3DEuler pose;

      /// list of points extracted by hpm at that frame.
      LandmarksList landmarks;

      void removeLandmark(unsigned int id_) {
	landmarks.remove_if(LandmarkListRemovePred(id_));
      }

    };

    /** Generic frame database for loop closing.
     * \ingroup slampt
     */
    class FrameDataBase :  public slam::BoSlamEventAdapter {

    protected:

      void removeLandmark(unsigned int id) {
	JFR_DEBUG("FrameDataBase::removeLandmark: " << id);
	for (DBFrameList::iterator it = frameList.begin() ; it != frameList.end() ; ++it) {
	  it->removeLandmark(id);
	}
      }

      void removeLandmark(slam::BaseFeature const& landmark) {
	removeLandmark(landmark.id());
      }

      void removeTentativeLandmark(slam::InitFeature const& landmark) {
	removeLandmark(landmark.id());
      }

    public:

      // FIXME optimization
      typedef std::list<DBFrame> DBFrameList; 
      DBFrameList frameList;

      void changeLandmarkId(unsigned int oldId, unsigned int newId, 
			    unsigned int frameIndexBegin, unsigned int frameIndexEnd);

    }; // class FrameDataBase

    /** A feature point with a quality.
     * \ingroup slampt
     */
    struct QualityIndexPoint {

      QualityIndexPoint(std::size_t hpmIndex_,
                        double quality_ = 0.0) :
	hpmIndex(hpmIndex_), quality(quality_) {};

      std::size_t hpmIndex;
      double quality;
    };

    bool operator<(QualityIndexPoint const& p1, QualityIndexPoint const& p2);

  }
}

#endif // SLAM_MANAGER_TOOLS_HPP
