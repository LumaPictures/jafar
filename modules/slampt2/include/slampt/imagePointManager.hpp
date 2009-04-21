/* $Id$ */

#ifndef SLAM_IMAGE_POINT_MANAGER_HPP
#define SLAM_IMAGE_POINT_MANAGER_HPP

#include <string>
#include <ostream>
#include <list>
#include <algorithm>

#include "boost/tuple/tuple.hpp"

#include "kernel/dataLog.hpp"

#include "jmath/jblas.hpp"

#include "image/Image.hpp"
#ifndef THALES_TAROT_DISABLE
#include "datareader/ImageReader.hpp"
#include "datareader/StereoReader.hpp"
#include "datareader/MonoReader.hpp"
#endif

#include "geom/t3dEuler.hpp"

#include "camera/cameraBarreto.hpp"
#include "camera/mask.hpp"

#include "hpm/export.hpp"
#include "hpm/engine.hpp"

#include "slampt/managerTools.hpp"
#include "slam/bearingOnlySlam.hpp"

namespace jafar {
  namespace slampt {

// #ifndef SWIG

    /** Frame Database for monocular images. Tentative loop closing
     * frame is obtained considering the image pose only.  
     *
     * \ingroup slampt
     */
    class MonoImageFrameDataBase : public FrameDataBase {

    public:

      unsigned int loopClosingIndexDistanceMin;

      double loopClosingDistanceMax;

#ifndef SWIG
      DBFrameList::iterator tentativeLoopClosing(unsigned int currentFrameIndex,
						 geom::T3DEuler const& pose);
#endif
    }; // class MonoImageFrameDataBase

// #endif // SWIG

    /**
     * Base class for doing slam using points extracted in an image.
     * 
     * \ingroup slampt
     */
    class ImagePointManager : 
      public jafar::kernel::DataLoggable
    {
      bool ownFrameDataBase;
    protected :

      static unsigned int idFactory;

      slam::BaseSlam& slam;

      /// For loop closing
      jafar::hpm::Engine& hpmEngineLoopClosing;

      MonoImageFrameDataBase* frameDataBase;

      bool loopClosingDetection;
      unsigned int loopClosingFrameIndex;
      unsigned int loopClosingNbUsedFeatures;
      jafar::hpm::vecHarrisPoints loopClosingPoints;

      typedef std::vector<QualityIndexPoint> QualityIndexPoints;

      /// points from last frame which are candidate
      std::vector<QualityIndexPoints> pointsInZone;
      std::vector<unsigned int> nbFeaturesInZone;

      /// features observed in current frame
      typedef std::list<std::size_t> FeaturesList;
      FeaturesList knownFeatures;
      FeaturesList newFeatures;

      
      /// cells definition
      unsigned int imageWidth;
      unsigned int imageHeight;
      std::size_t nbZonesU;
      std::size_t nbZonesV;
      unsigned int nbDesiredFeaturesPerZone;

      virtual boost::tuple<bool, unsigned int> isPointInZone(double u_, double v_);

      static unsigned int computeZoneIndex(double x_, unsigned int nbInt_, double xMax_);

      unsigned int getNewFeatureId() {return ++idFactory; }

      virtual bool isFeatureCandidate(std::size_t index_) = 0;

      void addNewFeature(std::size_t index_, hpm::HarrisPoint& point_);

      void addKnownFeature(std::size_t index_, jafar::hpm::HarrisPoint const& point_);

      struct IdPredicate {
        IdPredicate( int _id ) : id(_id) {}
        int id;
        bool operator()(const hpm::HarrisPoint& pt)
        {
          return pt.id == id;
        }
      };

#ifndef THALES_TAROT_DISABLE
      template<class VecPoints>
      void loopClosing(VecPoints& framePoints_, unsigned int robotId_)
      {
	loopClosingDetection = false;
	loopClosingNbUsedFeatures = 0;

	JFR_DEBUG("ImagePointManager::loopClosing:");

/*        JFR_DEBUG("Already observed" );
        JFR_FOREACH( const hpm::HarrisPoint& hp, framePoints_ )
        {
          if(hp.id != 0)
          {
            JFR_DEBUG(hp.id);
          }
        }
        JFR_DEBUG("Already observed end" );*/
        
  // FIXME does this function work correctly with a MultiMapManager
	geom::T3DEuler robotPose;
	slam.getRobotPose(robotPose, robotId_);

        JFR_DEBUG("Current pose: " << robotPose);
	FrameDataBase::DBFrameList::iterator tentativeLoopClosingFrame = 
	  frameDataBase->tentativeLoopClosing(currentFrameIndex, robotPose);
  
	if (tentativeLoopClosingFrame != frameDataBase->frameList.end()) {
	  loopClosingDetection = true;
	  loopClosingFrameIndex = tentativeLoopClosingFrame->index;
	  JFR_DEBUG("Tentative loop closing: frame " << tentativeLoopClosingFrame->index);
	  JFR_DEBUG("pose: " << tentativeLoopClosingFrame->pose.getX());
	  JFR_DEBUG("nb tentative landmarks: " << tentativeLoopClosingFrame->landmarks.size());

	  jafar::hpm::vecHarrisPoints framePointsTmp; // FIXME optimization
	  hpm::mapMatches loopClosingMatches;

	  // (re)compute harris points of frame tentativeLoopClosingFrame->index
	  // compute matches between currentFrameIndex and tentativeLoopClosingFrame->index

	  image::Image* im1 = loadImage(tentativeLoopClosingFrame->index);
	  image::Image* im2 = loadImage(currentFrameIndex);
	  hpmEngineLoopClosing.computeMatch(*im1, *im2,
					    loopClosingPoints, framePointsTmp, loopClosingMatches);
	  delete im1;
	  delete im2;

	  JFR_DEBUG("matched points: " << loopClosingMatches.size() << JFR_PP_VAR(tentativeLoopClosingFrame->landmarks.size()));

	  // iterate on the past frame landmarks
	  for (DBFrame::LandmarksList::const_iterator it =  tentativeLoopClosingFrame->landmarks.begin() ; 
	       it != tentativeLoopClosingFrame->landmarks.end() ; ++it) {
	    // is the point corresponding to that landmark is matched ?
	    hpm::mapMatches::iterator match = loopClosingMatches.find(it->hpmIndex);
	    if (match != loopClosingMatches.end()
              and (
                std::find_if(framePoints_.begin(),
                              framePoints_.end(),
                              IdPredicate( it->id ) ) == framePoints_.end() ) ) {
	      loopClosingNbUsedFeatures++;
	      JFR_DEBUG("loop closing frame feature matched: " << it->id);
	      if (framePoints_[match->second].id == hpm::HarrisPoint::NO_ID) {
		// the point is not yet a landmark
		framePoints_[match->second].id = it->id;
	      }
	      else {
		// the point is already a landmark
		if (framePoints_[match->second].id != it->id) {
		  JFR_DEBUG("loop closing feature already in the map with another id, removing feature " 
			     << framePoints_[match->second].id);
		  unsigned int oldId = framePoints_[match->second].id;
		  unsigned int newId = it->id;

		  slam.removeFeature(oldId);
		  framePoints_[match->second].id = newId;

		  // back propagation of this change in the previous DBFrames
		  JFR_DEBUG("back propagation of id: " << oldId << " -> " << newId);
		  // FIXME
		  // 		  frameDataBase.changeLandmarkId(oldId, newId, 
		  // 						 tentativeLoopClosingFrame->index, 
		  // 						 frameDataBase.frameList.back().index);
		  frameDataBase->changeLandmarkId(oldId, newId, 
						 0,
						 frameDataBase->frameList.back().index);
		}
		else {
		  JFR_DEBUG("confirm loop closing match: " << it->id);
		}
	      }
	    }
	    else { 
	      JFR_DEBUG("loop closing frame feature NOT matched: " << it->id);
	    }
	  }
    
	  JFR_DEBUG("loop closing nb lanmarks: " << loopClosingNbUsedFeatures);

	  jafar::hpm::backPropagateId(loopClosingMatches, loopClosingPoints, framePoints_);

	}
      }
#endif

      template<class VecPoints>
      void featureSelection(VecPoints& points_)
      {
	// init the process
	newFeatures.clear();
	knownFeatures.clear();

	for (std::size_t i = 0 ; i < nbFeaturesInZone.size() ; i++) {
	  nbFeaturesInZone[i]=0;
	  pointsInZone[i].clear();
	}

	// fill in the zones
	for (std::size_t i = 0 ; i < points_.size() ; ++i) 
	  {
	    // features that are beeing tracked
	    if (points_[i].id != hpm::HarrisPoint::NO_ID)
	      addKnownFeature(i, points_[i]);

	    bool isInZone;
	    unsigned int zoneIndex;
	    boost::tie(isInZone, zoneIndex) = isPointInZone(points_[i].u, points_[i].v);
	    JFR_POSTCOND(zoneIndex >= 0 && zoneIndex < nbFeaturesInZone.size(),
			 "ImagePointManager::featureSelection invalid zone index: " << zoneIndex);
	    if (isInZone) {
	      if (points_[i].id != hpm::HarrisPoint::NO_ID)
		nbFeaturesInZone[zoneIndex]++;
	      else {
		if (isFeatureCandidate(i))
		  pointsInZone[zoneIndex].push_back(QualityIndexPoint(i,points_[i].lambdaLow));
		else
		  JFR_DEBUG("ImagePointManager::featureSelection: feature not candidate " << points_[i].id);
	      }
	    }
	  }
  
	// add new features from zones
	for (std::size_t i = 0 ; i < nbFeaturesInZone.size() ; i++) {
	  std::sort(pointsInZone[i].begin(), pointsInZone[i].end());
	  while (nbFeaturesInZone[i] < nbDesiredFeaturesPerZone) {
	    if (!pointsInZone[i].empty()) {
	      std::size_t newIndex = pointsInZone[i].back().hpmIndex;
	      addNewFeature(newIndex, points_[newIndex]);
	      pointsInZone[i].pop_back();
	      nbFeaturesInZone[i]++;
	      //         JFR_DEBUG("add feature: " << features.back()->id);
	    }
	    else {
	      JFR_DEBUG("featureSelection: not enough points in zone " << i);
	      break;
	    }
	  }
	}

      }

      template<class VecPoints>
      void updateFrameDataBase(VecPoints& points_, unsigned int robotId_)
      {
	frameDataBase->frameList.push_back(DBFrame());
	DBFrame& dbFrame = frameDataBase->frameList.back();
	dbFrame.index=currentFrameIndex;
	slam.getRobotPose(dbFrame.pose, robotId_);
// 	dbFrame.pose.set(slam.getRobotPose(), slam.getRobotPoseCov());
	for (FeaturesList::const_iterator it = knownFeatures.begin() ; it != knownFeatures.end() ; ++it) {
	  dbFrame.landmarks.push_back(DBFrameLandmark(*it, points_[*it].id));
	}
	for (FeaturesList::const_iterator it = newFeatures.begin() ; it != newFeatures.end() ; ++it) {
	  dbFrame.landmarks.push_back(DBFrameLandmark(*it, points_[*it].id));
	}
      }

#ifndef THALES_TAROT_DISABLE
      virtual jafar::image::Image* loadImage(unsigned int frameNumber) const = 0;
#endif

      virtual bool fillObservation(std::size_t index_, slam::Observation& obs, unsigned int sensorId_, unsigned int robotId_) const = 0;
      
      //FIXME should it be sensorId instead of robotId?
      /// there is only one point manager but the observations corresponds to each robotId
      void slamProcessObservations(unsigned int robotId_, unsigned int sensorId_) const;

      /// current frame number
      unsigned int currentFrameIndex;

    public :

      ImagePointManager(slam::BaseSlam& slam_, jafar::hpm::Engine& hpmEngineLoopClosing_);

      virtual ~ImagePointManager();

      void setFrameDataBase( MonoImageFrameDataBase* _frameDataBase );
      
      virtual slam::Observation::ObservationType getObservationType() const = 0;

      virtual void setupZones(unsigned int imageWidth_, unsigned int imageHeight_,
		      double reduction_,
		      std::size_t nbZonesU_,
		      std::size_t nbZonesV_,
		      unsigned int nbDesiredFeaturesPerZone_);

      void setupLoopClosing(double loopClosingDistanceMax_, 
			    unsigned int loopClosingIndexDistanceMin_);

      bool isLoopClosingDetected() const {return loopClosingDetection;}

      unsigned int getLoopClosingFrameIndex() const {return loopClosingFrameIndex;}

      jafar::hpm::vecHarrisPoints const& getLoopClosingPoints() const {return loopClosingPoints;}

      //       FeaturesList const& getKnownFeatures() const {return knownFeatures;}
      //       FeaturesList const& getNewFeatures() const {return newFeatures;}

      void writeLogHeader(jafar::kernel::DataLogger& dataLogger) const;
      void writeLogData(jafar::kernel::DataLogger& dataLogger) const;

      friend std::ostream& operator <<(std::ostream& s, const ImagePointManager& slam_);
      friend std::string getZones(ImagePointManager const& m);

    }; // class ImagePointManager
    std::ostream& operator <<(std::ostream& s, const ImagePointManager& manager_);


#ifndef THALES_TAROT_DISABLE
    /**
     * Class for doing bearing-only slam with a monocular camera and when features are interest
     * points extracted from an image.
     * \ingroup slampt
     */
    class MonoImagePointManager : 
      public ImagePointManager,
      public slam::BoSlamEventAdapter {

    protected:

      /// For tracking points
      jafar::hpm::TrackingEngine& hpmTrackingEngine;

      jafar::hpm::vecHarrisPoints vecPointsPrev;
      jafar::hpm::vecHarrisPoints vecPointsCur;

      bool isFeatureCandidate(std::size_t index_) { return true; }

      jafar::image::Image* loadImage(unsigned int frameNumber) const {
	return imageReader.loadImage(frameNumber);
      }

      jafar::datareader::ImageReader imageReader;

      bool fillObservation(std::size_t index_, slam::Observation& obs, unsigned int sensorId_, unsigned int robotId_) const;

      void removeLandmark(unsigned int id) {
	for (jafar::hpm::vecHarrisPoints::iterator it = vecPointsCur.begin();
	     it != vecPointsCur.end() ; ++it)
	  {
	    if (it->id == id) {
	      JFR_DEBUG("MonoImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }

	for (jafar::hpm::vecHarrisPoints::iterator it = loopClosingPoints.begin();
	     it != loopClosingPoints.end() ; ++it)
	  {
	    if (it->id == id) {
	      JFR_DEBUG("MonoImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }
      }

      void removeLandmark(slam::BaseFeature const& landmark) {
	removeLandmark(landmark.id());

      }
      void removeTentativeLandmark(slam::InitFeature const& landmark) {
	removeLandmark(landmark.id());
      }

    public:

      MonoImagePointManager(slam::BearingOnlySlam& slam_,
			    jafar::hpm::Engine& hpmEngineLoopClosing_,
			    jafar::hpm::TrackingEngine& hpmTrackingEngine_) : 
	ImagePointManager(slam_, hpmEngineLoopClosing_), 
	hpmTrackingEngine(hpmTrackingEngine_)
      {
	slam_.addBoEventListener(*this);
	slam_.addBoEventListener(*frameDataBase);
      }

      ~MonoImagePointManager() {}

      slam::Observation::ObservationType getObservationType() const {return slam::Observation::POINT_IMAGE;}

      jafar::datareader::ImageReader& getImageReader() {return imageReader;}

      jafar::hpm::vecHarrisPoints const& getCurrentPoints() const {return vecPointsCur;}
      jafar::hpm::vecHarrisPoints const& getPreviousPoints() const {return vecPointsPrev;}

      void initFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);
      void initFrame(unsigned int frameIndex_, jafar::image::Image* image,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);

      void processFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);
      void processFrame(unsigned int frameIndex_, jafar::image::Image* image,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);

    }; // class MonoImagePointManager

/**
     * Class for doing bearing-only slam with a monocular camera and when features are interest
     * points extracted from an image using Inverse Depth parameterization.
     * \ingroup slampt
     */
    class IdImagePointManager : 
      public ImagePointManager,
      public slam::SlamEventAdapter {

    protected:

      /// For tracking points
      jafar::hpm::TrackingEngine& hpmTrackingEngine;

      jafar::hpm::vecHarrisPoints vecPointsPrev;
      jafar::hpm::vecHarrisPoints vecPointsCur;

      bool isFeatureCandidate(std::size_t index_) { return true; }

      jafar::image::Image* loadImage(unsigned int frameNumber) const {
	return imageReader.loadImage(frameNumber);
      }

      jafar::datareader::ImageReader imageReader;

      bool fillObservation(std::size_t index_, slam::Observation& obs, unsigned int sensorId_, unsigned int robotId_) const;

      void removeLandmark(unsigned int id) {
	for (jafar::hpm::vecHarrisPoints::iterator it = vecPointsCur.begin();
	     it != vecPointsCur.end() ; ++it)
	  {
	    if (it->id == id) {
	      JFR_DEBUG("MonoImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }

	for (jafar::hpm::vecHarrisPoints::iterator it = loopClosingPoints.begin();
	     it != loopClosingPoints.end() ; ++it)
	  {
	    if (it->id == id) {
	      JFR_DEBUG("MonoImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }
      }

      void removeLandmark(slam::BaseFeature const& landmark) {
	removeLandmark(landmark.id());

      }

    public:

      IdImagePointManager(slam::BaseSlam& slam_,
			    jafar::hpm::Engine& hpmEngineLoopClosing_,
			    jafar::hpm::TrackingEngine& hpmTrackingEngine_) : 
	ImagePointManager(slam_, hpmEngineLoopClosing_), 
	hpmTrackingEngine(hpmTrackingEngine_) 
      {
	slam_.addEventListener(*this);
	slam_.addEventListener(*frameDataBase);
      }

      ~IdImagePointManager() {}

      slam::Observation::ObservationType getObservationType() const {return slam::Observation::POINT_IMAGE;}

      jafar::datareader::ImageReader& getImageReader() {return imageReader;}

      jafar::hpm::vecHarrisPoints const& getCurrentPoints() const {return vecPointsCur;}
      jafar::hpm::vecHarrisPoints const& getPreviousPoints() const {return vecPointsPrev;}

      void initFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);
      void initFrame(unsigned int frameIndex_, jafar::image::Image* image,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);

      void processFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);
      void processFrame(unsigned int frameIndex_, jafar::image::Image* image,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);

    }; // class IdImagePointManager
#endif

    /**
     * Class for stereovision slam with a stereovision bench and when features are interest
     * points extracted from an image.
     * \ingroup slampt
     */
    class StereoImagePointManager : 
      public ImagePointManager,
      public slam::SlamEventAdapter {

    protected:
      unsigned int lastUsedRobot;
#ifndef THALES_TAROT_DISABLE
      jafar::datareader::StereoReader* stereoReader;
#endif
      jafar::hpm::StereoTrackingEngine& hpmTrackingEngine;

      jafar::hpm::vecStereoHarrisPoints vecPointsPrev;
      jafar::hpm::vecStereoHarrisPoints vecPointsCur;
      jafar::hpm::vecStereoHarrisPoints vecPointsStereo;

      static bool checkRectification(hpm::StereoHarrisPoint const& leftPt, 
				     hpm::StereoHarrisPoint const& rightPt) 
      {
	double rectCheck = fabs(rightPt.v - leftPt.v);
	return rectCheck < 2.0;
      }
      
      static bool checkDisparity(hpm::StereoHarrisPoint const& leftPt,
				 hpm::StereoHarrisPoint const& rightPt) 
      {
	double dispCheck = leftPt.u - rightPt.u;
	return dispCheck > 2.0;
      }

      bool isFeatureCandidate(std::size_t index_);

#ifndef THALES_TAROT_DISABLE
      jafar::image::Image* loadImage(unsigned int frameNumber) const {
	return stereoReader->left()->loadImage(frameNumber);
      }
#endif

      bool fillObservation(std::size_t index_, slam::Observation& obs, unsigned int sensorId_, unsigned int robotId_) const;

      void removeLandmark(slam::BaseFeature const& landmark) {
	for (jafar::hpm::vecStereoHarrisPoints::iterator it = vecPointsCur.begin();
	     it != vecPointsCur.end() ; ++it)
	  {
	    if (it->id == landmark.id()) {
	      JFR_DEBUG("StereoImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	      vecPointsStereo[it->stereoIndex].id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }

	for (jafar::hpm::vecHarrisPoints::iterator it = loopClosingPoints.begin();
	     it != loopClosingPoints.end() ; ++it)
	  {
	    if (it->id == landmark.id()) {
	      JFR_DEBUG("MonoImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }
      }
    public:

      StereoImagePointManager(slam::BaseSlam& slam_,
			      jafar::hpm::Engine& hpmEngineLoopClosing_, 
			      jafar::hpm::StereoTrackingEngine& hpmTrackingEngine_ 
#ifndef THALES_TAROT_DISABLE
			      ,jafar::datareader::StereoReader* stereoReader_
#endif
			) :
	ImagePointManager(slam_, hpmEngineLoopClosing_),
#ifndef THALES_TAROT_DISABLE
        stereoReader(stereoReader_),
#endif
        hpmTrackingEngine(hpmTrackingEngine_)
      {
	slam_.addEventListener(*this);
	slam_.addEventListener(*frameDataBase);
      }

      ~StereoImagePointManager() {}

      slam::Observation::ObservationType getObservationType() const {return slam::Observation::POINT_STEREOIMAGE;}

      jafar::hpm::vecStereoHarrisPoints const& getCurrentPoints() const {return vecPointsCur;}
      jafar::hpm::vecStereoHarrisPoints const& getPreviousPoints() const {return vecPointsPrev;}
      jafar::hpm::vecStereoHarrisPoints const& getStereoPoints() const {return vecPointsStereo;}


#ifndef THALES_TAROT
      void initFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);
#endif
      void initFrame(unsigned int frameIndex_, jafar::image::Image* imgLeft, jafar::image::Image* imgRight, unsigned int robotId_ = 0, unsigned int sensorId_ = 0);

#ifndef THALES_TAROT
      void processFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);
#endif
      void processFrame(unsigned int frameIndex_, jafar::image::Image* imgLeft, jafar::image::Image* imgRight, unsigned int robotId_ = 0, unsigned int sensorId_ = 0);

    }; // class StereoImagePointManager


/**
     * Class for doing bearing-only slam with an omni camera and when features are interest
     * points extracted from an image using Inverse Depth parameterization.
     * \ingroup slampt
     */
    class IdOmniImagePointManager : 
      public ImagePointManager,
      public slam::SlamEventAdapter {

    protected:

      /// For tracking points
      jafar::hpm::TrackingEngine& hpmTrackingEngine;

      jafar::hpm::vecHarrisPoints vecPointsPrev;
      jafar::hpm::vecHarrisPoints vecPointsCur;

      bool isFeatureCandidate(std::size_t index_) { return true; }

#ifndef THALES_TAROT_DISABLE
      jafar::image::Image* loadImage(unsigned int frameNumber) const {
	return imageReader.loadImage(frameNumber);
      }

      jafar::datareader::ImageReader imageReader;
#endif

      bool fillObservation(std::size_t index_, slam::Observation& obs, unsigned int sensorId_, unsigned int robotId_) const;

      void removeLandmark(unsigned int id) {
	for (jafar::hpm::vecHarrisPoints::iterator it = vecPointsCur.begin();
	     it != vecPointsCur.end() ; ++it)
	  {
	    if (it->id == id) {
	      JFR_DEBUG("IdOmniImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }

	for (jafar::hpm::vecHarrisPoints::iterator it = loopClosingPoints.begin();
	     it != loopClosingPoints.end() ; ++it)
	  {
	    if (it->id == id) {
	      JFR_DEBUG("IdOmniImagePointManager: remove feature " << it->id);
	      it->id = jafar::hpm::HarrisPoint::NO_ID;
	    }
	  }
      }

      void removeLandmark(slam::BaseFeature const& landmark) {
	removeLandmark(landmark.id());

      }




    public:

// new for omni
      // cells definition
      jafar::camera::CameraParabolicBarreto camera;
      jafar::camera::PanoMask omniMask;
      unsigned int imageMargin;
      std::size_t nbZonesInRadius;
      std::size_t nbZonesInPeriphery;
// /new


      IdOmniImagePointManager(slam::BaseSlam& slam_,
			    jafar::hpm::Engine& hpmEngineLoopClosing_,
			    jafar::hpm::TrackingEngine& hpmTrackingEngine_) :
	ImagePointManager(slam_, hpmEngineLoopClosing_), 
	hpmTrackingEngine(hpmTrackingEngine_),
	camera(),
	omniMask()
      {
	slam_.addEventListener(*this);
	slam_.addEventListener(*frameDataBase);
      }

      ~IdOmniImagePointManager() {}

			// overload for omni images
      virtual void setupZones(jafar::camera::CameraParabolicBarreto& camera_,
					unsigned int imageMargin_,
					std::size_t nbZonesInRadius_,
					std::size_t nbZonesInPeriphery_,
					unsigned int nbDesiredFeaturesPerZone_);

			virtual boost::tuple<bool, unsigned int> isPointInZone(double u, double v);


      slam::Observation::ObservationType getObservationType() const {return slam::Observation::POINT_OMNIIMAGE;}

#ifndef THALES_TAROT_DISABLE
      jafar::datareader::ImageReader& getImageReader() {return imageReader;}
#endif

      jafar::hpm::vecHarrisPoints const& getCurrentPoints() const {return vecPointsCur;}
      jafar::hpm::vecHarrisPoints const& getPreviousPoints() const {return vecPointsPrev;}

#ifndef THALES_TAROT
      void initFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);
#endif
      void initFrame(unsigned int frameIndex_, jafar::image::Image* image,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);

#ifndef THALES_TAROT
      void processFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);
#endif
      void processFrame(unsigned int frameIndex_, jafar::image::Image* image,unsigned int robotId_ = 0, unsigned int sensortId_ = 0);

    }; // class IdOmniImagePointManager



  } // namespace slam
} // namespace jafar

#endif // SLAM_IMAGE_POINT_MANAGER_HPP

