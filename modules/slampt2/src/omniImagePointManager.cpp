/* $Id$ */

#ifndef THALES_TAROT_DISABLE

#include <cmath>
#include <sstream>

#include <highgui.h> // opencv

#include "kernel/jafarMacro.hpp"
#include "kernel/timingTools.hpp"

#include "slampt/omniImagePointManager.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar::slam;
using namespace jafar::slampt;
using namespace jafar;


/*
 * class OmniImagePointManager
 */

OmniImagePointManager::OmniImagePointManager(BearingOnlySlam& slam_,
                                             camera::CameraParabolicBarreto& camera_,
					     jafar::hpm::Engine& hpmEngineLoopClosing_,
					     jafar::hpm::TrackingEngine& hpmTrackingEngine_) :
  idFactory(),
  slam(slam_),
  hpmEngineLoopClosing(hpmEngineLoopClosing_),
  frameDataBase(),
  databaseImageAdded(false),
  databaseImageDistance(0),
  loopClosingDetected(false),
  loopClosingFrameIndex(0),
  loopClosingImageDistance(0),
  loopClosingNbUsedFeatures(0),
  pointsInZone(),
  nbFeaturesInZone(),
  knownFeatures(),
  newFeatures(),
  hpmTrackingEngine(hpmTrackingEngine_),
  camera(camera_),
  omniMask()
{
  slam_.addBoEventListener(*this);
  slam_.addBoEventListener(frameDataBase);
  omniMask.setup(camera);
}

OmniImagePointManager::~OmniImagePointManager() {}

void OmniImagePointManager::setupManager(unsigned int imageMargin_,
					 std::size_t nbZonesInRadius_,
					 std::size_t nbZonesInPeriphery_,
					 unsigned int nbDesiredFeaturesPerZone_)
{
  imageMargin = imageMargin_;
  nbDesiredFeaturesPerZone = nbDesiredFeaturesPerZone_;
  nbZonesInRadius = nbZonesInRadius_;
  nbZonesInPeriphery = nbZonesInPeriphery_;
  pointsInZone.resize(nbZonesInRadius*nbZonesInPeriphery);
  nbFeaturesInZone.resize(nbZonesInRadius*nbZonesInPeriphery); 
}

void OmniImagePointManager::setupDataBase(int databaseNbImagesMax,
					  double databaseImageDistanceMin,
					  int loopClosingIndexDistanceMin,
					  double loopClosingImageDistanceMax,
                                          std::string const& imageDir)
{
  frameDataBase.databaseNbImagesMax = databaseNbImagesMax;
  frameDataBase.databaseImageDistanceMin = databaseImageDistanceMin;
  frameDataBase.loopClosingIndexDistanceMin = loopClosingIndexDistanceMin;
  frameDataBase.loopClosingImageDistanceMax = loopClosingImageDistanceMax;
  dataBaseImageReader.setup(imageDir);
}

void OmniImagePointManager::addNewFeature(std::size_t index_, hpm::HarrisPoint& point_)
{
  JFR_PRECOND(point_.id == hpm::HarrisPoint::NO_ID, 
	      "OmniImagePointManager::addNewFeature: invalid new feature");
  point_.id = getNewFeatureId();
  JFR_DEBUG("OmniImagePointManager::addNewFeature: " << point_.id);
  newFeatures.push_back(index_);
}

void OmniImagePointManager::addKnownFeature(std::size_t index_, jafar::hpm::HarrisPoint const& point_)
{
  JFR_PRECOND(point_.id != hpm::HarrisPoint::NO_ID, 
	      "OmniImagePointManager::addFeature: invalid feature");
  knownFeatures.push_back(index_);
}

void OmniImagePointManager::slamProcessObservations() const
{
  slam::Observation obs(Observation::POINT_OMNIIMAGE);


  typedef std::list<Observation*> ObsList;
  ObsList knownFeaturesObs;

  for (FeaturesList::const_iterator it = knownFeatures.begin() ; it != knownFeatures.end() ; ++it) {
    if (fillObservation(*it, obs))
      knownFeaturesObs.push_back(new Observation(obs));
    else
      JFR_WARNING("OmniImagePointManager: known feature observation ignored");
  }

  ObsList newFeaturesObs;

  for (FeaturesList::const_iterator it = newFeatures.begin() ; it != newFeatures.end() ; ++it) {
    if (fillObservation(*it, obs))
      newFeaturesObs.push_back(new Observation(obs));
    else
      JFR_WARNING("OmniImagePointManager: new feature observation ignored");
  }

  JFR_TRACE_BEGIN;
  slam.processObservations(currentFrameIndex,
			   knownFeaturesObs, 
			   newFeaturesObs);
  JFR_TRACE_END("OmniImagePointManager::slamProcessObservations");
}

boost::tuple<bool, unsigned int> OmniImagePointManager::isPointInZone(double u, double v) {

  jblas::vec2 pt;
  pt(0) = u;
  pt(1) = v;
  jblas::vec2 centerToPt = pt - camera.getMirrorCenter();

  double rho = ublas::norm_2(centerToPt);
  double theta = atan2(centerToPt(1), centerToPt(0));

  // point outside of image
  if (rho < camera.maskRadius + imageMargin || rho > camera.imageRadius - imageMargin) {
//     JFR_WARNING("Point outside of omni image - "
// 		<< "point: " << pt 
// 		<< " - rho: " << rho 
// 		<< " [" << camera.maskRadius << "," << camera.imageRadius << "]");
    return boost::make_tuple(false,0);
  }

  double r = rho - camera.maskRadius - imageMargin;
  
  return boost::make_tuple(true, computeIndex(r, nbZonesInRadius, camera.imageRadius - camera.maskRadius - 2*imageMargin)*nbZonesInPeriphery 
			   + computeIndex(theta + M_PI, nbZonesInPeriphery, 2*M_PI) );
}

int OmniImagePointManager::computeIndex(double x_, int nbInt_, double xMax_) {
  return int(floor(x_ / (xMax_/nbInt_)));
}


void OmniImagePointManager::writeLogHeader(jafar::kernel::DataLogger& dataLogger) const
{
  dataLogger.writeComment("slam: OmniImagePointManager");
  dataLogger.writeLegend("nb new features");
  dataLogger.writeLegend("nb known features");
  dataLogger.writeLegend("database image added (0/1)");
  dataLogger.writeLegend("database image distance");
  dataLogger.writeLegend("loop closing image distance");
  dataLogger.writeLegend("loop closing detected (0/1)");
  dataLogger.writeLegend("loop closing frame index");
  dataLogger.writeLegend("loop closing nb features");
  dataLogger.writeLegend("loop closing elapsed time (ms)");
}

void OmniImagePointManager::writeLogData(jafar::kernel::DataLogger& dataLogger) const
{
  dataLogger.writeData(newFeatures.size());
  dataLogger.writeData(knownFeatures.size());
  dataLogger.writeData(databaseImageAdded);
  dataLogger.writeData(databaseImageDistance);
  dataLogger.writeData(loopClosingImageDistance);
  dataLogger.writeData(loopClosingDetected);
  dataLogger.writeData(loopClosingFrameIndex);
  dataLogger.writeData(loopClosingNbUsedFeatures);
  dataLogger.writeData(loopClosingElapsedTime);
}

bool OmniImagePointManager::fillObservation(std::size_t index_, Observation& obs) const
{
  vec z(2);

  hpm::HarrisPoint const& pt = getCurrentPoints()[index_];

  z(0) = pt.u;
  z(1) = pt.v;
  
  obs.set(pt.id, z);
  
  return true;
}

void OmniImagePointManager::initFrame(unsigned int frameIndex_)
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  initFrame(*im, frameIndex_);
  delete im;
}

void OmniImagePointManager::initFrame(boost::posix_time::time_duration const& curTime, image::Image& image)
{
  slam.init(curTime);
  initFrame(image, -1);
}

void OmniImagePointManager::initFrame(image::Image& im, int frameIndex_)
{
  if (frameIndex_ != -1)
    currentFrameIndex = frameIndex_;
  else
    currentFrameIndex=0;

  JFR_DEBUG("OmniImagePointManager: init frame: " << currentFrameIndex);

  JFR_TRACE_BEGIN;
  idFactory = 0;

  // apply omni mask
  omniMask.apply(im);

  frameDataBase.omniImageDB.init(camera, im, frameDataBase.databaseNbImagesMax);
  databaseImageDistance = 0;

  hpmTrackingEngine.initTracking(im, vecPointsCur);

  featureSelection(vecPointsCur);

  slamProcessObservations();

  addFrameToDataBase(vecPointsCur, im);

  JFR_TRACE_END("OmniImagePointManager::initFrame");
}

void OmniImagePointManager::processFrame(unsigned int frameIndex_) 
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  processFrame(*im, frameIndex_);
  delete im;
}

void OmniImagePointManager::processFrame(jafar::image::Image& im, int frameIndex_)
{
  if (frameIndex_ != -1)
    currentFrameIndex = frameIndex_;
  else
    ++currentFrameIndex;

  JFR_DEBUG("OmniImagePointManager: process frame: " << currentFrameIndex);

  // apply omni mask
  omniMask.apply(im);

  loopClosingDetected = loopClosingDetection(im);

  databaseImageAdded = false;
  databaseImageDistance = frameDataBase.omniImageDB.getImageDistance(im, frameDataBase.frameList.back().index);

  // swap vecPointsCur vecPointsPrev
  vecPointsPrev.swap(vecPointsCur);

  JFR_TRACE_BEGIN;

  hpmTrackingEngine.track(im, vecPointsPrev, vecPointsCur, true);

  if (loopClosingDetected) {
    // update points id according to loop closing process
    loopClosing(vecPointsCur, im);
  }

  featureSelection(vecPointsCur);

  slamProcessObservations();

  //   if (currentFrameIndex % 2 == 0) {
  if (databaseImageDistance > frameDataBase.databaseImageDistanceMin) {
    addFrameToDataBase(vecPointsCur, im);
  }

  JFR_TRACE_END("OmniImagePointManager::processFrame()");
}

bool OmniImagePointManager::loopClosingDetection(image::Image const& image)
{
  JFR_DEBUG("ImagePointManager::loopClosingDetection:");
  loopClosingElapsedTime = 0;

  // find the relevant frame
  kernel::Chrono chrono;

  if (currentFrameIndex <= frameDataBase.loopClosingIndexDistanceMin)
    return false;

  try {
    boost::tie(loopClosingFrameIndex, loopClosingImageDistance) = 
      frameDataBase.omniImageDB.getClosestImage(image, 0, currentFrameIndex - frameDataBase.loopClosingIndexDistanceMin);
    loopClosingElapsedTime = chrono.elapsed();
    JFR_DEBUG("locpano:getClosestImage: " << loopClosingFrameIndex << " - " << loopClosingImageDistance << " - " 
              << loopClosingElapsedTime << "ms");

    JFR_POSTCOND(currentFrameIndex - loopClosingFrameIndex > frameDataBase.loopClosingIndexDistanceMin,
                 "invalid index, problem in locpano::getClosestImage() " 
                 << loopClosingFrameIndex << " " << currentFrameIndex);
    return (loopClosingImageDistance < frameDataBase.loopClosingImageDistanceMax);
  }
  catch(kernel::Exception& e) {
    JFR_WARNING("OmniImagePointManager::loopClosingDetection: detetction failed:" << e);
    return false;
  }
}

void OmniImagePointManager::loopClosing(hpm::vecHarrisPoints& framePoints_, image::Image const& imCur)
{
  loopClosingNbUsedFeatures = 0;

  FrameDataBase::DBFrameList::iterator tentativeLoopClosingFrame = frameDataBase.frameList.begin();
  while(tentativeLoopClosingFrame != frameDataBase.frameList.end() && tentativeLoopClosingFrame->index != loopClosingFrameIndex)
    ++tentativeLoopClosingFrame;

  if (tentativeLoopClosingFrame == frameDataBase.frameList.end()) {
    JFR_RUN_TIME("OmniImagePointManager::loopClosing: frame database and omni image database are not syncronized ! frame " << loopClosingFrameIndex << " not found");
  }

  JFR_DEBUG("Tentative loop closing: frame " << tentativeLoopClosingFrame->index);
  JFR_VDEBUG("nb tentative landmarks: " << tentativeLoopClosingFrame->landmarks.size());

  hpm::vecHarrisPoints framePointsTmp; // FIXME optimization
  hpm::mapMatches loopClosingMatches;
  hpm::mapMatches frameDataBaseUpdateMatch;

  // (re)compute harris points of frame tentativeLoopClosingFrame->index
  // compute matches between currentFrameIndex and tentativeLoopClosingFrame->index

  image::Image* imDb = dataBaseImageReader.loadImage(tentativeLoopClosingFrame->index);
  omniMask.apply(*imDb);
  hpmEngineLoopClosing.computeMatch(*imDb, imCur,
				    loopClosingPoints, framePointsTmp, loopClosingMatches);
  delete imDb;
  frameDataBaseUpdateMatch.insert(loopClosingMatches.begin(), loopClosingMatches.end());
  JFR_DEBUG("matched points: " << loopClosingMatches.size());

  // iterate on the past frame landmarks
  for (DBFrame::LandmarksList::const_iterator it =  tentativeLoopClosingFrame->landmarks.begin();
       it != tentativeLoopClosingFrame->landmarks.end() ; ++it) {
    // is the point corresponding to that landmark is matched ?
    hpm::mapMatches::iterator match = loopClosingMatches.find(it->hpmIndex);
    if (match != loopClosingMatches.end()) {
      loopClosingNbUsedFeatures++;
      JFR_VDEBUG("loop closing frame feature matched: " << it->id);

      // iterate over framePoints_ to be sure this feature is not already tracked
      bool featureAlreadyTracked = false;
      for (std::size_t i = 0 ; i < framePoints_.size() ; ++i) {
	if (i != match->second && framePoints_[i].id == it->id) {
	  featureAlreadyTracked = true;
	  break;
	}
      }
      if (featureAlreadyTracked) {
	JFR_WARNING("OmniImagePointManager::loopClosing: feature matched during loop closing has the same id as an other feature which is already tracked, ignoring this match. id: " << it->id);
	continue;
      }

      if (framePoints_[match->second].id == hpm::HarrisPoint::NO_ID) {
	// the point is not yet a landmark
	framePoints_[match->second].id = it->id;
      }
      else {
	// the point is already a landmark
	if (framePoints_[match->second].id != it->id) {
	  JFR_VDEBUG("loop closing feature already in the map with another id, removing feature " 
		     << framePoints_[match->second].id);
	  unsigned int oldId = framePoints_[match->second].id;
	  unsigned int newId = it->id;

	  slam.removeFeature(oldId);
	  framePoints_[match->second].id = newId;

	  // back propagation of this change in the previous DBFrames
	  JFR_VDEBUG("back propagation of id: " << oldId << " -> " << newId);
	  // FIXME
	  // 		  frameDataBase.changeLandmarkId(oldId, newId, 
	  // 						 tentativeLoopClosingFrame->index, 
	  // 						 frameDataBase.frameList.back().index);
	  frameDataBase.changeLandmarkId(oldId, newId, 
					 0,
					 frameDataBase.frameList.back().index);
	}
	else {
	  JFR_VDEBUG("confirm loop closing match: " << it->id);
	}
      }
      frameDataBaseUpdateMatch.erase(match->first);
    }
    else {
      JFR_VDEBUG("loop closing frame feature NOT matched: " << it->id);
    }
  }

  JFR_DEBUG("loop closing nb lanmarks: " << loopClosingNbUsedFeatures);

  jafar::hpm::backPropagateId(loopClosingMatches, loopClosingPoints, framePoints_);

  // database frame update
  for (hpm::mapMatches::const_iterator match = frameDataBaseUpdateMatch.begin() ; match != frameDataBaseUpdateMatch.end() ; ++match)
    {
      if (framePoints_[match->second].id != hpm::HarrisPoint::NO_ID) {
	tentativeLoopClosingFrame->landmarks.push_back(DBFrameLandmark(match->first, framePoints_[match->second].id));
	JFR_VDEBUG("added feature " << framePoints_[match->second].id << " to DBFrame");
      }
    }
}

void OmniImagePointManager::featureSelection(hpm::vecHarrisPoints& points_)
{
  // init the process
  newFeatures.clear();
  knownFeatures.clear();

  for (std::size_t i = 0 ; i < nbFeaturesInZone.size() ; i++) {
    nbFeaturesInZone[i]=0;
    pointsInZone[i].clear();
  }

  // fill in the zones
  for (std::size_t i = 0 ; i < points_.size() ; ++i) {
    // features that are beeing tracked
    if (points_[i].id != hpm::HarrisPoint::NO_ID)
      addKnownFeature(i, points_[i]);
    
    bool isInZone;
    unsigned int zoneIndex;
    boost::tie(isInZone, zoneIndex) = isPointInZone(points_[i].u, points_[i].v);
    JFR_POSTCOND(zoneIndex >= 0 && zoneIndex < nbFeaturesInZone.size(),
                 "OmniImagePointManager::featureSelection invalid zone index: " << zoneIndex);
    if (isInZone) {
      if (points_[i].id != hpm::HarrisPoint::NO_ID) {
        nbFeaturesInZone[zoneIndex]++;
        JFR_DEBUG("feature " << points_[i] << "in zone " << zoneIndex);
      }
      else
        pointsInZone[zoneIndex].push_back(QualityIndexPoint(i,points_[i].lambdaLow));
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

void OmniImagePointManager::addFrameToDataBase(jafar::hpm::vecHarrisPoints& points_, image::Image const& image)
{
  databaseImageAdded = true;
  // save image to disk
  cvSaveImage(dataBaseImageReader.getFilePath(currentFrameIndex).c_str(), image);

  // harris point database
  frameDataBase.frameList.push_back(DBFrame());
  DBFrame& dbFrame = frameDataBase.frameList.back();
  dbFrame.index=currentFrameIndex;

  for (FeaturesList::const_iterator it = knownFeatures.begin() ; it != knownFeatures.end() ; ++it) {
    dbFrame.landmarks.push_back(DBFrameLandmark(*it, points_[*it].id));
  }
  for (FeaturesList::const_iterator it = newFeatures.begin() ; it != newFeatures.end() ; ++it) {
    dbFrame.landmarks.push_back(DBFrameLandmark(*it, points_[*it].id));
  }

  // omni images database
  kernel::Chrono chrono;
  frameDataBase.omniImageDB.addImage(image, currentFrameIndex, true);
  JFR_DEBUG("locpano:addImage: " << chrono.elapsed() << "ms");
}

void OmniImagePointManager::removeLandmark(BaseFeature const& landmark) {
  removeLandmark(landmark.id());
}

void OmniImagePointManager::removeTentativeLandmark(InitFeature const& landmark) {
  removeLandmark(landmark.id());
}

void OmniImagePointManager::removeLandmark(unsigned int id) {
  JFR_DEBUG("OmniImagePointManager::removeLandmark: " << id);
  for (jafar::hpm::vecHarrisPoints::iterator it = vecPointsCur.begin();
       it != vecPointsCur.end() ; ++it)
    {
      if (it->id == id) {
	JFR_DEBUG("OmniImagePointManager: remove current feature " << it->id);
	it->id = jafar::hpm::HarrisPoint::NO_ID;
      }
    }

  for (jafar::hpm::vecHarrisPoints::iterator it = loopClosingPoints.begin();
       it != loopClosingPoints.end() ; ++it)
    {
      if (it->id == id) {
	JFR_DEBUG("OmniImagePointManager: remove loop closing feature " << it->id);
	it->id = jafar::hpm::HarrisPoint::NO_ID;
      }
    }
	
}

// /*
//  * class BoOmniSlamVmeOdo
//  */

// BoOmniSlamVmeOdo::BoOmniSlamVmeOdo(std::string const& logDir_,
// 				   BearingOnlySlam& slam_,
// 				   OmniImagePointManager& omniImagePointManager_) :
//   logDir(logDir_),
//   dalaLogReader(logDir),
//   sensorData(),
//   vmePredictModel(),
//   odoPredictModel(),
//   slam(slam_),
//   omniImagePointManager(omniImagePointManager_),
//   frameNumber(0)
// {}

// void BoOmniSlamVmeOdo::init(int frameOffset {
//   JFR_TRACE_BEGIN;

//   dalaLogReader.nextData(sensorData);
  
//   while(sensorData.index != frameOffset || sensorData.type != SensorData::IMAGE_OMNI) {
//     dalaLogReader.nextData(sensorData);
//   }

//   JFR_DEBUG("BoOmniSlamVmeOdo::init: " << sensorData);

//   frameNumber = sensorData.index;

//   omniImagePointManager.initFrame(frameNumber);
//   JFR_TRACE_END("BoOmniSlamVmeOdo::init");
// }

// void BoOmniSlamVmeOdo::nextFrame() {
//   JFR_TRACE_BEGIN;
//   frameNumber++;
//   JFR_DEBUG("frame: " << frameNumber);

//   bool flagOdo = false;
//   jblas::vec2 odo;
//   geom::T3DEuler vme;

//   while(sensorData.index != frameNumber || sensorData.type != SensorData::IMAGE_OMNI) {
//     dalaLogReader.nextData(sensorData);
//     if (sensorData.type == SensorData::ODO) {
//       odo.assign(sensorData.data);
//       flagOdo = true;
//     }
//   }

//   // do we have vme ?
//   try {
//     using namespace std;
//     stringstream vmeFile;
//     vmeFile << logDir << "/vme/" 
// 	    << setw(4) << setfill('0') << (frameNumber-1) << "_to_" 
// 	    << setw(4) << setfill('0') << frameNumber << ".t3d";
//     vme.load(vmeFile.str());

//     vmePredictModel.computeU(vme.getX(), vmeNoiseFactor*vmeNoiseFactor * vme.getXCov());
//     slam.predict(vmePredictModel, vmePredictModel.u(), vmePredictModel.uCov());

//   }
//   catch(kernel::JafarException const& e) {

//     throw;

//     // FIXME different frame !

// //     JFR_DEBUG(e);
// //     JFR_DEBUG("VME is not available for " << frameNumber-1 << " -> " << frameNumber);
// //     JFR_DEBUG("using odometry: " << odo);
// //     JFR_ASSERT(flagOdo, "odometry is not available !!");
// //     slam.predict(odoPredictModel, odo);
//   }

//   omniImagePointManager.processFrame(frameNumber);
//   JFR_TRACE_END("BoOmniSlamVmeOdo::nextFrame");
// }

#endif
