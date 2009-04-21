/* $Id$ */

#include <cmath>
#include <sstream>

#include "kernel/jafarException.hpp"
#include "kernel/jafarDebug.hpp"

#include "jmath/ublasExtra.hpp"

#include "slam/feature.hpp"
#include "slampt/imagePointManager.hpp"

using std::cout;
using std::endl;
using std::flush;

using namespace jblas;
using namespace ublas;
using namespace jafar::slam;
using namespace jafar::slampt;
using namespace jafar;

/*
 * class MonoImageFrameDataBase
 */

FrameDataBase::DBFrameList::iterator 
MonoImageFrameDataBase::tentativeLoopClosing(unsigned int currentFrameIndex,
                                             geom::T3DEuler const& pose) {
  DBFrameList::iterator tentativeLoopClosingFrame = frameList.end();
  double dMin = loopClosingDistanceMax;
  
  for (DBFrameList::iterator it = frameList.begin() ; it != frameList.end() ; ++it) {
    vec3 z = pose.getT() - it->pose.getT();
    sym_mat zCov = project(pose.getXCov(), range(0,3), range(0,3)) + project(it->pose.getXCov(), range(0,3), range(0,3));

    double d = ublas::norm_2(z);
//     mat J(1,3);
//     ublasExtra::norm_2Jac(z,J);
//     double dCov = ublas::prod(J, prod(zCov, trans(J)))(0,0);

    JFR_DEBUG( JFR_PP_VAR(currentFrameIndex) << JFR_PP_VAR(it->index) << JFR_PP_VAR(loopClosingIndexDistanceMin)
               << JFR_PP_VAR(d) << JFR_PP_VAR(dMin) );
    if (currentFrameIndex - it->index > loopClosingIndexDistanceMin &&
	d < dMin) {
      dMin = d;
      tentativeLoopClosingFrame = it;
    }

//     sym_mat zCovInv(3,3);
//     jmath::ublasExtra::inv(zCov, zCovInv);
//     double dTmp = inner_prod(z, prod(zCovInv, z));
//     if (currentFrameIndex - it->index > loopClosingIndexDistanceMin &&
// 	dTmp < dMin) {
//       dMin = dTmp;
//       tentativeLoopClosingFrame = it;
//     }
  }
  return tentativeLoopClosingFrame;
}

/*
 * class ImagePointManager
 */

unsigned int ImagePointManager::idFactory = 1;

ImagePointManager::ImagePointManager(BaseSlam& slam_, jafar::hpm::Engine& hpmEngineLoopClosing_) :
  ownFrameDataBase(true),
  slam(slam_),
  hpmEngineLoopClosing(hpmEngineLoopClosing_),
  frameDataBase(new MonoImageFrameDataBase),
  loopClosingDetection(false),
  loopClosingNbUsedFeatures(0),
  pointsInZone(),
  nbFeaturesInZone(),
  knownFeatures(),
  newFeatures()
{}
  
ImagePointManager::~ImagePointManager()
{
  if( ownFrameDataBase )
  {
    delete frameDataBase;
  }
}


void ImagePointManager::setFrameDataBase( MonoImageFrameDataBase* _frameDataBase )
{
  if( ownFrameDataBase )
  {
    slam.removeEventListener( *frameDataBase );
    delete frameDataBase;
  }
  frameDataBase = _frameDataBase;
  slam.addEventListener( *frameDataBase );
  ownFrameDataBase = false;
}

void ImagePointManager::setupZones(unsigned int imageWidth_, unsigned int imageHeight_,
				   double reduction_,
				   std::size_t nbZonesU_,
				   std::size_t nbZonesV_,
				   unsigned int nbDesiredFeaturesPerZone_)
{
  imageWidth = (unsigned int)round(imageWidth_/reduction_);
  imageHeight = (unsigned int)round(imageHeight_/reduction_);
  nbDesiredFeaturesPerZone = nbDesiredFeaturesPerZone_;
  nbZonesU = nbZonesU_;
  nbZonesV = nbZonesV_;
  pointsInZone.resize(nbZonesU*nbZonesV);
  nbFeaturesInZone.resize(nbZonesU*nbZonesV); 
}

void ImagePointManager::setupLoopClosing(double loopClosingDistanceMax_, 
					 unsigned int loopClosingIndexDistanceMin_) {
  frameDataBase->loopClosingDistanceMax = loopClosingDistanceMax_;
  frameDataBase->loopClosingIndexDistanceMin = loopClosingIndexDistanceMin_;
}

void ImagePointManager::addNewFeature(std::size_t index_, hpm::HarrisPoint& point_)
{
  JFR_PRECOND(point_.id == hpm::HarrisPoint::NO_ID, 
	      "ImagePointManager::addNewFeature: invalid new feature");
  point_.id = getNewFeatureId();
  JFR_DEBUG("ImagePointManager::addNewFeature: " << point_.id);
  newFeatures.push_back(index_);
}

void ImagePointManager::addKnownFeature(std::size_t index_, jafar::hpm::HarrisPoint const& point_)
{
  JFR_PRECOND(point_.id != hpm::HarrisPoint::NO_ID, 
	      "ImagePointManager::addFeature: invalid feature");
  knownFeatures.push_back(index_);
}

#define CHECK_NO_TWICE_OBS

void ImagePointManager::slamProcessObservations(unsigned int robotId_, unsigned int sensorId_) const
{
  Observation obs(getObservationType());


  typedef std::list<Observation*> ObsList;
  ObsList knownFeaturesObs;
  ObsList newFeaturesObs;
  
#ifdef CHECK_NO_TWICE_OBS
  ObsList allObs;
#endif

  for (FeaturesList::const_iterator it = knownFeatures.begin() ; it != knownFeatures.end() ; ++it) {
    if (fillObservation(*it, obs, sensorId_, robotId_))
    {
      JFR_DEBUG("hasFeature?(" << obs.id << " :" << slam.hasFeature(obs.id));
      Observation* newObs = new Observation(obs);
      if( slam.hasFeature( obs.id, robotId_ ) )
      {
        knownFeaturesObs.push_back(newObs);
      } else {
        newFeaturesObs.push_back(newObs);
      }
#ifdef CHECK_NO_TWICE_OBS
      allObs.push_back(newObs);
#endif
    }
    else
      JFR_WARNING("ImagePointManager: known feature observation ignored");
  }

  for (FeaturesList::const_iterator it = newFeatures.begin() ; it != newFeatures.end() ; ++it) {
    if (fillObservation(*it, obs, sensorId_, robotId_))
    {
      Observation* newObs = new Observation(obs);
      newFeaturesObs.push_back(newObs);
#ifdef CHECK_NO_TWICE_OBS
      allObs.push_back(newObs);
#endif
    }
    else
      JFR_WARNING("ImagePointManager: new feature observation ignored");
  }
  
#ifdef CHECK_NO_TWICE_OBS
  for( ObsList::const_iterator it = allObs.begin(); it != allObs.end(); ++it)
  {
    for( ObsList::const_iterator it2 = allObs.begin(); it2 != it; ++it2)
    {
      JFR_ASSERT( (*it)->id != (*it2)->id, "Two observations of same id" << (*it)->id );
    }
  }
#endif

  JFR_DEBUG( JFR_PP_VAR( knownFeaturesObs.size() ) << JFR_PP_VAR( newFeaturesObs.size() )  );
  slam.processObservations(currentFrameIndex,
			   knownFeaturesObs, 
			   newFeaturesObs,robotId_);
}

boost::tuple<bool, unsigned int> ImagePointManager::isPointInZone(double u_, double v_) {
  return boost::make_tuple(true, computeZoneIndex(u_, nbZonesU, imageWidth) * nbZonesV
			   + computeZoneIndex(v_, nbZonesV, imageHeight) );
}

unsigned int ImagePointManager::computeZoneIndex(double x_, unsigned int nbInt_, double xMax_) {
  return int(floor(x_ / (xMax_/nbInt_)));
}

std::ostream& jafar::slampt::operator <<(std::ostream& s, const ImagePointManager& manager_) {
  for (std::size_t i = 0 ; i < manager_.pointsInZone.size() ; i++) {
    s << "zone " << i << ": " << manager_.pointsInZone[i].size() << std::endl;
  }
  return s;
}

void ImagePointManager::writeLogHeader(jafar::kernel::DataLogger& dataLogger) const
{
  dataLogger.writeComment("slam: ImagePointManager");
  dataLogger.writeLegend("nb new features");
  dataLogger.writeLegend("nb known features");
  dataLogger.writeLegend("loop closing detected (0/1)");
  dataLogger.writeLegend("loop closing frame index");
  dataLogger.writeLegend("loop closing nb features");
}

void ImagePointManager::writeLogData(jafar::kernel::DataLogger& dataLogger) const
{
  dataLogger.writeData(newFeatures.size());
  dataLogger.writeData(knownFeatures.size());
  dataLogger.writeData(loopClosingDetection);
  dataLogger.writeData(loopClosingFrameIndex);
  dataLogger.writeData(loopClosingNbUsedFeatures);
}

#ifndef THALES_TAROT_DISABLE

/*
 * class MonoImagePointManager
 */

bool MonoImagePointManager::fillObservation(std::size_t index_, Observation& obs, unsigned int sensorId_, unsigned int robotId_) const
{
  vec z(2);

  hpm::HarrisPoint const& pt = getCurrentPoints()[index_];

  z(0) = pt.u;
  z(1) = pt.v;
  
  obs.set(pt.id, z, sensorId_, robotId_);
  
  return true;
}

void MonoImagePointManager::initFrame(unsigned int frameIndex_,unsigned int robotId_, unsigned int sensorId_)
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  initFrame(frameIndex_, im, robotId_, sensorId_);
  delete im;
}

void MonoImagePointManager::initFrame(unsigned int frameIndex_, jafar::image::Image* im, unsigned int robotId_, unsigned int sensorId_)
{
  currentFrameIndex = frameIndex_;

  JFR_DEBUG("MonoImagePointManager: init frame: " << currentFrameIndex);

  JFR_TRACE_BEGIN;

  hpmTrackingEngine.initTracking(*im, vecPointsCur);

  featureSelection(vecPointsCur);

  slamProcessObservations(robotId_,sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

  JFR_TRACE_END("MonoImagePointManager::initFrame");
}

void MonoImagePointManager::processFrame(unsigned int frameIndex_, unsigned int robotId_, unsigned int sensorId_) 
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  processFrame(frameIndex_, im, robotId_, sensorId_);
  delete im;
}

void MonoImagePointManager::processFrame(unsigned int frameIndex_, jafar::image::Image* im, unsigned int robotId_, unsigned int sensorId_)
{
  //If there are several frame indexes this is not valid anymore
 //   JFR_PRECOND(frameIndex_ > currentFrameIndex, "MonoImagePointManager::processFrame: frame index should be increasing");

  currentFrameIndex = frameIndex_;

  JFR_DEBUG("MonoImagePointManager: process frame: " << currentFrameIndex);

  // swap vecPointsCur vecPointsPrev
  vecPointsPrev.swap(vecPointsCur);

  JFR_TRACE_BEGIN;
	JFR_DEBUG( im->width() << " " << im->height() << " " << im->depth() << " " << im->colorSpace() );
  hpmTrackingEngine.track(*im, vecPointsPrev, vecPointsCur, true);

  // update points id according to loop closing process
  loopClosing(vecPointsCur, robotId_);

  featureSelection(vecPointsCur);

  slamProcessObservations(robotId_,sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

  JFR_TRACE_END("MonoImagePointManager::processFrame()");
}

/*
 * class IdImagePointManager
 */

bool IdImagePointManager::fillObservation(std::size_t index_, Observation& obs, unsigned int sensorId_, unsigned int robotId_) const
{
  vec z(2);

  hpm::HarrisPoint const& pt = getCurrentPoints()[index_];

  z(0) = pt.u;
  z(1) = pt.v;
  
  obs.set(pt.id, z, sensorId_, robotId_);
  
  return true;
}

void IdImagePointManager::initFrame(unsigned int frameIndex_, unsigned int robotId_, unsigned int sensorId_)
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  initFrame(frameIndex_, im, robotId_, sensorId_);
//   delete im;
}

void IdImagePointManager::initFrame(unsigned int frameIndex_, jafar::image::Image* im, unsigned int robotId_, unsigned int sensorId_)
{
  currentFrameIndex = frameIndex_;

  JFR_DEBUG("IdImagePointManager: init frame: " << currentFrameIndex);

  JFR_TRACE_BEGIN;

  hpmTrackingEngine.initTracking(*im, vecPointsCur);

  featureSelection(vecPointsCur);

  slamProcessObservations(robotId_, sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

  JFR_TRACE_END("IdImagePointManager::initFrame");
}

void IdImagePointManager::processFrame(unsigned int frameIndex_, unsigned int robotId_, unsigned int sensorId_) 
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  processFrame(frameIndex_, im, robotId_, sensorId_);
  delete im;
}

void IdImagePointManager::processFrame(unsigned int frameIndex_, jafar::image::Image* im, unsigned int robotId_, unsigned int sensorId_)
{
    JFR_PRECOND(frameIndex_ > currentFrameIndex,
	      "IdImagePointManager::processFrame: frame index should be increasing");

  currentFrameIndex = frameIndex_;

  JFR_DEBUG("IdImagePointManager: process frame: " << currentFrameIndex);

  // swap vecPointsCur vecPointsPrev
  vecPointsPrev.swap(vecPointsCur);

//   JFR_TRACE_BEGIN;
	JFR_DEBUG( im->width() << " " << im->height() << " " << im->depth() << " " << im->colorSpace() );
  hpmTrackingEngine.track(*im, vecPointsPrev, vecPointsCur, true);

  // update points id according to loop closing process
  loopClosing(vecPointsCur, robotId_);

  featureSelection(vecPointsCur);

  slamProcessObservations(robotId_,sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

//   JFR_TRACE_END("IdImagePointManager::processFrame()");
}
#endif

/*
 * class StereoImagePointManager
 */

bool StereoImagePointManager::isFeatureCandidate(std::size_t index_) {
  hpm::StereoHarrisPoint const& point = getCurrentPoints()[index_];
  
  if (point.stereoIndex == hpm::StereoHarrisPoint::NO_STEREO_INDEX)
    return false;

  return checkDisparity(point, getStereoPoints()[point.stereoIndex]) && 
    checkRectification(point, getStereoPoints()[point.stereoIndex]);
}

bool StereoImagePointManager::fillObservation(std::size_t index_, Observation& obs, unsigned int sensorId_, unsigned int robotId_) const
{
  hpm::StereoHarrisPoint const& leftPt = getCurrentPoints()[index_];
  hpm::StereoHarrisPoint const& rightPt = getStereoPoints()[leftPt.stereoIndex];

  vec z(3);
  z(0) = leftPt.u;
  z(1) = leftPt.v;
  z(2) = leftPt.u - rightPt.u;

  if (!checkDisparity(leftPt, rightPt)) {
    JFR_WARNING("ImagePointManager::fillObservation: ignored observation ! disparity too small=" 
		<< z(2) << " (id: " << leftPt.id << ")");
    return false;
  }
  else if (!checkRectification(leftPt, rightPt)) {
    JFR_WARNING("ImagePointManager::fillObservation: ignored observation ! Check images rectification |uRight-uLeft|=" 
		<< fabs(rightPt.u - leftPt.u) << " (id: " << leftPt.id << ")");
    return false;
  }
  else {
    obs.set(leftPt.id, z, sensorId_, robotId_);
    return true;
  }
}


void StereoImagePointManager::initFrame(unsigned int frameIndex_, jafar::image::Image* imgLeft, jafar::image::Image* imgRight, unsigned int robotId_, unsigned int sensorId_)
{
  lastUsedRobot = robotId_;
  currentFrameIndex = frameIndex_;

  JFR_DEBUG("StereoImagePointManager: init frame: " << currentFrameIndex);

//   JFR_TRACE_BEGIN;

  hpmTrackingEngine.initTracking(*imgLeft, *imgRight, vecPointsCur, vecPointsStereo);

  featureSelection(vecPointsCur);

  // this is not needed by the algos, but useful for the display...
  hpm::propagateId(hpmTrackingEngine.getStereoMatches(), vecPointsCur, vecPointsStereo);

  slamProcessObservations(robotId_,sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

//   JFR_TRACE_END("StereoImagePointManager::initFrame()");
}

#ifndef THALES_TAROT
void StereoImagePointManager::initFrame(unsigned int frameIndex_, unsigned int robotId_, unsigned int sensorId_)
{
  image::Image* imL = stereoReader->left()->loadImage(frameIndex_);
  image::Image* imR = stereoReader->right()->loadImage(frameIndex_);
  initFrame(frameIndex_, imL, imR, robotId_, sensorId_);
  delete imL;
  delete imR;
}
#endif

void StereoImagePointManager::processFrame(unsigned int frameIndex_, jafar::image::Image* imgLeft, jafar::image::Image* imgRight, unsigned int robotId_, unsigned int sensorId_)
{
  lastUsedRobot = robotId_;

  JFR_PRECOND(frameIndex_ > currentFrameIndex,
	      "MonoImagePointManager::processFrame: frame index should be increasing");

  currentFrameIndex = frameIndex_;

  JFR_DEBUG("MonoImagePointManager: process frame: " << currentFrameIndex);
 
  // swap vecPointsCur vecPointsPrev
  vecPointsPrev.swap(vecPointsCur);

//   JFR_TRACE_BEGIN;

  hpmTrackingEngine.track(*imgLeft, *imgRight, vecPointsPrev, vecPointsCur, vecPointsStereo, true);

#ifndef THALES_TAROT_DISABLE
  // update points id according to loop closing process
  loopClosing(vecPointsCur, robotId_);
#endif

  featureSelection(vecPointsCur);

  // this is not needed by the algos, but useful for the display...
  hpm::propagateId(hpmTrackingEngine.getStereoMatches(), vecPointsCur, vecPointsStereo);

  slamProcessObservations(robotId_,sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

//   JFR_TRACE_END("StereoImagePointManager::processFrame()");
}

#ifndef THALES_TAROT
void StereoImagePointManager::processFrame(unsigned int frameIndex_, unsigned int robotId_, unsigned int sensorId_)
{
  image::Image* imL = stereoReader->left()->loadImage(frameIndex_);
  image::Image* imR = stereoReader->right()->loadImage(frameIndex_);
  processFrame(frameIndex_, imL, imR, robotId_, sensorId_);
  delete imL;
  delete imR;
}
#endif



/*
 * class IdOmniImagePointManager
 */


void IdOmniImagePointManager::setupZones(jafar::camera::CameraParabolicBarreto& camera_,
					unsigned int imageMargin_,
					std::size_t nbZonesInRadius_,
					std::size_t nbZonesInPeriphery_,
					unsigned int nbDesiredFeaturesPerZone_)
			{
				camera = camera_;
				omniMask.setup(camera);

				imageMargin = imageMargin_;
				nbDesiredFeaturesPerZone = nbDesiredFeaturesPerZone_;
				nbZonesInRadius = nbZonesInRadius_;
				nbZonesInPeriphery = nbZonesInPeriphery_;
				pointsInZone.resize(nbZonesInRadius*nbZonesInPeriphery);
				nbFeaturesInZone.resize(nbZonesInRadius*nbZonesInPeriphery); 
			}




boost::tuple<bool, unsigned int> IdOmniImagePointManager::isPointInZone(double u, double v) {

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
  
  return boost::make_tuple(true, computeZoneIndex(r, nbZonesInRadius, camera.imageRadius - camera.maskRadius - 2*imageMargin)*nbZonesInPeriphery 
			   + computeZoneIndex(theta + M_PI, nbZonesInPeriphery, 2*M_PI) );
}




bool IdOmniImagePointManager::fillObservation(std::size_t index_, Observation& obs, unsigned int sensorId_, unsigned int robotId_) const
{
  vec z(2);

  hpm::HarrisPoint const& pt = getCurrentPoints()[index_];

  z(0) = pt.u;
  z(1) = pt.v;
  
  obs.set(pt.id, z, sensorId_, robotId_);
  
  return true;
}

void IdOmniImagePointManager::initFrame(unsigned int frameIndex_, jafar::image::Image* im, unsigned int robotId_, unsigned int sensorId_)
{
  currentFrameIndex = frameIndex_;

  JFR_DEBUG("IdOmniImagePointManager: init frame: " << currentFrameIndex);

  JFR_TRACE_BEGIN;
	omniMask.apply(*im);

  hpmTrackingEngine.initTracking(*im, vecPointsCur);
//  delete im;
  featureSelection(vecPointsCur);

  slamProcessObservations(robotId_, sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

  JFR_TRACE_END("IdOmniImagePointManager::initFrame");
}

#ifndef THALES_TAROT
void IdOmniImagePointManager::initFrame(unsigned int frameIndex_, unsigned int robotId_, unsigned int sensorId_)
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  initFrame(frameIndex_, im, robotId_, sensorId_);
}
#endif

void IdOmniImagePointManager::processFrame(unsigned int frameIndex_, jafar::image::Image* im, unsigned int robotId_, unsigned int sensorId_)
{
    JFR_PRECOND(frameIndex_ > currentFrameIndex,
	      "IdOmniImagePointManager::processFrame: frame index should be increasing");

  currentFrameIndex = frameIndex_;

  JFR_DEBUG("IdOmniImagePointManager: process frame: " << currentFrameIndex);

   JFR_TRACE_BEGIN;
	omniMask.apply(*im);
  // swap vecPointsCur vecPointsPrev
  vecPointsPrev.swap(vecPointsCur);

	JFR_DEBUG( im->width() << " " << im->height() << " " << im->depth() << " " << im->colorSpace() );
  hpmTrackingEngine.track(*im, vecPointsPrev, vecPointsCur, true);

#ifndef THALES_TAROT_DISABLE
  // update points id according to loop closing process
  loopClosing(vecPointsCur, robotId_);
#endif

  featureSelection(vecPointsCur);

  slamProcessObservations(robotId_,sensorId_);

  updateFrameDataBase(vecPointsCur, robotId_);

   JFR_TRACE_END("IdOmniImagePointManager::processFrame()");
}

#ifndef THALES_TAROT
void IdOmniImagePointManager::processFrame(unsigned int frameIndex_, unsigned int robotId_, unsigned int sensorId_) 
{
  image::Image* im = imageReader.loadImage(frameIndex_);
  processFrame(frameIndex_, im, robotId_, sensorId_);
}
#endif

