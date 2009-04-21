/* $Id$ */

#include <slamseg/MonoImageSegmentManager.hpp>

#include <datareader/ImageReader.hpp>
#include <dseg/DirectSegmentsDetector.hpp>
#include <dseg/DirectSegmentsTracker.hpp>
#include <dseg/SegmentHypothesis.hpp>
#include <dseg/SegmentsSet.hpp>
#include <dseg/StaticPredictor.hpp>
#include <slam/baseSlam.hpp>
#include <slam/segmentFeature.hpp>
#include <dseg/SegmentsSelector.hpp>

using namespace jafar;
using namespace jafar::slamseg;

struct MonoImageSegmentManager::Private {
  Private( slam::BaseSlam& _slam ) : slam(_slam), id(1) {}
  slam::BaseSlam& slam;
  dseg::DirectSegmentsDetector* detector;
  dseg::SegmentsSelector segmentsSelector;
  dseg::DirectSegmentsTracker* tracker;
  const dseg::Predictor* trackerPredictor;
  jafar::datareader::ImageReader* imageReader;
  dseg::SegmentsSet currentLines;
  uint id;
  slam::Observation::ObservationType observationType;
};


MonoImageSegmentManager::MonoImageSegmentManager( slam::BaseSlam& _slam, jafar::datareader::ImageReader* _imageReader, const dseg::Predictor* _predictor, slam::Observation::ObservationType type_ ) : d(new Private(_slam))
{
  d->detector = new dseg::DirectSegmentsDetector;
  d->tracker = new dseg::DirectSegmentsTracker;
  d->imageReader = _imageReader;
  d->trackerPredictor = _predictor;
  d->observationType = type_;
  if( not d->trackerPredictor )
  {
    d->trackerPredictor = new dseg::StaticPredictor(4.0, 0.2);
  }
}

MonoImageSegmentManager::~MonoImageSegmentManager()
{
}

void MonoImageSegmentManager::processFrame(unsigned int frameIndex_,unsigned int robotId_, unsigned int stereoSensorId_ )
{
  image::Image* imL = d->imageReader->loadImage(frameIndex_);
  if( d->currentLines.count() > 0 )
  {
    dseg::SegmentsSet trackedLines;
    d->tracker->trackSegment( *imL, d->currentLines, d->trackerPredictor, trackedLines );
    d->currentLines = trackedLines;
  }
  
  if( d->currentLines.count() <= 5 ) // look for segment before the count reach 0
  {
    dseg::SegmentsSet previous = d->currentLines;
    d->detector->detectSegment( *imL, d->currentLines, 0 );
    d->currentLines = d->segmentsSelector.select( d->currentLines, &previous );
  }
  
  std::list<slam::Observation*> newFeaturesObs;
  std::list<slam::Observation*> knownFeaturesObs;
  
  for( uint i = 0; i < d->currentLines.count(); ++i )
  {
    dseg::SegmentHypothesis* sh = d->currentLines.segmentAt( i );
    jblas::vec2 ext1; ext1(0) = sh->x1(); ext1(1) = sh->y1();
    jblas::vec2 ext2; ext2(0) = sh->x2(); ext2(1) = sh->y2();
    jblas::vec2 line = slam::ImageEuclideanPluckerFeatureObserveModel::extToLine(ext1, ext2);
    slam::SegmentObservation* obs = new slam::SegmentObservation( d->observationType, robotId_ );
    obs->set(sh->id(), line);
    obs->setExtremities(ext1, ext2);
    if(d->slam.hasFeature(sh->id()))
    {
      knownFeaturesObs.push_back(obs);
    } else {
      newFeaturesObs.push_back(obs);
    }
  }
  JFR_DEBUG( newFeaturesObs.size() );
  d->slam.processObservations( frameIndex_, knownFeaturesObs, newFeaturesObs, robotId_ );
  
  delete imL;
}

dseg::DirectSegmentsDetectorBase* MonoImageSegmentManager::detector()
{
  return d->detector;
}

const dseg::SegmentsSet& MonoImageSegmentManager::currentLines() const
{
  return d->currentLines;
}
