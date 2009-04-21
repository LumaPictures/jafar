/* $Id$ */

#include <slamseg/ImageSegmentManager.hpp>

using namespace jafar;
using namespace jafar::slamseg;

#ifdef HAVE_STEREOPIXEL

#include <datareader/MonoReader.hpp>
#include <datareader/StereoReader.hpp>
#include <dseg3d/DirectSegments3DTracker.hpp>
#include <geom/Segment.hpp>
#include <slam/baseSlam.hpp>
#include <slam/lineTools.hpp>
#include <dseg3d/Segment3DObservation.hpp>

ImageSegmentManager::ImageSegmentManager( slam::BaseSlam& _slam, dseg3d::DirectSegments3DTracker* _tracker, jafar::datareader::StereoReader* _stereoReader, slam::Observation::ObservationType type_ ) : 
    m_slam( _slam ),
    m_tracker( _tracker ),
    m_stereoReader( _stereoReader ),
    m_observationType(type_)
{
  
}

ImageSegmentManager::~ImageSegmentManager()
{
}

void ImageSegmentManager::processFrame(unsigned int frameIndex_, unsigned int robotId_ , unsigned int sensorId_, unsigned int leftSensorId_, unsigned int rightSensorId_ )
{
  image::Image* imL = m_stereoReader->left()->loadImage(frameIndex_);
  image::Image* imR = m_stereoReader->right()->loadImage(frameIndex_);
  
  m_tracker->process( *imL, *imR );
  
  // Process observations
  const std::list< jafar::dseg3d::Segment3DObservation >& segments = m_tracker->segment3DObservations();
  ublas::range range(0, 3);
  std::list<slam::Observation*> knownFeaturesObs;
  std::list<slam::Observation*> knownFeaturesObs2;
  std::list<slam::Observation*> newFeaturesObs;
  jblas::vec6 uvds;
  JFR_DEBUG( JFR_PP_VAR(segments.size()) );
  JFR_FOREACH( const jafar::dseg3d::Segment3DObservation& segmentObs, segments )
  {
    
    bool newFeature = not m_slam.hasFeature( segmentObs.segment3D().id() );
    
    if( newFeature or ! segmentObs.isHorizontal() )
    {
      slam::StereoSegmentObservation* obs = new slam::StereoSegmentObservation( robotId_ );
      obs->setExtremities( segmentObs.uvd1(), segmentObs.uvd2() );
      obs->sensorId = sensorId_;
      obs->id = segmentObs.segment3D().id();
    
      ublas::subrange( uvds, 0, 3 ).assign( segmentObs.uvd1() );
      ublas::subrange( uvds, 3, 6 ).assign( segmentObs.uvd2() );
      JFR_DEBUG( obs->id << " " << uvds );
      
      jblas::vec2 le1 = ublas::subrange( segmentObs.uvd1(), 0, 2 );
      jblas::vec2 le2 = ublas::subrange( segmentObs.uvd2(), 0, 2 );
      JFR_DEBUG( JFR_PP_VAR( jafar::slam::lineTools::extToRhoTheta(le1, le2 ) ) );
      JFR_DEBUG( JFR_PP_VAR( jafar::slam::lineTools::uvdsToStereoRhoTheta(uvds) ) );
      
      obs->z.assign( jafar::slam::lineTools::uvdsToStereoRhoTheta(uvds) );
      if( newFeature ) {
        newFeaturesObs.push_back( obs );
      } else {
        knownFeaturesObs.push_back( obs );
      }
    } else {
      // Create observation for the left camera
      slam::SegmentObservation* obsLeft = new slam::SegmentObservation( m_observationType, robotId_ );
      jblas::vec2 le1 = ublas::subrange( segmentObs.uvd1(), 0, 2 );
      jblas::vec2 le2 = ublas::subrange( segmentObs.uvd2(), 0, 2 );
      obsLeft->setExtremities( le1, le2 );
      obsLeft->sensorId = leftSensorId_;
      obsLeft->id = segmentObs.segment3D().id();
      obsLeft->z.assign( jafar::slam::lineTools::extToRhoTheta(le1, le2 ) );
      
      JFR_DEBUG( JFR_PP_VAR( obsLeft->id ) );
      JFR_DEBUG( JFR_PP_VAR( le1 ) << JFR_PP_VAR( le2 ) );
      JFR_DEBUG( JFR_PP_VAR( jafar::slam::lineTools::extToRhoTheta(le1, le2 ) ) );
      // Create observation for the right camera
      slam::SegmentObservation* obsRight = new slam::SegmentObservation( m_observationType, robotId_ );
      jblas::vec2 re1 = le1; re1(0) += segmentObs.uvd1()(2);
      jblas::vec2 re2 = le2; re2(0) += segmentObs.uvd2()(2);
      obsRight->setExtremities( re1, re2 );
      obsRight->sensorId = rightSensorId_;
      obsRight->id = segmentObs.segment3D().id();
      obsRight->z.assign( jafar::slam::lineTools::extToRhoTheta(re1, re2 ) );
      
      knownFeaturesObs.push_back( obsLeft );
    }
  }
  JFR_DEBUG( newFeaturesObs.size() << " new features and " << knownFeaturesObs.size() << " reobservations");
  m_slam.processObservations( frameIndex_, knownFeaturesObs, newFeaturesObs, robotId_ );

  std::list<slam::Observation*> knownFeaturesObs3;
  
  JFR_FOREACH( slam::Observation* obsRight, knownFeaturesObs2 )
  {
    if( m_slam.hasFeature( obsRight->id ) )
    {
      knownFeaturesObs3.push_back( obsRight );
    } else {
      delete obsRight;
    }
  }
  m_slam.processObservations( frameIndex_, knownFeaturesObs3, std::list<slam::Observation*>(), robotId_ );
}

#endif
