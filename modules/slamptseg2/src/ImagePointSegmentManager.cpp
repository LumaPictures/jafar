/* $Id$ */

#include "slamptseg/ImagePointSegmentManager.hpp"

#include "slam/imagePointManager.hpp"

using namespace jafar::slamptseg;

PointSegmentManager::PointSegmentManager( slam::StereoImagePointManager* _stereoImagePointManager, slamseg::ImageSegmentManager* _imageSegmentManager) :
    m_stereoImagePointManager( _stereoImagePointManager ), m_imageSegmentManager(_imageSegmentManager)
{
}

PointSegmentManager::~PointSegmentManager()
{
}

void PointSegmentManager::initFrame(unsigned int frameIndex_,unsigned int robotId_, unsigned int sensorId_ )
{
  _stereoImagePointManager->initFrame( frameIndex_, robotId_, sensorId_ );
  _imageSegmentManager->initFrame( frameIndex_, robotId_, sensorId_ );
}

void PointSegmentManager::processFrame(unsigned int frameIndex_,unsigned int robotId_, unsigned int sensorId_ )
{
  _stereoImagePointManager->processFrame( frameIndex_, robotId_, sensorId_ );
  _stereoImagePointManager->initFrame( frameIndex_, robotId_, sensorId_ );
}

