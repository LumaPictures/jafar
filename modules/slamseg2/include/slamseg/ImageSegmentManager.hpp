/* $Id$ */

#ifndef _JAFAR_SLAMSEG_IMAGE_SEGMENT_MANAGER_HPP_
#define _JAFAR_SLAMSEG_IMAGE_SEGMENT_MANAGER_HPP_

#include <jafarConfig.h>

#include <slam/segmentFeature.hpp>

namespace jafar {
  namespace datareader {
    class StereoReader;
  }
  namespace dseg3d {
    class DirectSegments3DTracker;
  }
  namespace slam {
    class BaseSlam;
  }
  namespace slamseg {
    /**
     * @ingroup slamseg
     * 
     * Allow to control a segment-based slam.
     */
#ifdef HAVE_STEREOPIXEL
    class ImageSegmentManager {
      public:
        ImageSegmentManager( slam::BaseSlam& _slam, dseg3d::DirectSegments3DTracker* _tracker, jafar::datareader::StereoReader* _stereoReader, 
             slam::Observation::ObservationType type_ = slam::Observation::SEGMENT_IMAGE );
        ~ImageSegmentManager();
        void processFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int stereoSensorId_ = 0, unsigned int leftSensorId_ = 1, unsigned int rightSensorId_ = 2);
      private:
        slam::BaseSlam& m_slam;
        dseg3d::DirectSegments3DTracker* m_tracker;
        jafar::datareader::StereoReader* m_stereoReader;
        slam::Observation::ObservationType m_observationType;
    };
#endif
  }
}

#endif
