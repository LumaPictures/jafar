/* $Id$ */

#ifndef _JAFAR_SLAMSEG_IMAGE_SEGMENT_MANAGER_HPP_
#define _JAFAR_SLAMSEG_IMAGE_SEGMENT_MANAGER_HPP_

#include <jafarConfig.h>
#include <slam/segmentFeature.hpp>

namespace jafar {
  namespace datareader {
    class ImageReader;
  }
  namespace dseg {
    class DirectSegmentsDetectorBase;
    class DirectSegmentsTracker;
    class Predictor;
    class SegmentsSet;
  }
  namespace slam {
    class BaseSlam;
  }
  namespace slamseg {
  
    class MonoImageSegmentManager {
      public:
        MonoImageSegmentManager( slam::BaseSlam& _slam, jafar::datareader::ImageReader* _imageReader, const dseg::Predictor* _predictor = 0, 
             slam::Observation::ObservationType type_ = slam::Observation::SEGMENT_IMAGE );
        ~MonoImageSegmentManager();
        void processFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int stereoSensorId_ = 0);
        dseg::DirectSegmentsDetectorBase* detector();
        const dseg::SegmentsSet& currentLines() const;
      private:
        struct Private;
        Private* const d;
    };
  }
}

#endif
