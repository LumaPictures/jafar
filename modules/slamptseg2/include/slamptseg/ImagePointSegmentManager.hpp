/* $Id$ */

#ifndef _SLAMPTSEG_POINTSEGMENTMANAGER_HPP_
#define _SLAMPTSEG_POINTSEGMENTMANAGER_HPP_

namespace jafar {
  namespace slam {
    class StereoImagePointManager;
  }
  namespace slamseg {
    class ImageSegmentManager;
  }
  namespace slamptseg {
    class PointSegmentManager {
      public:
        PointSegmentManager( slam::StereoImagePointManager* _stereoImagePointManager, slamseg::ImageSegmentManager* _imageSegmentManager);
        ~PointSegmentManager();
        void initFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);
        void processFrame(unsigned int frameIndex_,unsigned int robotId_ = 0, unsigned int sensorId_ = 0);
      private:
        slam::StereoImagePointManager* _stereoImagePointManager;
        slamseg::ImageSegmentManager* _imageSegmentManager;
    };
  }
}


#endif
