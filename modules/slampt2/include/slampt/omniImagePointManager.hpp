/* $Id$ */

#ifndef SLAM_OMNI_IMAGE_POINT_MANAGER_HPP
#define SLAM_OMNI_IMAGE_POINT_MANAGER_HPP

#include "boost/tuple/tuple.hpp"

#include "kernel/dataLog.hpp"

#include "jmath/jblas.hpp"

#include "image/Image.hpp"

#ifndef THALES_TAROT_DISABLE
#include "datareader/ImageReader.hpp"
#endif

#include "geom/t3dEuler.hpp"

#include "camera/cameraBarreto.hpp"
#include "camera/mask.hpp"

#ifndef THALES_TAROT_DISABLE
#include "locpano/engine.hpp"
#endif

#include "hpm/export.hpp"
#include "hpm/engine.hpp"

#include "locpano/engine.hpp"

#include <slam/full3dPredictModel.hpp>
#include "slampt/managerTools.hpp"
#include <slam/bearingOnlySlam.hpp>

namespace jafar {
  namespace slampt {

#ifndef SWIG

    /** Frame Database for omni images. Tentative loop closing
     * frame is obtained considering the image hitogramms (module locpano).  
     *
     * \ingroup slampt
     */
    class OmniImageFrameDataBase : public FrameDataBase {

    public:

      int databaseNbImagesMax;
      double databaseImageDistanceMin;
      int loopClosingIndexDistanceMin;
      double loopClosingImageDistanceMax;

      jafar::locpano::Engine omniImageDB;

      DBFrameList::iterator tentativeLoopClosing();

    }; // class OmniImageFrameDataBase

#endif // SWIG

    /** Class for omni slam
     * \ingroup slampt
     */
    class OmniImagePointManager :
      public jafar::kernel::DataLoggable,
      public slam::BoSlamEventAdapter
    {

    private :

      unsigned int idFactory;

      slam::BearingOnlySlam& slam;

      /// For loop closing
      jafar::hpm::Engine& hpmEngineLoopClosing;

      OmniImageFrameDataBase frameDataBase;
      jafar::datareader::ImageReader dataBaseImageReader;

      /* for logging */
      bool databaseImageAdded;
      double databaseImageDistance;
      bool loopClosingDetected;
      unsigned int loopClosingFrameIndex;
      double loopClosingImageDistance;
      unsigned int loopClosingNbUsedFeatures;
      unsigned int loopClosingElapsedTime;
      /**/

      jafar::hpm::vecHarrisPoints loopClosingPoints;

      typedef std::vector<QualityIndexPoint> QualityIndexPoints;

      /// points from last frame which are candidate
      std::vector<QualityIndexPoints> pointsInZone;
      std::vector<unsigned int> nbFeaturesInZone;

      /// features observed in current frame
      typedef std::list<std::size_t> FeaturesList;
      FeaturesList knownFeatures;
      FeaturesList newFeatures;

      boost::tuple<bool, unsigned int> isPointInZone(double u, double v);

      static int computeIndex(double x_, int nbInt_, double xMax_);

      unsigned int getNewFeatureId() { return ++idFactory;}

      void addNewFeature(std::size_t index_, hpm::HarrisPoint& point_);

      void addKnownFeature(std::size_t index_, jafar::hpm::HarrisPoint const& point_);

      bool loopClosingDetection(image::Image const& image);
      void loopClosing(jafar::hpm::vecHarrisPoints& framePoints_, jafar::image::Image const& image);

      void featureSelection(jafar::hpm::vecHarrisPoints& points_);

      void addFrameToDataBase(jafar::hpm::vecHarrisPoints& points_, jafar::image::Image const& image);

      void slamProcessObservations() const;

      /// current frame number
      unsigned int currentFrameIndex;

      jafar::datareader::ImageReader imageReader;

      /// For tracking points
      jafar::hpm::TrackingEngine& hpmTrackingEngine;

      jafar::hpm::vecHarrisPoints vecPointsPrev;
      jafar::hpm::vecHarrisPoints vecPointsCur;

      bool fillObservation(std::size_t index_, slam::Observation& obs) const;

      // BoSlamEventAdapter events
      void removeLandmark(slam::BaseFeature const& landmark);
      void removeTentativeLandmark(slam::InitFeature const& landmark);
      void removeLandmark(unsigned int id);

    public :

      // cells definition
      jafar::camera::CameraParabolicBarreto camera;
      jafar::camera::PanoMask omniMask;
      unsigned int imageMargin;
      std::size_t nbZonesInRadius;
      std::size_t nbZonesInPeriphery;
      unsigned int nbDesiredFeaturesPerZone;

      OmniImagePointManager(slam::BearingOnlySlam& slam_,
                            jafar::camera::CameraParabolicBarreto& camera_,
			    jafar::hpm::Engine& hpmEngineLoopClosing_,
			    jafar::hpm::TrackingEngine& hpmTrackingEngine_);

      ~OmniImagePointManager();
      
      void setupManager(unsigned int imageMargin_,
			std::size_t nbZonesInRadius_,
			std::size_t nbZonesInPeriphery_,
			unsigned int nbDesiredFeaturesPerZone_);

      void setupDataBase(int databaseNbImagesMax,
			 double databaseImageDistanceMin,
			 int loopClosingIndexDistanceMin,
			 double loopClosingImageDistanceMax,
                         std::string const& imageDir);

      bool isLoopClosingDetected() const {return loopClosingDetected;}

      unsigned int getLoopClosingFrameIndex() const {return loopClosingFrameIndex;}

      jafar::hpm::vecHarrisPoints const& getLoopClosingPoints() const {return loopClosingPoints;}

      //       FeaturesList const& getKnownFeatures() const {return knownFeatures;}
      //       FeaturesList const& getNewFeatures() const {return newFeatures;}

      void writeLogHeader(jafar::kernel::DataLogger& dataLogger) const;
      void writeLogData(jafar::kernel::DataLogger& dataLogger) const;      

      jafar::datareader::ImageReader& getImageReader() {return imageReader;}

      jafar::hpm::vecHarrisPoints const& getCurrentPoints() const {return vecPointsCur;}
      jafar::hpm::vecHarrisPoints const& getPreviousPoints() const {return vecPointsPrev;}

      void initFrame(unsigned int frameIndex_);
      void initFrame(jafar::image::Image& image, int frameIndex_=-1);
      void initFrame(boost::posix_time::time_duration const& curTime, jafar::image::Image& image);

      void processFrame(unsigned int frameIndex_);
      void processFrame(jafar::image::Image& image, int frameIndex_=-1);

//       void nextFrame(jblas::vec const& dx, jblas::sym_mat const& dxCov);

    }; // class OmniImagePointManager

//     class BoOmniSlamVmeOdo {

//     private:

//       std::string logDir;
//       kernel::DalaLogReader dalaLogReader;

//       SensorData sensorData;

//       double vmeNoiseFactor;
//       Full3dPredictModel vmePredictModel;

//       Odo3dPredictModel odoPredictModel;

//       BearingOnlySlam& slam;

//       OmniImagePointManager& omniImagePointManager;

//       unsigned int frameNumber;

//     public:

//       BoOmniSlamVmeOdo(std::string const& logDir_,
// 		       BearingOnlySlam& slam_,
// 		       OmniImagePointManager& omniImagePointManager_);

//       Full3dPredictModel& getVmePredictModel() {return vmePredictModel;}
//       Odo3dPredictModel& getOdoPredictModel() {return odoPredictModel;}

//       void setup(double vmeNoiseFactor_) {
// 	vmeNoiseFactor = vmeNoiseFactor_;
//       }
      
//       void init(int frameOffset);

//       void nextFrame();

//     };

  } // namespace slam
} // namespace jafar

#endif // SLAM_OMNI_IMAGE_POINT_MANAGER_HPP
