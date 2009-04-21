require 'jafar/kernel'
require 'jafar/slam/slam'
require 'jafar/filter.rb'
Jafar.register_module Jafar::Slam


module Jafar
  module Slam
    module Display
      def Display.showStereoPtPred( imageview, slamekf, obsmodel, robotId = 0 )
        robot = slamekf.robot( robotId );
        slamekf.beginBrowseFeatures()
        while( slamekf.hasNextFeature() )
          feature = slamekf.nextFeature();
          if( feature.typeObs == Slam::Observation::POINT_STEREOIMAGE )
            pred = obsmodel.predictObservation( robot.getX, feature.getX)
            predArr = Jmath::vecToArray( pred )
            s = Qdisplay::Shape.new( Qdisplay::Shape::ShapeCross, predArr[0], predArr[1], 3, 3 )
            s.setColor( 255, 0, 0 )
            s.setLabel( feature.id.to_s )
            imageview.addShape( s )
          end
        end
      end
    end
    def Slam.shortTerm(startindex, endindex, reductionRate = 3, dodisplayTraj = true, dodisplayPoints = false, datareader = Datareader::DataReader.new)
      nbZonesU = 4
      nbZonesV = 4
      nbDesiredFeatures = 2
      loopClosingDistanceMax = 3.0
      loopClosingIndexDistanceMin = 80
      nbDesiredFeaturesTotal = 2 * nbDesiredFeatures  * nbZonesU * nbZonesV
      preScale = 2
      uvStdDev = 1.0
      dispStdDev = 1.0


      stereoReader = datareader.getStereoReader(0)
      
      # create and initialize the slam object
      slamEkf = Slam::SlamEkf.new(350, 6, 6)
      slamEkf.getFilter.setCovUpdateType(Filter::BaseKalmanFilter::STANDARD)
      slamEkf.getFilter.setupConsistencyCheck(Filter::BaseKalmanFilter::CONSISTENCY_EXCEPTION, 10.0)
      
      # create a landmark model
      ptModel = Slam::PointFeatureModel.new
      # create an associated observe model
      sb = Camera::StereoBench.new
      sb.load(stereoReader.calibrationFileName())
      sb.applyScale(1.0 / reductionRate)
      
#                           set ptObsModel [slam::new_StereoImagePointFeatureObserveModel $ptModel $stereoBench]
# 
# #                   image::delete_StereoBench $stereoBench
# 
#                     set obsNoise [jmath::new_vec 3]
#                     set uvCov [expr pow($uvStdDev,2)]
#                     set dispCov [expr pow($dispStdDev,2)]
#                     jmath::setValue $obsNoise "($uvCov, $uvCov, $dispCov)"
#                     $ptObsModel setUncorrelatedR $obsNoise
#                     $slamEkf setSensor $ptObsModel

      ptObsModel = Slam::StereoImagePointFeatureObserveModel.new( ptModel, sb )
      
#       obsNoise = Jmath::Vec.new(3)
#       uvCov = uvStdDev ** 2;
#       dispCov = dispStdDev ** 2;
#       Jmath::setValue(obsNoise, "(#{uvCov}, #{uvCov}, #{dispCov})")
#       ptObsModel.setUncorrelatedR(obsNoise)
      
      
#       ptObsModel.setNoiseValues( 0.1, Jmath::degToRad(2), Jmath::degToRad(2))
      # add the defined sensor to Slam
      slamEkf.setSensor(ptObsModel)
      # set the robot to sensor t3d
#       robotToSensor = Jmath::Vec.new(6)
#       Jmath::setValue(robotToSensor, "(0,0,1.0,#{Jmath::degToRad(90)},0,0)")
#       slamEkf.setRobotToSensor(robotToSensor)

      # set the Rcam(cam) to Rcam(rob)
      rCamToSensor = Geom::T3DEuler.new
      rCamToSensorX = Jmath::Vec.new(6)
      Jmath::setValue(rCamToSensorX, "(0,0,0,#{-Math::PI / 2},0,#{-Math::PI / 2})")
      rCamToSensor.set(rCamToSensorX)
      # set the robot to sensor t3d
      
      leftposInit = stereoReader.left().loadPosition(startindex)
#       sensorToRobot = Geom::T3DEuler.new
#       sensorToRobotX = Jmath::Vec.new(6)
#       Jmath::setValue(sensorToRobotX, "(0.079,0.040495,1.016151,1.570796,0.523524,0.0)")
#       Jmath::setValue(sensorToRobotX, "(0.290486, 0.165000, 1.012333,0.000000,0.611264, 0.000000)")
#       sensorToRobot.set(sensorToRobotX)
      
      robotToSensor = Geom::T3DEuler.new
      Geom::T3D.compose( leftposInit.sensorToMain, rCamToSensor, robotToSensor )
#       Geom::T3D.compose( sensorToRobot, rCamToSensor, robotToSensor )
      slamEkf.setRobotToSensor( robotToSensor.getX, 0)
      
      puts "robotToSensor = #{Geom::print( robotToSensor )}"
      
#       slamEkf.setRobotToSensor( leftposInit.sensorToMain.getX, 0)



      # create a prediction model
      odoPredictModel = Slam::Odo3dPredictModel.new
#       odoPredictModel.odoNoiseModel.set(0.001, 0, 0.001, 0.001 )
      odoPredictModel.odoNoiseModel.set(0.05 ** 2, 0, 0.05 ** 2 , 0 )
      baseRobot = Slam::BaseRobot.new( 0, odoPredictModel)
      slamEkf.addRobot( baseRobot )

      #
      # run
      #

      # predict with command u [v,w]
      u = Jmath::Vec.new(2)
      slamEkf.predict(0, u)

      # map manager
      lmm = LocalMapManager.new(slamEkf, nbDesiredFeaturesTotal, 3 )
      slamEkf.setMapManager(lmm)
      
      initImage = stereoReader.left().loadImage(startindex)
      hpmEngineLoopClosing = Hpm::Engine.new(preScale*initImage.width, preScale*initImage.height, reductionRate)
      hpmTrackingEngine = Hpm::StereoTrackingEngine.new( preScale*initImage.width, preScale*initImage.height, reductionRate)
      
      imagePointManager = Slam::StereoImagePointManager.new( slamEkf, hpmEngineLoopClosing, hpmTrackingEngine, stereoReader) # , false

      
      imagePointManager.setupZones( preScale*initImage.width, preScale*initImage.height, reductionRate, nbZonesU, nbZonesV, nbDesiredFeatures )
      
      require 'jafar/preprocessing'
      leftpreproc = Jafar::Preprocessing::Preprocessing.new
      leftpreproc.load(stereoReader.left().calibrationFileName())
      leftProc = Image::ImagePreprocessor.new(initImage.width, initImage.height, initImage.depth, initImage.colorSpace)
#       leftProc.appendNode(leftpreproc)
      stereoReader.left().setImagePreprocessor( leftProc )


      rightpreproc = Jafar::Preprocessing::Preprocessing.new
      rightpreproc.load(stereoReader.right().calibrationFileName())
      rightProc = Image::ImagePreprocessor.new(initImage.width, initImage.height, initImage.depth, initImage.colorSpace)
#       rightProc.appendNode(rightpreproc)
      stereoReader.right().setImagePreprocessor( rightProc )

          
      imagePointManager.setupLoopClosing(loopClosingDistanceMax, loopClosingIndexDistanceMin)

#       if(startindex != 0)
        leftpos = stereoReader.left().loadPosition(startindex)
        mainToOrigin1 = leftpos.mainToOrigin
        slamEkf.setRobotPose( mainToOrigin1.getX )
#       end
      
      
      mainToOrigin1 = nil
      imagePointManager.initFrame(startindex)
      
      if(dodisplayPoints)
        load "jafar/qdisplay.rb"
        viewer = Jafar::Qdisplay::Viewer.new
      end
      if(dodisplayTraj)
        load "jafar/qdisplay.rb"
        viewerTraj = Jafar::Qdisplay::Viewer.new
        robotSlamTrajectory = Qdisplay::RobotTrajectory.new(10.0)
        robotSlamTrajectory.setColor(255,0,0)
        viewerTraj.addPolyLine(robotSlamTrajectory)
        robotOdoTrajectory = Qdisplay::RobotTrajectory.new(10.0)
        robotOdoTrajectory.setColor(0,255,0)
        viewerTraj.addPolyLine(robotOdoTrajectory)
        robotPredictionTrajectory = Qdisplay::RobotTrajectory.new(10.0)
        robotPredictionTrajectory.setColor(0,0,255)
        viewerTraj.addPolyLine(robotPredictionTrajectory)
        viewerTrajXZ = Jafar::Qdisplay::Viewer.new
        robotSlamTrajectoryXZ = Qdisplay::RobotTrajectory.new(10.0, 0, 2)
        robotSlamTrajectoryXZ.setColor(255,0,0)
        viewerTrajXZ.addPolyLine(robotSlamTrajectoryXZ)
        robotOdoTrajectoryXZ = Qdisplay::RobotTrajectory.new(10.0, 0,2)
        robotOdoTrajectoryXZ.setColor(0,255,0)
        viewerTrajXZ.addPolyLine(robotOdoTrajectoryXZ)
      end

      for i in (startindex+1)..endindex
        # Compute prediction
        leftpos2 = stereoReader.left().loadPosition(i)
        
        mainToOrigin2 = leftpos2.mainToOrigin
        
        # Estimate robot movement
        invSensorMovement = Geom::T3DEuler.new
        robotMovement = Geom::T3DEuler.new
        if(mainToOrigin1.nil?)
          identity = Jmath::Vec.new(6)
          Jmath::setValue(identity, "(0,0,0,0,0,0)")
          robotMovement.set(identity)
        else
          invMainToOrigin1 = Geom::T3DEuler.new
          Geom::T3D::inv( mainToOrigin1, invMainToOrigin1)
          Geom::T3D.compose( invMainToOrigin1, mainToOrigin2,  robotMovement)
        end
        mainToOrigin1 = mainToOrigin2
        leftpos2.mainToOrigin
        
        # prediction
        u = Jmath::Vec.new(2)
        robotMovementOdo = Jmath::vecToArray(robotMovement.getX)
        Jmath::setValue(u, "(#{Math.sqrt(robotMovementOdo[0]**2 + robotMovementOdo[1]**2)},#{robotMovementOdo[3]})")
        slamEkf.predict(0, u)
        puts "<=========> Robot movement:#{Geom::print(robotMovement)} #{robotMovementOdo.join(' ')} <=========>"
        
        predictionPose = Geom::T3DEuler.new
        slamEkf.getRobotPose(predictionPose)
        # Estimation
#         if(robotMovementOdo[0] > 0.01)
        if( i != 680 )
          imagePointManager.processFrame(i)
        end
        if(dodisplayPoints)
#           viewer = Jafar::Qdisplay::Viewer.new
          # Display tracking
          previousItem = Jafar::Qdisplay::ImageView.new( hpmTrackingEngine.getPreviousImage() )
          viewer.setImageView(previousItem,0,0)
          Hpm::Display.showPoints(previousItem, imagePointManager.getPreviousPoints, true)
          currentItem = Jafar::Qdisplay::ImageView.new( hpmTrackingEngine.getCurrentImage() )
          viewer.setImageView(currentItem,1,0)
          Hpm::Display.showPoints(currentItem, imagePointManager.getCurrentPoints, true)
          trackingMatches = hpmTrackingEngine.getTrackingMatches
          Hpm::Display.showMatches(previousItem, imagePointManager.getPreviousPoints, currentItem, imagePointManager.getCurrentPoints, trackingMatches, false)
          # Display stereo
          leftItem = Jafar::Qdisplay::ImageView.new(hpmTrackingEngine.getCurrentImage())
          viewer.setImageView(leftItem,0,1)
          Hpm::Display.showPoints(leftItem, imagePointManager.getCurrentPoints, true)
          rightItem = Jafar::Qdisplay::ImageView.new(hpmTrackingEngine.getStereoImage())
          viewer.setImageView(rightItem,1,1)
          Hpm::Display.showPoints(rightItem, imagePointManager.getStereoPoints, true)
          stereoMatches = hpmTrackingEngine.getStereoMatches
          Hpm::Display.showMatches(leftItem, imagePointManager.getCurrentPoints, rightItem, imagePointManager.getStereoPoints, stereoMatches)
        end
        # Display robot pose
        robotPose = Geom::T3DEuler.new
        slamEkf.getRobotPose(robotPose)
        puts "<=========> Robot position: #{Geom::print(robotPose)} <=========>"
        puts "<=========> Prediction: #{Geom::print(predictionPose)} <=========>"
        # Display trajectory
        if(dodisplayTraj)
          robotPredictionTrajectory.moveTo(predictionPose)
          robotOdoTrajectory.moveTo( leftpos2.mainToOrigin )
          robotSlamTrajectory.moveTo(robotPose)
          arr = Jmath.vecToArray(robotPose.getX)
#           shape = Qdisplay::Shape.new(Qdisplay::Shape::ShapeEllipse, arr[0] * robotSlamTrajectory.scale, arr[1] * robotSlamTrajectory.scale, 3,3)
#           shape.setFontSize(1)
#           shape.setLabel( i.to_s )
#           viewerTraj.addShape( shape )
          vecStdDev = Jmath.vecToArray(robotPose.getXStdDev)
          shape = Qdisplay::Shape.new(Qdisplay::Shape::ShapeEllipse, arr[0] * robotSlamTrajectory.scale, arr[1] * robotSlamTrajectory.scale, 2 * vecStdDev[0] * robotSlamTrajectory.scale, 2 * vecStdDev[1] * robotSlamTrajectory.scale)
          viewerTraj.addShape( shape )
# XZ
          robotOdoTrajectoryXZ.moveTo(leftpos2.mainToOrigin)
          robotSlamTrajectoryXZ.moveTo(robotPose)
        end
      end
    end
  end
end
