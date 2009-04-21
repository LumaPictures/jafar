/* $Id$ */

#if 1

#include <boost/test/auto_unit_test.hpp>

#include <camera/cameraPinhole.hpp>

#include <geom/t3dEuler.hpp>

#include <slam/slamEkf.hpp>
#include <slam/full3dPredictModel.hpp>
#include <slam/robot.hpp>

#include <jmath/ublasExtra.hpp>

#include <kernel/jafarMacro.hpp>
#include <kernel/jafarTestMacro.hpp>

namespace ublasExtra = jafar::jmath::ublasExtra;
using namespace jafar;
using namespace jafar::slam;

using jmath::ublasExtra::prettyFormat;

#if 0

class IdentifyFeatureObserveModel : public FeatureObserveModel {
  public:
    IdentifyFeatureObserveModel(FeatureModel& model) : FeatureObserveModel(model, 3)
    {
      JobsSensor.assign(jblas::identity_mat(3));
      JinvObs.assign(jblas::identity_mat(3));
      std::stringstream s;
      s << "[3,3]((0.01, 0.0,0.0),(0.0,0.01,0.0),(0.0,0.0,0.01))";
      jblas::mat m(6,6);
      JFR_IO_STREAM(s >> m, "reading matrix from string");
      R.assign(m);
    }
    virtual Observation::ObservationType typeObs() const
    {
      return Observation::POINT_CARTESIAN;
    }
  protected:
    virtual jblas::vec inverseObservationInSensorFrame(const jblas::vec& z_)
    {
      return z_;
    }
    virtual void inverseObservationInSensorFrameJac(const jblas::vec& z_)
    {
    }
    virtual jblas::vec const& predictObservationInSensorFrame(jblas::vec const& feature_)
    {
      z.assign(feature_);
      return z;
    }
    virtual void predictObservationInSensorFrameJac(jblas::vec const& featureInSensorFrame_)
    {
    }
};

Observation* createObservation( unsigned int _id, unsigned int _robotId, double _x, double _y, double _z, const geom::T3DEuler& _euler, double _noiseX, double _noiseY, double _noiseZ )
{
  Observation* obs = new Observation( Observation::POINT_STEREOIMAGE, _robotId );
  jblas::vec point(4);
  point(0) = _x; point(1) = _y; point(2) = _z; point(3) = 1.0;
  JFR_DEBUG(point << " " << _euler.getX());
  point = ublas::prod( _euler.getM(), point );
  JFR_DEBUG(point);
  point(0) += _noiseX;
  point(1) += _noiseY;
  point(2) += _noiseZ;
  obs->set( _id, ublas::project( point, ublas::range(0,3)) / point(3), 0);
  return obs;
}

JAFAR_AUTO_TEST_CASE_BEGIN( test_merge_map )
{
// BOOST_CHECK_MESSAGE(false,"test");
  SlamEkf* slamEkf1 = new SlamEkf( 30, 6, 6 );
  SlamEkf* slamEkf2 = new SlamEkf( 30, 6, 6 );
  
      
  // Init robots
  Full3dPredictModel predictModel;
  BaseRobot* robot1 = new BaseRobot( 0, predictModel );
  BaseRobot* robot2 = new BaseRobot( 1, predictModel );
  
  slamEkf1->addRobot( robot1 );
  slamEkf2->addRobot( robot2 );
  
  // Init sensors
  slamEkf1->setDeleteSensors( false);
  slamEkf2->setDeleteSensors( false);
  
  PointFeatureModel ptModel;
  IdentifyFeatureObserveModel ptObsModel( ptModel);
  
  slamEkf1->setSensor( &ptObsModel );
  slamEkf1->setRobotToSensor( jblas::zero_vec(6) );
  slamEkf2->setSensor( &ptObsModel );
  slamEkf2->setRobotToSensor( jblas::zero_vec(6) );
   
  // Initial robot1 position, real world = -5 0 0
  jblas::vec robot1X = jblas::zero_vec(6);
  robot1X(0) = -5;
  geom::T3DEuler robot1T3D;
  robot1T3D.set( robot1X, jblas::identity_mat(6,6) * 0.01  );
  
  slamEkf1->setRobotPose(jblas::zero_vec(6), 0); // In the map the robot is in (0,0,0)
  robot1->refPoseCov()->assign( jblas::identity_mat(6,6) * 0.01);
  
  // Initial robot2 position, real word = 8 0 0 -M_PI 0 0
  jblas::vec robot2X = jblas::zero_vec(6);
  robot2X(0) = 8;
  robot2X(3) = -M_PI;
  geom::T3DEuler robot2T3D;
  robot2T3D.set( robot2X, jblas::identity_mat(6,6) * 0.01 );
  slamEkf2->setRobotPose(jblas::zero_vec(6), 1); // In the map the robot is in (0,0,0)
  robot2->refPoseCov()->assign( jblas::identity_mat(6,6) * 0.01);
  
  // Initial Observation for first map
  {
    std::list<Observation*> newObs;
    geom::T3DEuler robot1T3DInv;
    geom::T3D::inv( robot1T3D, robot1T3DInv);
    newObs.push_back( createObservation( 0, 0, 1, 2, 0, robot1T3DInv, 0.001, -0.004, 0.000) );
    newObs.push_back( createObservation( 1, 0, 1, -2, 0, robot1T3DInv, -0.001, -0.009, -0.000) );
    newObs.push_back( createObservation( 2, 0, -4, 2, 0, robot1T3DInv, 0.005, 0.004, 0.000) );
    newObs.push_back( createObservation( 3, 0, -4, -2, 0, robot1T3DInv, 0.004, 0.009, 0.00) );
    std::list<Observation*> obs;
    slamEkf1->processObservations(0, obs, newObs, 0);
  }
  // Initial Observation for second map
  {
    std::list<Observation*> newObs;
    geom::T3DEuler robot2T3DInv;
    geom::T3D::inv( robot2T3D, robot2T3DInv);
    newObs.push_back( createObservation( 4, 1, 7, 2, 0, robot2T3DInv, 0.00008, 0.0004, 0.000) );
    newObs.push_back( createObservation( 5, 1, 7, -2, 0, robot2T3DInv, 0.003, -0.009, -0.000) );
    newObs.push_back( createObservation( 0, 1, 1, 2, 0, robot2T3DInv, 0.0014, -0.001, 0.000) );
    newObs.push_back( createObservation( 1, 1, 1, -2, 0, robot2T3DInv, -0.002, 0.006, 0.00) );
    std::list<Observation*> obs;
    slamEkf2->processObservations(0, obs, newObs, 1);
  }
  
  geom::T3DEuler mapTransfo;
  geom::T3DEuler inv;
  geom::T3D::inv( robot1T3D, inv);
  geom::T3D::compose( inv, robot2T3D, mapTransfo);
  
//   JFR_DEBUG( "robot1T3D = " << robot1T3D );
//   JFR_DEBUG( "robot2T3D = " << robot2T3D );
//   JFR_DEBUG( "mapTransfo = " << mapTransfo );
  { // Test when the two robots are expressed in the same frame (which is the case when the covariance of the map transformation is null)
    SlamEkf* slamEkf = SlamEkf::mergeMap( slamEkf1, slamEkf2, mapTransfo.getX(), jblas::zero_mat(6,6) );
    jblas::mat a = slamEkf->robot(0)->getP();
    jblas::mat b = slamEkf->robot(1)->getP();
    JFR_CHECK(jmath::ublasExtra::trace( ublas::prod( ((a) - (b)), ublas::trans((a)-(b)) ) ) < 1e-5 );
  }
  {
    SlamEkf* slamEkf = SlamEkf::mergeMap( slamEkf1, slamEkf2, mapTransfo.getX(), mapTransfo.getXCov() );
#if 0  
    // TODO test that vectors are kept at the same value    
    JFR_DEBUG( slamEkf->getFilter().getX() );
    JFR_DEBUG( slamEkf->getFilter().getP() );
    JFR_DEBUG("Robots state");
    JFR_DEBUG( "Before" );
    JFR_DEBUG( "1 " << robot1->getX() << " " << robot1->getP() << ublasExtra::det( robot1->getP()) );
    JFR_DEBUG( "2 " << robot2->getX() << " " << robot2->getP() << " " << jmath::ublasExtra::det( robot2->getP() ) );
    for(int i = 0; i < 2; ++i)
    {
      BaseRobot* robot = slamEkf->robot(i);
      JFR_DEBUG( i << " " << robot->getX() << " " << prettyFormat(robot->getP())  << " " << jmath::ublasExtra::det( robot->getP() ) );
    }
    
    JFR_DEBUG("Feature state");
    JFR_DEBUG("First filter");
    for( SlamEkf::FeaturesMapType::iterator it = slamEkf1->featuresMap.begin();
        it != slamEkf1->featuresMap.end(); ++it)
    {
      BaseFeature* feature = it->second;
      JFR_DEBUG( it->first << " " << feature->getX() << " " << feature->getP() << " " << jmath::ublasExtra::det( feature->getP() ) );
    }
    JFR_DEBUG("Second filter");
    JFR_DEBUG( slamEkf2->getFilter().getX() );
    for( SlamEkf::FeaturesMapType::iterator it = slamEkf2->featuresMap.begin();
        it != slamEkf2->featuresMap.end(); ++it)
    {
      BaseFeature* feature = it->second;
      JFR_DEBUG( it->first << " " << feature->getX() << " " << feature->getP() << " " << jmath::ublasExtra::det( feature->getP() ) << " " << feature->filterIndex() );
    }
    JFR_DEBUG("After");
    for( SlamEkf::FeaturesMapType::iterator it = slamEkf->featuresMap.begin();
        it != slamEkf->featuresMap.end(); ++it)
    {
      BaseFeature* feature = it->second;
      JFR_DEBUG( it->first << " " << feature->getX() << " " << feature->getP() << " " << jmath::ublasExtra::det( feature->getP() ) );
    }
    
    BaseRobot* robot1Fusion = slamEkf->robot(0);
#endif
  }
}
JAFAR_AUTO_TEST_CASE_END()

#endif
               
#endif
