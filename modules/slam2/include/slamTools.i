
/* $Id$ */

/** tcl tools for module slam.
 *
 * \file slamTools.i
 * \ingroup slam
 */   


%{
#include <sstream>
#include <list>

#include "boost/date_time/posix_time/posix_time_types.hpp"

#include "jafarConfig.h"

#ifdef HAVE_BOOST_SANDBOX
#ifdef HAVE_LAPACK
#include "boost/numeric/bindings/lapack/syev.hpp"
#include "boost/numeric/bindings/traits/ublas_matrix.hpp"
#include "boost/numeric/bindings/traits/ublas_vector.hpp"
#endif
#endif

#include "jmath/jblas.hpp"
#include "jmath/ublasExtra.hpp"
#include "jmath/angle.hpp"

#include "geom/t3dEuler.hpp"
#include "geom/Segment.hpp"

#include "slam/eulerTools.hpp"
#include "slam/bearingOnlyFeature.hpp"
#include "slam/bearingOnlySlam.hpp"
#include "slam/map3d.hpp"
#include "slam/feature.hpp"
#include "slam/pointInvDepthFeature.hpp"
#include "slam/segmentFeature.hpp"
#include "slam/segmentInvDepthFeature.hpp"

  %}

%inline %{

  namespace jafar {namespace slam {

    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };

#ifdef HAVE_BOOST_SANDBOX
#ifdef HAVE_LAPACK
    template<class X, class XCov>
    std::string covToEllipsoid(X const& x, XCov const & xCov)
    {
      namespace lapack = boost::numeric::bindings::lapack;
      using namespace ublas;
      JFR_PRECOND((x.size() == 3 && xCov.size1() == 3) || (x.size() == 6 && xCov.size1() == 6),
		  "covToEllipsoid: this function works in 3D");
        

      jblas::vec lambda(3);
      ublas::matrix<double, ublas::column_major> A(ublas::project(xCov, range(0,3), range(0,3))); 

      int ierr = lapack::syev( 'V', 'U', A, lambda, lapack::optimal_workspace() );
//       JFR_POSTCOND(ierr==0,
// 		   "slam::covToEllipsoid: error in lapack::syev() function, ierr=" << ierr);
      if (!ierr==0) {
	JFR_WARNING("slam::covToEllipsoid: error in lapack::syev() function, ierr=" << ierr);
	lambda.clear();
	A.clear();
      }

      jblas::mat M(4,4);
      project(M, range(0,3), range(0,3)).assign(A);
      M(0,3) = x(0);
      M(1,3) = x(1);
      M(2,3) = x(2);
      M(3,0) = 0.0;
      M(3,1) = 0.0;
      M(3,2) = 0.0;
      M(3,3) = 1.0;

      for (std::size_t i = 0 ; i < 3 ; ++i) {
        if (lambda(i) < 0) {
          lambda(i) = 0;
          JFR_WARNING("slam::covToEllipsoid: negative eigen value...");
        }
      } 

      std::stringstream ss;
      ss << "{ " << sqrt(lambda(0)) << " " << sqrt(lambda(1)) << " " << sqrt(lambda(2)) << " } { ";
      for(std::size_t i = 0 ; i < 4 ; ++i) {
	for(std::size_t j = 0 ; j < 4 ; ++j) {
	  ss << M(j,i) << " ";
	}
      }
      ss << "}";
      return ss.str();
    };

    std::string covToEllipsoid(jafar::geom::T3DEuler const& t3d) {
      using namespace ublas;
      return covToEllipsoid(project(t3d.getX(), range(0,3)), project(t3d.getXCov(), range(0,3), range(0,3)));
    };
#endif // HAVE_LAPACK
#endif // HAVE_BOOST_SANDBOX

    std::string weightToColor(double w) {
      JFR_PRECOND(w >= 0 && w <= 1.0,
		  "slam::weightToColor w=" << w);

      std::stringstream os;
      int r, g, b;
      r = 255;
      g = int(255*w);
      b = 0;
      os << r << " " << g << " " << b;
      return os.str();
    }

    std::string boSlamTrajectory(BearingOnlySlam const& slam)
    {
      JFR_TRACE_BEGIN;
      std::stringstream os;

      for (BearingOnlySlam::TrajectoryPoseRecordsType::const_iterator it = slam.trajectoryPoseRecords.begin() ; it != slam.trajectoryPoseRecords.end() ; ++it) {
	os << "{ ";
	os << it->first.frameIndex << " ";
	jblas::vec_range const& pose = slam.trajectory[it->second.trajIndex];
	os << "{ ";
	for (std::size_t i = 0 ; i < 6 ; ++i) {
	  os << pose(i) << " ";
	}
	os << "} ";
	os << "} ";
      }
      return os.str();
      JFR_TRACE_END("slamTools: boSlamTrajectory");
    }

    std::string boPtFeatureInitState(InitFeature& f, BearingOnlySlam& slam)
    {
      using namespace jblas;
      using namespace ublas;
      JFR_TRACE_BEGIN;
      std::stringstream s;
      jblas::vec3 x;
      jblas::sym_mat xCov(3,3);
      jblas::mat Jf(3,6);
      jblas::mat Jx(3,3);
      std::size_t refObsIndex = f.initObs.begin()->first;
      jblas::vec refFrame = slam.getTrajectoryPose(refObsIndex);
      for (InitFeature::InitStateType::const_iterator it = f.initState.begin();
	   it != f.initState.end() ; ++it)
	{
	  EulerTools::fromFrame(refFrame, (*it)->x, EulerTools::h_1, x);
	  EulerTools::fromFrameJac(refFrame, (*it)->x, EulerTools::h_1, Jf, Jx);
	  xCov.assign(prod(Jx, mat(prod((*it)->P,trans(Jx)))));

	  s << " { " 
	    << (**it).w << " "
	    << "{ " << x(0) << " "<< x(1) << " " << x(2) << " } "
	    << "{ " << xCov(0,0) << " " << xCov(0,1) << " " << xCov(0,2) << " " << " " << xCov(1,1) << " " << xCov(1,2) << " " << xCov(2,2) << " } "
	    << " } ";
	}
      return s.str();
      JFR_TRACE_END("slamTools: boPtFeatureInitState");
    };

    std::string boSegFeatureInitState(InitFeature const& f_, BearingOnlySlam& slam_)
    {
      JFR_TRACE_BEGIN;

      std::stringstream s;
      jblas::vec3 ext1;
      jblas::vec3 ext2;
      SegmentObservation const& refObs = dynamic_cast<SegmentObservation const&>(f_.getRefObservation());
      unsigned int refObsIndex = f_.getRefFrameIndex();

      jblas::vec refFrame(slam_.getTrajectoryPose(refObsIndex));

      jblas::vec lineExt1Sensor(6);
      jblas::vec lineExt2Sensor(6);
      jblas::vec lineExt1Ref(6);
      jblas::vec lineExt2Ref(6);

      double s1,s2;
      double tmp1, tmp2;
      ImageEuclideanPluckerFeatureObserveModel const& m = 
	dynamic_cast<ImageEuclideanPluckerFeatureObserveModel const&>
	(slam_.getBoFeatureObserveModel(refObs.sensorId));

      for (InitFeature::InitStateType::const_iterator it = f_.initState.begin() ; it != f_.initState.end() ; ++it) {
	jblas::vec3 n(ublas::project((*it)->x, ublas::range(0,3)));
	jblas::vec3 u(ublas::project((*it)->x, ublas::range(3,6)));
	jblas::vec3 po;
	jmath::ublasExtra::crossProd(u,n,po);

	m.segmentExtremitiesPluckerLines(refObs, lineExt1Sensor, lineExt2Sensor);

	EulerTools::lineFromFrame(m.robotToSensor().getX(), lineExt1Sensor, lineExt1Ref); // robotToSensor must not have changed !
	EulerTools::lineFromFrame(m.robotToSensor().getX(), lineExt2Sensor, lineExt2Ref); // robotToSensor must not have changed !

	boost::tie(s1,tmp1,tmp2) = ImageEuclideanPluckerFeatureObserveModel::pluckerLinesDistance((*it)->x, lineExt1Ref);
	boost::tie(s2,tmp1,tmp2) = ImageEuclideanPluckerFeatureObserveModel::pluckerLinesDistance((*it)->x, lineExt2Ref);

	EulerTools::fromFrame(refFrame, po + s1*u, EulerTools::h_1, ext1);
	EulerTools::fromFrame(refFrame, po + s2*u, EulerTools::h_1, ext2);

	s << " { " 
	  << (**it).w << " "
	  << "{ " << ext1(0) << " " << ext1(1) << " " << ext1(2) << " } "
	  << "{ " << ext2(0) << " " << ext2(1) << " " << ext2(2) << " } "
	  << " } ";
      }
      return s.str();
      
      JFR_TRACE_END("boSegFeatureInitState()");
    };

//     std::string boSegFeatureState(BaseBearingOnlyFeature& f_)
//     {
//       double displayLength = 20;
//       std::stringstream s;

//       jblas::vec3 n = project(f_.getX(), ublas::range(0,3));
//       jblas::vec3 u = project(f_.getX(), ublas::range(3,6));
//       jblas::vec3 po;
//       jmath::ublasExtra::crossProd(u, n, po);
//       jblas::vec3 ext1 = po + displayLength*u;
//       jblas::vec3 ext2 = po - displayLength*u;
	  
//       s << "{ " << ext1(0) << " " << ext1(1) << " " << ext1(2) << " } "
// 	<< "{ " << ext2(0) << " " << ext2(1) << " " << ext2(2) << " } ";

//       return s.str();
//     };

    // 0: nothing special
    // 1: newly observed feature
    // 2: uninitialised feature
    // 3: feature matched during loop closing
    // 4: feature at infinity
    int landmarkCategory(jafar::slam::BaseFeature const& f) {
      if (f.frameIndexes.size()==1)
	return 1;
      
      if (f.frameIndexes.size() >= 2) {
	unsigned int indexN = f.frameIndexes.back();
	
	BaseFeature::FrameIndexesType::const_iterator it = f.frameIndexes.end();
	--it;--it;
	unsigned int indexN_1 = *it;
	if ( indexN_1 < indexN - 1 )
	  return 3;
      }

      if (f.atInfinity)
	return 4;

      return 0;
    };

    int landmarkCategory(jafar::slam::InitFeature const& f) {
      if (f.frameIndexes.size()==1)
	return 1;
      else
	return 2;
    };

    std::size_t sizeInitStateType(InitFeature::InitStateType is) {
      return is.size();
    }

    jafar::slam::InitStateMember const& getInitStateType(InitFeature::InitStateType const& is, 
							 std::size_t index) 
    {
      std::size_t i = 0;
      InitFeature::InitStateType::const_iterator it = is.begin();
      while (i < index) {
	JFR_PRECOND(it != is.end(),
		    "slam::getInitStateType");
	it++;
	i++;
      }
      return **it;
    }

    std::string gnuplotBoPtInitState(BearingOnlySlam const& slam, unsigned int id) {
      using namespace ublas;

      InitFeature const& f = slam.getInitFeature(id);

      std::size_t refObsIndex = f.initObs.begin()->first;
      jblas::vec refPose = project(slam.getTrajectoryPose(refObsIndex), range(0,3));

      std::stringstream gpCmd;
      std::stringstream gpCmdSum;

      gpCmd << "plot ";

      for (InitFeature::InitStateType::const_iterator it = f.initState.begin() ; it != f.initState.end() ; ++it) {
	InitStateMember const& ism = **it;
	double m = ism.initParams[0].x(0);
	double sig = sqrt(ism.initParams[0].P(0,0));
	gpCmd << ism.w / (sig*2*M_PI) << "*exp(" << -1/(2*sig*sig) << "*(x-" << m << ")**2) lt 1 lw 2, ";
	gpCmdSum << ism.w / (sig*2*M_PI) << "*exp(" << -1/(2*sig*sig) << "*(x-" << m << ")**2)+ ";
      }

      gpCmd << gpCmdSum.str() << "0 lt 2 lw 4";

      return gpCmd.str();
    }

#ifdef HAVE_TTL
    std::string demFromMap(jafar::slam::Map const& m) 
    {
      using namespace hed;

      std::stringstream os;

      list<Edge*> const& leadingEdges = m.triangulation.getLeadingEdges();
      list<Edge*>::const_iterator it;
      for (it = leadingEdges.begin(); it != leadingEdges.end(); ++it) {
	Edge* edge = *it;
	os << "{ ";
	for (int i = 0; i < 3; ++i) {
	  Node& node = *(edge->getSourceNode());	
	  os << node.x() << " " << node.y() << " " << node.z() << " ";
	  edge = edge->getNextEdgeInFace();
	}
	os << "} ";
      }
      return os.str();
    };
#endif // HAVE_TTL

//     // FIXME maybe some bug in swig, should not need it
//     jafar::slam::Observation* segObsToObs(jafar::slam::SegmentObservation* obs) {
//       return obs;
//     };

//     // FIXME swig
//     jafar::kernel::DataLoggable* toDataLoggable(jafar::slam::SlamEkf* s) {
//       return s;
//     }
//     // FIXME swig
//     jafar::kernel::DataLoggable* toDataLoggable(jafar::slam::BearingOnlySlam* s) {
//       return s;
//     }

    jafar::slam::SegmentFeature* dynamic_cast_SegmentFeature(jafar::slam::BaseFeature* f) {
      return dynamic_cast<jafar::slam::SegmentFeature*>(f);
    };

    jafar::slam::SegmentIDFeature* dynamic_cast_SegmentIDFeature(jafar::slam::BaseFeature* f) {
      return dynamic_cast<jafar::slam::SegmentIDFeature*>(f);
    };

    jafar::slam::ImageEuclideanPluckerFeatureObserveModel* dynamic_cast_ImageEuclideanPluckerFeatureObserveModel(jafar::slam::BearingOnlyFeatureObserveModel* om) {
      return dynamic_cast<jafar::slam::ImageEuclideanPluckerFeatureObserveModel*>(om);
    };
	
	jafar::slam::ImagePluckerFeatureObserveModel* dynamic_cast_ImagePluckerFeatureObserveModel(jafar::slam::FeatureObserveModel* om) {
		return dynamic_cast<jafar::slam::ImagePluckerFeatureObserveModel*>(om);
	};

    jafar::slam::RhoThetaImagePluckerFeatureObserveModel* dynamic_cast_RhoThetaImagePluckerFeatureObserveModel(jafar::slam::FeatureObserveModel* om) {
        return dynamic_cast<jafar::slam::RhoThetaImagePluckerFeatureObserveModel*>(om);
    };

    jafar::slam::StereoImagePluckerFeatureObserveModel* dynamic_cast_StereoImagePluckerFeatureObserveModel(jafar::slam::FeatureObserveModel* om) {
        return dynamic_cast<jafar::slam::StereoImagePluckerFeatureObserveModel*>(om);
    };

    jafar::slam::ImageSegInvDepthFeatureObserveModel* dynamic_cast_ImageSegInvDepthFeatureObserveModel(jafar::slam::FeatureObserveModel* om) {
        return dynamic_cast<jafar::slam::ImageSegInvDepthFeatureObserveModel*>(om);
    };

    jafar::slam::ImageSegInvDepthInovFeatureObserveModel* dynamic_cast_ImageSegInvDepthInovFeatureObserveModel(jafar::slam::FeatureObserveModel* om) {
        return dynamic_cast<jafar::slam::ImageSegInvDepthInovFeatureObserveModel*>(om);
    };

    void pluckerAngle(jafar::slam::BearingOnlySlam const& slam, 
		      unsigned int id1, 
		      unsigned int id2)
    {
      using namespace jblas;
      using namespace ublas;
      BaseFeature const& f1 = slam.getFeature(id1);
      BaseFeature const& f2 = slam.getFeature(id2);
      vec u1 = project(f1.getX(),range(3,6));
      u1 /= norm_2(u1);
      vec u2 = project(f2.getX(),range(3,6));
      u2 /= norm_2(u2);
      double a = acos(inner_prod(u1,u2));

      JFR_DEBUG("a=" << jmath::radToDeg(a) << " deg");

      mat J(1,6);
      jmath::ublasExtra::inner_prodJac<3>(u1,u2,J);

      sym_mat u1Cov = project(f1.getP(), range(3,6),range(3,6));
      sym_mat u2Cov = project(f2.getP(), range(3,6),range(3,6));

      mat u12Cov = project(slam.getFilter().getP(), range(f1.filterIndex()+3, f1.filterIndex()+6), range(f2.filterIndex()+3, f2.filterIndex()+6));

      mat J1 = project(J,range(0,1),range(0,3));
      mat J2 = project(J,range(0,1),range(3,6));

      double aCov = prod(J1, mat(prod(u1Cov, trans(J1))))(0,0) + prod(J2, mat(prod(u2Cov, trans(J2))))(0,0) 
	+ 2*prod(J1, mat(prod(u12Cov, trans(J2))))(0,0);

      JFR_DEBUG("sigma_a=" << jmath::radToDeg(sqrt(aCov)) << " deg");
      
    }

    /*
     * inverse depth points
     */

    void computeInverseDepthPointCoordinate(BaseFeature const& f, 
					    FeatureModel& model, 
					    jblas::vec& x, jblas::sym_mat& xCov) 
    {
      using namespace ublas;
      using namespace jblas;

      JFR_PRECOND(x.size() == 3 && xCov.size1() == 3,
		  "slam::computeInverseDepthPointCoordinate: invalid size");
      
      JFR_TRACE_BEGIN;
      PointInvDepthFeatureModel& m = dynamic_cast<PointInvDepthFeatureModel&>(model);

      m.compute3dPoint(f.getX(), x);
      m.compute3dPointJac(f.getX());

      xCov.assign( prod(m.J3dPoint, mat(prod(f.getP(), trans(m.J3dPoint))) ) );
      JFR_TRACE_END("slam::computeInverseDepthPointCoordinate");
    };
    
    /**
     * Make a list of segment 3D from the features of a slam filter
     */
    std::list<jafar::geom::Segment3D> featuresToSegment3D( const SlamEkf& _slamEkf )
    {
      std::list<jafar::geom::Segment3D> list;
      for( SlamEkf::FeaturesMapType::const_iterator it = _slamEkf.featuresMap.begin();
          it != _slamEkf.featuresMap.end(); ++it )
      {
        SegmentFeature* feature = dynamic_cast<SegmentFeature*>( it->second );
        if( feature )
        {
          geom::Segment3D seg( new geom::Segment3D::TwoPointsDriver( feature->getExt1(), feature->getExt2() ) );
          seg.setId( feature->id());
          list.push_back( seg );
        }
      }
      return list;
    }

  }} // namespace
  
  %}

