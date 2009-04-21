/* $Id$ */

#include "slam/slamEkf.hpp"
#include "dseg/SegmentHypothesis.hpp"
#include "dseg/LineFitterKalman2.hpp"

namespace jafar {
  namespace slamseg {

template<class _TModel_>
SlamPredictor<_TModel_>::SlamPredictor( slam::SlamEkf& _slam, _TModel_& _model, unsigned int _robotId ) : m_slam( _slam ), m_obsModel(_model), m_robotId( _robotId )
{
}

template<class _TModel_>
SlamPredictor<_TModel_>::~SlamPredictor()
{
}

template<class _TModel_>
dseg::SegmentHypothesis* SlamPredictor<_TModel_>::predictionFor( const dseg::SegmentHypothesis* _segmentHypothesis ) const
{
  if( m_slam.hasFeature( _segmentHypothesis->id() ) )
  {
    slam::BaseRobot* baseRobot = m_slam.robot( m_robotId );
    slam::BaseFeature& bf = m_slam.getFeature( _segmentHypothesis->id() );
    slam::SegmentFeature* segFeature = dynamic_cast<slam::SegmentFeature*>( &bf );
    if( segFeature )
    {
      jblas::vec zPredExt1(2);
      jblas::vec zPredExt2(2);
      jblas::sym_mat zPredExt1Cov(2,2);
      jblas::sym_mat zPredExt2Cov(2,2);
      m_slam.computeExtObs( m_robotId, *segFeature, m_obsModel, zPredExt1, zPredExt1Cov, zPredExt2, zPredExt2Cov );
      
      double x1 = zPredExt1(0);
      double y1 = zPredExt1(1);
      double x2 = zPredExt2(0);
      double y2 = zPredExt2(1);
      
      double x_o = 0.5 * (x1 + x2);
      double y_o = 0.5 * (y1 + y2);
      
  
      double angle = -atan2( x1 - x2, y1 - y2);
      double angle2 = angle + M_PI;
      double anglel = _segmentHypothesis->lineFitter().angle();
      
      JFR_DEBUG( JFR_PP_VAR( anglel ) << JFR_PP_VAR( angle2 ) << JFR_PP_VAR( angle ) );
      JFR_DEBUG( JFR_PP_VAR( bf.getP() ) );
      
      if( cos( angle2 - anglel) > cos(angle - anglel) ) // This is needed to recover the direction
      {
        angle = angle2;
      }
      
    
//       JFR_DEBUG( JFR_PP_VAR(angle) << JFR_PP_VAR( x_o ) << JFR_PP_VAR( y_o ) << JFR_PP_VAR( x1 ) << JFR_PP_VAR( y1 ) << JFR_PP_VAR( x2 ) << JFR_PP_VAR( y2 ));
//       JFR_DEBUG( JFR_PP_VAR( _segmentHypothesis->lineFitter().angle() ) << JFR_PP_VAR(_segmentHypothesis->x1() )<< JFR_PP_VAR( _segmentHypothesis->y1() ) << JFR_PP_VAR( _segmentHypothesis->x2() ) << JFR_PP_VAR( _segmentHypothesis->y2() ) );
      
      dseg::SegmentHypothesis* segHyp = new dseg::SegmentHypothesis( x_o, y_o, angle );
      segHyp->setExtremity1( x1, y1 );
      segHyp->setExtremity2( x2, y2 );
//       JFR_DEBUG( JFR_PP_VAR( segHyp->lineFitter().angle() ) << JFR_PP_VAR( segHyp->x1() ) << JFR_PP_VAR( segHyp->y1() ) << JFR_PP_VAR( segHyp->x2() ) << JFR_PP_VAR( segHyp->y2() ) );
      
      // Compute uncertainty of rho/theta
      jblas::vec_range& x1_ = *baseRobot->refPose();
      jblas::vec_range& x2_ = bf.getX();
      
      m_obsModel.predictObservationJac(x1_, x2_);

      
      ublas::range r1( baseRobot->filterIndex(), baseRobot->filterIndex() + m_obsModel.sizeState1());
      ublas::range r2( bf.filterIndex(), bf.filterIndex() + m_obsModel.sizeState2());

      jblas::sym_mat_range& P1_ = *baseRobot->refPoseCov();
      jblas::sym_mat_range& P2_ = bf.getP();
      
      jblas::sym_mat_range P12(m_slam.getFilter().getP(), r1, r2);
      jblas::sym_mat_range P21(m_slam.getFilter().getP(), r2, r1);
      
      jblas::mat P = ublas::prod( m_obsModel.Jobs1, jblas::mat(
              ublas::prod(P1_, ublas::trans(m_obsModel.Jobs1))))
              + ublas::prod(m_obsModel.Jobs1, jblas::mat(ublas::prod(P12, ublas::trans(m_obsModel.Jobs2)))) +
                ublas::prod(m_obsModel.Jobs2, jblas::mat(ublas::prod(P2_, ublas::trans(m_obsModel.Jobs2))))
              + ublas::prod(m_obsModel.Jobs2, jblas::mat(ublas::prod(P21, ublas::trans(m_obsModel.Jobs1)))) +
                m_obsModel.getR();

      double rhoCov = sqrt( P(0,0) );
      double thetaCov = sqrt( P(1,1) );
      JFR_DEBUG( JFR_PP_VAR(P) );
      if(rhoCov > 4) rhoCov = 4;
      if(thetaCov > 0.1) thetaCov = 0.1;
      segHyp->setUncertainty( thetaCov, rhoCov, thetaCov, rhoCov );
//       segHyp->setUncertainty( 0.1, sqrt(xoCov), 0.1, sqrt(yoCov) );
      return segHyp;
    }
  }
  return 0;
}

  }
}
