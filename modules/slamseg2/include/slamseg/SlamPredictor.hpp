/* $Id$ */

#ifndef _SLAMSEG_SLAM_PREDICTOR_HPP_
#define _SLAMSEG_SLAM_PREDICTOR_HPP_

#include <dseg/StaticPredictor.hpp>

namespace jafar {
  namespace slam {
    class SlamEkf;
    class StereoImagePluckerFeatureObserveModel;
  }
  namespace slamseg {
    template<class _TModel_>
    class SlamPredictor : public dseg::Predictor {
      public:
        SlamPredictor( slam::SlamEkf& _slam, _TModel_& _model, unsigned int _robotId = 0 );
        virtual ~SlamPredictor( );
        virtual dseg::SegmentHypothesis* predictionFor( const dseg::SegmentHypothesis* ) const;
      private:
        slam::SlamEkf& m_slam;
        _TModel_& m_obsModel;
        unsigned int m_robotId;
    };
  }
}

#include "SlamPredictor.tpp"

#endif
