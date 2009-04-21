/* $Id$ */

#ifndef SLAM_FEATURE_HPP
#define SLAM_FEATURE_HPP

#include <ostream>

#include "kernel/dataLog.hpp"

#include "jmath/jblas.hpp"
#include "jmath/gaussianVector.hpp"

#include "filter/constraintModel.hpp"

#include "slam/observation.hpp"
#include "slam/abstractMapObject.hpp"

namespace jafar {
  namespace filter {
    class JacobianBlockPredictModel;
  }
  namespace slam {

    // forward declaration
    class FeatureModel;

    /** Generic feature.
     *
     * \ingroup slam
     */
    class BaseFeature : public AbstractMapObject,
			public jafar::kernel::DataLoggable {

    public:

      Observation::ObservationType typeObs;
      bool atInfinity;

      /** Indexes of frames where the feature has been observed (in
       * chronological order).
       */
      typedef std::list<unsigned int> FrameIndexesType;
      FrameIndexesType frameIndexes;

      FeatureModel& model;

      typedef std::list<jafar::filter::JacobianStrongConstraintModel*> ConstraintsListType;
      /// list of constraints to be applied on the feature state vector
      ConstraintsListType constraints;

      /**
       * Prediction of observation. Filled by SlamEkf::observeKnownFeature.
       */
      jafar::jmath::GaussianVector zPred;

      // FIXME sizeObs should not be necessary !
      BaseFeature(unsigned int id,
		  FeatureModel& model, std::size_t sizeObs, Observation::ObservationType typeObs_);
      virtual ~BaseFeature();

      std::size_t sizeState() const;

      void addObservation(unsigned int frameIndex_);

     /** Model to apply the contraints over features (this model is only implemented 
     for segment features)
      */
      jafar::filter::JacobianBlockPredictModel *featureConstraintModel;

//       virtual void applyConstraints() {};

    protected:
      
      virtual void writeLogHeader(jafar::kernel::DataLogger& log) const;
      virtual void addMembersToLog(jafar::kernel::DataLogger& log) const;
      virtual void writeLogData(jafar::kernel::DataLogger& log) const;

      friend std::ostream& operator <<(std::ostream& s, const BaseFeature& f_);

    private:

      jblas::vec_range* m_x;
      jblas::sym_mat_range* m_P;

    }; // class BaseFeature

    std::ostream& operator <<(std::ostream& s, const BaseFeature& f_);


  } // namespace slam
} // namespace jafar


#endif // SLAM_FEATURE_HPP
