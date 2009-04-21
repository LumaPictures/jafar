/* $Id$*/

#include <sstream>

#include "jmath/ublasExtra.hpp"

#include "slam/slamException.hpp"
#include "slam/feature.hpp"
#include "slam/featureModel.hpp"

using namespace ublas;
using namespace jblas;
using namespace jafar::slam;


/*
 * class BaseFeature
 */
//FIXME dynamic sizeObs
BaseFeature::BaseFeature(unsigned int id,
			 FeatureModel& featureModel, std::size_t sizeObs, Observation::ObservationType typeObs_) :
  AbstractMapObject(id),
  typeObs(typeObs_),
  atInfinity(false),
  frameIndexes(),
  model(featureModel),
  zPred(sizeObs),
  featureConstraintModel(0),
  m_x(0),
  m_P(0)
{}

// BaseFeature::BaseFeature(IdType id_,
//                          FeatureModel& featureModel_,
//                          std::size_t sizeObs) :
//         id(id_),
//         frameIndexes(),
//         m_featureModel(&featureModel_),
// zPred(sizeObs) {}

BaseFeature::~BaseFeature() {
  if (m_x) delete m_x;
  if (m_P) delete m_P;

  for (ConstraintsListType::iterator it = constraints.begin() ;
       it != constraints.end() ; ++it) {
    delete (*it);
  }
}


std::size_t BaseFeature::sizeState() const {
  return model.sizeState();
}


void BaseFeature::addObservation(unsigned int frameIndex_) {
  frameIndexes.push_back(frameIndex_);
}


void BaseFeature::writeLogHeader(jafar::kernel::DataLogger& log) const {
  log.writeComment("slam: BaseFeature");
  {
    std::ostringstream os;
    os << "id: " << id();
    log.writeComment(os.str());
  }
  {
    std::ostringstream os;
    for (std::size_t i = 0 ; i < sizeState() ; ++i) {
      os << "state(" << i << ") ";
    }
    log.writeLegendTokens(os.str());
  }

  {
    std::ostringstream os;
    for (std::size_t i = 0 ; i < sizeState() ; ++i) {
      os << "sigma(" << i << ") ";
    }
    log.writeLegendTokens(os.str());
  }
  log.writeLegend("state uncertainty volume");
  log.writeLegend("number of observations");
}


void BaseFeature::addMembersToLog(jafar::kernel::DataLogger& log) const {
  for (ConstraintsListType::const_iterator it = constraints.begin() ; it != constraints.end() ; ++it) {
    log.addLoggable(**it);
  }
}


void BaseFeature::writeLogData(jafar::kernel::DataLogger& log) const {
  JFR_TRACE_BEGIN;
  if (m_x) {
    log.writeDataVector(getX());
  } 
  // FIXME
// else {
//     jblas::zero_vec nullState (sizeState());
//     log.writeDataVector(nullState);
//   }

  if (m_P) {
    for (std::size_t i=0 ; i < sizeState() ; ++i) {
      log.writeData(sqrt(getP()(i,i)));
    }
    log.writeData(jmath::ublasExtra::lu_det(getP()));
   }
    // FIXME
  // else {
//     jblas::zero_vec nullStateStdDev (sizeState());
//     log.writeDataVector(nullStateStdDev);
//     log.writeData(0);
//   }

  log.writeData(frameIndexes.size());

  JFR_TRACE_END("");
}


std::ostream& jafar::slam::operator <<(std::ostream& s, const BaseFeature& f) {
  s << f.id() << " (index: " << f.filterIndex() << "): " << f.getX() << ", " << f.getP();
//   s << " observed: { ";
//   for (BaseFeature::FrameIndexesType::const_iterator it = f.frameIndexes.begin() ; it != f.frameIndexes.end() ; ++it) {
//     s << *it << " ";
//   }
//   s << "}";
  return s;
}
