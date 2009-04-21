/* $Id$ */

#include "geom/t3dIdentity.hpp"

#include "slam/eulerTools.hpp"
#include "slam/full3dPredictModel.hpp"

using namespace jafar;
using namespace jafar::slam;

Full3dPredictModel::Full3dPredictModel() : 
  JacobianBlockCommandPredictModel(6,6)
{
  // dynamic noise model is null
  Q.clear();
  m_predToRef = new geom::T3DIdentity();
  m_refToPred = new geom::T3DIdentity();
}

Full3dPredictModel::~Full3dPredictModel() {
  delete m_predToRef;
  delete m_refToPred;
}

void Full3dPredictModel::setPredToRef(geom::T3DEuler const& predToRef)
{
  if (predToRef.hasCov())
    JFR_WARNING("Full3dPredictModel::setPredToRef: predToRef covariance is ignored");
  
  delete m_predToRef;
  delete m_refToPred;
  m_predToRef = new geom::T3DEuler(predToRef);
  m_refToPred = new geom::T3DEuler();
  geom::T3D::inv(*m_predToRef, *m_refToPred);
  
  JFR_DEBUG("Full3dPredictModel::setPredToRef: " << m_predToRef->getX());

}

void Full3dPredictModel::predict(jblas::vec_range& x_r, jblas::vec const& dx) {
  JFR_PRECOND(x_r.size() == sizeState(),
	      "Odo3dPredictModel::predict: invalid state size");
  JFR_PRECOND(dx.size() == sizeCommand(),
	      "Odo3dPredictModel::predict: invalid command size");

  JFR_TRACE_BEGIN;

  jblas::vec ref_dx(6);
  

  if (predToRef().isIdentity()) {
    ref_dx.assign(dx);
  }
  else {
    jblas::vec tmp(6); 
    EulerTools::composeFrame(refToPred().getX(), dx, tmp);
    EulerTools::composeFrame(tmp, predToRef().getX(), ref_dx);
  }

  jblas::vec xPred(6);

  EulerTools::composeFrame(x_r, ref_dx, xPred);
  EulerTools::composeFrameJac(x_r, ref_dx, F, G);

  x_r.assign(xPred);

  JFR_TRACE_END("Full3dPredictModel::predict()");

}
