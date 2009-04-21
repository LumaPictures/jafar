/* $Id$ */

#include "slam/robot.hpp"
#include "filter/predictModel.hpp"

using namespace jblas;
using namespace ublas;
using namespace jafar::slam;

BaseRobot::BaseRobot(unsigned int id, jafar::filter::JacobianBlockCommandPredictModel& _model, std::size_t sizePose ) : 
AbstractMapObject( id ), 
model(_model), 
m_sizePose( sizePose ), 
m_refPose(0), 
m_refPoseCov(0)
{
}

BaseRobot::~BaseRobot()
{
  delete m_refPose;
  delete m_refPoseCov;
}

std::size_t BaseRobot::sizeState() const
{
  return model.sizeState();
}

