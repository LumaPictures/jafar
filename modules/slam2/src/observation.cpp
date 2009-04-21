/* $Id$ */

#include "slam/observation.hpp"

using namespace jblas;
using namespace jafar::slam;

/*
 * class Observation
 */

std::ostream& jafar::slam::operator <<(std::ostream& s, const Observation::ObservationType& t) {
  switch (t) {
  case  Observation::POINT_CARTESIAN:
    s << "POINT_CARTESIAN";
    break;
  case Observation::POINT_POLAR:
    s << "POINT_POLAR";
    break;
  case Observation::POINT_STEREOIMAGE:
    s << "POINT_STEREOIMAGE";
    break;
  case Observation::POINT_BEARING:
    s << "POINT_BEARING";
    break;
  case Observation::POINT_IMAGE:
    s << "POINT_IMAGE";
    break;
  case Observation::POINT_OMNIIMAGE:
    s << "POINT_OMNIIMAGE";
    break;
  case Observation::SEGMENT_IMAGE:
    s << "SEGMENT_IMAGE";
    break;
  case Observation::SEGMENTID_IMAGE:
    s << "SEGMENTID_IMAGE";
    break;
  case Observation::SEGMENTID_EXT_IMAGE:
    s << "SEGMENTID_EXT_IMAGE";
    break;
  case Observation::SEGMENT_STEREOIMAGE:
    s << "SEGMENT_STEREOIMAGE";
    break;
  case Observation::BASIS:
    s << "BASIS";
    break;
  default:
    JFR_WARNING("<< operator for ObservationType::ObservationType is not defined for value: " << (int)t);
    s << (int)t;
    break;
  }
  return s;
}

std::ostream& jafar::slam::operator <<(std::ostream& s, const jafar::slam::Observation& o_) {
  s << "id: " << o_.id << " (" << o_.sensorId << ", " << o_.type << "): "
    << " z: " << o_.z;
  return s;
}
