/* $Id$ */

#include "slam/slamException.hpp"

#include <sstream>

using std::string;

using namespace jafar::slam;

/*
 * class SlamException
 */

SlamException::SlamException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "slam", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

SlamException::~SlamException() throw() {}

SlamException::ExceptionId SlamException::getExceptionId() const throw() {
  return id;
}

string SlamException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
  case UNKNOWN_FEATURE:
    return "UNKNOWN_FEATURE";
  case UNKNOWN_POSE_COPY:
    return "UNKNOWN_POSE_COPY";
  case LOCAL_MAP_FULL:
    return "LOCAL_MAP_FULL";
  case UNKNOWN_SENSOR:
    return "UNKNOWN_SENSOR";
  case INVALID_INITSTATE_UPDATE:
    return "INVALID_INITSTATE_UPDATE";
  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

