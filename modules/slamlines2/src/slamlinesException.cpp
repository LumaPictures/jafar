/* $Id$ */

#include "slamlines/slamlinesException.hpp"

#include <sstream>

using std::string;

using namespace jafar::slamlines;

/*
 * class SlamlinesException
 */

SlamlinesException::SlamlinesException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "slamlines", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

SlamlinesException::~SlamlinesException() throw() {}

SlamlinesException::ExceptionId SlamlinesException::getExceptionId() const throw() {
  return id;
}

string SlamlinesException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
//   case MY_ERROR:
//     return "MY_ERROR";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

