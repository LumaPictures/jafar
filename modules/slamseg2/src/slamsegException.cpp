/* $Id$ */

#include "slamseg/slamsegException.hpp"

#include <sstream>

using std::string;

using namespace jafar::slamseg;

/*
 * class SlamsegException
 */

SlamsegException::SlamsegException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "slamseg", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

SlamsegException::~SlamsegException() throw() {}

SlamsegException::ExceptionId SlamsegException::getExceptionId() const throw() {
  return id;
}

string SlamsegException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
//   case MY_ERROR:
//     return "MY_ERROR";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

