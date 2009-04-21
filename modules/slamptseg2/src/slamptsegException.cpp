/* $Id$ */

#include "slamptseg/slamptsegException.hpp"

#include <sstream>

using std::string;

using namespace jafar::slamptseg;

/*
 * class SlamptsegException
 */

SlamptsegException::SlamptsegException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "slamptseg", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

SlamptsegException::~SlamptsegException() throw() {}

SlamptsegException::ExceptionId SlamptsegException::getExceptionId() const throw() {
  return id;
}

string SlamptsegException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
//   case MY_ERROR:
//     return "MY_ERROR";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

