/* $Id$ */

#include "slampt/slamptException.hpp"

#include <sstream>

using std::string;

using namespace jafar::slampt;

/*
 * class SlamptException
 */

SlamptException::SlamptException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "slampt", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

SlamptException::~SlamptException() throw() {}

SlamptException::ExceptionId SlamptException::getExceptionId() const throw() {
  return id;
}

string SlamptException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
//   case MY_ERROR:
//     return "MY_ERROR";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

