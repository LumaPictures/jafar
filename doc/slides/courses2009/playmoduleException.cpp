/* $Id$ */

#include "playmodule/playmoduleException.hpp"

#include <sstream>

using std::string;

using namespace jafar::playmodule;

/*
 * class PlaymoduleException
 */

PlaymoduleException::PlaymoduleException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "playmodule", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

PlaymoduleException::~PlaymoduleException() throw() {}

PlaymoduleException::ExceptionId PlaymoduleException::getExceptionId() const throw() {
  return id;
}

string PlaymoduleException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
//   case MY_ERROR:
//     return "MY_ERROR";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

