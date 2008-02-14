/* $Id$ */

#include "lines/linesException.hpp"

#include <sstream>

using std::string;

using namespace jafar::lines;

/*
 * class LinesException
 */

LinesException::LinesException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "lines", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

LinesException::~LinesException() throw() {}

LinesException::ExceptionId LinesException::getExceptionId() const throw() {
  return id;
}

string LinesException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
//   case MY_ERROR:
//     return "MY_ERROR";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

