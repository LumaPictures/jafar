/* $Id$ */

#include <sstream>

#include "kernel/jafarException.hpp"

using std::string;
using std::cerr;
using std::endl;

using namespace jafar::kernel;

/*
 * class Exception
 */

Exception::Exception(const string& message_, 
                     const string& module_, const string& id_,
                     const string& file_, int line_) throw() : _what(), trace()
{
  try {
    std::stringstream s;
    s << "\n** Exception from jafar module " << module_ << endl;
    s << "Id: " << id_ << endl;
    _what = s.str();
    addTrace(file_, line_, message_);
  }
  catch (std::exception& e) {
    cerr << "Exception: " << endl;
    cerr << e.what() << endl;
    cerr << "thrown in JafarException constructor:" << endl;
    cerr << "** Exception from jafar module " << module_ << endl;
    cerr << "Id: " << id_ << endl;  
    cerr << file_ << ":" << line_ << ": " << message_ << endl; 
  }
}

Exception::~Exception() throw() {}

const string& Exception::what() const throw() {
  return _what;
}

void Exception::addTrace(const std::string& file_, int line_, const std::string& message_) {
  std::stringstream s;
  s << file_ << ":" << line_ << ":\n  " << message_;
  trace.push_back(s.str());
}

std::ostream& jafar::kernel::operator <<(std::ostream& s, const Exception& e) {
  s << e._what;
  s << "trace:" << endl;
  for (std::list<string>::const_iterator it = e.trace.begin() ; it != e.trace.end() ; it++) {
    s << (*it) << endl;
  }
  return s;
}

/*
 * class JafarException
 */

JafarException::JafarException(ExceptionId id_, const string& message_, 
                               const string& module_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, module_,  exceptionIdToString(id_), file_, line_),
  id(id_)
{}

JafarException::~JafarException() throw() {}

JafarException::ExceptionId JafarException::getExceptionId() const throw() {
  return id;
}

string JafarException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
  case PRECONDITION:
    return "JafarException::PRECONDITION";
  case POSTCONDITION:
    return "JafarException::POSTCONDITION";
  case INVARIANT:
    return "JafarException::INVARIANT";
  case RUN_TIME:
    return "JafarException::RUN_TIME";
  case IO_STREAM:
    return "JafarException::IO_STREAM";
  case NUMERIC:
    return "JafarException::NUMERIC";
  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}


