/* $Id$ */

#if (defined(__MACH__) ||  defined(__APPLE__) || defined(__NetBSD__) \
		|| defined(__OpenBSD__) || defined(__FreeBSD__))
#include <libgen.h>
#endif

#include <cstring>

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
  std::stringstream s;
  s << "\n** Exception from jafar module " << module_ << endl;
  s << "Id: " << id_ << endl;
  s << "Message: " << message_ << endl;
  _what = s.str();
  addTrace(module_, file_, line_, message_);
}

Exception::~Exception() throw() {}

const string& Exception::what() const throw() {
  return _what;
}

void Exception::addTrace(std::string const& module_, std::string const& file_, int line_, const std::string& message_) {
  std::stringstream s;

#ifndef JFR_DEBUG_FULL_PATH
#ifdef __NetBSD__
  s << module_ << "/" << basename(const_cast<char*> (file_.c_str())) << ":" << line_ 
			   << ":\n  " << message_;
#else
  s << module_ << "/" << basename(file_.c_str()) << ":" << line_ << ":\n  " << message_;
#endif
#else
  s << file_ << ":" << line_ << ":\n  " << message_;
#endif

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
  case ASSERT:
    return "JafarException::ASSERT";
  case PRECONDITION:
    return "JafarException::PRECONDITION";
  case POSTCONDITION:
    return "JafarException::POSTCONDITION";
  case INVALID_PARAM:
    return "JafarException::INVALID_PARAM";
  case RUN_TIME:
    return "JafarException::RUN_TIME";
  case IO_STREAM:
    return "JafarException::IO_STREAM";
  case NUMERIC:
    return "JafarException::NUMERIC";
  case INVARIANT:
    return "JafarException::INVARIANT";
  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}


