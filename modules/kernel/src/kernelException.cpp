/* $Id$ */

#include "kernel/kernelException.hpp"

using std::string;

using namespace jafar::kernel;

/*
 * class KernelException
 */

KernelException::KernelException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "kernel", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

KernelException::~KernelException() throw() {}

KernelException::ExceptionId KernelException::getExceptionId() const throw() {
  return id;
}

string KernelException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
  case KEYVALUEFILE_UNKNOWN_KEY:
    return "KEYVALUEFILE_UNKNOWN_KEY";
  case KEYVALUEFILE_INVALID_LINE:
    return "KEYVALUEFILE_INVALID_LINE";
  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

