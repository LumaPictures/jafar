/* $Id:$ */

#include "_jfr_module_/_jfr_module_Exception.hpp"

#include <sstream>

using std::string;

using namespace jafar::_jfr_module_;

/*
 * class _jfr_Module_Exception
 */

_jfr_Module_Exception::_jfr_Module_Exception(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "_jfr_module_", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

_jfr_Module_Exception::~_jfr_Module_Exception() throw() {}

_jfr_Module_Exception::ExceptionId _jfr_Module_Exception::getExceptionId() const throw() {
  return id;
}

string _jfr_Module_Exception::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
//   case MY_ERROR:
//     return "MY_ERROR";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}
