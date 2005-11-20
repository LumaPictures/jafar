/* $Id$ */

#include <cstdlib>

#include "helloworld/helloworldException.hpp"

using std::string;
using std::cerr;
using std::endl;

using namespace jafar::helloworld;

/*
 * class HelloworldException
 */

HelloworldException::HelloworldException(ExceptionId id_, const string& message_, const string& file_, int line_) throw() :
  jafar::kernel::Exception(message_, "helloworld", exceptionIdToString(id_), file_, line_),
  id(id_)
{}

HelloworldException::~HelloworldException() throw() {}

HelloworldException::ExceptionId HelloworldException::getExceptionId() const throw() {
  return id;
}

string HelloworldException::exceptionIdToString(ExceptionId id_) throw() {
  switch(id_) {
  case EMPTY_HELLO:
    return "HelloworldException::EMPTY_HELLO";
  case BAD_FORMAT:
    return "HelloworldException::BAD_FORMAT";

  default:
    std::stringstream s;
    s << id_;
    return s.str();
  }
}

/*
 * class HelloworldFormatException
 */

HelloworldFormatException::HelloworldFormatException(const string& hello_, 
                                                     const string& message_, 
                                                     const string& file_, int line_) throw() :
  HelloworldException(BAD_FORMAT, 
                      message_, 
                      file_, line_),
  hello()
{
  try {
    hello = hello_;
    _what +=  "given string: " + hello + "\n";
  }
  catch (std::exception& e) {
    cerr << "Exception: " << endl;
    cerr << e.what() << endl;
    cerr << "thrown in HelloworldFormatException constructor:" << endl;
    cerr << _what;
    cerr << "given string: " << hello << endl;
  }
}

HelloworldFormatException::~HelloworldFormatException() throw() {}

const string& HelloworldFormatException::getHello() const throw() {
  return hello;
}


