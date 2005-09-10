/* $Id$ */

#include <fstream>

#include "kernel/jafarDebug.hpp"

using std::string;
using std::ofstream;
using std::endl; 
using std::flush;
using std::cerr;
using std::ostream;

using namespace jafar::kernel;

const std::string JafarDebug::_dev_null = "/dev/null";

bool JafarDebug::isDbgStreamAllocated = false;

ostream* JafarDebug::dbg = &cerr;

void JafarDebug::puts(const string& message_) {
  (*dbg) << "D: " 
         << message_ << endl << flush;
}

void JafarDebug::setOn() {
  if (isDbgStreamAllocated) {
    delete dbg;
  }
  dbg = &cerr;
}

void JafarDebug::setOff() {
  setOutputFile(_dev_null);
}

void JafarDebug::setOutputDefault() {
  setOn();
}

void JafarDebug::setOutputFile(const string& file_) {
  ofstream* f = new ofstream();
  f->open(file_.c_str(), std::ios_base::out);
  dbg = f;
  isDbgStreamAllocated = true;
}

void JafarDebug::setOutputStream(ostream* stream_) {
  if (isDbgStreamAllocated) {
    delete dbg;
  }
  dbg = stream_;
}


JafarDebug::JafarDebug() {}

JafarDebug::~JafarDebug() {}
