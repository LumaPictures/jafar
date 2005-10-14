/* $Id$ */

#include "helloworld/helloWorld.hpp"
#include "kernel/jafarDebug.hpp"

using namespace std;

using namespace jafar::helloworld;

HelloWorld::HelloWorld() : hello() {
  JFR_VDEBUG("HelloWorld object created");
}

HelloWorld::HelloWorld(const string& hello_) :
  hello() 
{
  JFR_VDEBUG("HelloWorld object created");
  setHello(hello_);
}

HelloWorld::~HelloWorld() {
  JFR_VDEBUG("HelloWorld object destroyed");
}

const string& HelloWorld::getHello() const {
  return hello;
}

void HelloWorld::setHello(const string& hello_)
{
  JFR_PRECOND(hello_.size() > 0, "HelloWorld::setString: empty string");
  if (!checkHello(hello_)) {
    throw HelloworldFormatException(hello_, 
                                    "message should begin with \"Hello\"", 
                                    __FILE__, __LINE__);
  }
  hello = hello_;
  JFR_DEBUG("set hello message to \"" << hello_ << "\"");
}

void HelloWorld::clearHello() {
  hello.clear();
}

void HelloWorld::printHello()
{
  if (hello.size() > 0) {
    cout << hello << endl << flush;
  }
  else {
    JFR_ERROR(HelloworldException, HelloworldException::EMPTY_HELLO, "hello is not initialized");
  }
}

bool  HelloWorld::checkHello(const string& hello_) {
  JFR_DEBUG("check hello string: \"" << hello_ << "\"");
  return (hello_.substr(0,5) == "Hello");
}

std::ostream& jafar::helloworld::operator<<(std::ostream& s, const HelloWorld& h_) {
  s << "Hello string: " << h_.hello;
  return s;
}
