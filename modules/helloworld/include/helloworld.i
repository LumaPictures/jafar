/* $Id$ */

/** swig/tcl interface file for module helloworld.
 *
 * \file helloworld.i
 * \ingroup helloworld
 */  

%module helloworld

%{
  /* 
   * headers necessary to compile the wrapper
   *
   * example:
   *
   * #include <cmath>
   * #include "toto.hpp"
   *
   */

#include <string>

#include "helloworld/helloWorld.hpp"

%}

%include "jafar.i"

/*
 * headers to be wrapped goes here
 *
 * example:
 *
 * %include "toto.hpp"
 *
 */

%include "helloworldException.i"
%include "helloworldTools.i"
%template(print) jafar::helloworld::print<jafar::helloworld::HelloWorld>;

%include "helloworld/helloWorld.hpp"

