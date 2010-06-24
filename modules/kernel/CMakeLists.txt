# $Id$ #

#
# User part of the makefiles
# module kernel
#

build_jafar_module(kernel
  VERSION 0
  REVISION 2
  #REQUIRED_MODULES
  #OPTIONAL_MODULES
  REQUIRED_EXTLIBS boost_date_time boost_thread
  #OPTIONAL_EXTLIBS
  CXXFLAGS -g -ggdb -Wall
  CPPFLAGS_MODULE -Wno-long-long -pedantic -pedantic-errors -Werror 
)
