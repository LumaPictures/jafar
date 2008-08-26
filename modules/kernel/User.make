# $Id$ #

#
# User part of the makefiles
# module kernel
#

# module version 
MODULE_VERSION = 0
MODULE_REVISION = 2

# modules dependencies
REQUIRED_MODULES =  
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS =  

LDFLAGS += $(BOOST_LDFLAGS)
LIBS += $(BOOST_DATETIME_LIBS)

CPPFLAGS += $(BOOST_CPPFLAGS)
#CPPFLAGS += -DJFR_NDEBUG

CXXFLAGS += -g -ggdb -Wall
CPPFLAGS_MODULE = -Wno-long-long -pedantic -pedantic-errors -Werror 
