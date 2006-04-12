# $Id$ #

# module version 
MODULE_VERSION = 0
MODULE_REVISION = 3 

# modules dependencies
REQUIRED_MODULES = kernel 
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

LDFLAGS += $(BOOST_LDFLAGS)
LIBS += -lkernel 

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall




