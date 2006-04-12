# $Id$ #

#
# User part of the makefiles
# module _jfr_module_
# 

# module version 
MODULE_VERSION = 0
MODULE_REVISION = 1 

# modules dependencies
REQUIRED_MODULES = kernel 
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

# LDFLAGS +=
LIBS += -lkernel 

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall



