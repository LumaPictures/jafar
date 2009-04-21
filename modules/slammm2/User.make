# $Id$ #

#
# User part of the makefiles
# module slammm
# 

# module version 
MODULE_VERSION = 2
MODULE_REVISION = 0

# modules dependencies
REQUIRED_MODULES = kernel slam
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

# LDFLAGS +=
LIBS += -lkernel -lslam

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS) -DBOOST_UBLAS_USE_EXCEPTIONS
CPPFLAGS += $(BOOST_SANDBOX_CPPFLAGS)
CPPFLAGS += $(BOOST_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall




