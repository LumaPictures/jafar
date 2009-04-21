# $Id$ #

#
# User part of the makefiles
# module slampt
# 

# module version 
MODULE_VERSION = 2
MODULE_REVISION = 0

# modules dependencies
REQUIRED_MODULES = kernel hpm slam locpano datareader
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

# LDFLAGS +=
LIBS += -lkernel -lhpm -lslam -llocpano -ldatareader

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS) -DBOOST_UBLAS_USE_EXCEPTIONS $(OPENCV_CPPFLAGS)
CPPFLAGS += $(BOOST_SANDBOX_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall




