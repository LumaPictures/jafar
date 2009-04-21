# $Id$ #

#
# User part of the makefiles
# module slamlines
# 

# module version 
MODULE_VERSION = 2
MODULE_REVISION = 0

# modules dependencies
REQUIRED_MODULES = kernel lines slam
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

# LDFLAGS +=
LIBS += -lkernel -llines -lslam $(OPENCV_LDFLAGS) $(OPENCV_LIBS)

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS) -DBOOST_UBLAS_USE_EXCEPTIONS $(OPENCV_CPPFLAGS)
CPPFLAGS += $(BOOST_SANDBOX_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall




