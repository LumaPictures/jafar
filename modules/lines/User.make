# $Id$ #

#
# User part of the makefiles
# module lines
# 

# module version 
MODULE_VERSION = 0
MODULE_REVISION = 1 

# modules dependencies
REQUIRED_MODULES = kernel image
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = opencv
OPTIONAL_EXTLIBS = 

LDFLAGS += $(OPENCV_LDFLAGS) $(BOOST_LDFLAGS)
LIBS += -lkernel $(OPENCV_LIBS) -lboost_serialization

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS) $(OPENCV_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall




