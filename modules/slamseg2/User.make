# $Id$ #

#
# User part of the makefiles
# module slamseg
# 

# module version 
MODULE_VERSION = 2
MODULE_REVISION = 0

# modules dependencies
REQUIRED_MODULES = kernel slam dseg datareader
OPTIONAL_MODULES = dseg3d

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

# LDFLAGS +=
LIBS += -lkernel -lslam -ldseg -ldatareader

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS) $(BOOST_SANDBOX_CPPFLAGS) $(OPENCV_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall




