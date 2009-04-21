# $Id$ #

#
# User part of the makefiles
# module slamptseg
# 

# module version 
MODULE_VERSION = 0
MODULE_REVISION = 1 

# modules dependencies
REQUIRED_MODULES = kernel hpm dseg3d slam slamseg
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

# LDFLAGS +=
LIBS += -lkernel 

# CPPFLAGS += -DJFR_NDEBUG
CPPFLAGS += $(BOOST_CPPFLAGS) $(OPENCV_CPPFLAGS)
CPPFLAGS += $(BOOST_SANDBOX_CPPFLAGS)

CXXFLAGS += -g -ggdb -Wall




