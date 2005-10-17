# $Id$ #

#
# User part of the makefiles
# module kernel
#

# module version 
MODULE_VERSION = 0
MODULE_REVISION = 2

# modules dependencies
USE_MODULES = 

LDFLAGS += $(BOOST_LDFLAGS)
LIBS += $(BOOST_DATETIME_LIBS)

CPPFLAGS += $(BOOST_CPPFLAGS)
# CPPFLAGS += -DNDEBUG

CXXFLAGS += -g -ggdb -Wall




