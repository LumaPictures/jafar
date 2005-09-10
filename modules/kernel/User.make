# $Id$ #

#
# User part of the makefiles
# module kernel
#

# module version 
MODULE_VERSION = 0
MODULE_REVISION = 1

# modules dependencies
USE_MODULES = 

LDFLAGS += $(BOOST_LDFLAGS)
LIBS += -lboost_date_time

CPPFLAGS += $(BOOST_CPPFLAGS)
# CPPFLAGS += -DNDEBUG

CXXFLAGS += -g -ggdb -Wall




