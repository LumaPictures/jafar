 # $Id$ #

# module slam

# module version 
MODULE_VERSION = 2
MODULE_REVISION = 0

# modules dependencies
REQUIRED_MODULES = kernel jmath filter camera geom
OPTIONAL_MODULES = 

# external libraries dependencies
REQUIRED_EXTLIBS = 
OPTIONAL_EXTLIBS = 

LIBS += $(LAPACK_LDFLAGS) $(LAPACK_LIBS)

LIBS += -lcamera -lfilter -lgeom -ljmath -lkernel $(OPENCV_LDFLAGS) $(OPENCV_LIBS)

CPPFLAGS += $(BOOST_CPPFLAGS) -DBOOST_UBLAS_USE_EXCEPTIONS $(OPENCV_CPPFLAGS)
CPPFLAGS += $(BOOST_SANDBOX_CPPFLAGS)
#CPPFLAGS += -I/home/thomas/usr/local/ttl/include -DHAVE_TTL

#CPPFLAGS += -DNDEBUG

CXXFLAGS += -g -ggdb -Wall
