CXX:=ccache $(CXX)

CXXFLAGS+=-std=c++17 -Wno-ignored-attributes

CPPFLAGS+=$(LOCAL_CFLAGS)
LDFLAGS+=$(LOCAL_LDFLAGS)

CPPFLAGS+=$(IMR-H_CFLAGS) $(BOOST_CFLAGS) $(CAIRO_CFLAGS) $(LOG4CXX_CPPFLAGS) $(RRT_CFLAGS)
LDFLAGS+=$(IMR-H-ALGORITHM) $(IMR-H-GUI_LDFLAGS) $(IMR-H_LDFLAGS) $(CAIRO_LDFLAGS) $(BOOST_LDFLAGS) $(LOG4CXX_LDFLAGS) $(OPENDUBINS_LDFLAGS) $(RRT_LDFLAGS)


CXXFLAGS+=-O2 -Wno-ignored-attributes
#-pg #for profiler 
#-Wall -lmcheck
#CXXFLAGS+= -march=native
#CXXFLAGS+=-std=c++11

