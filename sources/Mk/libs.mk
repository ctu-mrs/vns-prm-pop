#
# Date:      2011/07/11 17:55
# Author:    Jan Faigl
#

OPSYS=$(shell uname)

PLATFORM=$(shell uname -p)
ARCH=.$(PLATFORM)

ifeq ($(OPSYS),FreeBSD)
   BOOST_CFLAGS=-I/usr/local/include
   BOOST_LDFLAGS=-L/usr/local/lib

else
   LOG4CXX_CPPFLAGS=$(shell pkg-config --cflags liblog4cxx)
   LOG4CXX_LDFLAGS=$(shell pkg-config --libs liblog4cxx)

   CAIRO_LDFLAGS:=-L/usr/X11/lib
   CAIRO_CFLAGS:=-I/usr/X11/include
endif

CPLEX_LIBS=-L$(CPLEX_ROOT_DIR)/cplex/lib/x86-64_linux/static_pic -L$(CPLEX_ROOT_DIR)/opl/lib/x86-64_linux/static_pic -lilocplex -lconcert -lcplex -lm -lpthread 
CPLEX_INCLUDE=-DIL_STD -I$(CPLEX_ROOT_DIR)/cplex/include -I$(CPLEX_ROOT_DIR)/opl/include

BOOST_LDFLAGS+=-lboost_program_options -lboost_thread -lboost_filesystem -lboost_iostreams -lboost_system
LOG4CXX_LDFLAGS+=-llog4cxx
CAIRO_LDFLAGS+=-lcairo -pthread -lX11

#$(CPLEX_INCLUDE)
#$(CPLEX_LIBS)
LOCAL_CFLAGS=-I./lib/jf_vendors/include -I./lib/flann/install/include
LOCAL_LDFLAGS=-L./lib/jf_vendors/lib  -L./lib/flann/install/lib/ -lflann_cpp_s

RRT_LDFLAGS=-L./lib/rapid-2.01 -lRAPID 
RRT_CFLAGS=-I./lib/rapid-2.01 -I./lib/ann/ann/include -I./lib/mpnn/MPNN/include

#LOCAL_CFLAGS+=-I../opendubins/build/include
#LOCAL_LDFLAGS+=-L../opendubins/build/lib 

#OPENDUBINS_LDFLAGS=-lopendubins_planning -lopendubins_core 
#OPENDUBINS_LDFLAGS=-lopendubins_core

IMR-H-GUI_LDFLAGS=-lcrl-gui 
IMR-H_LDFLAGS=-lcrl 
IMR-H-ALGORITHM=-lcrl-algorithm 

CAIRO_LDFLAGS+=-lcairo -pthread -lX11

