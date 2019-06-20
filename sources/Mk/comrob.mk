#
# Date:      2013/06/10 15:21
# Author:    Jan Faigl
#

#CXX:=ccache $(CXX)

uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

CXXFLAGS:=$(call uniq,$(CXXFLAGS))

OBJS_W_DIRS:=$(addprefix $(OBJ_DIR)/,$(OBJS))
OBJS_SUBDIRS:=$(addprefix $(OBJ_DIR)/,$(SUBDIRS))
OBJS_SUBDIRS_ALL:=$(addsuffix /*.o,$(OBJS_SUBDIRS))
OBJS_W_DIRS+=$(OBJ_DIR)/triangle.o 

bin: all_targets

$(OBJS): %.o: %.cpp
	$(CXX) -c $< $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$@

all_targets: create_directories triangle obj_subdirs $(OBJS) 
	@for i in $(TARGETS) ;\
	do \
	echo "compiling target $$i"; \
	echo "$(CXX) -c $$i.cpp $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$$i.o $(LDFLAGS)"; \
	$(CXX) -c $$i.cpp $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$$i.o $(LDFLAGS); \
	echo "$(CXX) -o $$i $(OBJ_DIR)/$$i.o $(OBJS_W_DIRS) $(OBJS_SUBDIRS_ALL) $(LDFLAGS) $(CXXFLAGS)"; \
	$(CXX) -o $$i $(OBJ_DIR)/$$i.o $(OBJS_W_DIRS) $(OBJS_SUBDIRS_ALL) $(LDFLAGS) $(CXXFLAGS);\
    done

all_subdirs:    
	echo $(OPSYS)
	@for i in $(SUBDIRS) ;\
	do \
	echo "making" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i all || exit 1 ;\
	done
	$(MAKE) -C triangle trilibrary

obj_subdirs: all_subdirs
	echo "Copy objs"
	@for i in $(SUBDIRS) ;\
	do \
	$(MAKE) --ignore-errors -C $$i obj ;\
	echo "coping all in $(CURRENT_DIR)/$$i..."; \
	cp $$i/*.o $(OBJ_DIR)/$$i/; \
	done
	cp triangle/*.o $(OBJ_DIR)/; \
	
create_directories:
	echo "create dircetory $(OBJ_DIR)"
	mkdir -p $(OBJ_DIR)
	@for i in $(SUBDIRS) ;\
	do \
	mkdir -p $(OBJ_DIR)/$$i ;\
	done
	
dependencies: rapid_lib vendors_lib flann_lib

rapid_lib:
	echo "making rapid"
	$(MAKE) -C lib/rapid-2.01/

vendors_lib:
	echo "making jf_vendors"
	cd ./lib/jf_vendors/crl; ./install.sh

flann_lib:
	echo "making flann"
	cd ./lib/flann; mkdir build; cd build; cmake .. -DCMAKE_INSTALL_PREFIX=../install; make -j4; make install

clean_dependencies: clean_rapid_lib clean_vendors_lib clean_flann_lib

clean_rapid_lib:
	echo "cleaning rapid"
	cd lib/rapid-2.01/ ; make clean

clean_vendors_lib:
	echo "cleaning jf_vendors"
	cd ./lib/jf_vendors/crl; ./clean.sh

clean_flann_lib:
	echo "cleaning flann"
	cd ./lib/flann; rm -rf build; rm -rf install

clean:
	$(RM) $(TARGETS)
	$(RM) -r $(OBJ_DIR)/*
	@for i in $(SUBDIRS) ;\
        do \
        echo "cleaning" all "in $(CURRENT_DIR)/$$i..."; \
        $(MAKE) -C $$i clean; \
        done
	$(MAKE) -C triangle clean

clean_all: clean clean_dependencies

