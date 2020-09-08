## Makefile
SHELL = /bin/bash

source_dir := src
binary_dir := bin
data_dir := data

rpcfolder = ./deps/rpclib

program := gtrack
modules := $(shell basename -a $(shell find ./src -mindepth 1 -type d))

sources := $(shell find $(source_dir) -type f -name "*.cpp")
objects := $(patsubst $(source_dir)/%, $(binary_dir)/%, $(subst .cpp,.o, $(sources)))

include_dirs := ./include
include_dirs += $(rpcfolder)/include

CXXFLAGS += -g

CPPFLAGS += $(addprefix -I, $(include_dirs))
CPPFLAGS += `pkg-config opencv4 --cflags` `pkg-config eigen3 --cflags`

LDFLAGS += `pkg-config opencv4 --libs` `pkg-config eigen3 --libs` -lrealsense2 -lpthread -ljsoncpp
LDFLAGS += -L$(rpcfolder)/build -lrpc

_obj_fld := $(dir $(objects))
_build_dirs := $(shell for d in $(_obj_fld) $(data_dir); \
	do \
	echo -e "$$d\n";\
	[ ! -d $$d ] && mkdir -p $$d;\
	done)

vpath %.cpp $(source_dir)



#CPPFLAGS += -DGATLAS_DEBUG
#CPPFLAGS += -DDEVINTERFACE_DEBUG
CPPFLAGS += -DHIST_DEBUG
CPPFLAGS += -DMASK_DEBUG
CPPFLAGS += -DARUCO_DEBUG
#CPPFLAGS += -DMMTRACKER_DEBUG
CPPFLAGS += -DRSTRACKER_DEBUG





.PHONY: program
program: $(objects)
	$(CXX) -o $(binary_dir)/$(program) $^ $(LDFLAGS) 

$(binary_dir)/%.o: $(source_dir)/%.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $^ -o $@


.PHONY: clean
clean:
	@rm -fr bin/*

.PHONY: print
print:
	@echo "Application: " $(program)
	@echo "Modules: " $(modules)
	@echo "Sources: " $(sources)
	@echo "Objects:" $(objects)
