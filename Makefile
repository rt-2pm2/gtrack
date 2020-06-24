## Makefile
SHELL = /bin/bash

source_dir := src
binary_dir := bin

program := gtrak
modules := $(shell basename -a $(shell find ./src -mindepth 1 -type d))

sources := $(shell find $(source_dir) -type f -name "*.cpp")
objects := $(patsubst $(source_dir)/%, $(binary_dir)/%, $(subst .cpp,.o, $(sources)))

include_dirs := ./include

CXXFLAGS += -g

CPPFLAGS += $(addprefix -I, $(include_dirs))
CPPFLAGS += `pkg-config opencv4 --cflags` `pkg-config eigen3 --cflags`

LDFLAGS += `pkg-config opencv4 --libs` `pkg-config eigen3 --libs` -lrealsense2 -lpthread

_obj_fld := $(dir $(objects))
_build_dirs := $(shell for d in $(_obj_fld); \
	do \
	echo -e "$$d\n";\
	[ ! -d $$d ] && mkdir -p $$d;\
	done)

vpath %.cpp $(source_dir)

.PHONY: program
program: $(objects)
	$(CXX) -o $(program) $^ $(LDFLAGS) 

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
