# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/m16a/Documents/phys/eduPhys

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/m16a/Documents/phys/eduPhys

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/m16a/Documents/phys/eduPhys/CMakeFiles /home/m16a/Documents/phys/eduPhys/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/m16a/Documents/phys/eduPhys/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named quaternion_demo

# Build rule for target.
quaternion_demo: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 quaternion_demo
.PHONY : quaternion_demo

# fast build rule for target.
quaternion_demo/fast:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/build
.PHONY : quaternion_demo/fast

# target to build an object file
box.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/box.o
.PHONY : box.o

# target to preprocess a source file
box.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/box.i
.PHONY : box.i

# target to generate assembly for a file
box.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/box.s
.PHONY : box.s

# target to build an object file
camera.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/camera.o
.PHONY : camera.o

# target to preprocess a source file
camera.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/camera.i
.PHONY : camera.i

# target to generate assembly for a file
camera.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/camera.s
.PHONY : camera.s

# target to build an object file
core.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/core.o
.PHONY : core.o

# target to preprocess a source file
core.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/core.i
.PHONY : core.i

# target to generate assembly for a file
core.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/core.s
.PHONY : core.s

# target to build an object file
geometry.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/geometry.o
.PHONY : geometry.o

# target to preprocess a source file
geometry.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/geometry.i
.PHONY : geometry.i

# target to generate assembly for a file
geometry.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/geometry.s
.PHONY : geometry.s

# target to build an object file
gpuhelper.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/gpuhelper.o
.PHONY : gpuhelper.o

# target to preprocess a source file
gpuhelper.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/gpuhelper.i
.PHONY : gpuhelper.i

# target to generate assembly for a file
gpuhelper.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/gpuhelper.s
.PHONY : gpuhelper.s

# target to build an object file
icosphere.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/icosphere.o
.PHONY : icosphere.o

# target to preprocess a source file
icosphere.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/icosphere.i
.PHONY : icosphere.i

# target to generate assembly for a file
icosphere.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/icosphere.s
.PHONY : icosphere.s

# target to build an object file
my_utils.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/my_utils.o
.PHONY : my_utils.o

# target to preprocess a source file
my_utils.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/my_utils.i
.PHONY : my_utils.i

# target to generate assembly for a file
my_utils.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/my_utils.s
.PHONY : my_utils.s

# target to build an object file
quaternion_demo.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/quaternion_demo.o
.PHONY : quaternion_demo.o

# target to preprocess a source file
quaternion_demo.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/quaternion_demo.i
.PHONY : quaternion_demo.i

# target to generate assembly for a file
quaternion_demo.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/quaternion_demo.s
.PHONY : quaternion_demo.s

# target to build an object file
sphere.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/sphere.o
.PHONY : sphere.o

# target to preprocess a source file
sphere.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/sphere.i
.PHONY : sphere.i

# target to generate assembly for a file
sphere.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/sphere.s
.PHONY : sphere.s

# target to build an object file
trackball.o:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/trackball.o
.PHONY : trackball.o

# target to preprocess a source file
trackball.i:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/trackball.i
.PHONY : trackball.i

# target to generate assembly for a file
trackball.s:
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/trackball.s
.PHONY : trackball.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... quaternion_demo"
	@echo "... rebuild_cache"
	@echo "... box.o"
	@echo "... box.i"
	@echo "... box.s"
	@echo "... camera.o"
	@echo "... camera.i"
	@echo "... camera.s"
	@echo "... core.o"
	@echo "... core.i"
	@echo "... core.s"
	@echo "... geometry.o"
	@echo "... geometry.i"
	@echo "... geometry.s"
	@echo "... gpuhelper.o"
	@echo "... gpuhelper.i"
	@echo "... gpuhelper.s"
	@echo "... icosphere.o"
	@echo "... icosphere.i"
	@echo "... icosphere.s"
	@echo "... my_utils.o"
	@echo "... my_utils.i"
	@echo "... my_utils.s"
	@echo "... quaternion_demo.o"
	@echo "... quaternion_demo.i"
	@echo "... quaternion_demo.s"
	@echo "... sphere.o"
	@echo "... sphere.i"
	@echo "... sphere.s"
	@echo "... trackball.o"
	@echo "... trackball.i"
	@echo "... trackball.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

