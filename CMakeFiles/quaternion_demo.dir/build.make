# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# Include any dependencies generated for this target.
include CMakeFiles/quaternion_demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quaternion_demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quaternion_demo.dir/flags.make

quaternion_demo.moc: quaternion_demo.h
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating quaternion_demo.moc"
	/usr/lib/x86_64-linux-gnu/qt4/bin/moc @/home/m16a/Documents/phys/eduPhys/quaternion_demo.moc_parameters

CMakeFiles/quaternion_demo.dir/gpuhelper.o: CMakeFiles/quaternion_demo.dir/flags.make
CMakeFiles/quaternion_demo.dir/gpuhelper.o: gpuhelper.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quaternion_demo.dir/gpuhelper.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quaternion_demo.dir/gpuhelper.o -c /home/m16a/Documents/phys/eduPhys/gpuhelper.cpp

CMakeFiles/quaternion_demo.dir/gpuhelper.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quaternion_demo.dir/gpuhelper.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/m16a/Documents/phys/eduPhys/gpuhelper.cpp > CMakeFiles/quaternion_demo.dir/gpuhelper.i

CMakeFiles/quaternion_demo.dir/gpuhelper.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quaternion_demo.dir/gpuhelper.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/m16a/Documents/phys/eduPhys/gpuhelper.cpp -o CMakeFiles/quaternion_demo.dir/gpuhelper.s

CMakeFiles/quaternion_demo.dir/gpuhelper.o.requires:
.PHONY : CMakeFiles/quaternion_demo.dir/gpuhelper.o.requires

CMakeFiles/quaternion_demo.dir/gpuhelper.o.provides: CMakeFiles/quaternion_demo.dir/gpuhelper.o.requires
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/gpuhelper.o.provides.build
.PHONY : CMakeFiles/quaternion_demo.dir/gpuhelper.o.provides

CMakeFiles/quaternion_demo.dir/gpuhelper.o.provides.build: CMakeFiles/quaternion_demo.dir/gpuhelper.o

CMakeFiles/quaternion_demo.dir/icosphere.o: CMakeFiles/quaternion_demo.dir/flags.make
CMakeFiles/quaternion_demo.dir/icosphere.o: icosphere.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quaternion_demo.dir/icosphere.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quaternion_demo.dir/icosphere.o -c /home/m16a/Documents/phys/eduPhys/icosphere.cpp

CMakeFiles/quaternion_demo.dir/icosphere.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quaternion_demo.dir/icosphere.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/m16a/Documents/phys/eduPhys/icosphere.cpp > CMakeFiles/quaternion_demo.dir/icosphere.i

CMakeFiles/quaternion_demo.dir/icosphere.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quaternion_demo.dir/icosphere.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/m16a/Documents/phys/eduPhys/icosphere.cpp -o CMakeFiles/quaternion_demo.dir/icosphere.s

CMakeFiles/quaternion_demo.dir/icosphere.o.requires:
.PHONY : CMakeFiles/quaternion_demo.dir/icosphere.o.requires

CMakeFiles/quaternion_demo.dir/icosphere.o.provides: CMakeFiles/quaternion_demo.dir/icosphere.o.requires
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/icosphere.o.provides.build
.PHONY : CMakeFiles/quaternion_demo.dir/icosphere.o.provides

CMakeFiles/quaternion_demo.dir/icosphere.o.provides.build: CMakeFiles/quaternion_demo.dir/icosphere.o

CMakeFiles/quaternion_demo.dir/camera.o: CMakeFiles/quaternion_demo.dir/flags.make
CMakeFiles/quaternion_demo.dir/camera.o: camera.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quaternion_demo.dir/camera.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quaternion_demo.dir/camera.o -c /home/m16a/Documents/phys/eduPhys/camera.cpp

CMakeFiles/quaternion_demo.dir/camera.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quaternion_demo.dir/camera.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/m16a/Documents/phys/eduPhys/camera.cpp > CMakeFiles/quaternion_demo.dir/camera.i

CMakeFiles/quaternion_demo.dir/camera.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quaternion_demo.dir/camera.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/m16a/Documents/phys/eduPhys/camera.cpp -o CMakeFiles/quaternion_demo.dir/camera.s

CMakeFiles/quaternion_demo.dir/camera.o.requires:
.PHONY : CMakeFiles/quaternion_demo.dir/camera.o.requires

CMakeFiles/quaternion_demo.dir/camera.o.provides: CMakeFiles/quaternion_demo.dir/camera.o.requires
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/camera.o.provides.build
.PHONY : CMakeFiles/quaternion_demo.dir/camera.o.provides

CMakeFiles/quaternion_demo.dir/camera.o.provides.build: CMakeFiles/quaternion_demo.dir/camera.o

CMakeFiles/quaternion_demo.dir/trackball.o: CMakeFiles/quaternion_demo.dir/flags.make
CMakeFiles/quaternion_demo.dir/trackball.o: trackball.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quaternion_demo.dir/trackball.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quaternion_demo.dir/trackball.o -c /home/m16a/Documents/phys/eduPhys/trackball.cpp

CMakeFiles/quaternion_demo.dir/trackball.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quaternion_demo.dir/trackball.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/m16a/Documents/phys/eduPhys/trackball.cpp > CMakeFiles/quaternion_demo.dir/trackball.i

CMakeFiles/quaternion_demo.dir/trackball.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quaternion_demo.dir/trackball.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/m16a/Documents/phys/eduPhys/trackball.cpp -o CMakeFiles/quaternion_demo.dir/trackball.s

CMakeFiles/quaternion_demo.dir/trackball.o.requires:
.PHONY : CMakeFiles/quaternion_demo.dir/trackball.o.requires

CMakeFiles/quaternion_demo.dir/trackball.o.provides: CMakeFiles/quaternion_demo.dir/trackball.o.requires
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/trackball.o.provides.build
.PHONY : CMakeFiles/quaternion_demo.dir/trackball.o.provides

CMakeFiles/quaternion_demo.dir/trackball.o.provides.build: CMakeFiles/quaternion_demo.dir/trackball.o

CMakeFiles/quaternion_demo.dir/quaternion_demo.o: CMakeFiles/quaternion_demo.dir/flags.make
CMakeFiles/quaternion_demo.dir/quaternion_demo.o: quaternion_demo.cpp
CMakeFiles/quaternion_demo.dir/quaternion_demo.o: quaternion_demo.moc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quaternion_demo.dir/quaternion_demo.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quaternion_demo.dir/quaternion_demo.o -c /home/m16a/Documents/phys/eduPhys/quaternion_demo.cpp

CMakeFiles/quaternion_demo.dir/quaternion_demo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quaternion_demo.dir/quaternion_demo.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/m16a/Documents/phys/eduPhys/quaternion_demo.cpp > CMakeFiles/quaternion_demo.dir/quaternion_demo.i

CMakeFiles/quaternion_demo.dir/quaternion_demo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quaternion_demo.dir/quaternion_demo.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/m16a/Documents/phys/eduPhys/quaternion_demo.cpp -o CMakeFiles/quaternion_demo.dir/quaternion_demo.s

CMakeFiles/quaternion_demo.dir/quaternion_demo.o.requires:
.PHONY : CMakeFiles/quaternion_demo.dir/quaternion_demo.o.requires

CMakeFiles/quaternion_demo.dir/quaternion_demo.o.provides: CMakeFiles/quaternion_demo.dir/quaternion_demo.o.requires
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/quaternion_demo.o.provides.build
.PHONY : CMakeFiles/quaternion_demo.dir/quaternion_demo.o.provides

CMakeFiles/quaternion_demo.dir/quaternion_demo.o.provides.build: CMakeFiles/quaternion_demo.dir/quaternion_demo.o

CMakeFiles/quaternion_demo.dir/sphere.o: CMakeFiles/quaternion_demo.dir/flags.make
CMakeFiles/quaternion_demo.dir/sphere.o: sphere.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quaternion_demo.dir/sphere.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quaternion_demo.dir/sphere.o -c /home/m16a/Documents/phys/eduPhys/sphere.cpp

CMakeFiles/quaternion_demo.dir/sphere.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quaternion_demo.dir/sphere.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/m16a/Documents/phys/eduPhys/sphere.cpp > CMakeFiles/quaternion_demo.dir/sphere.i

CMakeFiles/quaternion_demo.dir/sphere.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quaternion_demo.dir/sphere.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/m16a/Documents/phys/eduPhys/sphere.cpp -o CMakeFiles/quaternion_demo.dir/sphere.s

CMakeFiles/quaternion_demo.dir/sphere.o.requires:
.PHONY : CMakeFiles/quaternion_demo.dir/sphere.o.requires

CMakeFiles/quaternion_demo.dir/sphere.o.provides: CMakeFiles/quaternion_demo.dir/sphere.o.requires
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/sphere.o.provides.build
.PHONY : CMakeFiles/quaternion_demo.dir/sphere.o.provides

CMakeFiles/quaternion_demo.dir/sphere.o.provides.build: CMakeFiles/quaternion_demo.dir/sphere.o

CMakeFiles/quaternion_demo.dir/core.o: CMakeFiles/quaternion_demo.dir/flags.make
CMakeFiles/quaternion_demo.dir/core.o: core.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/m16a/Documents/phys/eduPhys/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quaternion_demo.dir/core.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/quaternion_demo.dir/core.o -c /home/m16a/Documents/phys/eduPhys/core.cpp

CMakeFiles/quaternion_demo.dir/core.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quaternion_demo.dir/core.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/m16a/Documents/phys/eduPhys/core.cpp > CMakeFiles/quaternion_demo.dir/core.i

CMakeFiles/quaternion_demo.dir/core.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quaternion_demo.dir/core.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/m16a/Documents/phys/eduPhys/core.cpp -o CMakeFiles/quaternion_demo.dir/core.s

CMakeFiles/quaternion_demo.dir/core.o.requires:
.PHONY : CMakeFiles/quaternion_demo.dir/core.o.requires

CMakeFiles/quaternion_demo.dir/core.o.provides: CMakeFiles/quaternion_demo.dir/core.o.requires
	$(MAKE) -f CMakeFiles/quaternion_demo.dir/build.make CMakeFiles/quaternion_demo.dir/core.o.provides.build
.PHONY : CMakeFiles/quaternion_demo.dir/core.o.provides

CMakeFiles/quaternion_demo.dir/core.o.provides.build: CMakeFiles/quaternion_demo.dir/core.o

# Object files for target quaternion_demo
quaternion_demo_OBJECTS = \
"CMakeFiles/quaternion_demo.dir/gpuhelper.o" \
"CMakeFiles/quaternion_demo.dir/icosphere.o" \
"CMakeFiles/quaternion_demo.dir/camera.o" \
"CMakeFiles/quaternion_demo.dir/trackball.o" \
"CMakeFiles/quaternion_demo.dir/quaternion_demo.o" \
"CMakeFiles/quaternion_demo.dir/sphere.o" \
"CMakeFiles/quaternion_demo.dir/core.o"

# External object files for target quaternion_demo
quaternion_demo_EXTERNAL_OBJECTS =

quaternion_demo: CMakeFiles/quaternion_demo.dir/gpuhelper.o
quaternion_demo: CMakeFiles/quaternion_demo.dir/icosphere.o
quaternion_demo: CMakeFiles/quaternion_demo.dir/camera.o
quaternion_demo: CMakeFiles/quaternion_demo.dir/trackball.o
quaternion_demo: CMakeFiles/quaternion_demo.dir/quaternion_demo.o
quaternion_demo: CMakeFiles/quaternion_demo.dir/sphere.o
quaternion_demo: CMakeFiles/quaternion_demo.dir/core.o
quaternion_demo: CMakeFiles/quaternion_demo.dir/build.make
quaternion_demo: /usr/lib/x86_64-linux-gnu/libQtCore.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libQtGui.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libGLU.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libGL.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libSM.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libICE.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libX11.so
quaternion_demo: /usr/lib/x86_64-linux-gnu/libXext.so
quaternion_demo: CMakeFiles/quaternion_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable quaternion_demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quaternion_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quaternion_demo.dir/build: quaternion_demo
.PHONY : CMakeFiles/quaternion_demo.dir/build

CMakeFiles/quaternion_demo.dir/requires: CMakeFiles/quaternion_demo.dir/gpuhelper.o.requires
CMakeFiles/quaternion_demo.dir/requires: CMakeFiles/quaternion_demo.dir/icosphere.o.requires
CMakeFiles/quaternion_demo.dir/requires: CMakeFiles/quaternion_demo.dir/camera.o.requires
CMakeFiles/quaternion_demo.dir/requires: CMakeFiles/quaternion_demo.dir/trackball.o.requires
CMakeFiles/quaternion_demo.dir/requires: CMakeFiles/quaternion_demo.dir/quaternion_demo.o.requires
CMakeFiles/quaternion_demo.dir/requires: CMakeFiles/quaternion_demo.dir/sphere.o.requires
CMakeFiles/quaternion_demo.dir/requires: CMakeFiles/quaternion_demo.dir/core.o.requires
.PHONY : CMakeFiles/quaternion_demo.dir/requires

CMakeFiles/quaternion_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quaternion_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quaternion_demo.dir/clean

CMakeFiles/quaternion_demo.dir/depend: quaternion_demo.moc
	cd /home/m16a/Documents/phys/eduPhys && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/m16a/Documents/phys/eduPhys /home/m16a/Documents/phys/eduPhys /home/m16a/Documents/phys/eduPhys /home/m16a/Documents/phys/eduPhys /home/m16a/Documents/phys/eduPhys/CMakeFiles/quaternion_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quaternion_demo.dir/depend

