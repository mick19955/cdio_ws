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
CMAKE_SOURCE_DIR = /home/cdio/cdio_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cdio/cdio_ws/build

# Include any dependencies generated for this target.
include flight_controller/CMakeFiles/flight_controller.dir/depend.make

# Include the progress variables for this target.
include flight_controller/CMakeFiles/flight_controller.dir/progress.make

# Include the compile flags for this target's objects.
include flight_controller/CMakeFiles/flight_controller.dir/flags.make

flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o: flight_controller/CMakeFiles/flight_controller.dir/flags.make
flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o: /home/cdio/cdio_ws/src/flight_controller/src/flight_main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cdio/cdio_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o"
	cd /home/cdio/cdio_ws/build/flight_controller && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/flight_controller.dir/src/flight_main.cpp.o -c /home/cdio/cdio_ws/src/flight_controller/src/flight_main.cpp

flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flight_controller.dir/src/flight_main.cpp.i"
	cd /home/cdio/cdio_ws/build/flight_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cdio/cdio_ws/src/flight_controller/src/flight_main.cpp > CMakeFiles/flight_controller.dir/src/flight_main.cpp.i

flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flight_controller.dir/src/flight_main.cpp.s"
	cd /home/cdio/cdio_ws/build/flight_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cdio/cdio_ws/src/flight_controller/src/flight_main.cpp -o CMakeFiles/flight_controller.dir/src/flight_main.cpp.s

flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.requires:
.PHONY : flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.requires

flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.provides: flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.requires
	$(MAKE) -f flight_controller/CMakeFiles/flight_controller.dir/build.make flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.provides.build
.PHONY : flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.provides

flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.provides.build: flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o

# Object files for target flight_controller
flight_controller_OBJECTS = \
"CMakeFiles/flight_controller.dir/src/flight_main.cpp.o"

# External object files for target flight_controller
flight_controller_EXTERNAL_OBJECTS =

/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: flight_controller/CMakeFiles/flight_controller.dir/build.make
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libcv_bridge.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libimage_transport.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libmessage_filters.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libclass_loader.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/libPocoFoundation.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libdl.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libroslib.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/librospack.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libroscpp.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/librosconsole.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/liblog4cxx.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/librostime.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /opt/ros/indigo/lib/libcpp_common.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller: flight_controller/CMakeFiles/flight_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller"
	cd /home/cdio/cdio_ws/build/flight_controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/flight_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
flight_controller/CMakeFiles/flight_controller.dir/build: /home/cdio/cdio_ws/devel/lib/flight_controller/flight_controller
.PHONY : flight_controller/CMakeFiles/flight_controller.dir/build

flight_controller/CMakeFiles/flight_controller.dir/requires: flight_controller/CMakeFiles/flight_controller.dir/src/flight_main.cpp.o.requires
.PHONY : flight_controller/CMakeFiles/flight_controller.dir/requires

flight_controller/CMakeFiles/flight_controller.dir/clean:
	cd /home/cdio/cdio_ws/build/flight_controller && $(CMAKE_COMMAND) -P CMakeFiles/flight_controller.dir/cmake_clean.cmake
.PHONY : flight_controller/CMakeFiles/flight_controller.dir/clean

flight_controller/CMakeFiles/flight_controller.dir/depend:
	cd /home/cdio/cdio_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cdio/cdio_ws/src /home/cdio/cdio_ws/src/flight_controller /home/cdio/cdio_ws/build /home/cdio/cdio_ws/build/flight_controller /home/cdio/cdio_ws/build/flight_controller/CMakeFiles/flight_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : flight_controller/CMakeFiles/flight_controller.dir/depend

