# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/abobco/RaspberryPi/OCV_Projects

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abobco/RaspberryPi/OCV_Projects/build

# Include any dependencies generated for this target.
include CMakeFiles/xn_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/xn_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xn_test.dir/flags.make

CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.o: CMakeFiles/xn_test.dir/flags.make
CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.o: ../src/XNlib/xn_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.o -c /home/abobco/RaspberryPi/OCV_Projects/src/XNlib/xn_test.cpp

CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/src/XNlib/xn_test.cpp > CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.i

CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/src/XNlib/xn_test.cpp -o CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.s

# Object files for target xn_test
xn_test_OBJECTS = \
"CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.o"

# External object files for target xn_test
xn_test_EXTERNAL_OBJECTS =

xn_test: CMakeFiles/xn_test.dir/src/XNlib/xn_test.cpp.o
xn_test: CMakeFiles/xn_test.dir/build.make
xn_test: /usr/local/lib/libopencv_dnn.so.4.5.0
xn_test: /usr/local/lib/libopencv_gapi.so.4.5.0
xn_test: /usr/local/lib/libopencv_highgui.so.4.5.0
xn_test: /usr/local/lib/libopencv_ml.so.4.5.0
xn_test: /usr/local/lib/libopencv_objdetect.so.4.5.0
xn_test: /usr/local/lib/libopencv_photo.so.4.5.0
xn_test: /usr/local/lib/libopencv_stitching.so.4.5.0
xn_test: /usr/local/lib/libopencv_video.so.4.5.0
xn_test: /usr/local/lib/libopencv_videoio.so.4.5.0
xn_test: /home/abobco/RaspberryPi/pigpio-master/install/lib/libpigpio.so
xn_test: /home/abobco/RaspberryPi/pigpio-master/install/lib/libpigpiod_if2.so
xn_test: librobolib.a
xn_test: _deps/libwebsockets-build/lib/libwebsockets.a
xn_test: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
xn_test: /usr/local/lib/libopencv_calib3d.so.4.5.0
xn_test: /usr/local/lib/libopencv_features2d.so.4.5.0
xn_test: /usr/local/lib/libopencv_flann.so.4.5.0
xn_test: /usr/local/lib/libopencv_imgproc.so.4.5.0
xn_test: /usr/local/lib/libopencv_core.so.4.5.0
xn_test: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libssl.so
xn_test: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libcrypto.so
xn_test: CMakeFiles/xn_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable xn_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xn_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xn_test.dir/build: xn_test

.PHONY : CMakeFiles/xn_test.dir/build

CMakeFiles/xn_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xn_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xn_test.dir/clean

CMakeFiles/xn_test.dir/depend:
	cd /home/abobco/RaspberryPi/OCV_Projects/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles/xn_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xn_test.dir/depend

