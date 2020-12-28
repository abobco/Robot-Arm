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
include CMakeFiles/wavetimer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/wavetimer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/wavetimer.dir/flags.make

CMakeFiles/wavetimer.dir/src/wavetimer.cpp.o: CMakeFiles/wavetimer.dir/flags.make
CMakeFiles/wavetimer.dir/src/wavetimer.cpp.o: ../src/wavetimer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/wavetimer.dir/src/wavetimer.cpp.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/wavetimer.dir/src/wavetimer.cpp.o -c /home/abobco/RaspberryPi/OCV_Projects/src/wavetimer.cpp

CMakeFiles/wavetimer.dir/src/wavetimer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/wavetimer.dir/src/wavetimer.cpp.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/src/wavetimer.cpp > CMakeFiles/wavetimer.dir/src/wavetimer.cpp.i

CMakeFiles/wavetimer.dir/src/wavetimer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/wavetimer.dir/src/wavetimer.cpp.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/src/wavetimer.cpp -o CMakeFiles/wavetimer.dir/src/wavetimer.cpp.s

# Object files for target wavetimer
wavetimer_OBJECTS = \
"CMakeFiles/wavetimer.dir/src/wavetimer.cpp.o"

# External object files for target wavetimer
wavetimer_EXTERNAL_OBJECTS =

wavetimer: CMakeFiles/wavetimer.dir/src/wavetimer.cpp.o
wavetimer: CMakeFiles/wavetimer.dir/build.make
wavetimer: /usr/local/lib/libopencv_dnn.so.4.5.0
wavetimer: /usr/local/lib/libopencv_gapi.so.4.5.0
wavetimer: /usr/local/lib/libopencv_highgui.so.4.5.0
wavetimer: /usr/local/lib/libopencv_ml.so.4.5.0
wavetimer: /usr/local/lib/libopencv_objdetect.so.4.5.0
wavetimer: /usr/local/lib/libopencv_photo.so.4.5.0
wavetimer: /usr/local/lib/libopencv_stitching.so.4.5.0
wavetimer: /usr/local/lib/libopencv_video.so.4.5.0
wavetimer: /usr/local/lib/libopencv_videoio.so.4.5.0
wavetimer: /home/abobco/RaspberryPi/pigpio-master/install/lib/libpigpio.so
wavetimer: /home/abobco/RaspberryPi/pigpio-master/install/lib/libpigpiod_if2.so
wavetimer: librobolib.a
wavetimer: _deps/libwebsockets-build/lib/libwebsockets.a
wavetimer: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libvcos.so
wavetimer: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libbrcmEGL.so
wavetimer: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libbrcmGLESv2.so
wavetimer: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libbcm_host.so
wavetimer: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libvchiq_arm.so
wavetimer: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
wavetimer: /usr/local/lib/libopencv_calib3d.so.4.5.0
wavetimer: /usr/local/lib/libopencv_features2d.so.4.5.0
wavetimer: /usr/local/lib/libopencv_flann.so.4.5.0
wavetimer: /usr/local/lib/libopencv_imgproc.so.4.5.0
wavetimer: /usr/local/lib/libopencv_core.so.4.5.0
wavetimer: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libssl.so
wavetimer: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libcrypto.so
wavetimer: CMakeFiles/wavetimer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable wavetimer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/wavetimer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/wavetimer.dir/build: wavetimer

.PHONY : CMakeFiles/wavetimer.dir/build

CMakeFiles/wavetimer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/wavetimer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/wavetimer.dir/clean

CMakeFiles/wavetimer.dir/depend:
	cd /home/abobco/RaspberryPi/OCV_Projects/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles/wavetimer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/wavetimer.dir/depend

