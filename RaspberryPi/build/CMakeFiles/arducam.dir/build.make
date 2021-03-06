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
include CMakeFiles/arducam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/arducam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/arducam.dir/flags.make

CMakeFiles/arducam.dir/src/arducam.cpp.o: CMakeFiles/arducam.dir/flags.make
CMakeFiles/arducam.dir/src/arducam.cpp.o: ../src/arducam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/arducam.dir/src/arducam.cpp.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/arducam.dir/src/arducam.cpp.o -c /home/abobco/RaspberryPi/OCV_Projects/src/arducam.cpp

CMakeFiles/arducam.dir/src/arducam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/arducam.dir/src/arducam.cpp.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/src/arducam.cpp > CMakeFiles/arducam.dir/src/arducam.cpp.i

CMakeFiles/arducam.dir/src/arducam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/arducam.dir/src/arducam.cpp.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++ --sysroot=/home/abobco/RaspberryPi/sysroot $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/src/arducam.cpp -o CMakeFiles/arducam.dir/src/arducam.cpp.s

# Object files for target arducam
arducam_OBJECTS = \
"CMakeFiles/arducam.dir/src/arducam.cpp.o"

# External object files for target arducam
arducam_EXTERNAL_OBJECTS =

arducam: CMakeFiles/arducam.dir/src/arducam.cpp.o
arducam: CMakeFiles/arducam.dir/build.make
arducam: /usr/local/lib/libopencv_dnn.so.4.5.0
arducam: /usr/local/lib/libopencv_gapi.so.4.5.0
arducam: /usr/local/lib/libopencv_highgui.so.4.5.0
arducam: /usr/local/lib/libopencv_ml.so.4.5.0
arducam: /usr/local/lib/libopencv_objdetect.so.4.5.0
arducam: /usr/local/lib/libopencv_photo.so.4.5.0
arducam: /usr/local/lib/libopencv_stitching.so.4.5.0
arducam: /usr/local/lib/libopencv_video.so.4.5.0
arducam: /usr/local/lib/libopencv_videoio.so.4.5.0
arducam: /home/abobco/RaspberryPi/pigpio-master/install/lib/libpigpio.so
arducam: /home/abobco/RaspberryPi/pigpio-master/install/lib/libpigpiod_if2.so
arducam: librobolib.a
arducam: _deps/libwebsockets-build/lib/libwebsockets.a
arducam: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libvcos.so
arducam: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libbrcmEGL.so
arducam: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libbrcmGLESv2.so
arducam: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libbcm_host.so
arducam: /home/abobco/RaspberryPi/sysroot/opt/vc/lib/libvchiq_arm.so
arducam: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
arducam: /usr/local/lib/libopencv_calib3d.so.4.5.0
arducam: /usr/local/lib/libopencv_features2d.so.4.5.0
arducam: /usr/local/lib/libopencv_flann.so.4.5.0
arducam: /usr/local/lib/libopencv_imgproc.so.4.5.0
arducam: /usr/local/lib/libopencv_core.so.4.5.0
arducam: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libssl.so
arducam: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libcrypto.so
arducam: CMakeFiles/arducam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable arducam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/arducam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/arducam.dir/build: arducam

.PHONY : CMakeFiles/arducam.dir/build

CMakeFiles/arducam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/arducam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/arducam.dir/clean

CMakeFiles/arducam.dir/depend:
	cd /home/abobco/RaspberryPi/OCV_Projects/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles/arducam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/arducam.dir/depend

