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
include _deps/libwebsockets-build/CMakeFiles/test-server.dir/depend.make

# Include the progress variables for this target.
include _deps/libwebsockets-build/CMakeFiles/test-server.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/libwebsockets-build/CMakeFiles/test-server.dir/flags.make

_deps/libwebsockets-build/CMakeFiles/test-server.dir/test-apps/test-server.c.o: _deps/libwebsockets-build/CMakeFiles/test-server.dir/flags.make
_deps/libwebsockets-build/CMakeFiles/test-server.dir/test-apps/test-server.c.o: _deps/libwebsockets-src/test-apps/test-server.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object _deps/libwebsockets-build/CMakeFiles/test-server.dir/test-apps/test-server.c.o"
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test-server.dir/test-apps/test-server.c.o   -c /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/test-server.c

_deps/libwebsockets-build/CMakeFiles/test-server.dir/test-apps/test-server.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test-server.dir/test-apps/test-server.c.i"
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/test-server.c > CMakeFiles/test-server.dir/test-apps/test-server.c.i

_deps/libwebsockets-build/CMakeFiles/test-server.dir/test-apps/test-server.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test-server.dir/test-apps/test-server.c.s"
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/test-server.c -o CMakeFiles/test-server.dir/test-apps/test-server.c.s

# Object files for target test-server
test__server_OBJECTS = \
"CMakeFiles/test-server.dir/test-apps/test-server.c.o"

# External object files for target test-server
test__server_EXTERNAL_OBJECTS =

_deps/libwebsockets-build/bin/libwebsockets-test-server: _deps/libwebsockets-build/CMakeFiles/test-server.dir/test-apps/test-server.c.o
_deps/libwebsockets-build/bin/libwebsockets-test-server: _deps/libwebsockets-build/CMakeFiles/test-server.dir/build.make
_deps/libwebsockets-build/bin/libwebsockets-test-server: _deps/libwebsockets-build/lib/libwebsockets.a
_deps/libwebsockets-build/bin/libwebsockets-test-server: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libssl.so
_deps/libwebsockets-build/bin/libwebsockets-test-server: /home/abobco/RaspberryPi/sysroot/usr/lib/arm-linux-gnueabihf/libcrypto.so
_deps/libwebsockets-build/bin/libwebsockets-test-server: _deps/libwebsockets-build/CMakeFiles/test-server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable bin/libwebsockets-test-server"
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-server.dir/link.txt --verbose=$(VERBOSE)
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E make_directory /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/libwebsockets-test-server.key.pem /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/libwebsockets-test-server.pem /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/favicon.ico /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/leaf.jpg /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/candide.zip /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/libwebsockets.org-logo.svg /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/http2.png /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/wss-over-h2.png /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/lws-common.js /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/test.html /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/test.css /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && /usr/bin/cmake -E copy /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src/test-apps/test.js /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/bin/../share/libwebsockets-test-server

# Rule to build all files generated by this target.
_deps/libwebsockets-build/CMakeFiles/test-server.dir/build: _deps/libwebsockets-build/bin/libwebsockets-test-server

.PHONY : _deps/libwebsockets-build/CMakeFiles/test-server.dir/build

_deps/libwebsockets-build/CMakeFiles/test-server.dir/clean:
	cd /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build && $(CMAKE_COMMAND) -P CMakeFiles/test-server.dir/cmake_clean.cmake
.PHONY : _deps/libwebsockets-build/CMakeFiles/test-server.dir/clean

_deps/libwebsockets-build/CMakeFiles/test-server.dir/depend:
	cd /home/abobco/RaspberryPi/OCV_Projects/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-src /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build /home/abobco/RaspberryPi/OCV_Projects/build/_deps/libwebsockets-build/CMakeFiles/test-server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/libwebsockets-build/CMakeFiles/test-server.dir/depend

