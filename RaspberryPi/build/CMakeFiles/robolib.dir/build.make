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
include CMakeFiles/robolib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/robolib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robolib.dir/flags.make

CMakeFiles/robolib.dir/src/ws/message_queue.c.o: CMakeFiles/robolib.dir/flags.make
CMakeFiles/robolib.dir/src/ws/message_queue.c.o: ../src/ws/message_queue.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/robolib.dir/src/ws/message_queue.c.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/robolib.dir/src/ws/message_queue.c.o   -c /home/abobco/RaspberryPi/OCV_Projects/src/ws/message_queue.c

CMakeFiles/robolib.dir/src/ws/message_queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robolib.dir/src/ws/message_queue.c.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/src/ws/message_queue.c > CMakeFiles/robolib.dir/src/ws/message_queue.c.i

CMakeFiles/robolib.dir/src/ws/message_queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robolib.dir/src/ws/message_queue.c.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/src/ws/message_queue.c -o CMakeFiles/robolib.dir/src/ws/message_queue.c.s

CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.o: CMakeFiles/robolib.dir/flags.make
CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.o: ../SPI_Camera/src/EZCAM.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.o   -c /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/EZCAM.c

CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/EZCAM.c > CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.i

CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/EZCAM.c -o CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.s

CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.o: CMakeFiles/robolib.dir/flags.make
CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.o: ../SPI_Camera/src/ArduCAM.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.o   -c /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/ArduCAM.c

CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/ArduCAM.c > CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.i

CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/ArduCAM.c -o CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.s

CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.o: CMakeFiles/robolib.dir/flags.make
CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.o: ../SPI_Camera/src/bcm283x_board_driver.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.o   -c /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/bcm283x_board_driver.c

CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/bcm283x_board_driver.c > CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.i

CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/bcm283x_board_driver.c -o CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.s

CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.o: CMakeFiles/robolib.dir/flags.make
CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.o: ../SPI_Camera/src/sccb_bus.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.o"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.o   -c /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/sccb_bus.c

CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.i"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/sccb_bus.c > CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.i

CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.s"
	/home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --sysroot=/home/abobco/RaspberryPi/sysroot $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/sccb_bus.c -o CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.s

# Object files for target robolib
robolib_OBJECTS = \
"CMakeFiles/robolib.dir/src/ws/message_queue.c.o" \
"CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.o" \
"CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.o" \
"CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.o" \
"CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.o"

# External object files for target robolib
robolib_EXTERNAL_OBJECTS =

librobolib.a: CMakeFiles/robolib.dir/src/ws/message_queue.c.o
librobolib.a: CMakeFiles/robolib.dir/SPI_Camera/src/EZCAM.c.o
librobolib.a: CMakeFiles/robolib.dir/SPI_Camera/src/ArduCAM.c.o
librobolib.a: CMakeFiles/robolib.dir/SPI_Camera/src/bcm283x_board_driver.c.o
librobolib.a: CMakeFiles/robolib.dir/SPI_Camera/src/sccb_bus.c.o
librobolib.a: CMakeFiles/robolib.dir/build.make
librobolib.a: CMakeFiles/robolib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C static library librobolib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/robolib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robolib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robolib.dir/build: librobolib.a

.PHONY : CMakeFiles/robolib.dir/build

CMakeFiles/robolib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robolib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robolib.dir/clean

CMakeFiles/robolib.dir/depend:
	cd /home/abobco/RaspberryPi/OCV_Projects/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build /home/abobco/RaspberryPi/OCV_Projects/build/CMakeFiles/robolib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robolib.dir/depend

