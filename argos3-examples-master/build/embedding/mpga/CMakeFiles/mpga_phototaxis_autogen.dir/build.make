# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/lin/argos3/argos3-examples-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lin/argos3/argos3-examples-master/build

# Utility rule file for mpga_phototaxis_autogen.

# Include the progress variables for this target.
include embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/progress.make

embedding/mpga/CMakeFiles/mpga_phototaxis_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/lin/argos3/argos3-examples-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target mpga_phototaxis"
	cd /home/lin/argos3/argos3-examples-master/build/embedding/mpga && /usr/bin/cmake -E cmake_autogen /home/lin/argos3/argos3-examples-master/build/embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/AutogenInfo.json Release

mpga_phototaxis_autogen: embedding/mpga/CMakeFiles/mpga_phototaxis_autogen
mpga_phototaxis_autogen: embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/build.make

.PHONY : mpga_phototaxis_autogen

# Rule to build all files generated by this target.
embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/build: mpga_phototaxis_autogen

.PHONY : embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/build

embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/clean:
	cd /home/lin/argos3/argos3-examples-master/build/embedding/mpga && $(CMAKE_COMMAND) -P CMakeFiles/mpga_phototaxis_autogen.dir/cmake_clean.cmake
.PHONY : embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/clean

embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/depend:
	cd /home/lin/argos3/argos3-examples-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lin/argos3/argos3-examples-master /home/lin/argos3/argos3-examples-master/embedding/mpga /home/lin/argos3/argos3-examples-master/build /home/lin/argos3/argos3-examples-master/build/embedding/mpga /home/lin/argos3/argos3-examples-master/build/embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : embedding/mpga/CMakeFiles/mpga_phototaxis_autogen.dir/depend

