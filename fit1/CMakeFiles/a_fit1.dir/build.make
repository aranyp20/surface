# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peti/surface/fit1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peti/surface/fit1

# Include any dependencies generated for this target.
include CMakeFiles/a_fit1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/a_fit1.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/a_fit1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/a_fit1.dir/flags.make

ui_mainwindow.h: src/visualization/qt/mainwindow.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_mainwindow.h"
	/usr/lib/qt5/bin/uic -o /home/peti/surface/fit1/ui_mainwindow.h /home/peti/surface/fit1/src/visualization/qt/mainwindow.ui

CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o: a_fit1_autogen/mocs_compilation.cpp
CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o -MF CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o -c /home/peti/surface/fit1/a_fit1_autogen/mocs_compilation.cpp

CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peti/surface/fit1/a_fit1_autogen/mocs_compilation.cpp > CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.i

CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peti/surface/fit1/a_fit1_autogen/mocs_compilation.cpp -o CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.s

CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o: src/visualization/qt/main.cpp
CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o -MF CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o.d -o CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o -c /home/peti/surface/fit1/src/visualization/qt/main.cpp

CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peti/surface/fit1/src/visualization/qt/main.cpp > CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.i

CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peti/surface/fit1/src/visualization/qt/main.cpp -o CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.s

CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o: src/visualization/qt/mainwindow.cpp
CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o -MF CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o.d -o CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o -c /home/peti/surface/fit1/src/visualization/qt/mainwindow.cpp

CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peti/surface/fit1/src/visualization/qt/mainwindow.cpp > CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.i

CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peti/surface/fit1/src/visualization/qt/mainwindow.cpp -o CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.s

CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o: src/visualization/src/canvas.cpp
CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o -MF CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o.d -o CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o -c /home/peti/surface/fit1/src/visualization/src/canvas.cpp

CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peti/surface/fit1/src/visualization/src/canvas.cpp > CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.i

CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peti/surface/fit1/src/visualization/src/canvas.cpp -o CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.s

CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o: src/core/src/discretefairer.cpp
CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o -MF CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o.d -o CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o -c /home/peti/surface/fit1/src/core/src/discretefairer.cpp

CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peti/surface/fit1/src/core/src/discretefairer.cpp > CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.i

CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peti/surface/fit1/src/core/src/discretefairer.cpp -o CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.s

# Object files for target a_fit1
a_fit1_OBJECTS = \
"CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o" \
"CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o" \
"CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o" \
"CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o"

# External object files for target a_fit1
a_fit1_EXTERNAL_OBJECTS =

a_fit1: CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/build.make
a_fit1: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
a_fit1: /usr/local/lib/libOpenMeshCore.so
a_fit1: /usr/local/lib/libOpenMeshTools.so
a_fit1: /usr/lib/x86_64-linux-gnu/libGLX.so
a_fit1: /usr/lib/x86_64-linux-gnu/libOpenGL.so
a_fit1: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
a_fit1: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
a_fit1: CMakeFiles/a_fit1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable a_fit1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a_fit1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/a_fit1.dir/build: a_fit1
.PHONY : CMakeFiles/a_fit1.dir/build

CMakeFiles/a_fit1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/a_fit1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/a_fit1.dir/clean

CMakeFiles/a_fit1.dir/depend: ui_mainwindow.h
	cd /home/peti/surface/fit1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peti/surface/fit1 /home/peti/surface/fit1 /home/peti/surface/fit1 /home/peti/surface/fit1 /home/peti/surface/fit1/CMakeFiles/a_fit1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/a_fit1.dir/depend

