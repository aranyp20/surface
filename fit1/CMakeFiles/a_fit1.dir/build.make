# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.29.2/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.29.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/peti/surface/fit1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/peti/surface/fit1

# Include any dependencies generated for this target.
include CMakeFiles/a_fit1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/a_fit1.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/a_fit1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/a_fit1.dir/flags.make

ui_mainwindow.h: src/visualization/qt/mainwindow.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_mainwindow.h"
	/opt/homebrew/bin/uic -o /Users/peti/surface/fit1/ui_mainwindow.h /Users/peti/surface/fit1/src/visualization/qt/mainwindow.ui

a_fit1_autogen/timestamp: /opt/homebrew/bin/moc
a_fit1_autogen/timestamp: /opt/homebrew/bin/uic
a_fit1_autogen/timestamp: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Automatic MOC and UIC for target a_fit1"
	/opt/homebrew/Cellar/cmake/3.29.2/bin/cmake -E cmake_autogen /Users/peti/surface/fit1/CMakeFiles/a_fit1_autogen.dir/AutogenInfo.json Release
	/opt/homebrew/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/peti/surface/fit1/a_fit1_autogen/timestamp

CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o: a_fit1_autogen/mocs_compilation.cpp
CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o -MF CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o -c /Users/peti/surface/fit1/a_fit1_autogen/mocs_compilation.cpp

CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/a_fit1_autogen/mocs_compilation.cpp > CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.i

CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/a_fit1_autogen/mocs_compilation.cpp -o CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.s

CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o: src/visualization/qt/main.cpp
CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o -MF CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o.d -o CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o -c /Users/peti/surface/fit1/src/visualization/qt/main.cpp

CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/src/visualization/qt/main.cpp > CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.i

CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/src/visualization/qt/main.cpp -o CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.s

CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o: src/visualization/qt/mainwindow.cpp
CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o -MF CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o.d -o CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o -c /Users/peti/surface/fit1/src/visualization/qt/mainwindow.cpp

CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/src/visualization/qt/mainwindow.cpp > CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.i

CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/src/visualization/qt/mainwindow.cpp -o CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.s

CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o: src/visualization/src/canvas.cpp
CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o -MF CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o.d -o CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o -c /Users/peti/surface/fit1/src/visualization/src/canvas.cpp

CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/src/visualization/src/canvas.cpp > CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.i

CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/src/visualization/src/canvas.cpp -o CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.s

CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o: src/visualization/src/camera.cpp
CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o -MF CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o.d -o CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o -c /Users/peti/surface/fit1/src/visualization/src/camera.cpp

CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/src/visualization/src/camera.cpp > CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.i

CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/src/visualization/src/camera.cpp -o CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.s

CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o: src/core/src/curvaturecalculator.cpp
CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o -MF CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o.d -o CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o -c /Users/peti/surface/fit1/src/core/src/curvaturecalculator.cpp

CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/src/core/src/curvaturecalculator.cpp > CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.i

CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/src/core/src/curvaturecalculator.cpp -o CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.s

CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o: src/framework/src/objectloader.cpp
CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o -MF CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o.d -o CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o -c /Users/peti/surface/fit1/src/framework/src/objectloader.cpp

CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/src/framework/src/objectloader.cpp > CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.i

CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/src/framework/src/objectloader.cpp -o CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.s

CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o: src/core/src/discretefairer.cpp
CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o -MF CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o.d -o CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o -c /Users/peti/surface/fit1/src/core/src/discretefairer.cpp

CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/src/core/src/discretefairer.cpp > CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.i

CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/src/core/src/discretefairer.cpp -o CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.s

CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o: CMakeFiles/a_fit1.dir/flags.make
CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o: dependencies/lsq-plane.cc
CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o: CMakeFiles/a_fit1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o -MF CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o.d -o CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o -c /Users/peti/surface/fit1/dependencies/lsq-plane.cc

CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/peti/surface/fit1/dependencies/lsq-plane.cc > CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.i

CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/peti/surface/fit1/dependencies/lsq-plane.cc -o CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.s

# Object files for target a_fit1
a_fit1_OBJECTS = \
"CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o" \
"CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o" \
"CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o" \
"CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o" \
"CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o" \
"CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o" \
"CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o" \
"CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o"

# External object files for target a_fit1
a_fit1_EXTERNAL_OBJECTS =

a_fit1: CMakeFiles/a_fit1.dir/a_fit1_autogen/mocs_compilation.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/visualization/qt/main.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/visualization/qt/mainwindow.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/visualization/src/canvas.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/visualization/src/camera.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/core/src/curvaturecalculator.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/framework/src/objectloader.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/src/core/src/discretefairer.cpp.o
a_fit1: CMakeFiles/a_fit1.dir/dependencies/lsq-plane.cc.o
a_fit1: CMakeFiles/a_fit1.dir/build.make
a_fit1: /Library/Developer/CommandLineTools/SDKs/MacOSX14.4.sdk/System/Library/Frameworks/OpenGL.framework
a_fit1: /opt/homebrew/lib/QtWidgets.framework/QtWidgets
a_fit1: /opt/homebrew/lib/libOpenMeshCore.dylib
a_fit1: /opt/homebrew/lib/libOpenMeshTools.dylib
a_fit1: /opt/homebrew/lib/QtGui.framework/QtGui
a_fit1: /opt/homebrew/lib/QtCore.framework/QtCore
a_fit1: CMakeFiles/a_fit1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/peti/surface/fit1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX executable a_fit1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a_fit1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/a_fit1.dir/build: a_fit1
.PHONY : CMakeFiles/a_fit1.dir/build

CMakeFiles/a_fit1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/a_fit1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/a_fit1.dir/clean

CMakeFiles/a_fit1.dir/depend: a_fit1_autogen/timestamp
CMakeFiles/a_fit1.dir/depend: ui_mainwindow.h
	cd /Users/peti/surface/fit1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/peti/surface/fit1 /Users/peti/surface/fit1 /Users/peti/surface/fit1 /Users/peti/surface/fit1 /Users/peti/surface/fit1/CMakeFiles/a_fit1.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/a_fit1.dir/depend

