# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.24

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "D:\Program Files\JetBrains\CLion 2022.3.2\bin\cmake\win\x64\bin\cmake.exe"

# The command to remove a file.
RM = "D:\Program Files\JetBrains\CLion 2022.3.2\bin\cmake\win\x64\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "D:\Documents\School\Computer Graphics\graphics-engine"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/engine.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/engine.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/engine.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/engine.dir/flags.make

CMakeFiles/engine.dir/src/utils/easy_image.cc.obj: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/src/utils/easy_image.cc.obj: D:/Documents/School/Computer\ Graphics/graphics-engine/src/utils/easy_image.cc
CMakeFiles/engine.dir/src/utils/easy_image.cc.obj: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/engine.dir/src/utils/easy_image.cc.obj"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/src/utils/easy_image.cc.obj -MF CMakeFiles\engine.dir\src\utils\easy_image.cc.obj.d -o CMakeFiles\engine.dir\src\utils\easy_image.cc.obj -c "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\easy_image.cc"

CMakeFiles/engine.dir/src/utils/easy_image.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/src/utils/easy_image.cc.i"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\easy_image.cc" > CMakeFiles\engine.dir\src\utils\easy_image.cc.i

CMakeFiles/engine.dir/src/utils/easy_image.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/src/utils/easy_image.cc.s"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\easy_image.cc" -o CMakeFiles\engine.dir\src\utils\easy_image.cc.s

CMakeFiles/engine.dir/src/engine.cc.obj: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/src/engine.cc.obj: D:/Documents/School/Computer\ Graphics/graphics-engine/src/engine.cc
CMakeFiles/engine.dir/src/engine.cc.obj: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/engine.dir/src/engine.cc.obj"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/src/engine.cc.obj -MF CMakeFiles\engine.dir\src\engine.cc.obj.d -o CMakeFiles\engine.dir\src\engine.cc.obj -c "D:\Documents\School\Computer Graphics\graphics-engine\src\engine.cc"

CMakeFiles/engine.dir/src/engine.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/src/engine.cc.i"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "D:\Documents\School\Computer Graphics\graphics-engine\src\engine.cc" > CMakeFiles\engine.dir\src\engine.cc.i

CMakeFiles/engine.dir/src/engine.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/src/engine.cc.s"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "D:\Documents\School\Computer Graphics\graphics-engine\src\engine.cc" -o CMakeFiles\engine.dir\src\engine.cc.s

CMakeFiles/engine.dir/src/utils/ini_configuration.cc.obj: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/src/utils/ini_configuration.cc.obj: D:/Documents/School/Computer\ Graphics/graphics-engine/src/utils/ini_configuration.cc
CMakeFiles/engine.dir/src/utils/ini_configuration.cc.obj: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/engine.dir/src/utils/ini_configuration.cc.obj"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/src/utils/ini_configuration.cc.obj -MF CMakeFiles\engine.dir\src\utils\ini_configuration.cc.obj.d -o CMakeFiles\engine.dir\src\utils\ini_configuration.cc.obj -c "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\ini_configuration.cc"

CMakeFiles/engine.dir/src/utils/ini_configuration.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/src/utils/ini_configuration.cc.i"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\ini_configuration.cc" > CMakeFiles\engine.dir\src\utils\ini_configuration.cc.i

CMakeFiles/engine.dir/src/utils/ini_configuration.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/src/utils/ini_configuration.cc.s"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\ini_configuration.cc" -o CMakeFiles\engine.dir\src\utils\ini_configuration.cc.s

CMakeFiles/engine.dir/src/LSystems/LSystems.cc.obj: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/src/LSystems/LSystems.cc.obj: D:/Documents/School/Computer\ Graphics/graphics-engine/src/LSystems/LSystems.cc
CMakeFiles/engine.dir/src/LSystems/LSystems.cc.obj: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/engine.dir/src/LSystems/LSystems.cc.obj"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/src/LSystems/LSystems.cc.obj -MF CMakeFiles\engine.dir\src\LSystems\LSystems.cc.obj.d -o CMakeFiles\engine.dir\src\LSystems\LSystems.cc.obj -c "D:\Documents\School\Computer Graphics\graphics-engine\src\LSystems\LSystems.cc"

CMakeFiles/engine.dir/src/LSystems/LSystems.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/src/LSystems/LSystems.cc.i"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "D:\Documents\School\Computer Graphics\graphics-engine\src\LSystems\LSystems.cc" > CMakeFiles\engine.dir\src\LSystems\LSystems.cc.i

CMakeFiles/engine.dir/src/LSystems/LSystems.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/src/LSystems/LSystems.cc.s"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "D:\Documents\School\Computer Graphics\graphics-engine\src\LSystems\LSystems.cc" -o CMakeFiles\engine.dir\src\LSystems\LSystems.cc.s

CMakeFiles/engine.dir/src/utils/draw2DLines.cc.obj: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/src/utils/draw2DLines.cc.obj: D:/Documents/School/Computer\ Graphics/graphics-engine/src/utils/draw2DLines.cc
CMakeFiles/engine.dir/src/utils/draw2DLines.cc.obj: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/engine.dir/src/utils/draw2DLines.cc.obj"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/src/utils/draw2DLines.cc.obj -MF CMakeFiles\engine.dir\src\utils\draw2DLines.cc.obj.d -o CMakeFiles\engine.dir\src\utils\draw2DLines.cc.obj -c "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\draw2DLines.cc"

CMakeFiles/engine.dir/src/utils/draw2DLines.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/src/utils/draw2DLines.cc.i"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\draw2DLines.cc" > CMakeFiles\engine.dir\src\utils\draw2DLines.cc.i

CMakeFiles/engine.dir/src/utils/draw2DLines.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/src/utils/draw2DLines.cc.s"
	"D:\Program Files\JetBrains\CLion 2022.3.2\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "D:\Documents\School\Computer Graphics\graphics-engine\src\utils\draw2DLines.cc" -o CMakeFiles\engine.dir\src\utils\draw2DLines.cc.s

# Object files for target engine
engine_OBJECTS = \
"CMakeFiles/engine.dir/src/utils/easy_image.cc.obj" \
"CMakeFiles/engine.dir/src/engine.cc.obj" \
"CMakeFiles/engine.dir/src/utils/ini_configuration.cc.obj" \
"CMakeFiles/engine.dir/src/LSystems/LSystems.cc.obj" \
"CMakeFiles/engine.dir/src/utils/draw2DLines.cc.obj"

# External object files for target engine
engine_EXTERNAL_OBJECTS =

engine.exe: CMakeFiles/engine.dir/src/utils/easy_image.cc.obj
engine.exe: CMakeFiles/engine.dir/src/engine.cc.obj
engine.exe: CMakeFiles/engine.dir/src/utils/ini_configuration.cc.obj
engine.exe: CMakeFiles/engine.dir/src/LSystems/LSystems.cc.obj
engine.exe: CMakeFiles/engine.dir/src/utils/draw2DLines.cc.obj
engine.exe: CMakeFiles/engine.dir/build.make
engine.exe: CMakeFiles/engine.dir/linklibs.rsp
engine.exe: CMakeFiles/engine.dir/objects1.rsp
engine.exe: CMakeFiles/engine.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable engine.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\engine.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/engine.dir/build: engine.exe
.PHONY : CMakeFiles/engine.dir/build

CMakeFiles/engine.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\engine.dir\cmake_clean.cmake
.PHONY : CMakeFiles/engine.dir/clean

CMakeFiles/engine.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" "D:\Documents\School\Computer Graphics\graphics-engine" "D:\Documents\School\Computer Graphics\graphics-engine" "D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug" "D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug" "D:\Documents\School\Computer Graphics\graphics-engine\cmake-build-debug\CMakeFiles\engine.dir\DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/engine.dir/depend

