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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/noah/sproject/build

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/noah/sproject/build

# Include any dependencies generated for this target.
include CMakeFiles/senior.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/senior.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/senior.dir/flags.make

CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o: /home/noah/sproject/source/Track.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o -c /home/noah/sproject/source/Track.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Track.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Track.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o: /home/noah/sproject/source/WorldObject.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o -c /home/noah/sproject/source/WorldObject.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/WorldObject.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/WorldObject.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o: /home/noah/sproject/source/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o -c /home/noah/sproject/source/main.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/main.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/main.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o: /home/noah/sproject/source/Scene.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o -c /home/noah/sproject/source/Scene.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Scene.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Scene.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o: /home/noah/sproject/source/Vehicle.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o -c /home/noah/sproject/source/Vehicle.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Vehicle.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Vehicle.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o: /home/noah/sproject/source/ShapeObj.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o -c /home/noah/sproject/source/ShapeObj.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/ShapeObj.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/ShapeObj.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o: /home/noah/sproject/source/Texture.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o -c /home/noah/sproject/source/Texture.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Texture.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Texture.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o: /home/noah/sproject/source/Image.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o -c /home/noah/sproject/source/Image.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Image.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Image.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o: /home/noah/sproject/source/CollisionBox.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o -c /home/noah/sproject/source/CollisionBox.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/CollisionBox.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/CollisionBox.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o: /home/noah/sproject/source/Shape.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o -c /home/noah/sproject/source/Shape.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Shape.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Shape.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o: /home/noah/sproject/source/GLSL.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o -c /home/noah/sproject/source/GLSL.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/GLSL.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/GLSL.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o: /home/noah/sproject/source/Program.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_12)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o -c /home/noah/sproject/source/Program.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Program.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Program.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o: /home/noah/sproject/source/MatrixStack.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_13)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o -c /home/noah/sproject/source/MatrixStack.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/MatrixStack.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/MatrixStack.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o: /home/noah/sproject/source/Camera.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_14)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o -c /home/noah/sproject/source/Camera.cpp

CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/Camera.cpp > CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.i

CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/Camera.cpp -o CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.s

CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o

CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o: CMakeFiles/senior.dir/flags.make
CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o: /home/noah/sproject/source/tiny_obj_loader.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/noah/sproject/build/CMakeFiles $(CMAKE_PROGRESS_15)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o -c /home/noah/sproject/source/tiny_obj_loader.cc

CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/noah/sproject/source/tiny_obj_loader.cc > CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.i

CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/noah/sproject/source/tiny_obj_loader.cc -o CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.s

CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.requires:
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.requires

CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.provides: CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.requires
	$(MAKE) -f CMakeFiles/senior.dir/build.make CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.provides.build
.PHONY : CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.provides

CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.provides.build: CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o

# Object files for target senior
senior_OBJECTS = \
"CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o" \
"CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o"

# External object files for target senior
senior_EXTERNAL_OBJECTS =

senior: CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o
senior: CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o
senior: CMakeFiles/senior.dir/build.make
senior: /usr/lib/x86_64-linux-gnu/libGLU.so
senior: /usr/lib/x86_64-linux-gnu/libGL.so
senior: /usr/lib/x86_64-linux-gnu/libSM.so
senior: /usr/lib/x86_64-linux-gnu/libICE.so
senior: /usr/lib/x86_64-linux-gnu/libX11.so
senior: /usr/lib/x86_64-linux-gnu/libXext.so
senior: /usr/lib/x86_64-linux-gnu/libglut.so
senior: /usr/lib/x86_64-linux-gnu/libXmu.so
senior: /usr/lib/x86_64-linux-gnu/libXi.so
senior: CMakeFiles/senior.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable senior"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/senior.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/senior.dir/build: senior
.PHONY : CMakeFiles/senior.dir/build

CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Track.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/WorldObject.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/main.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Scene.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Vehicle.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/ShapeObj.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Texture.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Image.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/CollisionBox.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Shape.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/GLSL.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Program.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/MatrixStack.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/Camera.cpp.o.requires
CMakeFiles/senior.dir/requires: CMakeFiles/senior.dir/home/noah/sproject/source/tiny_obj_loader.cc.o.requires
.PHONY : CMakeFiles/senior.dir/requires

CMakeFiles/senior.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/senior.dir/cmake_clean.cmake
.PHONY : CMakeFiles/senior.dir/clean

CMakeFiles/senior.dir/depend:
	cd /home/noah/sproject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/noah/sproject/build /home/noah/sproject/build /home/noah/sproject/build /home/noah/sproject/build /home/noah/sproject/build/CMakeFiles/senior.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/senior.dir/depend

