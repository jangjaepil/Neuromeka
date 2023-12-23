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
CMAKE_SOURCE_DIR = /home/jang/neuromeka_ws/src/osqp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jang/neuromeka_ws/src/osqp

# Include any dependencies generated for this target.
include CMakeFiles/osqp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/osqp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/osqp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/osqp.dir/flags.make

CMakeFiles/osqp.dir/src/osqp_api.c.o: CMakeFiles/osqp.dir/flags.make
CMakeFiles/osqp.dir/src/osqp_api.c.o: src/osqp_api.c
CMakeFiles/osqp.dir/src/osqp_api.c.o: CMakeFiles/osqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/osqp.dir/src/osqp_api.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/osqp.dir/src/osqp_api.c.o -MF CMakeFiles/osqp.dir/src/osqp_api.c.o.d -o CMakeFiles/osqp.dir/src/osqp_api.c.o -c /home/jang/neuromeka_ws/src/osqp/src/osqp_api.c

CMakeFiles/osqp.dir/src/osqp_api.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osqp.dir/src/osqp_api.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jang/neuromeka_ws/src/osqp/src/osqp_api.c > CMakeFiles/osqp.dir/src/osqp_api.c.i

CMakeFiles/osqp.dir/src/osqp_api.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osqp.dir/src/osqp_api.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jang/neuromeka_ws/src/osqp/src/osqp_api.c -o CMakeFiles/osqp.dir/src/osqp_api.c.s

# Object files for target osqp
osqp_OBJECTS = \
"CMakeFiles/osqp.dir/src/osqp_api.c.o"

# External object files for target osqp
osqp_EXTERNAL_OBJECTS = \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/auxil.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/error.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/scaling.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/util.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/polish.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/derivative.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/interrupt_unix.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/timing_linux.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/codegen.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/csc_math.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/csc_utils.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/builtin/algebra_libs.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/builtin/vector.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/builtin/matrix.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/SuiteSparse_config.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_1.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_2.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_aat.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_control.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_defaults.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_info.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_order.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_post_tree.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_postorder.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_preprocess.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_valid.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/kkt.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/qdldl_interface.c.o" \
"/home/jang/neuromeka_ws/src/osqp/_deps/qdldl-build/CMakeFiles/qdldlobject.dir/src/qdldl.c.o"

out/libosqp.so: CMakeFiles/osqp.dir/src/osqp_api.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/auxil.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/error.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/scaling.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/util.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/polish.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/derivative.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/interrupt_unix.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/timing_linux.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/src/codegen.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/csc_math.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/csc_utils.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/builtin/algebra_libs.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/builtin/vector.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/builtin/matrix.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/SuiteSparse_config.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_1.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_2.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_aat.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_control.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_defaults.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_info.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_order.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_post_tree.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_postorder.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_preprocess.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_valid.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/kkt.c.o
out/libosqp.so: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/qdldl_interface.c.o
out/libosqp.so: _deps/qdldl-build/CMakeFiles/qdldlobject.dir/src/qdldl.c.o
out/libosqp.so: CMakeFiles/osqp.dir/build.make
out/libosqp.so: CMakeFiles/osqp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library out/libosqp.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/osqp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/osqp.dir/build: out/libosqp.so
.PHONY : CMakeFiles/osqp.dir/build

CMakeFiles/osqp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/osqp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/osqp.dir/clean

CMakeFiles/osqp.dir/depend:
	cd /home/jang/neuromeka_ws/src/osqp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp/CMakeFiles/osqp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/osqp.dir/depend

