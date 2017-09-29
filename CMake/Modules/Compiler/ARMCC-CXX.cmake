# This file is processed when the ARMCC compiler is used for a C++ file

include(Compiler/ARMCC)

set(CMAKE_CXX_FLAGS_INIT "")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-g")
set(CMAKE_CXX_FLAGS_MINSIZEREL_INIT "-Ospace -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Otime -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")

set(CMAKE_CXX_OUTPUT_EXTENSION ".o")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

set(CMAKE_CXX_LINK_EXECUTABLE      "<CMAKE_LINKER> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <LINK_LIBRARIES> <OBJECTS> -o <TARGET> --list <TARGET_BASE>.map")
set(CMAKE_CXX_CREATE_STATIC_LIBRARY  "<CMAKE_AR> --create -cr <LINK_FLAGS> <TARGET> <OBJECTS>")

# add the target specific include directory:
get_filename_component(_compilerDir "${CMAKE_CXX_COMPILER}" PATH)
get_filename_component(_compilerDir "${_compilerDir}" PATH)
include_directories("${_compilerDir}/inc" )
