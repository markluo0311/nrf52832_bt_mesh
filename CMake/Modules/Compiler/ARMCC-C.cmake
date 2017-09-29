# This file is processed when the ARMCC compiler is used for a C file

include(Compiler/ARMCC)

set(CMAKE_C_FLAGS_INIT                "")
set(CMAKE_C_FLAGS_DEBUG_INIT          "-O0 -g")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT     "-O3 -Ospace -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE_INIT        "-O3 -Otime -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O3 -Otime -g")

#set(CMAKE_C_COMPILER "armcc")
#set(CMAKE_AR "armar")
#set(CMAKE_LINKER "armlink")

set(CMAKE_C_OUTPUT_EXTENSION ".o")
set(CMAKE_C_OUTPUT_EXTENSION_REPLACE 1)

set(CMAKE_C_LINK_EXECUTABLE      "<CMAKE_LINKER> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <LINK_LIBRARIES> <OBJECTS> -o <TARGET> --list <TARGET_BASE>.map")
set(CMAKE_C_CREATE_STATIC_LIBRARY  "<CMAKE_AR> --create -cr <TARGET> <LINK_FLAGS> <OBJECTS>")

# add the target specific include directory:
get_filename_component(_compilerDir "${CMAKE_C_COMPILER}" PATH)
get_filename_component(_compilerDir "${_compilerDir}" PATH)
include_directories("${_compilerDir}/inc" )
