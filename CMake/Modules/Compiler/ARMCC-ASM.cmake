# This file is processed when the ARMCC compiler is used for an assembler file

include(Compiler/ARMCC)
set(CMAKE_ASM_OUTPUT_EXTENSION ".o")
set(CMAKE_ASM_OUTPUT_EXTENSION_REPLACE 1)

SET(ASM_DIALECT "-ARMCC")

set(CMAKE_ASM_COMPILE_OBJECT       "<CMAKE_ASM_COMPILER> <FLAGS> -o <OBJECT> <SOURCE>")
set(CMAKE_ASM_SOURCE_FILE_EXTENSIONS s;asm;msa)

# add the target specific include directory:
get_filename_component(_compilerDir "${CMAKE_ASM_COMPILER}" PATH)
get_filename_component(_compilerDir "${_compilerDir}" PATH)
include_directories("${_compilerDir}/inc" )
