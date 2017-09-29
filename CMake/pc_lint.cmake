# This file contains functions and configurations for generating PC-Lint build
# targets for your CMake projects.

set(PC_LINT_EXECUTABLE "c:/lint/lint-nt.exe" CACHE STRING "full path to the pc-lint executable. NOT the generated lin.bat")
mark_as_advanced(PC_LINT_EXECUTABLE)
set(PC_LINT_CONFIG_DIR "c:/lint/" CACHE STRING "full path to the directory containing pc-lint configuration files")
mark_as_advanced(PC_LINT_CONFIG_DIR)
set(PC_LINT_USER_FLAGS "-b" CACHE STRING "additional pc-lint command line options -- some flags of pc-lint cannot be set in option files (most notably -b)")
mark_as_advanced(PC_LINT_USER_FLAGS)
# a phony target which causes all available *_LINT targets to be executed
add_custom_target(ALL_LINT)

separate_arguments(PC_LINT_USER_FLAGS)

# add_pc_lint(target source1 [source2 ...])
#
# Takes a list of source files and generates a build target which can be used
# for linting all files
#
# The generated lint commands assume that a top-level config file named
# 'std.lnt' resides in the configuration directory 'PC_LINT_CONFIG_DIR'. This
# config file must include all other config files. This is standard lint
# behaviour.
#
# Parameters:
#  - target: the name of the target to which the sources belong. You will get a
#            new build target named ${target}_LINT
#  - source1 ... : a list of source files to be linted. Just pass the same list
#            as you passed for add_executable or add_library. Everything except
#            C and CPP files (*.c, *.cpp, *.cxx) will be filtered out.
#
# Example:
#  If you have a CMakeLists.txt which generates an executable like this:
#
#    set(MAIN_SOURCES main.c foo.c bar.c)
#    add_executable(main ${MAIN_SOURCES})
#
#  include this file
#
#    include(/path/to/pc_lint.cmake)
#
#  and add a line to generate the main_LINT target
#
#   if(COMMAND add_pc_lint)
#    add_pc_lint(main ${MAIN_SOURCES})
#   endif(COMMAND add_pc_lint)
#
function(add_pc_lint target)
    # Get include directories for target
    get_property(lint_include_directories TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)

    # Get defines for target (for some reason directory defines are not included in target variable)
    get_property(lint_target_defines TARGET ${target} PROPERTY COMPILE_DEFINITIONS)
    get_directory_property(lint_defines COMPILE_DEFINITIONS)

    # Convert all include dirs to use relative path in order to stay below maximum allowed
    # limit of 8196 characters for windows commands.
    set(lint_include_directories_transformed)
    foreach(include_dir ${lint_include_directories})
        file(RELATIVE_PATH include_dir_rel ${PROJECT_BINARY_DIR} ${include_dir})
        list(APPEND lint_include_directories_transformed -i"${include_dir_rel}")
        #message(lint include_dir_rel = ${include_dir_rel})
    endforeach(include_dir)

    # prepend each definition with "-d"
    set(lint_defines_transformed)
    foreach(definition ${lint_defines})
        list(APPEND lint_defines_transformed -d"${definition}")
    endforeach(definition)

    foreach(definition ${lint_target_defines})
        list(APPEND lint_defines_transformed -d${definition})
    endforeach()

    # If building for host, add -dHOST=1, otherwise add -dHOST=0:
    if(BUILD_HOST)
        list(APPEND lint_defines_transformed -dHOST=1)
    else()
        list(APPEND lint_defines_transformed -dHOST=0)
    endif()

    # Set target platform define:
    if(TARGET_PLATFORM MATCHES "51")
        list(APPEND lint_defines_transformed -dNRF51)
    elseif(TARGET_PLATFORM MATCHES "52")
        list(APPEND lint_defines_transformed -dNRF52)
    endif()

    # list of all commands, one for each given source file
    set(pc_lint_properties_file -i"${CMAKE_SOURCE_DIR}/CMake" mesh.lnt)

    foreach(argument ${ARGN})
        # only include c and cpp files
        if( argument MATCHES \\.c$|\\.cxx$|\\.cpp$ )
            # Make filename absolute
            get_filename_component(sourcefile_abs ${argument} ABSOLUTE)
            # Include source folders as well, if it has not been included yet
            get_filename_component(sourcefile_dir ${argument} DIRECTORY)
            list(FIND lint_include_directories_transformed ${sourcefile_dir} dir_index)
            if(${dir_index} GREATER -1)
                list(APPEND lint_include_directories_transformed -i"${sourcefile_dir}")
            endif()

            # Use relative path for all src files to stay below windows command line limit
            file(RELATIVE_PATH sourcefile_rel ${PROJECT_BINARY_DIR} ${sourcefile_abs})
            list(APPEND sourcefiles_rel ${sourcefile_rel} ${PC_LINT_USER_FLAGS})
            #message(lint sourcefile_rel = ${sourcefile_rel})
        endif()
    endforeach(argument)

    #Create a LINT target for this project.
    add_custom_target(${target}_lint COMMAND ${PC_LINT_EXECUTABLE}
        -i"${PC_LINT_CONFIG_DIR}" std.lnt
        ${pc_lint_properties_file}
        "-u" ${PC_LINT_USER_FLAGS}
        ${lint_include_directories_transformed}
        ${lint_defines_transformed}
        ${sourcefiles_rel}
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMENT "Running lint for target ${target}"
        )

    # make the lint target depend on each and every *_LINT target
    add_dependencies(ALL_LINT ${target}_lint)
    #message( FATAL_ERROR "End of lint generation, CMake will exit." )
endfunction(add_pc_lint)
