#! find_dependency : Resolves external dependencies.
#
# This function will try to resolve external dependencies, e.g., the Unity
# framework path.
#
# \arg:var          Variable to store dependency in.
# \arg:description  Variable description.
# \arg:default_path Default path to check for the dependency.
# \arg:file_check   File that should be found in the default path.
#
function (find_dependency var description default_path file_check)
    set(${var} ${${var}} CACHE STRING ${description})
    if (${var} STREQUAL "")
        if (DEFINED ENV{${var}})
            file(TO_CMAKE_PATH "$ENV{${var}}" ${var})
            set(var_source "set with system ENVIRONMENT")
        else ()
            set(${var} ${default_path})
            set(var_source "set with default PATH")
        endif ()
    else()
        set(var_source "set with cmd line ARG")
    endif ()

    # CMake is stupid and we need to elevate the variable to the parent
    # scope (caller of function) even though it is supposed to be global...
    message(STATUS "${var}=${${var}} --- ${var_source}")

    if (NOT (file_check STREQUAL ""))
        if (NOT (EXISTS "${${var}}/${file_check}"))
            message(STATUS "${${var}} is not a valid path to ${var}...")
            set(${var} "NOT FOUND")
        endif ()
    endif()

    set(${var} ${${var}} PARENT_SCOPE)
endfunction()

function (check_dependency dep_list)
    foreach(dep ${dep_list})
        if (${dep} MATCHES "NOT FOUND")
            message(FATAL_ERROR "${dep} not found")
        endif ()
    endforeach()
endfunction()