# Library for writing multithreaded tests, used to test for IRQ-safety problems.

find_package(Threads REQUIRED)
include_directories(".")
add_library_platforms(mttest mttest.c)
