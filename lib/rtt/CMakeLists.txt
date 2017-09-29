set(RTT_SRCS
  src/SEGGER_RTT.c
  src/SEGGER_RTT_printf.c
  )

include_directories(
  "include"
  )

add_library_platforms(RTT_LIB STATIC ${RTT_SRCS})
