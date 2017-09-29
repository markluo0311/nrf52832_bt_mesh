# ARM Compiler Toolchain
set(_compiler_id_pp_test "defined(__ARMCC_VERSION)")

set(_compiler_id_version_compute "
  /* __ARMCC_VERSION = PVVbbbb */
# define @PREFIX@COMPILER_VERSION_MAJOR @MACRO_DEC@(__ARMCC_VERSION/1000000)
# define @PREFIX@COMPILER_VERSION_MINOR @MACRO_DEC@(__ARMCC_VERSION/10000 % 10000)
# define @PREFIX@COMPILER_VERSION_PATCH @MACRO_DEC@(__ARMCC_VERSION       % 10000)")
