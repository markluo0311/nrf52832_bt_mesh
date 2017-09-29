# Windows

**Important:** Ensure that all installed tools are available in a folder referenced by the
system path (e.g. the `PATH` environment variable).

## CMake

Download the installer from the @link_cmake_download<!-- https://cmake.org/download/ -->
and follow the installation instructions.

## Toolchain

For Windows, both the armcc v5 and GCC ARM Embedded toolchains are supported.

* For ARMCC/Keil, follow the instructions provided for @link_armcc<!-- https://developer.arm.com/products/software-development-tools/compilers/arm-compiler/downloads/version-5 -->.
  The armcc v5 toolchain is also provided by @link_keil<!--Keil: http://www2.keil.com/mdk5/compiler/5/-->
  and comes bundled with the @link_keiluvision<!--http://www2.keil.com/mdk5/uvision/-->.
* For the GCC ARM Embedded toolchain, download the @link_armnone<!-- https://developer.arm.com/open-source/gnu-toolchain/gnu-rm -->
  installer and follow the installation instructions.

## Build system
The Ninja build system is preferred on Windows. Download the binary from the @link_ninja_download<!-- https://ninja-build.org/ -->
and place it in a folder reachable from the system path.

## Building unit tests (host)
The unit test (host) build uses the standard GCC compiler, available on Windows through @link_mingw<!-- https://sourceforge.net/projects/mingw/files/-->.
Ruby is used by the mocking framework @link_cmock<!-- https://github.com/ThrowTheSwitch/CMock -->.
Install it from the @link_ruby<!-- https://www.ruby-lang.org/ --> website.
