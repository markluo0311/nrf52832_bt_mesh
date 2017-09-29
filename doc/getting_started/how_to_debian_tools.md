# Debian/Ubuntu

For Debian/Ubuntu, most tools are available from the system package manager `apt-get`.

## CMake

To install CMake, simply call:

    $ sudo apt-get install cmake cmake-curses-gui


## Toolchain

For Debian/Ubuntu, only the GNU ARM Embedded toolchain is supported. The version usually found in
the Debian package manager is quite old (4.9.3). Therefore, install the toolchain in the following
way:

    $ sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
    $ sudo apt-get update
    $ sudo apt-get install gcc-arm-embedded


This will also install GDB (the GNU Debugger) for ARM.

## Build system

@link_make is the default build system on Debian/Ubuntu and usually comes with the distribution.
However, if @link_ninja is preferred, it can be installed with:

    $ sudo apt-get install ninja-build

## Building unit tests (host)

The unit test (host) build uses the standard GCC compiler, which should be available in the
Debian/Ubuntu distribution by default. To enable compilation for a 32-bit architecture on a 64-bit
system (`-m32`), `multilib` is required. Install it with:

    $ sudo apt-get install gcc-multilib

Ruby is used by the mocking framework @link_cmock. Install it with:

    $ sudo apt-get install ruby

