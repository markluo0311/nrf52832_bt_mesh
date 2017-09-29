# Installing the mesh toolchain

To build the example applications, a toolchain based on either CMake or SEGGER Embedded Studio is required.
Install instructions are provided for Windows and Debian/Ubuntu. The steps should be similar for
other platforms.

## SEGGER Embedded Studio

Please see the @link_seggerstudio
<!--https://www.segger.com/products/development-tools/embedded-studio/?L=0--> website for download and
installation instructions.

## CMake based setup

@link_cmake <!--CMake: https://cmake.org/--> is a build management system used for managing an
environment that is independent of the compiler and build system used. Version 3.0 or above is
required by the mesh stack.

The following tools must be installed:
* @link_cmake <!--CMake: https://cmake.org/-->
* A _toolchain_. Supported toolchaings are the @link_armnone <!--arm-none-eabi-gcc: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm-->
  toolchain and the @link_armcc <!--armcc v5: https://developer.arm.com/products/software-development-tools/compilers/arm-compiler/downloads/version-5-->
  toolchain. The armcc v5 toolchain is also provided by @link_keil <!--Keil: http://www2.keil.com/mdk5/compiler/5/-->
  and comes bundled with the @link_keiluvision<!--Keil uVision IDE: http://www2.keil.com/mdk5/uvision/-->.
* A _build system_. CMake supports a range of build systems, e.g., @link_ninja and @link_make.

Please follow the instructions for your platform:
* @subpage md_doc_getting_started_how_to_windows_tools
* @subpage md_doc_getting_started_how_to_debian_tools

