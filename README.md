# Introduction to nRF5 SDK for Bluetooth Mesh

The nRF5 SDK for Bluetooth Mesh is Nordic Semiconductor's implementation of the Bluetooth Mesh. It allows
applications to utilize the features provided by the Bluetooth Mesh when running on Nordic's
nRF5 Series chips.

See the [Getting started guides](@ref md_doc_getting_started_getting_started) for more information
on
[Bluetooth Mesh](@ref md_doc_getting_started_basic_concepts),
[installing the mesh toolchain](@ref md_doc_getting_started_how_to_toolchain),
[building the mesh stack](@ref md_doc_getting_started_how_to_build), and more.

Make sure to check the [release notes](@ref md_RELEASE_NOTES) for the current release.

## Compatibility
The nRF5 SDK for Bluetooth Mesh has no direct dependencies on
the @link_nRF5SDK<!--nRF5 SDK: https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK-->, but see @subpage md_doc_getting_started_how_to_nordicSDK for instructions on how to integrate the two SDKs.

The mesh stack is compatible with the following configurations:

| SoC       | Board             | SoftDevice  | nRF5 SDK |
| --------- | ----------------- | ----------- | -------- |
| nRF52832  | PCA10040          | S132 v3.0   | v12.1.0  |
| nRF51xxx  | PCA10028/PCA10031 | S130 v2.0.1 | v12.1.0  |
| nRF51xxx  | PCA10028/PCA10031 | S110 v8.0   | v10.0.0  |

The nRF5 SDK for Bluetooth Mesh is compatible with the nRF52840 and nRF52810 SoCs. However, the build system
has not been updated to support those SoCs yet.

## Resource usage
For information about resource usage, see the @subpage md_doc_getting_started_mesh_hw_resources document.

## Repository Structure
The Bluetooth Mesh repository is organized as follows:
  - `mesh` contains the source code and unit tests for the mesh stack.
    - `mesh/api` contains the main API headers for the library.
    - `mesh/include` contains the internal headers.
    - `mesh/src` contains the source code.
    - `mesh/test` contains the unit tests.
  - `lib` contains libraries used by the mesh stack and examples (mainly uECC and Segger RTT and the SoftDevice).
  - `examples` contains example applications using the mesh stack and supporting modules such as `nrf_mesh_sdk.c`.
  - `scripts` contains useful scripts, such as a parser and communication script for the serial
  interface provided by the mesh stack.
  - `tools` contains tools useful for development.
  - `bin` contains prebuilt binaries such as the bootloader (required for DFU).
  - `toolchain` contains target-specific build files such as startup, linker, scatter, and CMSIS files.

