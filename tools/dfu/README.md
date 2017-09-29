# DFU Utilities and Tools

To work with the mesh DFU, the following scripts are provided:

- Device page generator tool, `device_page.py`
- Bootloader verification tool, `bootloader_verify.py`
- Device page reader tool, `read_devpage.py`

The following sections describe each of the tools in more detail.

## Device Page Generator Tool (`device_page.py`)

This simple tool will generate a device page from a specified input file. A device page is an in-flash storage area from which
the bootloader can read device information required for correct operation. Values like the application start address, public key
for verifying DFU transfers and firmware IDs, allows the bootloader to make qualified decisions about whether it should accept
incoming DFU transfers and where the data belongs.

### Requirements

The device page generator works in both Python 2.7 and 3.2 and above. It depends on the [Intel Hex
package](https://pypi.python.org/pypi/IntelHex) by Alexander Belchenko, available on pypi.

### Usage

```

python device_page.py <info-file> [--nrf52]
    <info-file> Must be the filename of a info file following the format described below.
    --nrf52     Optional argument to generate the device page file to fit on an nRF52, rather than an nRF51.

```

### Input file

The input file is a text file consisting of a set of handle-value pairs. The handle-value pair assignment must follow this format:

```
<handle>: <value>
```

There can only be one handle per line. Lines starting with # are comments.

The example files in this directory contain all mandatory handles, and some common values for each.

Below is a list of all allowed handles, their format, whether they are mandatory for proper operation and a short description.

## Device page handles
| Handle name           | Data type     | Required | Description
|-----------------------|---------------|----------|-------------|
| `SD_START`            | `uint32_t`    | Yes      | Start of permitted Softdevice area. Should be right after the MBR (see the Softdevice Specification for your favorite Softdevice)
| `SD_SIZE`             | `uint32_t`    | Yes      | Size of permitted Softdevice area.
| `APP_START`           | `uint32_t`    | Yes      | Start of permitted application area. Should be immediately after the Softdevice.
| `APP_SIZE`            | `uint32_t`    | Yes      | Size of permitted application area.
| `BL_START`            | `uint32_t`    | Yes      | Start of permitted bootloader area. Should be towards the end of the available flash-memory.
| `BL_SIZE`             | `uint32_t`    | Yes      | Size of permitted bootloader area.
| `COMPANY_ID`          | `uint32_t`    | Yes      | Company ID associated with the application on this device.
| `APP_ID`              | `uint16_t`    | Yes      | Application ID associated with the application on this device.
| `APP_VERSION`         | `uint32_t`    | Yes      | Currently flashed version of the application.
| `SD_VERSION`          | `uint16_t`    | Yes      | Required Softdevice for the application on this device, or 0xFFFE if it doesn't matter.
| `BL_ID`               | `uint8_t`     | Yes      | ID of the bootloader on this device.
| `BL_VERSION`          | `uint8_t`     | Yes      | Version of the bootloader on this device.
| `VERIFICATION KEY QX` | `uint8_t[32]` | No       | First part of the public signing key used to verify incoming DFU transfers.
| `VERIFICATION KEY QY` | `uint8_t[32]` | No       | Second part of the public signing key used to verify incoming DFU transfers.
| `PUBLIC_KEY`          | `uint8_t[64]` | No       | Alternative full-version of the public signing key used to verify incoming DFU transfers.

For further documentation on the different handles and their uses, see the guide under /docs/dfu/.

## Bootloader Verification Tool (`bootloader_verify.py`)

The bootloader verification script is used to berify that the bootloader is flashed and initialized correctly.

## Device Page Reader (`read_devpage.py`)

This script allows you to read the device page from a device. It provides output in the same format as the
Device Page Generator script expects.