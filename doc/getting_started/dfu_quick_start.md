# DFU quick start guide

A Device Firmware Update (DFU) is the process of updating the firmware on a mesh device. The following guide offers step-by-step instructions on how to prepare and program the DFU example application, create a DFU file that contains example firmware, and transfer it. This guide should make it easy to use the mesh DFU to transfer any firmware to any mesh.

The required steps are listed below, followed by a more detailed explanation for each step. 

Before you start, note the following information:
- This guide assumes that you have at least 2 development kits or devices, where one is to be interfaced over the serial port, while the other receives the DFU from the first device over the mesh. To specify which device to use in which context, add the `-s <serial-number>` option for each call to the `nrfjprog` command, where `<serial-number>` is the Segger ID of your device.
- The @link_ic_nrfutil <!--nrfutil: http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.tools/dita/tools/nrfutil/nrfutil_intro.html--> tool that is required to transfer the firmware image is available at @link_nrfutil_github<!--https://github.com/NordicSemiconductor/pc-nrfutil/tree/mesh_dfu-->. The tool is open source.

  > **Important:** The master branch of the pc-nrfutil repository does not contain the additional code that is needed to handle a mesh DFU. To use the tool with mesh DFU, use the `mesh_dfu` branch. See the tool's documentation for more information about installation and prerequisites.
- Make sure to use the correct precompiled bootloader for your chip variant (nRF51/nRF52, xxAA, xxAB, xxAC). These variants have different flash and RAM sizes, as specified in the Product Specification for @link_ic_nRF51PS <!--nRF51: http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf51/dita/nrf51/pdflinks/51822_ps.html--> and @link_ic_nRF52832PS<!--nRF52832: http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52/dita/nrf52/chips/nrf52832_ps.html-->.

## Steps

1. Optional: Generate a signing key file with nrfutil.
2. Optional: Paste the public key from nrfutil into your device page.
3. Generate a HEX version of your device page with the tool in `tools/dfu`.
4. Erase all chip memory (including the UICR) on all devices.
5. Flash the Softdevice on all devices.
6. Flash the serial bootloader on all devices.
7. Flash the first application on all devices.
8. Flash the device page on all devices.
9. Generate a DFU archive with nrfutil, giving arguments that match the device page.
10. Transfer the DFU archive over serial with nrfutil.

### 1. Optional: Generate a signing key file with nrfutil

DFU images can be signed to ensure they stem from a trusted source. If you want to use this signature verification functionality, you need a signing key. The nrfutil tool can be used to generate a signing key:

```
nrfutil keys --gen-key private_key.txt
```

This will create a text file in your current directory named `private_key.txt`. This key is top secret and must not be shared with anyone! Make sure you do not lose it, because you would also lose authorization to do DFU updates to your devices in the future, and the only way to recover from this is to reflash the device manually.

### 2. Optional: Paste the public key from nrfutil into your device page

Now that you have a private key, you can generate the public key for it:

```
nrfutil keys --show-vk hex private_key.txt
```

This will output something like this:

```
Verification key Qx: ed09a58df6db5cd15b8637304f31d31f4042492ed7c7e4839fbe903f260a2ba1
Verification key Qy: a855e92b72885825481ad56282bcf549ad7455ec46f000f0f62d97eeec883ba6
```

These two HEX strings make up your public key, which you can safely share with everyone. Note that the keys above are example keys and will only work with a specific private key.

All your devices require a device page that contains information about the device and the firmware that is installed on the device. The device page is generated from a device page file. Example files for nRF51 and nRF52 are available in `tools/dfu`. The example file already has entries for the Softdevice (S110 or S132 for nRF51 and nRF52, respectively), application, and bootloader sections and the firmware IDs. Paste the public key entry that you generated above to two empty lines in the file. This allows your device to verify that the person that initiated the DFU transfer has your private key.

You may also want to change the company ID entry. In the example, the company ID is set to 0x00000059, which is the 32-bit version of Nordic Semiconductor's @link_companyID<!--Bluetooth SIG assigned Company ID: https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers-->. If your company has an assigned ID, use that one. If you do not represent a company with an assigned ID, use a random 32-bit number higher than 65535. The company ID works as a namespace for application IDs in the mesh DFU. This way, any company with an assigned company ID may safely use any application ID for their products, without risking an application ID conflict. If you use a random number, the risk of getting conflicts is present, but unlikely given the large pool of numbers. You might still want to go with something more clever than 0x12345678 though, as someone else might have been just as lazy.

### 3. Generate a HEX version of your device page

In the same folder as the example device page file, there is a Python script `device_page.py` that can generate device pages (works for both Python 2.7 and Python 3, requires the `intelhex` package from PyPi). See the README file in the `tools/dfu` folder for instructions. Run one of the following commands from inside the `tools/dfu` folder to generate an example device page file:

- For nRF51:
  ```
  python device_page.py example
  ```
- For nRF52:
  ```
  python device_page.py example52 --nrf52
  ```

This creates a device page .hex file (called `example.hex` or `example52.hex`). Hang onto this file, for we will use it in step 8.

### 4. Erase all chip memory (including UICR) on all devices

Use nrfjprog (available on @link_nordicsemi<!--http://www.nordicsemi.com/-->) to erase all previous data on your device (including UICR):

```
nrfjprog --eraseall
```

### 5. Flash the SoftDevice on all devices

> **Important:** Steps 5--8 must be executed in order.

SoftDevices for nRF51 and nRF52 are located in the `lib/softdevices` folder. If you used the example device page file for nRF51, use the S110 SoftDevice. For nRF52, use S132.

```
nrfjprog --program <Softdevice HEX file>
```

### 6. Flash the serial bootloader on all devices

Flash the precompiled `bootloader_serial.hex` (in `bin/`) to your device using nrfjprog:

```
nrfjprog --program <bootloader serial HEX file>
```

### 7. Flash the first application on all devices

To be able to do Device Firmware Updates, you must flash an application that supports DFU. The DFU example application can be found in `examples/dfu/`.

From your build folder, flash the HEX file of the DFU example application to all devices. For nRF51 devices, use the `examples/dfu/51_dfu.hex` file. For nRF52, use `examples/dfu/52_dfu.hex`.

Flash the file with the following command:
```
nrfjprog --family <nRF51|nRF52> --program <application HEX file>
```

Then reset the device to start the application:
```
nrfjprog --reset
```

### 8. Flash the device page on all devices

Flash the device page HEX file that you generated in step 3 to the devices:

```
nrfjprog --program <device page HEX file>
```

### 9. Generate a DFU file with nrfutil

To do a DFU transfer, you must create a DFU archive. The DFU archive is a zip file that contains the application binary along with some metadata. 

A HEX file of an example application that causes an LED to blink on the boards is located in `examples/dfu/bin`. Use the HEX file that corresponds to the SoftDevice you are using.

Matching the firmware IDs in your device page file, use the `nrfutil` tool to generate the DFU archive:

- For nRF51:
  ```
  nrfutil dfu genpkg --application examples/dfu/bin/app_green_s110.hex \
      --company-id 0x00000059 \
      --application-id 1 \
      --application-version 2 \
      --key-file private.txt \
      --sd-req 0x0064 \
      --mesh dfu_test.zip
  ```
- For nRF52:
  ```
  nrfutil dfu genpkg --application examples/dfu/bin/app_green_s132.hex \
      --company-id 0x00000059 \
      --application-id 1 \
      --application-version 2 \
      --key-file private.txt \
      --sd-req 0x0064 \
      --mesh dfu_test.zip
  ```

These commands generate a DFU archive called `dfu_test.zip` in the current directory. You can call `nrfutil dfu genpkg --help` to get a list of possible command line arguments and their meaning. Note that some of the options do not apply to the mesh DFU, because the tool also supports the regular Nordic Semiconductor DFU transfer.

The example commands use the Nordic Semiconductor company ID, so make sure you use your own instead. Also note that the application version is set to 2. A device will only accept application transfers that match its current company and application IDs and have a higher version number.

### 10. Transfer the DFU archive over serial with nrfutil

> **Important:** Close all running instances of nRFgo Studio before you continue. If running, nRFgo Studio might cause problems with the reset procedure for the nRF51.

Now we come to the interesting part: Doing the DFU! First, figure out to which COM port your serial device is connected:

- On Windows, serial ports are called COMxx, where xx is an integer. To figure out which COM port is used for a device, open Windows Device Manager and look under "Ports (COM & LPT)" for the number of the port.
- On Linux, serial ports for JLink devices are called ttyACMx, where x is an integer, and live in the `/dev` directory. Use the `dmesg` command after you have plugged in a device to see which serial port file has been assigned to the device.

To start the DFU, run the following command:
```
nrfutil dfu serial -pkg dfu_test.zip -p <COM port> -b 115200 -fc --mesh
```

A progress bar should pop up, and the transfer should take a couple of minutes.

When finished, the bootloader should switch to the application and the LEDs should start blinking. Note that you cannot do the DFU twice with the same DFU archive, because the application version in the device page on your device is incremented to the latest version. Therefore, the bootloader will reject any attempt to transfer the same firmware again.

To try another DFU, re-run steps 9--10 with an increased version number, for example `--application-version 3`, and use the new zip file to do the DFU again.

## Troubleshooting: Verifying your bootloader with the bootloader_verify.py script

To verify that the bootloader is working correctly, run the bootloader verification script located in `tools/dfu`. Note that it requires the @link_pyserial <!--pyserial package: https://pypi.python.org/pypi/pyserial--> and that `nrfjprog` is present in your `PATH`.

- For nRF51:
  ```
  python bootloader_verify_nRF51.py <serial number> <COM port>
  ```
- For nRF52:
  ```
  python bootloader_verify_nRF52.py <serial number> <COM port>
  ```

The output should look like this:

```
Reading UICR..                  OK.
Reading Device page..           OK.
Resetting device..              OK.
Checking serial connection..    OK.

Bootloader verification OK
```

Run `nrfjprog --reset` to reset the board back to a well-known state of operation after running the bootloader verification script.
