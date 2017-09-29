
# Serial commands

# Serial command overview {#serial-commands}
@ingroup LIBRARIES
Serial commands are messages sent from the host controller to the nRF5. Most
serial commands result in a _CMD RSP_ event, indicating whether the command was
successful and returning any relevant data depending on the command type.

The serial commands are broken into groups:

- **Device:** HW commands for device operation

- **Application:** Single opcode made available to the application

- **Segmentation And Reassembly:** Segmentation and reassembly of serial packets, to allow packets larger than the largest serial packet size

- **Configuration:** Configuration of various device parameter, like addresses and radio properties

- **Provisioning:** Provisioning-specific commands and operations

- **nRF Open Mesh:** Set of commands used by the nRF Open Mesh

- **Bluetooth Mesh:** Bluetooth Mesh commands for controlling a device's behavior on the mesh

- **Direct Firmware Upgrade:** Commands controlling the behavior of the Device Firmware Update part of the mesh stack

- **Access Layer:** Commands to interface the access layer on mesh

- **Model Specific:** Commands for initializing and commanding specific models



See the tables below for a list of serial commands available for each command
group in the nRF5 mesh serial interface and their opcodes. Each entry links
to their respective "Details" section, where the parameters and effect of each
command are described.


## Device Commands {#device-commands}

Command                                 | Opcode
----------------------------------------|-------
[Echo](#device-echo)                             | `0x02`
[Internal Events Report](#device-internal-events-report)           | `0x03`
[Serial Version Get](#device-serial-version-get)               | `0x09`
[FW Info Get](#device-fw-info-get)                      | `0x0a`
[Radio Reset](#device-radio-reset)                      | `0x0e`
[Beacon Start](#device-beacon-start)                     | `0x10`
[Beacon Stop](#device-beacon-stop)                      | `0x11`
[Beacon Params Get](#device-beacon-params-get)                | `0x13`
[Beacon Params Set](#device-beacon-params-set)                | `0x12`


## Application Commands {#application-commands}

Command                                 | Opcode
----------------------------------------|-------
[Application](#application-application)                 | `0x20`


## Segmentation And Reassembly Commands {#segmentation-and-reassembly-commands}

Command                                 | Opcode
----------------------------------------|-------
[Start](#segmentation-and-reassembly-start)       | `0x21`
[Continue](#segmentation-and-reassembly-continue)    | `0x22`


## Configuration Commands {#configuration-commands}

Command                                 | Opcode
----------------------------------------|-------
[Adv Addr Set](#configuration-adv-addr-set)              | `0x40`
[Adv Addr Get](#configuration-adv-addr-get)              | `0x41`
[Channel Map Set](#configuration-channel-map-set)           | `0x42`
[Channel Map Get](#configuration-channel-map-get)           | `0x43`
[TX Power Set](#configuration-tx-power-set)              | `0x44`
[TX Power Get](#configuration-tx-power-get)              | `0x45`
[UUID Set](#configuration-uuid-set)                  | `0x53`
[UUID Get](#configuration-uuid-get)                  | `0x54`


## Provisioning Commands {#provisioning-commands}

Command                                 | Opcode
----------------------------------------|-------
[Scan Start](#provisioning-scan-start)                 | `0x61`
[Scan Stop](#provisioning-scan-stop)                  | `0x62`
[Provision](#provisioning-provision)                  | `0x63`
[Listen](#provisioning-listen)                     | `0x64`
[OOB Use](#provisioning-oob-use)                    | `0x66`
[Auth Data](#provisioning-auth-data)                  | `0x67`
[ECDH Secret](#provisioning-ecdh-secret)                | `0x68`
[Keypair Set](#provisioning-keypair-set)                | `0x69`
[Capabilities Set](#provisioning-capabilities-set)           | `0x6a`


## nRF Open Mesh Commands {#nrf-open-mesh-commands}

Command                                 | Opcode
----------------------------------------|-------
[Init](#nrf-open-mesh-init)                      | `0x70`
[Value Set](#nrf-open-mesh-value-set)                 | `0x71`
[Value Enable](#nrf-open-mesh-value-enable)              | `0x72`
[Value Disable](#nrf-open-mesh-value-disable)             | `0x73`
[Start](#nrf-open-mesh-start)                     | `0x74`
[Stop](#nrf-open-mesh-stop)                      | `0x75`
[Flag Set](#nrf-open-mesh-flag-set)                  | `0x76`
[Flag Get](#nrf-open-mesh-flag-get)                  | `0x77`
[DFU Data](#nrf-open-mesh-dfu-data)                  | `0x78`
[Value Get](#nrf-open-mesh-value-get)                 | `0x7a`
[Build Version Get](#nrf-open-mesh-build-version-get)         | `0x7b`
[Access Addr Get](#nrf-open-mesh-access-addr-get)           | `0x7c`
[Channel Get](#nrf-open-mesh-channel-get)               | `0x7d`
[Interval Min ms Get](#nrf-open-mesh-interval-min-ms-get)       | `0x7f`


## Bluetooth Mesh Commands {#bluetooth-mesh-commands}

Command                                 | Opcode
----------------------------------------|-------
[Enable](#bluetooth-mesh-enable)                   | `0x90`
[Disable](#bluetooth-mesh-disable)                  | `0x91`
[Subnet Add](#bluetooth-mesh-subnet-add)               | `0x92`
[Subnet Update](#bluetooth-mesh-subnet-update)            | `0x93`
[Subnet Delete](#bluetooth-mesh-subnet-delete)            | `0x94`
[Subnet Get All](#bluetooth-mesh-subnet-get-all)           | `0x95`
[Subnet Count Max Get](#bluetooth-mesh-subnet-count-max-get)     | `0x96`
[Appkey Add](#bluetooth-mesh-appkey-add)               | `0x97`
[Appkey Update](#bluetooth-mesh-appkey-update)            | `0x98`
[Appkey Delete](#bluetooth-mesh-appkey-delete)            | `0x99`
[Appkey Get All](#bluetooth-mesh-appkey-get-all)           | `0x9a`
[Appkey Count Max Get](#bluetooth-mesh-appkey-count-max-get)     | `0x9b`
[Devkey Add](#bluetooth-mesh-devkey-add)               | `0x9c`
[Devkey Delete](#bluetooth-mesh-devkey-delete)            | `0x9d`
[Devkey Count Max Get](#bluetooth-mesh-devkey-count-max-get)     | `0x9e`
[Addr Local Unicast Set](#bluetooth-mesh-addr-local-unicast-set)   | `0x9f`
[Addr Local Unicast Get](#bluetooth-mesh-addr-local-unicast-get)   | `0xa0`
[Addr Get](#bluetooth-mesh-addr-get)                 | `0xa7`
[Addr Get All](#bluetooth-mesh-addr-get-all)             | `0xa8`
[Addr Nonvirtual Count Max Get](#bluetooth-mesh-addr-nonvirtual-count-max-get)| `0xa9`
[Addr Virtual Count Max Get](#bluetooth-mesh-addr-virtual-count-max-get)| `0xaa`
[Addr Subscription Add](#bluetooth-mesh-addr-subscription-add)    | `0xa1`
[Addr Subscription Add Virtual](#bluetooth-mesh-addr-subscription-add-virtual)| `0xa2`
[Addr Subscription Remove](#bluetooth-mesh-addr-subscription-remove) | `0xa3`
[Addr Publication Add](#bluetooth-mesh-addr-publication-add)     | `0xa4`
[Addr Publication Add Virtual](#bluetooth-mesh-addr-publication-add-virtual)| `0xa5`
[Addr Publication Remove](#bluetooth-mesh-addr-publication-remove)  | `0xa6`
[Packet Send](#bluetooth-mesh-packet-send)              | `0xab`
[State Clear](#bluetooth-mesh-state-clear)              | `0xac`


## Direct Firmware Upgrade Commands {#direct-firmware-upgrade-commands}

Command                                 | Opcode
----------------------------------------|-------
[Jump To Bootloader](#direct-firmware-upgrade-jump-to-bootloader)| `0xd0`
[Request](#direct-firmware-upgrade-request)         | `0xd1`
[Relay](#direct-firmware-upgrade-relay)           | `0xd2`
[Abort](#direct-firmware-upgrade-abort)           | `0xd3`
[Bank Info Get](#direct-firmware-upgrade-bank-info-get)   | `0xd4`
[Bank Flash](#direct-firmware-upgrade-bank-flash)      | `0xd5`
[State Get](#direct-firmware-upgrade-state-get)       | `0xd6`


## Access Layer Commands {#access-layer-commands}

Command                                 | Opcode
----------------------------------------|-------
[Model Pub Addr Set](#access-layer-model-pub-addr-set)         | `0xe0`
[Model Pub Addr Get](#access-layer-model-pub-addr-get)         | `0xe1`
[Model Pub Period Set](#access-layer-model-pub-period-set)       | `0xe2`
[Model Pub Period Get](#access-layer-model-pub-period-get)       | `0xe3`
[Model Subs Add](#access-layer-model-subs-add)             | `0xe4`
[Model Subs Remove](#access-layer-model-subs-remove)          | `0xe5`
[Model Subs Get](#access-layer-model-subs-get)             | `0xe6`
[Model App Bind](#access-layer-model-app-bind)             | `0xe7`
[Model App Unbind](#access-layer-model-app-unbind)           | `0xe8`
[Model App Get](#access-layer-model-app-get)              | `0xe9`
[Model Pub App Set](#access-layer-model-pub-app-set)          | `0xea`
[Model Pub App Get](#access-layer-model-pub-app-get)          | `0xeb`
[Model Pub TTL Set](#access-layer-model-pub-ttl-set)          | `0xec`
[Model Pub TTL Get](#access-layer-model-pub-ttl-get)          | `0xed`
[Elem Loc Set](#access-layer-elem-loc-set)               | `0xee`
[Elem Loc Get](#access-layer-elem-loc-get)               | `0xef`
[Elem Sig Model Count Get](#access-layer-elem-sig-model-count-get)   | `0xf0`
[Elem Vendor Model Count Get](#access-layer-elem-vendor-model-count-get)| `0xf1`
[Model ID Get](#access-layer-model-id-get)               | `0xf2`
[Handle Get](#access-layer-handle-get)                 | `0xf3`
[Elem Models Get](#access-layer-elem-models-get)            | `0xf4`
[Access Flash Store](#access-layer-access-flash-store)         | `0xf5`


## Model Specific Commands {#model-specific-commands}

Command                                 | Opcode
----------------------------------------|-------
[Models Get](#model-specific-models-get)               | `0xfc`
[Init](#model-specific-init)                     | `0xfd`
[Command](#model-specific-command)                  | `0xfe`


## Serial Command Details {#serial-command-details}


### Device Echo {#device-echo}

_Opcode:_ `0x02`

_Total length: 1..98 bytes_

A simple loopback test command, to verify that the serial transport layer is working as intended.

_Echo Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[97]` | Data                                    | 0..97 | 0      | Data to echo back.

### Response

Potential status codes:

- `INVALID_LENGTH`

_Echo Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[97]` | Data                                    | 0..97 | 0      | Data received in the echo command.


### Device Internal Events Report {#device-internal-events-report}

_Opcode:_ `0x03`

_Total length: 1 byte_

Start reporting internal events, if supported.

_Internal Events Report takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Device Serial Version Get {#device-serial-version-get}

_Opcode:_ `0x09`

_Total length: 1 byte_

Request the implemented version of the serial interface.

_Serial Version Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Serial Version Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Serial Ver                              | 2    | 0      | Serial interface version.


### Device FW Info Get {#device-fw-info-get}

_Opcode:_ `0x0a`

_Total length: 1 byte_

Request the firmware version info structure for the firmware.

_FW Info Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_FW Info Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`nrf_mesh_fwid_t` | FWID                                    | 10   | 0      | Firmware ID data.


### Device Radio Reset {#device-radio-reset}

_Opcode:_ `0x0e`

_Total length: 1 byte_

Restart the device.

_Radio Reset takes no parameters._

### Response

_This command does not yield any response._

### Device Beacon Start {#device-beacon-start}

_Opcode:_ `0x10`

_Total length: 2..33 bytes_

Start an application controlled beacon with the given payload. Will hotswap the payload if the beacon is already running.

_Beacon Start Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to set the payload for.
`uint8_t[31]` | Data                                    | 0..31 | 1      | Beacon payload.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `ERROR_BUSY`

- `INVALID_LENGTH`

_The response has no parameters._

### Device Beacon Stop {#device-beacon-stop}

_Opcode:_ `0x11`

_Total length: 2 bytes_

Stop transmitting the current beacon.

_Beacon Stop Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to stop.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Device Beacon Params Get {#device-beacon-params-get}

_Opcode:_ `0x13`

_Total length: 2 bytes_

Set parameters for application controlled beacon.

_Beacon Params Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to get the parameters of.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_Beacon Params Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to start.
`uint8_t`     | TX Power                                | 1    | 1      | TX Power value, must be a value from @ref serial_cmd_tx_power_value_t.
`uint8_t`     | Channel Map                             | 1    | 2      | Channel map bitfield for beacon, starting at channel 37.
`uint32_t`    | Interval ms                             | 4    | 3      | TX interval in milliseconds.


### Device Beacon Params Set {#device-beacon-params-set}

_Opcode:_ `0x12`

_Total length: 8 bytes_

Set parameters for application controlled beacon.

_Beacon Params Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to start.
`uint8_t`     | TX Power                                | 1    | 1      | TX Power value, must be a value from @ref serial_cmd_tx_power_value_t.
`uint8_t`     | Channel Map                             | 1    | 2      | Channel map bitfield for beacon, starting at channel 37.
`uint32_t`    | Interval ms                             | 4    | 3      | TX interval in milliseconds.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Application Application {#application-application}

_Opcode:_ `0x20`

_Total length: 1..98 bytes_

Application specific command, has no functionality in the framework, but is forwarded to the application.

_Application Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[97]` | Data                                    | 0..97 | 0      | Application data.

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Segmentation And Reassembly Start {#segmentation-and-reassembly-start}

_Opcode:_ `0x21`

_Total length: 1 byte_

Opening message of a Segmentation and Reassembly message.

_Start takes no parameters._

### Response

Potential status codes:

- `TRANSACTION_CONTINUE`

- `TRANSACTION_COMPLETE`

- `ERROR_DATA_SIZE`

- `INVALID_LENGTH`

_The response has no parameters._

### Segmentation And Reassembly Continue {#segmentation-and-reassembly-continue}

_Opcode:_ `0x22`

_Total length: 1 byte_

Continuation of a Segmentation and Reassembly message.

_Continue takes no parameters._

### Response

Potential status codes:

- `TRANSACTION_CONTINUE`

- `TRANSACTION_COMPLETE`

- `INVALID_LENGTH`

_The response has no parameters._

### Configuration Adv Addr Set {#configuration-adv-addr-set}

_Opcode:_ `0x40`

_Total length: 8 bytes_

Set the device's BLE advertisement address used for all BLE advertisment messages sent by the device.

_Adv Addr Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Addr Type                               | 1    | 0      | BLE advertising address type.
`uint8_t[6]`  | Adv Addr                                | 6    | 1      | BLE advertising address.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Configuration Adv Addr Get {#configuration-adv-addr-get}

_Opcode:_ `0x41`

_Total length: 1 byte_

Get the device's BLE advertisement address.

_Adv Addr Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Adv Addr Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Addr Type                               | 1    | 0      | Advertisement address type.
`uint8_t[6]`  | Addr                                    | 6    | 1      | Advertisement address.


### Configuration Channel Map Set {#configuration-channel-map-set}

_Opcode:_ `0x42`

_Total length: 2 bytes_

Set the channel map for advertisement packets. The device will send the advertisement packets on all enabled channels in increasing order. The channel map parameter is a bitmap, where the first bit represents channel 37, the second bit channel 38 and the third bit channel 39. The rest of the byte is ignored. Set to `0x07` to enable all channels, `0x01` to only enable channel 37, and so on.

_Channel Map Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Channel Map                             | 1    | 0      | Channel map bitfield for mesh to use, starting at channel 37.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Configuration Channel Map Get {#configuration-channel-map-get}

_Opcode:_ `0x43`

_Total length: 1 byte_

Get the channel map used for advertisement packets.

_Channel Map Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Configuration TX Power Set {#configuration-tx-power-set}

_Opcode:_ `0x44`

_Total length: 2 bytes_

Set the transmission power of the radio. Must be a valid enumeration in `serial_cmd_config_tx_power_value_t`.

_TX Power Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | TX Power                                | 1    | 0      | Transmit power of radio, @see serial_cmd_tx_power_value_t.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Configuration TX Power Get {#configuration-tx-power-get}

_Opcode:_ `0x45`

_Total length: 1 byte_

Get the transmission power of the radio.

_TX Power Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Configuration UUID Set {#configuration-uuid-set}

_Opcode:_ `0x53`

_Total length: 17 bytes_

Set the device UUID used for identifying the device during provisioning. If the UUID isn't set, the device will use a preprogrammed UUID.

_UUID Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | UUID                                    | 16   | 0      | Device UUID.

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Configuration UUID Get {#configuration-uuid-get}

_Opcode:_ `0x54`

_Total length: 1 byte_

Get the device UUID used for identifying the device during provisioning.

_UUID Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_UUID Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | Device UUID                             | 16   | 0      | Device UUID.


### Provisioning Scan Start {#provisioning-scan-start}

_Opcode:_ `0x61`

_Total length: 1 byte_

Start reporting of incoming unprovisioned beacons.

_Scan Start takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Provisioning Scan Stop {#provisioning-scan-stop}

_Opcode:_ `0x62`

_Total length: 1 byte_

Stop reporting of incoming unprovisioned beacons.

_Scan Stop takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Provisioning Provision {#provisioning-provision}

_Opcode:_ `0x63`

_Total length: 44 bytes_

Start provisioning of a device. When a provisioning link has been successfully established, a _Provisioning Link Established_ event is received. If an error occurs, a _Provisioning Link Closed_ event is received. After a link has been established, a _Provisioning Capabilities Received_ event will be emitted upon receiving the peer node's OOB capabilities. To continue the provisioning process, a _Provisioning OOB Use_ command must be sent to select which kind of OOB authentication to use.

_Provision Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | Context ID to use for this provisioning session.
`uint8_t[16]` | Target UUID                             | 16   | 1      | UUID of the device to provision.
`uint8_t[16]` | Network Key                             | 16   | 17     | Network key to give to the device.
`uint16_t`    | Network Key Index                       | 2    | 33     | Network key index.
`uint32_t`    | Iv Index                                | 4    | 35     | Initial IV index of the network.
`uint16_t`    | Address                                 | 2    | 39     | Unicast address to assign to the device.
`uint8_t`     | Iv Update Flag                          | 1    | 41     | IV update in progress flag.
`uint8_t`     | Key Refresh Flag                        | 1    | 42     | Key refresh in progress flag.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `ERROR_INVALID_DATA`

- `INVALID_LENGTH`

_Provision Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


### Provisioning Listen {#provisioning-listen}

_Opcode:_ `0x64`

_Total length: 1 byte_

As an uprovisioned device, listen for incoming provisioning requests.

_Listen takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Provisioning OOB Use {#provisioning-oob-use}

_Opcode:_ `0x66`

_Total length: 4 bytes_

Used to respond to the _Provisioning Capabilities Received_ event. It is used to select which kind of OOB authentication method to use. The values can be found in nrf_mesh_prov.h.  If authentication is enabled, the application will receive a _Provisioning Auth Request_ event requesting authentication data.  A _Provisioning ECDH Request_ will be received when the provisioner needs to calculate the ECDH shared secret for the nodes.  The _Provisioning Complete_ event is received when the provisioning procedure has completed successfully. At this point, a provisioner must wait for the _Provisioning Link Closed_ event before re-using the provisioning context.

_OOB Use Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | ID of context to set the oob method for.
`uint8_t`     | OOB Method                              | 1    | 1      | OOB method to use, see @ref nrf_mesh_prov_oob_method_t for values.
`uint8_t`     | Size                                    | 1    | 2      | Size of the OOB data.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_OOB Use Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


### Provisioning Auth Data {#provisioning-auth-data}

_Opcode:_ `0x67`

_Total length: 2..18 bytes_

Used to respond to a _Provisioning Auth Request_ event. It passes OOB authentication data back to the mesh stack.

_Auth Data Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | ID of the context to set the authentication data for.
`uint8_t[16]` | Data                                    | 0..16 | 1      | Authentication data.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Auth Data Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


### Provisioning ECDH Secret {#provisioning-ecdh-secret}

_Opcode:_ `0x68`

_Total length: 34 bytes_

Used to respond to a _Provisioning ECDH Request_ event. It passes the calculated ECDH shared secret back to the mesh stack.

_ECDH Secret Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | ID of the context to set the shared secret for.
`uint8_t[32]` | Shared Secret                           | 32   | 1      | ECDH shared secret.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_ECDH Secret Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


### Provisioning Keypair Set {#provisioning-keypair-set}

_Opcode:_ `0x69`

_Total length: 97 bytes_

Send a public/private keypair to the device. These keys are used for some of the encryption involved in provisioning.

_Keypair Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[32]` | Private Key                             | 32   | 0      | Private key.
`uint8_t[64]` | Public Key                              | 64   | 32     | Public key.

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Provisioning Capabilities Set {#provisioning-capabilities-set}

_Opcode:_ `0x6a`

_Total length: 10 bytes_

Used to set the out-of-band authentication capabilities of a device. The values for the parameters can be found in the various defines in the nrf_mesh_prov.h header file.

_Capabilities Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Num Elements                            | 1    | 0      | The number of elements in the device
`uint8_t`     | Public Key Type                         | 1    | 1      | The type of public key used in the device.
`uint8_t`     | Static OOB Types                        | 1    | 2      | The types of static OOB authentication methods.
`uint8_t`     | Output OOB Size                         | 1    | 3      | Maximum size of the OOB authentication output.
`uint16_t`    | Output OOB Actions                      | 2    | 4      | Available output actions for OOB authentication.
`uint8_t`     | Input OOB Size                          | 1    | 6      | Maximum size of the OOB authentication input.
`uint16_t`    | Input OOB Actions                       | 2    | 7      | Available input actions for OOB authentication.

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Init {#nrf-open-mesh-init}

_Opcode:_ `0x70`

_Total length: 1 byte_

Not implemented.

_Init takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Value Set {#nrf-open-mesh-value-set}

_Opcode:_ `0x71`

_Total length: 1 byte_

Not implemented.

_Value Set takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Value Enable {#nrf-open-mesh-value-enable}

_Opcode:_ `0x72`

_Total length: 1 byte_

Not implemented.

_Value Enable takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Value Disable {#nrf-open-mesh-value-disable}

_Opcode:_ `0x73`

_Total length: 1 byte_

Not implemented.

_Value Disable takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Start {#nrf-open-mesh-start}

_Opcode:_ `0x74`

_Total length: 1 byte_

Not implemented.

_Start takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Stop {#nrf-open-mesh-stop}

_Opcode:_ `0x75`

_Total length: 1 byte_

Not implemented.

_Stop takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Flag Set {#nrf-open-mesh-flag-set}

_Opcode:_ `0x76`

_Total length: 1 byte_

Not implemented.

_Flag Set takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Flag Get {#nrf-open-mesh-flag-get}

_Opcode:_ `0x77`

_Total length: 1 byte_

Not implemented.

_Flag Get takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh DFU Data {#nrf-open-mesh-dfu-data}

_Opcode:_ `0x78`

_Total length: 25 bytes_

Send DFU data to the device.

_DFU Data Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`nrf_mesh_dfu_packet_t` | DFU Packet                              | 24   | 0      | DFU packet data.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Value Get {#nrf-open-mesh-value-get}

_Opcode:_ `0x7a`

_Total length: 1 byte_

Not implemented.

_Value Get takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Build Version Get {#nrf-open-mesh-build-version-get}

_Opcode:_ `0x7b`

_Total length: 1 byte_

Not implemented.

_Build Version Get takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Access Addr Get {#nrf-open-mesh-access-addr-get}

_Opcode:_ `0x7c`

_Total length: 1 byte_

Not implemented.

_Access Addr Get takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Channel Get {#nrf-open-mesh-channel-get}

_Opcode:_ `0x7d`

_Total length: 1 byte_

Not implemented.

_Channel Get takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### nRF Open Mesh Interval Min ms Get {#nrf-open-mesh-interval-min-ms-get}

_Opcode:_ `0x7f`

_Total length: 1 byte_

Not implemented.

_Interval Min ms Get takes no parameters._

### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

### Bluetooth Mesh Enable {#bluetooth-mesh-enable}

_Opcode:_ `0x90`

_Total length: 1 byte_

Enable mesh operation. Starts radio scanning and transmissions.

_Enable takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Bluetooth Mesh Disable {#bluetooth-mesh-disable}

_Opcode:_ `0x91`

_Total length: 1 byte_

Disable mesh operation. Stops radio scanning and transmissions.

_Disable takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Bluetooth Mesh Subnet Add {#bluetooth-mesh-subnet-add}

_Opcode:_ `0x92`

_Total length: 19 bytes_

Add a mesh subnetwork to the device.

_Subnet Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Net Key Index                           | 2    | 0      | Mesh-global key index.
`uint8_t[16]` | Key                                     | 16   | 2      | Key to add.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Add Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Subnetwork handle operated on.


### Bluetooth Mesh Subnet Update {#bluetooth-mesh-subnet-update}

_Opcode:_ `0x93`

_Total length: 19 bytes_

Update a mesh subnetwork's root key.

_Subnet Update Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the subnet to change.
`uint8_t[16]` | Key                                     | 16   | 2      | Key to change to.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Update Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Subnetwork handle operated on.


### Bluetooth Mesh Subnet Delete {#bluetooth-mesh-subnet-delete}

_Opcode:_ `0x94`

_Total length: 3 bytes_

Delete a subnetwork from the device.

_Subnet Delete Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the subnet to delete.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Delete Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Subnetwork handle operated on.


### Bluetooth Mesh Subnet Get All {#bluetooth-mesh-subnet-get-all}

_Opcode:_ `0x95`

_Total length: 1 byte_

Get all known subnetwork key indexes.

_Subnet Get All takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Get All Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t[47]` | Subnet Key Index                        | 94   | 0      | List of all subnetwork key indexes known by the device.


### Bluetooth Mesh Subnet Count Max Get {#bluetooth-mesh-subnet-count-max-get}

_Opcode:_ `0x96`

_Total length: 1 byte_

Get the maximum number of subnetworks the device can fit.

_Subnet Count Max Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Subnet Count Max Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


### Bluetooth Mesh Appkey Add {#bluetooth-mesh-appkey-add}

_Opcode:_ `0x97`

_Total length: 21 bytes_

Add a mesh application key to the device.

_Appkey Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | App Key Index                           | 2    | 0      | Mesh-global key index.
`uint16_t`    | Subnet Handle                           | 2    | 2      | Handle of the subnetwork to add the appkey to.
`uint8_t[16]` | Key                                     | 16   | 4      | Key to add.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Add Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Application key handle operated on.


### Bluetooth Mesh Appkey Update {#bluetooth-mesh-appkey-update}

_Opcode:_ `0x98`

_Total length: 19 bytes_

Update a mesh application key.

_Appkey Update Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Handle of the appkey to change.
`uint8_t[16]` | Key                                     | 16   | 2      | Key to change to.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Update Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Application key handle operated on.


### Bluetooth Mesh Appkey Delete {#bluetooth-mesh-appkey-delete}

_Opcode:_ `0x99`

_Total length: 3 bytes_

Delete a application key from the device.

_Appkey Delete Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Handle of the appkey to delete.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Delete Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Application key handle operated on.


### Bluetooth Mesh Appkey Get All {#bluetooth-mesh-appkey-get-all}

_Opcode:_ `0x9a`

_Total length: 3 bytes_

Get all the application key indices of the stored application keys associated with a specific subnetwork.

_Appkey Get All Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the subnet to get all appkeys of.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Get All Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the Subnetwork associated with the application keys.
`uint16_t[46]` | Appkey Key Index                        | 92   | 2      | List of all application key indexes known by the device.


### Bluetooth Mesh Appkey Count Max Get {#bluetooth-mesh-appkey-count-max-get}

_Opcode:_ `0x9b`

_Total length: 1 byte_

Get the maximum number of application keys the device can fit.

_Appkey Count Max Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Appkey Count Max Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


### Bluetooth Mesh Devkey Add {#bluetooth-mesh-devkey-add}

_Opcode:_ `0x9c`

_Total length: 21 bytes_

Add a mesh device key to the device.

_Devkey Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Owner Addr                              | 2    | 0      | Unicast address of the device that owns the given devkey.
`uint16_t`    | Subnet Handle                           | 2    | 2      | Handle of the subnetwork to bind the devkey to.
`uint8_t[16]` | Key                                     | 16   | 4      | Key to add.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Devkey Add Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Devkey Handle                           | 2    | 0      | Device key handle operated on.


### Bluetooth Mesh Devkey Delete {#bluetooth-mesh-devkey-delete}

_Opcode:_ `0x9d`

_Total length: 3 bytes_

Delete a device key from the device.

_Devkey Delete Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Devkey Handle                           | 2    | 0      | Handle of the devkey to delete.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Devkey Delete Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Devkey Handle                           | 2    | 0      | Device key handle operated on.


### Bluetooth Mesh Devkey Count Max Get {#bluetooth-mesh-devkey-count-max-get}

_Opcode:_ `0x9e`

_Total length: 1 byte_

Get the maximum number of device keys the device can fit.

_Devkey Count Max Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Devkey Count Max Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


### Bluetooth Mesh Addr Local Unicast Set {#bluetooth-mesh-addr-local-unicast-set}

_Opcode:_ `0x9f`

_Total length: 5 bytes_

Set the start and count of the device's local unicast address.

_Addr Local Unicast Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Start Address                           | 2    | 0      | First address in the range of unicast addresses.
`uint16_t`    | Count                                   | 2    | 2      | Number of addresses in the range of unicast addresses.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Bluetooth Mesh Addr Local Unicast Get {#bluetooth-mesh-addr-local-unicast-get}

_Opcode:_ `0xa0`

_Total length: 1 byte_

Get the start and count of the device's local unicast addresses.

_Addr Local Unicast Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Local Unicast Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Start                           | 2    | 0      | First address in the range of unicast addresses.
`uint16_t`    | Count                                   | 2    | 2      | Number of addresses in the range of unicast addresses.


### Bluetooth Mesh Addr Get {#bluetooth-mesh-addr-get}

_Opcode:_ `0xa7`

_Total length: 3 bytes_

Get the raw representation of the address with the given handle. If the given address is a virtual address, the virtual UUID will be included in the response.

_Addr Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Handle of address to get raw representation of.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle requested.
`uint8_t`     | Addr Type                               | 1    | 2      | Address type of the given address. @see nrf_mesh_address_type_t.
`uint8_t`     | Subscribed                              | 1    | 3      | Flag indicating whether the given address is subscribed to or not.
`uint16_t`    | Raw Short Addr                          | 2    | 4      | Raw representation of the address.
`uint8_t[16]` | Virtual UUID                            | 16   | 6      | Optional virtual UUID of the given address.


### Bluetooth Mesh Addr Get All {#bluetooth-mesh-addr-get-all}

_Opcode:_ `0xa8`

_Total length: 1 byte_

Get a list of all address handles in the address pool, not including local unicast addresses.

_Addr Get All takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Get All Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t[47]` | Address Handles                         | 94   | 0      | List of all address handles known by the device, not including local unicast addresses.


### Bluetooth Mesh Addr Nonvirtual Count Max Get {#bluetooth-mesh-addr-nonvirtual-count-max-get}

_Opcode:_ `0xa9`

_Total length: 1 byte_

Get the maximum number of nonvirtual addresses the device can fit.

_Addr Nonvirtual Count Max Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Nonvirtual Count Max Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


### Bluetooth Mesh Addr Virtual Count Max Get {#bluetooth-mesh-addr-virtual-count-max-get}

_Opcode:_ `0xaa`

_Total length: 1 byte_

Get the maximum number of virtual addresses the device can fit.

_Addr Virtual Count Max Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Virtual Count Max Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


### Bluetooth Mesh Addr Subscription Add {#bluetooth-mesh-addr-subscription-add}

_Opcode:_ `0xa1`

_Total length: 3 bytes_

Add the specified address to the set of active address subscriptions.

_Addr Subscription Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address                                 | 2    | 0      | Address to add as a subscription address.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Subscription Add Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


### Bluetooth Mesh Addr Subscription Add Virtual {#bluetooth-mesh-addr-subscription-add-virtual}

_Opcode:_ `0xa2`

_Total length: 17 bytes_

Add the virtual address with the specified UUID to the set of active address subscriptions.

_Addr Subscription Add Virtual Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | UUID                                    | 16   | 0      | Virtual address UUID.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Subscription Add Virtual Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


### Bluetooth Mesh Addr Subscription Remove {#bluetooth-mesh-addr-subscription-remove}

_Opcode:_ `0xa3`

_Total length: 3 bytes_

Remove the address with the given handle from the set of active address subscriptions.

_Addr Subscription Remove Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Handle of address to remove from address subscription list.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Subscription Remove Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


### Bluetooth Mesh Addr Publication Add {#bluetooth-mesh-addr-publication-add}

_Opcode:_ `0xa4`

_Total length: 3 bytes_

Add the specified address to the set of active publish addresses.

_Addr Publication Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address                                 | 2    | 0      | Address to add as a publication address.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Publication Add Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


### Bluetooth Mesh Addr Publication Add Virtual {#bluetooth-mesh-addr-publication-add-virtual}

_Opcode:_ `0xa5`

_Total length: 17 bytes_

Add the virtual address with the specified UUID to the set of active publish addresses.

_Addr Publication Add Virtual Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | UUID                                    | 16   | 0      | Virtual address UUID.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Publication Add Virtual Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


### Bluetooth Mesh Addr Publication Remove {#bluetooth-mesh-addr-publication-remove}

_Opcode:_ `0xa6`

_Total length: 3 bytes_

Remove the address with the specified handle from the set of active publish addresses.

_Addr Publication Remove Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Handle of the address to remove from the publication address list.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Publication Remove Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


### Bluetooth Mesh Packet Send {#bluetooth-mesh-packet-send}

_Opcode:_ `0xab`

_Total length: 9..98 bytes_

Send a mesh packet. The source address handle must represent a local unicast address.

_Packet Send Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Appkey or devkey handle to use for packet sending. Subnetwork will be picked automatically.
`uint16_t`    | SRC Addr                                | 2    | 2      | Raw unicast address to use as source address. Must be in the range of local unicast addresses.
`uint16_t`    | DST Addr Handle                         | 2    | 4      | Handle of destination address to use in packet.
`uint8_t`     | TTL                                     | 1    | 6      | Time To Live value to use in packet.
`uint8_t`     | Reliable                                | 1    | 7      | Whether or not to make the transmission reliable.
`uint8_t[89]` | Data                                    | 0..89 | 8      | Payload of the packet.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_DATA_SIZE`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Bluetooth Mesh State Clear {#bluetooth-mesh-state-clear}

_Opcode:_ `0xac`

_Total length: 1 byte_

Reset the device and network state and erase the flash copies.

_State Clear takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Direct Firmware Upgrade Jump To Bootloader {#direct-firmware-upgrade-jump-to-bootloader}

_Opcode:_ `0xd0`

_Total length: 1 byte_

Immediately jump to bootloader mode. If successful, this call will not yield a command response. It will however yield a _Device Started_ event if the current bootloader supports serial communication.

_Jump To Bootloader takes no parameters._

### Response

Potential status codes:

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Direct Firmware Upgrade Request {#direct-firmware-upgrade-request}

_Opcode:_ `0xd1`

_Total length: 16 bytes_

Request a DFU transfer.

_Request Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to request.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 1      | Firmware ID to request.
`uint32_t`    | Bank Addr                               | 4    | 11     | Address in which to bank firmware.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Direct Firmware Upgrade Relay {#direct-firmware-upgrade-relay}

_Opcode:_ `0xd2`

_Total length: 12 bytes_

Relay a DFU transfer.

_Relay Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to relay.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 1      | Firmware ID of firmware that should be relayed.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Direct Firmware Upgrade Abort {#direct-firmware-upgrade-abort}

_Opcode:_ `0xd3`

_Total length: 1 byte_

Abort the ongoing DFU transfer. Will fail if there's no ongoing transfer.

_Abort takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Direct Firmware Upgrade Bank Info Get {#direct-firmware-upgrade-bank-info-get}

_Opcode:_ `0xd4`

_Total length: 2 bytes_

Get information about the firmware bank of the given type, if it exists.

_Bank Info Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to get bank info about.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Bank Info Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU type of the bank.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 1      | Firmware ID of the bank.
`uint8_t`     | Is Signed                               | 1    | 11     | Flag indicating whether the bank is signed with an encryption key.
`uint32_t`    | Start Addr                              | 4    | 12     | Start address of the bank.
`uint32_t`    | Length                                  | 4    | 16     | Length of the firmware in the bank.


### Direct Firmware Upgrade Bank Flash {#direct-firmware-upgrade-bank-flash}

_Opcode:_ `0xd5`

_Total length: 2 bytes_

Flash the bank with the given firmware type. If successful, this serial call does not produce a command response. Note that all volatile memory will be lost, as the device will restart. If the new firmware supports serial communication, the device issues a device started event when it's ready to receive new commands.

_Bank Flash Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to flash.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Direct Firmware Upgrade State Get {#direct-firmware-upgrade-state-get}

_Opcode:_ `0xd6`

_Total length: 1 byte_

Get the current state of the DFU module. Only works if the DFU module has been initialized.

_State Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_State Get Response Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Role                                    | 1    | 0      | This device's intended role in the transfer, @see nrf_mesh_dfu_role_t for accepted values.
`uint8_t`     | Type                                    | 1    | 1      | The DFU type of the transfer, @see nrf_mesh_dfu_type_t for accepted values.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 2      | The FWID of the new data in the transfer.
`uint8_t`     | State                                   | 1    | 12     | The current global state of the transfer, @see nrf_mesh_dfu_state_t for accepted values.
`uint8_t`     | Data Progress                           | 1    | 13     | The progress of the transfer in percent (0-100).


### Access Layer Model Pub Addr Set {#access-layer-model-pub-addr-set}

_Opcode:_ `0xe0`

_Total length: 5 bytes_

Set the publish address for a model instance.

_Model Pub Addr Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Pub Addr Get {#access-layer-model-pub-addr-get}

_Opcode:_ `0xe1`

_Total length: 3 bytes_

Get the publish address for a model instance.

_Model Pub Addr Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Pub Period Set {#access-layer-model-pub-period-set}

_Opcode:_ `0xe2`

_Total length: 5 bytes_

Set the publish address for a model instance.

_Model Pub Period Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle of the model that the access module should operate on.
`uint8_t`     | Resolution                              | 1    | 2      | @see access_publish_resolution_t
`uint8_t`     | Step Number                             | 1    | 3      | Must not be larger than @ref ACCESS_PUBLISH_PERIOD_STEP_MAX.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Pub Period Get {#access-layer-model-pub-period-get}

_Opcode:_ `0xe3`

_Total length: 3 bytes_

Get the publish period for a model instance.

_Model Pub Period Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Subs Add {#access-layer-model-subs-add}

_Opcode:_ `0xe4`

_Total length: 5 bytes_

Add a subscription address to a model instance.

_Model Subs Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Subs Remove {#access-layer-model-subs-remove}

_Opcode:_ `0xe5`

_Total length: 5 bytes_

Remove a subscription address from a model instance.

_Model Subs Remove Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Subs Get {#access-layer-model-subs-get}

_Opcode:_ `0xe6`

_Total length: 3 bytes_

Get the list of subscription addresses from a model instance.

_Model Subs Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model App Bind {#access-layer-model-app-bind}

_Opcode:_ `0xe7`

_Total length: 5 bytes_

Bind an application key to a model instance.

_Model App Bind Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model App Unbind {#access-layer-model-app-unbind}

_Opcode:_ `0xe8`

_Total length: 5 bytes_

Unbind an application key from a model instance.

_Model App Unbind Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model App Get {#access-layer-model-app-get}

_Opcode:_ `0xe9`

_Total length: 3 bytes_

Get all the application keys bound to a model instance.

_Model App Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Pub App Set {#access-layer-model-pub-app-set}

_Opcode:_ `0xea`

_Total length: 5 bytes_

Set the application key to be used when publishing for a model instance.

_Model Pub App Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Pub App Get {#access-layer-model-pub-app-get}

_Opcode:_ `0xeb`

_Total length: 3 bytes_

Get the application key used when publishing for a model instance.

_Model Pub App Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Pub TTL Set {#access-layer-model-pub-ttl-set}

_Opcode:_ `0xec`

_Total length: 4 bytes_

Set the default ttl value used when publishing for a model instance.

_Model Pub TTL Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle of the model that the access module should operate on.
`uint8_t`     | TTL                                     | 1    | 2      | 

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model Pub TTL Get {#access-layer-model-pub-ttl-get}

_Opcode:_ `0xed`

_Total length: 3 bytes_

Get the default ttl value used when publishing for a model instance.

_Model Pub TTL Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Elem Loc Set {#access-layer-elem-loc-set}

_Opcode:_ `0xee`

_Total length: 5 bytes_

Set the location descriptor for an element.

_Elem Loc Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | The index of the element addressed.
`uint16_t`    | Location                                | 2    | 2      | Location value for the element.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Elem Loc Get {#access-layer-elem-loc-get}

_Opcode:_ `0xef`

_Total length: 3 bytes_

Get the location descriptor for an element.

_Elem Loc Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | 

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Elem Sig Model Count Get {#access-layer-elem-sig-model-count-get}

_Opcode:_ `0xf0`

_Total length: 3 bytes_

Get the number of Bluetooth SIG models for an element.

_Elem Sig Model Count Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | 

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Elem Vendor Model Count Get {#access-layer-elem-vendor-model-count-get}

_Opcode:_ `0xf1`

_Total length: 3 bytes_

Get the number of vendor specific models for an element.

_Elem Vendor Model Count Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | 

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Model ID Get {#access-layer-model-id-get}

_Opcode:_ `0xf2`

_Total length: 3 bytes_

Get the model ID of a model instance.

_Model ID Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Handle Get {#access-layer-handle-get}

_Opcode:_ `0xf3`

_Total length: 7 bytes_

Get the handle assigned to the model instance of a model based on the element index and model ID.

_Handle Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | 
`access_model_id_t` | Model ID                                | 4    | 2      | 

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Elem Models Get {#access-layer-elem-models-get}

_Opcode:_ `0xf4`

_Total length: 3 bytes_

Get the array of handles corresponding to an element.

_Elem Models Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | 

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_The response has no parameters._

### Access Layer Access Flash Store {#access-layer-access-flash-store}

_Opcode:_ `0xf5`

_Total length: 1 byte_

Store the access layer information to flash.

_Access Flash Store takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

### Model Specific Models Get {#model-specific-models-get}

_Opcode:_ `0xfc`

_Total length: 1 byte_

Get a list of all the models available on the device.

_Models Get takes no parameters._

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_The response has no parameters._

### Model Specific Init {#model-specific-init}

_Opcode:_ `0xfd`

_Total length: 7..98 bytes_

Calls the initializer of the addressed model in order to create a new instance.

_Init Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`serial_cmd_model_specific_init_header_t` | Model Init Info                         | 6    | 0      | Basic information that is always needed to initialize a model
`uint8_t[91]` | Data                                    | 0..91 | 6      | Additional data provided to the initializer

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

### Model Specific Command {#model-specific-command}

_Opcode:_ `0xfe`

_Total length: 3..98 bytes_

Forwards a model specific command to a model instance. See the serial handler for the specific model being commanded for more information.

_Command Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`serial_cmd_model_specific_command_header_t` | Model Cmd Info                          | 2    | 0      | Contains the handle of the model being addressed.
`uint8_t[95]` | Data                                    | 0..95 | 2      | Additional data provided to the event

### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

