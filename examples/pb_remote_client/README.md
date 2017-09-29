# Remote provisioning client example

This example demonstrates how to use the remote provisioning client model to
provision remote devices. To use this example, flash the client application
to one device and the server application to two or more devices.

For more information about remote provisioning, see the
[PB-remote API reference](@ref PB_REMOTE).

## User interface

The client is controlled over RTT with a few simple commands:

- Send `1` to start normal PB-ADV on the first unprovisioned device.
- Send `2N`, where `N` is the handle of the newly provisioned device, to set the handle of the
  remote provisioning server.
- Send `3` to start remote scanning on the current server.
- Send `4` to cancel remote scanning on the current server.
- Send `5N`, where `N` is the device number, to start remote provisioning of the first unprovisioned
  device known to the current provisioning server.

> **Important:** These commands must happen in order.

