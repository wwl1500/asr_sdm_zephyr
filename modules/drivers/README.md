# Driver Notes

The Dynamixel RS-485 driver currently applies a board-specific runtime
override so live XM430 bring-up can be validated without changing other
modules:

- UART bus is forced to `uart0`
- UART baudrate is forced to `57600`
- Default validated path uses a manual-direction transceiver with `GPIO28`
  (`D2` on XIAO RP2350) driving `DE/RE`

This override lives in [`src/dynamixel_bus.c`](./src/dynamixel_bus.c) and
intentionally ignores the current devicetree parent UART and `dir-gpios`
property until the higher-level communication path is migrated.
