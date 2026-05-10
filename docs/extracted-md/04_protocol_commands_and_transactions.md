# Protocol Commands And Transactions

LSM6DS3TR-C exposes 7-bit register addresses over I2C and SPI. The register map is byte-addressed, and output data is generally little-endian when `CTRL3_C.BLE=0` (default). Source: datasheet, pp. 40-41, 63.

## I2C Slave Protocol

| Item | Behavior | Source |
|---|---|---|
| 7-bit address | `0x6A` when `SA0=0`; `0x6B` when `SA0=1`. | Datasheet, pp. 39-40 |
| Single-byte write | START, SAD+W, sub-address, data, STOP. | Datasheet, p. 40 |
| Multi-byte write | START, SAD+W, sub-address, data bytes, STOP. | Datasheet, p. 40 |
| Single-byte read | START, SAD+W, sub-address, repeated START, SAD+R, data, NACK, STOP. | Datasheet, p. 40 |
| Multi-byte read | Same as single-byte read, with ACK after each byte until final NACK/STOP. | Datasheet, p. 40 |
| Auto-increment | Controlled by `CTRL3_C.IF_INC` (`0x12[2]`); default value is 1. | Datasheet, pp. 40, 63 |
| Bit order | Data bytes transmit most-significant bit first. | Datasheet, p. 40 |

## SPI Protocol

| Item | Behavior | Source |
|---|---|---|
| Bus role | SPI slave. | Datasheet, p. 41 |
| Lines | 4-wire uses `CS`, `SPC`, `SDI`, `SDO`; 3-wire uses `SDA` as serial data output. | Datasheet, pp. 20, 41, 44 |
| Clocking | `SDI` and `SDO` are driven on falling edge of `SPC` and captured on rising edge. | Datasheet, p. 41 |
| Control byte | Bit 0 in datasheet description is the read/write bit; `0` write, `1` read; address field is AD[6:0]. | Datasheet, p. 41 |
| Multi-byte access | Add 8 clocks per extra byte; address stays same when `IF_INC=0` and increments when `IF_INC=1`. | Datasheet, p. 41 |
| 3-wire mode | Set `CTRL3_C.SIM` (`0x12[3]`) to 1. | Datasheet, pp. 44, 63 |

## Output Reads

- Gyroscope outputs are `OUTX/Y/Z_L/H_G` at `0x22..0x27`, 16-bit two's-complement values. Source: datasheet, pp. 73-75.
- Accelerometer outputs are `OUTX/Y/Z_L/H_XL` at `0x28..0x2D`, 16-bit two's-complement values. Source: datasheet, pp. 75-76.
- Temperature output is `OUT_TEMP_L/H` at `0x20..0x21`, 16-bit two's-complement, with typ. 0 LSB at 25 degC and 256 LSB/degC. Source: datasheet, pp. 25, 73.
- With default `CTRL3_C.BLE=0`, output LSB bytes are at the lower addresses (`0x20`, `0x22`, `0x24`, `0x26`, `0x28`, `0x2A`, `0x2C`) and MSB bytes are at the next address. Setting `BLE=1` swaps byte order for output data. Source: datasheet, pp. 63, 73-76.

## Reserved And Access Notes

- The datasheet's register map marks many holes between `0x00` and `0x7F`; only documented user-interface addresses in Table 19 have defined behavior. Source: datasheet, pp. 49-52.
- Embedded function bank A/B registers are visible through `FUNC_CFG_ACCESS` (`0x01`); boot-loaded registers in those banks are marked as content not to be changed. Source: datasheet, pp. 53, 97-98.
