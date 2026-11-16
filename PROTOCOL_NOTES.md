# Belimo SBD2 NFC protocol notes

Recovered from **Belimo Assistant 2 v6.15.1 (build 3478)** with [blutter](https://github.com/worawit/blutter) (Dart 3.11.5) and jadx, then **verified byte‑for‑byte against a live EWC SBD2** and an NXP TagInfo scan. Source paths are `package:belimo_assistant/device_communication/...`.

Status: **confirmed working.** Reads and a setpoint write both run on hardware.

## Chip

Confirmed by TagInfo: **NXP NTAG I²C plus 1K, NT3H2111** (ISO 14443‑A, NFC Forum Type 2), 7‑byte UID starting `04`, CC `E1 10 6F 00` (888 bytes user memory, 64‑byte SRAM). The app also supports AMS and Panasonic NFC families on other devices; the SBD2 is the NXP path, which the PN532 speaks directly.

## Memory (sector 0)

| Pages | Content |
|---|---|
| 0x00–0x03 | UID / static lock / Capability Container |
| 0x04–0xE1 | User memory (EEPROM; Belimo TLV data blocks — identity, readable unpowered) |
| 0xE8–0xE9 | Configuration registers |
| 0xEC–0xED | **Session registers** (8 bytes) |
| 0xF0–0xFF | **SRAM**, 64 bytes (pass‑through "MP tunnel") |

Session‑register bytes used: `[0]` NC_REG, `[6]` NS_REG. No password (AUTH0 = 0xFF, ACCESS = 0x00).

## Tunnel handshake (`nxp_nfc_operations.dart`)

| State | NS.RF_LOCKED 0x20 | NC.PTHRU_ON 0x40 | NC.PTHRU_DIR 0x01 | NS.SRAM_RF_READY 0x08 |
|---|---|---|---|---|
| Ready to write request | 1 | 1 | 1 (RF→I²C) | 0 |
| Answer ready to read | 1 | 1 | 0 (I²C→RF) | 1 |

Sequence: poll session regs → write 64 B to SRAM `0xF0–0xFF` (writing page `0xFF` hands the buffer to the actuator) → poll until answer ready → read 64 B. If an answer is already pending before writing, read once to flush it first. Session registers read fine with a plain `READ (0x30)` of `0xEC` through the PN532 `InDataExchange`; `FAST_READ (0x3A)` is a fallback.

**No MP_Login and no write‑sequence‑number** are needed on the powered tunnel — the sequence counter (`NfcMobileAppWriteSequenceNumber`) is only used by the app's *powerless* (unpowered‑EEPROM) write path.

## Protocol version (Get‑BNPV)

Write `00 00` (zero‑padded to 64 B) and read back; first answer byte is the version (`02`/`03`; a "length mismatch" protocol error also implies v2). Version sets the SRAM frame offset: **0 for v2, 48 for v3**; the answer is read at the same offset. (The SBD2 tested is v2, offset 0.)

## Frames (`belimo_nfc_command_v2.dart`)

**Request:** `[LEN][CMD][MP bytes…][CRC hi][CRC lo]`, LEN = total length.

| CMD | Meaning |
|---|---|
| 01 | MP command |
| 02 | Get last MP answer (`04 02 01 83`) |

**Answer:** `[LEN][TYPE][MP header][data…][CRC hi][CRC lo]`

| TYPE | Meaning |
|---|---|
| 03 | MP answer |
| 04 | Delayed — poll with CMD 02, up to 7000 ms |
| 05 | MP answer error |
| 06 | NFC protocol error |

The MP answer carries a one‑byte **MP header** before the data whose **high nibble = data‑byte count** (`0x4D` = 4 data bytes, `0x1D` = 1 data byte). The value is those data bytes, **big‑endian**.

**CRC:** CRC‑16/ARC (poly `0xA001` reflected, init `0x0000`, no final XOR) over LEN‑2 bytes, stored **big‑endian** (hi, lo).

## MP‑Bus commands (`mp_command.dart`, operation table)

MP message = `[(len)<<4 | addrmode][opcode][params…]`, where `len` = 1 (opcode) + param bytes, addrmode PP = 1. Values big‑endian. Write value width = the datapoint's `numberOfBytes` (`_encodeRawBigEndian`).

| Operation | Opcode | Req params | Answer data |
|---|---|---|---|
| MP_Get_Data | 0x6F | pool hi, lo | 4 |
| MP_Set_Data | 0x6E | pool hi, lo, value… | 0 (ack) |
| MP_Get_NextBlock | 0x71 | block | 7 |
| MP_Set_NextBlock | 0x70 | … | — |
| MP_Login | 0x4E | — | (not used over NFC) |

### Verified frames

Reads (`08 01 31 6F 00 <pool> <crc hi> <crc lo>`):

```
SpSelect     (pool  8)              08 01 31 6F 00 08 AF 03   -> answer e.g. 06 03 1D 08 …  (1 data byte)
SpPos        (pool  9, x0.1    %)   08 01 31 6F 00 09 6F C2   -> answer      09 03 4D 00 00 00 53 …  (4 data, =83 ->8.3%)
DpSp         (pool 10, x0.0001 "wc) 08 01 31 6F 00 0A 6E 82
DP_FilterOut (pool 11, x0.0001 "wc) 08 01 31 6F 00 0B AE 43
FanOnTime    (pool 24, hours)       08 01 31 6F 00 18 63 02
BypassTime   (pool 25, hours)       08 01 31 6F 00 19 A3 C3
```

Write SpSelect (`09 01 41 6E 00 08 <value> <crc hi> <crc lo>`, header `0x41` = 4 MP bytes, PP):

```
SpSelect = 7 (0.55" wc)   09 01 41 6E 00 08 07 34 FE
SpSelect = 8 (0.60" wc)   09 01 41 6E 00 08 08 30 BE
SpSelect = 9 (0.65" wc)   09 01 41 6E 00 08 09 F0 7F
```

A write returns a short MP ack (`05 03 0D 34 D1`, TYPE 03, zero data bytes); success is confirmed by reading `SpSelect`/`DpSp` back. Writing a setpoint triggers an EEPROM save in the actuator, during which the chip briefly locks its memory to the internal side — reads can miss for ~1 cycle, so the component tolerates short gaps before blanking.
