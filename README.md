# ESPHome Belimo NFC

Read **live data** from a Belimo NFC actuator — and change its setpoint — with an **ESP32 + PN532**, straight into [Home Assistant](https://www.home-assistant.io/) via [ESPHome](https://esphome.io/). No gateway, no cloud, no Bluetooth adapter.

The first supported device is the **EWC SBD2 / Belimo Smart Bypass Damper** (component `belimo_sbd`). It exposes the damper **position (%)**, the **measured static pressure**, the **pressure setpoint**, and **fan / bypass run-time hours** as sensors, plus a **setpoint dropdown** you can change from HA.

Belimo's own documentation and most prior guidance says this can't be done without their handheld ZIP‑BT‑NFC / LINK.10 tools and the proprietary app. It can. This project talks to the actuator's NFC chip directly using the same protocol the Belimo Assistant app uses — recovered by decompiling that app — so a $5 reader does what the official adapter does, permanently and continuously.

> ⚠️ **Unofficial.** Not affiliated with, endorsed by, or supported by Belimo or EWC Controls. Use at your own risk. Writing the setpoint changes how a live HVAC damper regulates static pressure. See [Safety](#safety).

---

## How it works

The Belimo NFC chip is an **NXP NTAG I²C plus** (ISO 14443‑A, 13.56 MHz). Static identity lives in its EEPROM, but the **live values** (position, pressure, …) are not sitting in memory — they are fetched on demand over a **pass‑through "MP tunnel"**:

1. The ESP writes an **MP‑Bus request** into the chip's 64‑byte SRAM mailbox.
2. The actuator's processor (while powered) answers into the same mailbox.
3. The ESP reads the answer back.

Each value is addressed by an MP‑Bus **pool id** with a fixed‑point **scaling** factor, both taken from the device profile. The component implements the full handshake (session‑register polling, SRAM write/read, CRC‑16/ARC framing, protocol‑version detect) on top of ESPHome's stock `pn532_spi`. Details and the exact frames are in [`PROTOCOL_NOTES.md`](PROTOCOL_NOTES.md).

**The actuator must be powered (24 V)** for live data — that's what runs the processor behind the tunnel.

---

## Hardware

| Part | Notes |
|---|---|
| **ESP32** | Any ESP32 variant (classic, S2, S3, C3, …) with a **free hardware SPI bus** and one spare GPIO. ~Any ESPHome‑supported ESP32 board works. |
| **PN532 module** | The common red "PN532 V3" board (Elechouse / HiLetgo / ShillehTek) or Adafruit's breakout. **Set it to SPI mode** (see below). |
| **Wires** | 5 jumpers (4 SPI + 3V3/GND). Keep them short. |

**PN532 → SPI mode:** the red boards select interface with two DIP switches. SPI is usually **SW1 = OFF, SW2 = ON** (silk sometimes labels it `0 / 1`). I²C is intentionally **not** used here — the ESP32 I²C peripheral has clock‑stretching issues with the PN532 that cause lock‑ups under continuous polling; SPI is reliable.

**Mounting:** this is an RF link. Hold/mount the PN532 **antenna flat against the damper's NFC logo**, touching, and fix it so it can't drift (tape or a bracket). If it's off‑center you'll see intermittent reads. A phone's NFC antenna is far more forgiving than the little PN532 coil, so a position that "works on the phone" may still need nudging for the module.

### Wiring

Connect the PN532 to the ESP32:

| PN532 pin | ESP32 pin | Required? |
|---|---|---|
| **SCK** | SPI clock GPIO | **Yes** — must be a pin your board can use for SPI clock |
| **MISO** | SPI MISO GPIO | **Yes** — must be a valid SPI data‑in pin |
| **MOSI** | SPI MOSI GPIO | **Yes** — must be a valid SPI data‑out pin |
| **SS / CS / NSS** | any free GPIO | **Yes**, but **any** GPIO — this is just a chip‑select line |
| **VCC** | 3V3 | The red boards are 3.3 V logic; power from the ESP's 3V3 |
| **GND** | GND | |
| RSTPDN, IRQ | — | **Not used.** Leave unconnected. |

**Which pins can I use?**

- The **three SPI data pins** (SCK/MISO/MOSI) must be pins your ESP32 can route to a hardware SPI bus.
  - On the **original ESP32**, the safe choice is the VSPI default set: **SCK 18, MISO 19, MOSI 23** (HSPI 14/12/13 also works). Avoid input‑only pins (34–39) for SCK/MOSI, and avoid the strapping/flash pins (6–11).
  - On **ESP32‑S3 / S2 / C3**, the SPI matrix is flexible — almost any free GPIO works for all three.
- The **CS/SS pin can be any free GPIO** — it is not special. Pick anything that isn't a strapping or flash pin.

The example uses an ESP32‑S3 with **SCK 12, MISO 13, MOSI 11, CS 10**. Change these to whatever is convenient on your board.

---

## Install

You need ESPHome **2026.8 or newer** (the component adapts to the PN532 driver API used in recent versions).

### Option A — from GitHub (recommended)

Add this to your device YAML and ESPHome pulls the component automatically:

```yaml
external_components:
  - source: github://isaacbreuer/esphome-belimo-nfc@main
    components: [belimo_sbd]
```

### Option B — local copy

Copy the [`components/belimo_sbd`](components/belimo_sbd) folder into your ESPHome config directory so you have `config/components/belimo_sbd/…`, then:

```yaml
external_components:
  - source:
      type: local
      path: components
```

Either way, see [`example.yaml`](example.yaml) for a complete, working configuration. The short version:

```yaml
spi:
  clk_pin: GPIO12
  miso_pin: GPIO13
  mosi_pin: GPIO11

pn532_spi:
  id: my_pn532
  cs_pin: GPIO10
  update_interval: never      # belimo_sbd drives the reader

belimo_sbd:
  id: damper
  pn532_id: my_pn532
  update_interval: 3s
  sensors:
    - pool_id: 9              # position
      scale: 0.1
      name: "Damper Position"
      unit_of_measurement: "%"
    - pool_id: 11             # static pressure
      scale: 0.0001
      name: "Static Pressure"
      unit_of_measurement: '"wc'
      accuracy_decimals: 3
    # ...see example.yaml for setpoint, run-times, and the setpoint dropdown
```

---

## Configuration

### `belimo_sbd:`

| Option | Default | Description |
|---|---|---|
| `pn532_id` | — (required) | ID of the `pn532_spi` reader to drive. |
| `update_interval` | `10s` | How often to poll. A full sweep of several pools takes ~5 s, so values below that just poll continuously (harmless — reads don't wear anything). Setpoint writes fire immediately regardless. |
| `protocol_version` | `0` | `0` auto‑detects the Belimo NFC protocol version (recommended). Force `2` or `3` only if detection ever misbehaves. |
| `dump_memory` | `true` | On first contact, log a one‑time **read‑only** hex dump of the chip memory. Handy for exploring a new device; set `false` for normal use. |
| `stale_timeout` | `20s` | How long to keep showing the last values after reads start failing, before marking sensors *unknown*. This rides over the brief NFC lock that happens while the actuator saves a setpoint to EEPROM, so a write doesn't flicker the other sensors. A real disconnect still clears after this long. |
| `sensors:` | `[]` | List of values to read (below). |

### `sensors:` entries

Each entry is a normal ESPHome sensor **plus**:

| Option | Default | Description |
|---|---|---|
| `pool_id` | — (required) | MP‑Bus pool id of the value (see the profile table below). |
| `scale` | `1.0` | Multiply the raw integer by this. From the device profile's fixed‑point `scaling`. |

All standard sensor options (`name`, `unit_of_measurement`, `accuracy_decimals`, `state_class`, `filters`, …) work as usual.

### Setpoint control

`belimo_sbd` exposes one write method, hard‑scoped to the setpoint selector:

```cpp
id(damper).request_setpoint_select(uint8_t preset);   // preset 0..11, anything else ignored
```

Wire it to a `select` (dropdown) or buttons in YAML — see [`example.yaml`](example.yaml). The recommended pattern uses a template `select` whose options are exactly the allowed presets, and drives the dropdown's shown value from the pool‑8 read‑back so it always reflects the real damper. The write is applied on the next poll (kicked immediately, ~1–2 s), after which all values are read back.

---

## SBD2 device profile

Values available on the SBD2 (from the Belimo profile). Position, pressure, setpoint and the run‑times are the useful ones; the rest are internal control‑loop variables, all readable.

| Pool | Name | Scale | Unit | Meaning |
|---:|---|---|---|---|
| 8 | SpSelect | — | enum | Setpoint preset selector (**writable**) |
| 9 | SpPos | 0.1 | % | **Damper position** |
| 10 | DpSp | 0.0001 | "wc | Active pressure setpoint |
| 11 | DP_FilterOut | 0.0001 | "wc | **Measured static pressure** |
| 12 | SpPosPreAD | 0.1 | % | Position setpoint, pre anti‑dither |
| 13 | SpPosPostAD | 0.1 | % | Position setpoint, post anti‑dither |
| 14 | DPPreFilter | 0.0001 | "wc | Differential pressure, pre‑filter |
| 15 | DPPostFilter | 0.0001 | "wc | Differential pressure, post‑filter |
| 16–19 | Save0/100DpOut, Save0/100FbOut | 0.0001 / 0.1 | "wc / % | Commissioning saved points |
| 20 | PIDPrVal | 0.0001 | "wc | PID process value |
| 21 | PIDSp | 0.0001 | "wc | PID setpoint |
| 22 | PIDOut | 0.1 | % | PID output |
| 23 | DmprChrOut | 0.1 | % | Damper characteristic output |
| 24 | FanOnTime | 1 | h | Total fan run time |
| 25 | BypassTime | 1 | h | Total bypass‑active time |

**SpSelect (pool 8) presets:**

| Value | Setpoint | Value | Setpoint |
|---:|---|---:|---|
| 0 | Default / Learned | 6 | 0.50" wc |
| 1 | 0.25" wc | 7 | 0.55" wc |
| 2 | 0.30" wc | 8 | 0.60" wc |
| 3 | 0.35" wc | 9 | 0.65" wc |
| 4 | 0.40" wc | 10 | 0.70" wc |
| 5 | 0.45" wc | 11 | 0.75" wc |

---

## Adding other Belimo NFC devices

The transport — NTAG I²C plus pass‑through, the MP‑Bus framing, pool read/write, CRC, protocol‑version detect — is **device‑agnostic**. Reading is already generic: you just list the `pool_id`s and `scale`s for your device. Everything device‑specific lives in the **profile** (which pools exist, their scaling, which are writable).

Those profiles ship inside the **Belimo Assistant** app. Each device has an XML profile (`PROFILE_*.xml`) listing every datapoint with its `pool-id`, `basetype`, and fixed‑point `format` (scaling), plus a YAML profile with the friendly labels and units. That's exactly where this project's SBD map came from. For a different actuator (a VAV controller, a damper/valve actuator, an energy valve, …), pull its profile from the app and translate the pools into `sensors:` entries the same way.

**Writing** is the only part that's device‑specific in code (this build hard‑codes the SBD's `SpSelect` on pool 8). To publish support for another device as part of this family, add a sibling component — e.g. `belimo_vav` — reusing this one's NFC/tunnel engine and exposing that device's writable datapoint(s). The naming (`belimo_sbd`, `belimo_<device>`) keeps each device a clear, self‑contained type in the same repo. PRs welcome.

If you just want to **read** another device, you may well be able to do it today with `belimo_sbd` by listing that device's pools — the read path makes no SBD‑specific assumptions.

---

## Safety

- **Default is read‑only.** The only value this component can write is `SpSelect` (pool 8), and only via the explicit `request_setpoint_select()` you wire up. Every other datapoint is read‑only here; there is no code path that writes any other register.
- **Out‑of‑range writes are rejected twice:** the dropdown only offers the 12 valid presets (ESPHome rejects anything else before it reaches the firmware), and the firmware ignores any index outside 0–11. A stray value is never transmitted.
- **Reads never touch EEPROM.** Requests go to volatile SRAM; polling as fast as you like causes no wear. Only an actual setpoint *write* causes an EEPROM save in the actuator (occasional, by design).
- **It's a live HVAC device.** Changing the setpoint changes how the damper regulates building static pressure. Test when a brief excursion is acceptable, and remember any change is reversible by selecting another preset.
- Treat the included protocol notes and memory maps as "works for my unit." Firmware and profiles vary.

---

## How the protocol was reverse‑engineered

Short version: decompile the Belimo Assistant Android app, read how it builds its NFC requests, confirm against the chip. The device profiles (pool ids, scaling) are plain XML/YAML assets in the APK. The transport — how the app drives the NTAG I²C pass‑through SRAM, the MP‑Bus command/answer framing, the CRC, the protocol‑version query — came from the app's compiled Dart code, recovered with [blutter](https://github.com/worawit/blutter), then verified byte‑for‑byte against a live reader and an [NXP TagInfo](https://play.google.com/store/apps/details?id=com.nxp.taginfolite) scan. Full write‑up in [`PROTOCOL_NOTES.md`](PROTOCOL_NOTES.md).

---

## Credits

- **Protocol reverse‑engineering, decoding, and the ESPHome component** were done by **Claude** (Anthropic) — app decompilation and analysis, the NTAG I²C pass‑through / MP‑Bus tunnel decode, CRC and framing, and the full `belimo_sbd` implementation and debugging.
- **Hardware, on‑site testing, and the whole project direction** by [@isaacbreuer](https://github.com/isaacbreuer).
- Built on [ESPHome](https://esphome.io/)'s `pn532` component. Reverse engineering used [blutter](https://github.com/worawit/blutter) and [NXP TagInfo](https://www.nxp.com/products/nxp-product-information/nxp-mobile-technologies/mifare/taginfo-application:TAGINFO).

A collaboration between a human who wouldn't let "it can't be done" stand and an AI that read the app until it could. 🤝

---

## License

[MIT](LICENSE) — do what you like, no warranty. Not affiliated with Belimo or EWC Controls. "Belimo" and "EWC" are trademarks of their respective owners; used here only to describe compatibility.
