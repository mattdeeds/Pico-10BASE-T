# Pico-10BASE-T (with RX)

A fork of [kingyoPiyo/Pico-10BASE-T](https://github.com/kingyoPiyo/Pico-10BASE-T)
that adds a **receive (RX) path**, turning the original transmit-only design into a
**full bidirectional 10BASE-T link** on the Raspberry Pi Pico 2 (RP2350).

kingyoPiyo's project bit-bangs 10BASE-T **transmit** from a Pico using PIO and a
handful of resistors. This fork keeps that TX design and adds the missing half: a
PIO + DMA + software-Manchester **receiver** front-ended by an RS-485 transceiver,
so the Pico can both send and receive real Ethernet frames.

> **Status:** prototype / reference implementation. TX and RX both work end-to-end
> against a real host NIC — UDP frames decode with a valid FCS in both directions.
> It's an educational software-PHY experiment, not a production NIC.
> **Do not connect to PoE equipment.**

## What this fork adds

- **`src/eth_rx.c` / `src/eth_rx.h`** — the RX path. A 60 MHz PIO sampler
  (3 samples per Manchester half-bit) streams GP13 into a 32 KB DMA ring buffer;
  software then finds the active run, phase-locks to the preamble's first falling
  edge, Manchester-decodes, locates the SFD, and verifies the FCS
  (CRC-32/IEEE-802.3).
- **`src/rx_10base_t.pio`** — the PIO sampler program for the receive path.
- **RS-485 transceiver front end** — kingyoPiyo's original drives a bare resistor
  network (transmit only). To *receive*, this fork uses an **ISL3177E** RS-485
  transceiver + **HR911105A** RJ45 magnetics so the differential line can be sliced
  back to a single-ended digital stream. The circuit is based on the
  [Niccle](https://github.com/timonvo/niccle) project.
- **`src/main.c`** — wires the RX sampler in and decodes a frame once per second as
  a smoke test, alongside the original TX loop.

## Hardware

- **Raspberry Pi Pico 2** (RP2350)
- **ISL3177E** RS-485 transceiver (single-ended digital ↔ differential line)
- **HR911105A** RJ45 jack with integrated magnetics
- Passives per the Niccle reference circuit

| Signal | Pico 2 pin |
|---|---|
| ISL3177E `RO` (receiver out → MCU) | GP13 |
| ISL3177E `DI` (driver in ← MCU) | GP14 |
| Onboard LED | GP25 |

(kingyoPiyo's original TX-only build used GP16/GP17 into a resistor network; this
fork moves to GP13/GP14 into the transceiver.)

## Build & flash

A standard [Pico SDK](https://github.com/raspberrypi/pico-sdk) CMake project (the
project root is `src/`):

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cd src
mkdir build && cd build
cmake ..
make
# flash build/Pico-10BASE-T.uf2 — hold BOOTSEL while plugging in, then:
picotool load -x Pico-10BASE-T.uf2
```

The device logs over USB serial (`pico_enable_stdio_usb`).

## Host setup (10BASE-T peer)

The device emits Normal Link Pulses only (no auto-negotiation), so force the peer
NIC to 10 Mbit half-duplex:

```bash
sudo ethtool -s <iface> speed 10 duplex half autoneg off
sudo ip addr add 192.168.37.19/24 dev <iface>
```

- **TX:** the Pico broadcasts UDP on `192.168.37.0/24`; a listener bound to port
  1234 receives the "Hello World" payload.
- **RX:** blast UDP broadcasts at the Pico and watch it decode them over USB serial
  with `FCS OK`.

## Credits

- [kingyoPiyo/Pico-10BASE-T](https://github.com/kingyoPiyo/Pico-10BASE-T) — the
  original C/PIO 10BASE-T **transmit** design this forks (MIT, © 2022 kingyo). All
  of the TX PIO + Manchester encoding is their work.
- [Niccle](https://github.com/timonvo/niccle) — the RS-485-transceiver receive
  circuit and software Manchester-decode approach the RX path is based on.

A Rust rewrite of this project also exists — targeting the RP2350 Hazard3 RISC-V
cores with a [smoltcp](https://github.com/smoltcp-rs/smoltcp) IP stack, IRQ-driven
multicore RX, and a cyw43 wireless-router mode.

## License

MIT — see [LICENSE](LICENSE). Original work Copyright © 2022 kingyo; the RX
additions in this fork are released under the same license.
