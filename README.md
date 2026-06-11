# Capacitor Leakage Tester

> ⚠️ **Work in Progress** — this project is not yet complete.

A DIY high-sensitivity capacitor leakage current meter. Applies a selectable test
voltage to a capacitor under test and measures the resulting leakage current down
to the microamp range.

## Hardware

| Component | Part |
|-----------|------|
| MCU | RP2040-Zero |
| External ADC | ADS1115 (16-bit, I²C) |
| Display | HD44780 16×2 LCD (I²C) |
| Input | Rotary encoder |
| Test voltages | 10 V / 16 V / 25 V / 50 V |

KiCAD schematic and PCB files are in the [`kicad/`](kicad/) directory.

## Firmware

Built with [PlatformIO](https://platformio.org/) and the Arduino framework (Earle Philhower RP2040 core).

## Status

- [x] Schematic design
- [x] PCB layout
- [x] Firmware (core measurement loop)
- [ ] Pass/fail logic
- [ ] Testing & calibration
- [ ] Documentation

## License

[MIT](LICENSE)
