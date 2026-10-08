# ir20.py — Icom IC-R20 clone tool (Linux / Python)

Command-line tool to back up, restore, and edit memory on the **Icom IC-R20** wideband receiver, and to download internal recorder tracks to WAV (and optional MP3).

This is a Python port of the clone / memory / recorder side of
[DarksaCY/ic-r20-studio](https://github.com/DarksaCY/ic-r20-studio). The CLI shape follows that project’s `ir20` tool so the two can be used interchangeably for common tasks.

**Platform:** Linux (and other Unix-like systems with pyserial).  
**Cable:** OPC-1382 (FTDI USB) on the radio’s USB (clone) jack.

---

## Features

| Area | Capability |
|------|------------|
| **Memory** | Read / write the 30 528-byte settings image (`.icf`, same layout as CS-R20) |
| **Channels** | 1000 memories, banks A–Z, skip / PSkip, CTCSS / DTCS / TRAIN / VSC |
| **CSV** | CHIRP-compatible export / import (+ `Bank`, `BankSlot`, `SsbStep`) |
| **Program scan** | Export 25 × A/B scan edges |
| **Recorder** | List tracks, download to `.icw` + `.wav` (optional `.mp3` via ffmpeg) |
| **Offline** | Convert existing `.icw` files to WAV without the radio |
| **Safety** | Write path never touches the recorder track table or “has recordings” flag (same as CS-R20) |

Not included (use the Windows Studio app or a CI-V cable): live CI-V control panel, GUI, Russian localisation.

---

## Requirements

- Python 3.8+
- [pyserial](https://pypi.org/project/pyserial/)
- Optional: [ffmpeg](https://ffmpeg.org/) (only needed for `--mp3`)

```bash
# Arch / EndeavourOS
sudo pacman -S python-pyserial

# Debian / Ubuntu
sudo apt install python3-serial

# Or via pip
pip install pyserial
```

Add your user to the `uucp` or `dialout` group if the serial device is not writable:

```bash
sudo usermod -aG uucp "$USER"   # Arch
# or: sudo usermod -aG dialout "$USER"
```

Log out and back in after changing groups.

---

## Quick start

```bash
chmod +x ir20.py

# Identify the radio
./ir20.py info

# Full backup (always do this first)
./ir20.py read backup.icf

# Inspect an image
./ir20.py list backup.icf
./ir20.py csv-export backup.icf channels.csv
```

Serial port is auto-detected when an FTDI device is present. Override with:

```bash
./ir20.py --port /dev/ttyUSB0 info
```

Use `--verbose` to print raw clone frames (useful when debugging).

---

## Commands

### Radio — identification and memory

| Command | Description |
|---------|-------------|
| `info` | Model, firmware, region, recorder track end-blocks |
| `read <file.icf>` | Clone settings image from the radio |
| `write <file.icf> [--yes]` | Write an image to the radio (confirm unless `--yes`) |

After a successful **read**, the tool sends a clone-session terminator so the radio normally leaves CLONE OUT.

After a successful **write**, many firmwares **stay on CLONE** until you **power-cycle** the radio. That matches CS-R20 / ic-r20-studio behaviour.

### Image / CSV (no radio required)

| Command | Description |
|---------|-------------|
| `list <file.icf>` | Used channels, bank names, track summary |
| `csv-export <icf> <csv>` | CHIRP-style CSV (+ Bank / BankSlot / SsbStep) |
| `csv-import <icf> <csv> <out.icf> [--replace]` | Merge or replace channels from CSV |
| `scan-export <icf> <csv>` | Programmed scan edges (00A–24B) |
| `dump <icf> --start 0x… --length 0x…` | Hex dump of a region |
| `channel-dump <icf> [--start N] [--count N]` | Decoded channel table slice |

Blank channel names are stored as spaces (`0x20`), not NULs, so the radio does not fall back to the default `M:nnn` label.

### Recorder

| Command | Description |
|---------|-------------|
| `tracks` | List tracks on the radio (quality, duration, block range) |
| `download all [dir]` | Download every track → `.icw` + `.wav` |
| `download N [dir]` | Download track *N* (1-based) |
| `download … --mp3` | Also encode MP3 if `ffmpeg` is on `PATH` |
| `icw <file.icw> [--mp3]` | Offline ICW → WAV (+ optional MP3) |

Recorder flash is read with the clone protocol (address bit 28). Audio is OKI ADPCM (2-bit LONG @ 8 kHz, 4-bit NORMAL @ 8 kHz, 4-bit FINE @ 16 kHz), matching the reference decode used by ic-r20-studio.

---

## Typical workflows

### Edit channels in a spreadsheet / CHIRP

```bash
./ir20.py read backup.icf
./ir20.py csv-export backup.icf channels.csv
# edit channels.csv …
./ir20.py csv-import backup.icf channels.csv edited.icf
# or wipe all channels first:
./ir20.py csv-import backup.icf channels.csv edited.icf --replace
./ir20.py write edited.icf --yes
# power-cycle the radio
```

### Save recordings

```bash
./ir20.py tracks
./ir20.py download all ~/r20-tracks
# or one track as WAV+MP3:
./ir20.py download 1 ~/r20-tracks --mp3
```

### Restore a known-good backup

```bash
./ir20.py write known-good.icf --yes
# power-cycle
```

---

## Safety notes

1. **Always read a verified backup before the first write.**
2. Prefer writing images that originated from an IC-R20 (or a careful CSV edit of one). Random or truncated files can leave the radio with empty or invalid memory.
3. The write path **skips** the recorder track table (`0x729F–0x7300`) and the “has recordings” byte (`0x7667`), so cloning memory should not erase audio tracks.
4. After **write**, power-cycle if the display remains on **CLONE** / **CLONE OUT**.
5. Do not unplug the cable mid-transfer.

---

## Protocol summary

Clone frames on the USB port: `FE FE <to> <from> <cmd> <payload> FD`  
Radio address `0xEE`, PC `0xEF`. Baud rate is ignored by the radio for clone.

| Cmd | Role |
|-----|------|
| E0 / E1 | Identify |
| E2 | Read range (or `FFFFFFFFFFFFFFFF` to end session) |
| E3 | Start write |
| E4 | Data (ASCII-hex body + checksum) |
| E5 | End-of-transfer summary |
| E6 | Write result (`00` = accepted) |

Settings image size: **0x7740** (30 528) bytes.  
Recorder blocks: 512 bytes each at `(block << 9) \| 0x10000000`.

Full reverse-engineering notes live in the upstream project:

- [docs/PROTOCOL.md](https://github.com/DarksaCY/ic-r20-studio/blob/main/docs/PROTOCOL.md)

---

## Project layout

```text
ir20.py      # single-file CLI (this tool)
README.md    # this file
```

No installer and no extra packages beyond pyserial (and optional ffmpeg for MP3).

---

## Acknowledgements

- Protocol, memory map, write framing, and OKI ADPCM behaviour derived from
  [DarksaCY/ic-r20-studio](https://github.com/DarksaCY/ic-r20-studio) and earlier CS-R20 / CHIRP Icom clone work.
- Original commercial cloning software: Icom **CS-R20**.

---

## Disclaimer

This is unofficial software. Use at your own risk. The authors are not affiliated with Icom. Incorrect use of clone write can leave the radio with unexpected memory contents; keep backups and power-cycle after writes when needed.

---

## Licence

Choose a licence that fits your distribution (for example MIT or GPL-2.0) and add a `LICENSE` file before publishing. Upstream ic-r20-studio should be credited when redistributing protocol-derived code.
