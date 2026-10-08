#!/usr/bin/env python3
"""
ir20.py — Linux Python CLI for the Icom IC-R20 (based on ic-r20-studio).

A Python port of the clone / memory side of:
    https://github.com/DarksaCY/ic-r20-studio

Requires:
    python-pyserial

Examples (similar to the official ir20.exe):
    ./ir20.py info
    ./ir20.py read backup.icf
    ./ir20.py write backup.icf --yes
    ./ir20.py list backup.icf
    ./ir20.py csv-export backup.icf channels.csv
    ./ir20.py csv-import backup.icf channels.csv out.icf
    ./ir20.py csv-import backup.icf channels.csv out.icf --replace
    ./ir20.py scan-export backup.icf scans.csv
    ./ir20.py tracks
    ./ir20.py download all out
    ./ir20.py download 1 out --mp3
    ./ir20.py icw track01.icw --mp3
    ./ir20.py dump backup.icf --start 0x6716 --length 0x200
    ./ir20.py channel-dump backup.icf --start 0 --count 20

Global options:
    --port /dev/ttyUSB0
    --verbose

CSV is CHIRP-compatible (plus Bank / BankSlot / SsbStep columns), matching
the C# tool.  Recorder tracks: list, download to ICW/WAV (optional MP3 via
ffmpeg), and offline ICW conversion.

Before using "write", make a verified backup of the radio.
"""

import argparse
import csv
import sys
import time

import serial
import serial.tools.list_ports


# =============================================================================
# Protocol constants
# =============================================================================

ADDR_PC = 0xEF
ADDR_RADIO = 0xEE

PREAMBLE = b"\xFE\xFE"
END = 0xFD

CMD_E0 = 0xE0
CMD_E1 = 0xE1
CMD_E2 = 0xE2
CMD_E3 = 0xE3
CMD_E4 = 0xE4
CMD_E5 = 0xE5
CMD_E6 = 0xE6

MODEL = bytes([0x26, 0x99, 0x00, 0x00])

SETTINGS_START = 0x00000000
SETTINGS_END = 0x0000773F
SETTINGS_SIZE = 0x7740

FRAME_TIMEOUT = 10.0
INTER_FRAME_DELAY = 0.010


# =============================================================================
# Memory map
# =============================================================================

CHANNEL_SIZE = 22
NUM_CHANNELS = 1000
CHANNEL_START = 0x0000

PROGRAM_SCAN_START = 0x55F0
PROGRAM_SCAN_COUNT = 25
PROGRAM_SCAN_RECORD_SIZE = 44

EMPTY_BITMAP = 0x6716
SKIP_BITMAP = 0x679A
PSKIP_BITMAP = 0x681E

BANK_TABLE = 0x6997

RECORDER_NEXT_BLOCK = 0x729F
RECORDER_TRACK_TABLE = 0x72A1
RECORDER_TRACK_MAX = 32

# Bit 28 selects recorder flash instead of settings EEPROM.
# Address of block N = (N << 9) | SOUND_ADDRESS_FLAG
SOUND_ADDRESS_FLAG = 0x10000000
SOUND_BLOCK_SIZE = 512
SOUND_BLOCK_HEADER = 2
SOUND_BLOCK_PAYLOAD = 510

BANK_NAMES = 0x7301

CIV_ADDRESS = 0x7648
CIV_BAUD = 0x7649
CIV_TRANSCEIVE = 0x764A

FORMAT_SIGNATURE = 0x7730

WRITE_BORDERS = [
    0x6716,
    0x6997,
    0x71CB,
    0x7730,
]

NEVER_WRITE = [
    (0x729F, 0x7300),
    (0x7667, 0x7667),
]


# =============================================================================
# Channel decoding
# =============================================================================

# IC-R20 protocol mode values.
MODE_NAMES = {
    0x00: "FM",
    0x01: "WFM",
    0x02: "AM",
    0x03: "LSB",
    0x04: "USB",
    0x05: "CW",
}

MODE_VALUES = {
    "FM": 0x00,
    "WFM": 0x01,
    "AM": 0x02,
    "LSB": 0x03,
    "USB": 0x04,
    "CW": 0x05,
}

TONE_MODE_NAMES = {
    0: "OFF",
    2: "TSQL",
    4: "DTCS",
    5: "TRAIN",
    6: "MSK",
    7: "VSC",
}

STEP_NAMES = [
    "0.01k",
    "0.1k",
    "1k",
    "5k",
    "6.25k",
    "8.33k",
    "9k",
    "10k",
    "12.5k",
    "15k",
    "20k",
    "25k",
    "30k",
    "50k",
    "100k",
]

DTCS_CODES = [
    23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72, 73,
    74, 75, 76, 77, 78, 79, 80, 88, 95, 102, 107, 110, 113, 114,
    115, 116, 117, 118, 121, 122, 123, 124, 125, 126, 131, 132,
    133, 134, 142, 146, 151, 154, 156, 160, 162, 165, 172, 177,
    181, 187, 188, 189, 194, 196, 197, 198, 199, 201, 209, 210,
    211, 212, 215, 216, 217, 218, 219, 223, 227, 233, 234, 235,
    241, 244, 245, 246, 251, 254, 255, 261, 263, 265, 266, 268,
    271, 274, 302, 306, 311, 315, 320, 337, 346, 351, 357, 361,
    362, 365, 371,
]

CTCSS_TONES = [
    67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.1, 87.4, 89.9, 91.5,
    94.1, 97.3, 100.0, 103.5, 105.1, 107.2, 109.4, 110.9, 113.1,
    115.0, 116.9, 118.8, 120.3, 121.9, 123.8, 125.5, 127.3, 129.0,
    131.8, 134.2, 136.5, 138.9, 141.3, 143.5, 145.5, 147.4, 150.8,
    153.5, 156.7, 159.8, 162.2, 165.5, 167.9, 171.3, 173.8, 177.3,
    179.9, 183.5, 186.2, 189.9,
]


# =============================================================================
# Frame handling
# =============================================================================

def build_frame(
    to: int,
    frm: int,
    cmd: int,
    payload: bytes = b"",
) -> bytes:
    return (
        PREAMBLE
        + bytes([to, frm, cmd])
        + payload
        + bytes([END])
    )


def find_complete_frame(buf: bytearray):
    """
    Extract exactly one complete frame.

    Any subsequent frames remain in buf.
    """

    while True:

        start = buf.find(PREAMBLE)

        if start < 0:

            if buf and buf[-1] == PREAMBLE[0]:
                del buf[:-1]
            else:
                buf.clear()

            return None

        if start > 0:
            del buf[:start]

        if len(buf) < 6:
            return None

        end = buf.find(
            bytes([END]),
            5,
        )

        if end < 0:
            return None

        frame = bytes(
            buf[:end + 1]
        )

        del buf[:end + 1]

        if len(frame) < 6:
            continue

        to = frame[2]
        frm = frame[3]
        cmd = frame[4]
        payload = frame[5:-1]

        return (
            to,
            frm,
            cmd,
            payload,
        )


# =============================================================================
# Serial / radio
# =============================================================================

class ICR20:

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        verbose: bool = False,
    ):

        self.port = port
        self.verbose = verbose

        print(
            f"Opening {port}..."
        )

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.10,
            write_timeout=5.0,
        )

        self.rx_buffer = bytearray()

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    # -------------------------------------------------------------------------
    # TX
    # -------------------------------------------------------------------------

    def _send(
        self,
        frame: bytes,
    ):

        if self.verbose:
            print(
                f"TX: {frame.hex(' ')}"
            )

        self.ser.write(frame)
        self.ser.flush()

        time.sleep(
            INTER_FRAME_DELAY
        )

    # -------------------------------------------------------------------------
    # RX
    # -------------------------------------------------------------------------

    def _read_into_buffer(self):

        waiting = self.ser.in_waiting

        if waiting:

            data = self.ser.read(
                waiting
            )

            if data:
                self.rx_buffer.extend(
                    data
                )

    def _recv_frame(
        self,
        timeout: float = FRAME_TIMEOUT,
    ):

        deadline = (
            time.monotonic()
            + timeout
        )

        while time.monotonic() < deadline:

            frame = find_complete_frame(
                self.rx_buffer
            )

            if frame is not None:

                self._debug_rx(frame)

                return frame

            self._read_into_buffer()

            frame = find_complete_frame(
                self.rx_buffer
            )

            if frame is not None:

                self._debug_rx(frame)

                return frame

            time.sleep(
                0.002
            )

        raise TimeoutError(
            "No complete IC-R20 frame received "
            f"within {timeout:.1f} seconds"
        )

    def _debug_rx(
        self,
        frame,
    ):

        if not self.verbose:
            return

        to, frm, cmd, payload = frame

        print(
            "RX: FE FE "
            f"{to:02X} "
            f"{frm:02X} "
            f"{cmd:02X} "
            f"{payload.hex(' ')} FD"
        )

    def _recv_until_cmd(
        self,
        cmd: int,
        timeout: float = FRAME_TIMEOUT,
    ):

        deadline = (
            time.monotonic()
            + timeout
        )

        while True:

            remaining = (
                deadline
                - time.monotonic()
            )

            if remaining <= 0:

                raise TimeoutError(
                    f"Command 0x{cmd:02X} "
                    f"not received within "
                    f"{timeout:.1f} seconds"
                )

            (
                to,
                frm,
                received_cmd,
                payload,
            ) = self._recv_frame(
                timeout=remaining
            )

            if to != ADDR_PC:

                if self.verbose:
                    print(
                        f"Ignoring frame addressed "
                        f"to {to:02X}"
                    )

                continue

            if received_cmd == cmd:
                return payload

            if self.verbose:
                print(
                    f"Ignoring command "
                    f"0x{received_cmd:02X}; "
                    f"waiting for "
                    f"0x{cmd:02X}"
                )

    # -------------------------------------------------------------------------
    # Identification
    # -------------------------------------------------------------------------

    def query_id(self):

        self.rx_buffer.clear()
        self.ser.reset_input_buffer()

        self._send(
            build_frame(
                ADDR_RADIO,
                ADDR_PC,
                CMD_E0,
                MODEL,
            )
        )

        return self._recv_until_cmd(
            CMD_E1,
            timeout=5.0,
        )

    # -------------------------------------------------------------------------
    # Read memory
    # -------------------------------------------------------------------------

    def read_settings(self):

        request = (
            MODEL
            + (
                b"%08X%08X"
                % (
                    SETTINGS_START,
                    SETTINGS_END,
                )
            )
        )

        self.rx_buffer.clear()
        self.ser.reset_input_buffer()

        self._send(
            build_frame(
                ADDR_RADIO,
                ADDR_PC,
                CMD_E2,
                request,
            )
        )

        memory = bytearray(
            SETTINGS_SIZE
        )

        received_ranges = []

        expected_frames = None

        print(
            "  Reading frames..."
        )

        while True:

            (
                to,
                frm,
                cmd,
                payload,
            ) = self._recv_frame(
                timeout=FRAME_TIMEOUT
            )

            if to != ADDR_PC:
                continue

            # -------------------------------------------------------------
            # E4
            # -------------------------------------------------------------

            if cmd == CMD_E4:

                try:

                    ascii_payload = (
                        payload.decode(
                            "ascii"
                        )
                    )

                    body = bytes.fromhex(
                        ascii_payload
                    )

                except (
                    UnicodeDecodeError,
                    ValueError,
                ) as exc:

                    raise RuntimeError(
                        "Invalid E4 ASCII-hex payload: "
                        f"{payload!r}"
                    ) from exc

                if len(body) < 6:

                    raise RuntimeError(
                        f"E4 payload too short: "
                        f"{len(body)} bytes"
                    )

                addr = int.from_bytes(
                    body[0:4],
                    "big",
                )

                length = body[4]

                expected_length = (
                    4
                    + 1
                    + length
                    + 1
                )

                if len(body) != expected_length:

                    raise RuntimeError(
                        "Invalid E4 frame: "
                        f"address=0x{addr:08X}, "
                        f"length={length}, "
                        f"body={len(body)}"
                    )

                data = body[
                    5:5 + length
                ]

                if (
                    sum(body) & 0xFF
                ) != 0:

                    raise RuntimeError(
                        "E4 checksum error at "
                        f"0x{addr:08X}"
                    )

                if addr > SETTINGS_END:

                    raise RuntimeError(
                        "E4 address outside settings "
                        f"image: 0x{addr:08X}"
                    )

                end = addr + length

                if end > SETTINGS_SIZE:

                    raise RuntimeError(
                        "E4 frame overruns settings "
                        f"image: "
                        f"0x{addr:08X}+{length}"
                    )

                # Check overlapping data.
                for i, value in enumerate(data):

                    position = addr + i

                    for (
                        start,
                        stop,
                    ) in received_ranges:

                        if (
                            start
                            <= position
                            < stop
                        ):

                            if memory[position] != value:

                                raise RuntimeError(
                                    "Conflicting duplicate "
                                    f"E4 data at "
                                    f"0x{position:04X}"
                                )

                            break

                    else:
                        memory[position] = value

                received_ranges.append(
                    (addr, end)
                )

                received_ranges = (
                    merge_ranges(
                        received_ranges
                    )
                )

                received = sum(
                    stop - start
                    for start, stop
                    in received_ranges
                )

                percent = (
                    received
                    * 100.0
                    / SETTINGS_SIZE
                )

                print_progress(
                    percent,
                    received,
                    SETTINGS_SIZE,
                )

            # -------------------------------------------------------------
            # E5
            # -------------------------------------------------------------

            elif cmd == CMD_E5:

                print()

                if payload.startswith(
                    b"Icom Inc."
                ):

                    tail = payload[
                        len(b"Icom Inc."):]

                    if len(tail) >= 2:

                        try:

                            expected_frames = int(
                                tail[:2].decode(
                                    "ascii"
                                ),
                                16,
                            )

                        except ValueError:
                            expected_frames = None

                break

            elif cmd == CMD_E2:
                break

            else:

                if self.verbose:

                    print(
                        f"Ignoring command "
                        f"0x{cmd:02X} during read"
                    )

        received = sum(
            stop - start
            for start, stop
            in received_ranges
        )

        if received != SETTINGS_SIZE:

            raise RuntimeError(
                "Incomplete memory read: "
                f"received {received}/"
                f"{SETTINGS_SIZE} bytes"
            )

        if expected_frames is not None:

            print(
                f"  E5 reports "
                f"{expected_frames} data frames."
            )

        return bytes(memory)

    # -------------------------------------------------------------------------
    # General range read (settings or recorder flash)
    # -------------------------------------------------------------------------

    def read_range(
        self,
        start: int,
        end: int,
        label: str = "data",
    ) -> bytes:
        """
        Read inclusive address range [start, end] via E2/E4/E5.

        For recorder flash set bit 28 (SOUND_ADDRESS_FLAG) in both
        addresses.  Large transfers are split into 8 KiB chunks so a
        cancelled transfer leaves at most one chunk of tail traffic.
        """

        if end < start:
            return b""

        total = end - start + 1
        buf = bytearray(total)
        chunk = 8192
        done = 0

        print(f"  Reading {label} ({total:,} bytes)...")

        for off in range(0, total, chunk):
            length = min(chunk, total - off)
            chunk_start = start + off
            chunk_end = chunk_start + length - 1

            self.rx_buffer.clear()
            self.ser.reset_input_buffer()

            request = (
                MODEL
                + (
                    b"%08X%08X"
                    % (chunk_start, chunk_end)
                )
            )

            self._send(
                build_frame(
                    ADDR_RADIO,
                    ADDR_PC,
                    CMD_E2,
                    request,
                )
            )

            got = 0

            while True:
                (
                    to,
                    frm,
                    cmd,
                    payload,
                ) = self._recv_frame(
                    timeout=FRAME_TIMEOUT
                )

                if to != ADDR_PC:
                    continue

                if cmd == CMD_E4:
                    try:
                        body = bytes.fromhex(
                            payload.decode("ascii")
                        )
                    except (UnicodeDecodeError, ValueError) as exc:
                        raise RuntimeError(
                            f"Invalid E4 payload: {payload!r}"
                        ) from exc

                    if len(body) < 6:
                        raise RuntimeError(
                            f"E4 payload too short: {len(body)}"
                        )

                    addr = int.from_bytes(body[0:4], "big")
                    length_b = body[4]
                    expected = 4 + 1 + length_b + 1

                    if len(body) != expected:
                        raise RuntimeError(
                            f"Invalid E4 frame at 0x{addr:08X}"
                        )

                    data = body[5:5 + length_b]

                    if (sum(body) & 0xFF) != 0:
                        raise RuntimeError(
                            f"E4 checksum error at 0x{addr:08X}"
                        )

                    rel = addr - chunk_start
                    if rel < 0 or rel + len(data) > length:
                        raise RuntimeError(
                            f"E4 address out of chunk: "
                            f"0x{addr:08X}"
                        )

                    buf[off + rel:off + rel + len(data)] = data
                    got += len(data)
                    done += len(data)

                    percent = done * 100.0 / total
                    print_progress(percent, done, total)

                elif cmd == CMD_E5:
                    break

                elif cmd == CMD_E2:
                    break

            if got < length:
                raise RuntimeError(
                    f"Incomplete chunk read: "
                    f"got {got}/{length} at 0x{chunk_start:08X}"
                )

        print()
        return bytes(buf)

    def read_sound_blocks(
        self,
        start_block: int,
        end_block: int,
    ) -> bytes:
        """
        Read recorder blocks [start_block, end_block) (exclusive end).
        Each block is 512 bytes (2-byte header + 510 ADPCM).
        """

        if end_block <= start_block:
            return b""

        a = (start_block << 9) | SOUND_ADDRESS_FLAG
        b = ((end_block << 9) - 1) | SOUND_ADDRESS_FLAG
        n_blocks = end_block - start_block

        return self.read_range(
            a,
            b,
            label=f"blocks {start_block}-{end_block - 1} "
                  f"({n_blocks})",
        )

    # -------------------------------------------------------------------------
    # Write memory
    # -------------------------------------------------------------------------

    def write_settings(
        self,
        memory: bytes,
    ):
        """
        Write the settings image like CS-R20 / ic-r20-studio:

          E3 (start) → fire all E4 frames with no per-frame ACK →
          E5 (Icom Inc. + frame_count&0xFF) → wait for E6 00.

        The radio is silent during the E4 stream; only E6 acknowledges
        the whole transfer.  Waiting for E4/E5 replies is what caused
        the previous 5 s timeout (and left the radio stuck in CLONE).
        """

        if len(memory) != SETTINGS_SIZE:

            raise ValueError(
                f"Expected {SETTINGS_SIZE} bytes, "
                f"got {len(memory)}"
            )

        frames = self._build_write_frames(
            memory
        )

        if len(frames) != 956:

            raise RuntimeError(
                "Protocol sanity check failed: "
                f"generated {len(frames)} "
                "E4 frames; expected 956"
            )

        # Discard any leftover traffic from a previous session.
        self.rx_buffer.clear()
        self.ser.reset_input_buffer()
        time.sleep(0.05)
        self.ser.reset_input_buffer()

        print(
            "  Starting clone write..."
        )

        # E3 — enter write mode.  Do NOT wait for a reply; CS-R20
        # only pauses briefly, then streams E4.
        self._send(
            build_frame(
                ADDR_RADIO,
                ADDR_PC,
                CMD_E3,
                MODEL,
            )
        )

        time.sleep(0.050)

        print(
            f"  Writing {len(frames)} "
            f"E4 data frames..."
        )

        for index, frame_body in enumerate(
            frames,
            1,
        ):

            payload = (
                frame_body
                .hex()
                .upper()
                .encode("ascii")
            )

            self._send(
                build_frame(
                    ADDR_RADIO,
                    ADDR_PC,
                    CMD_E4,
                    payload,
                )
            )

            # Radio is normally silent until E6.  Drain anything that
            # arrives so a premature E6 (write rejected mid-stream)
            # is not missed, but never *wait* for an ACK.
            self._read_into_buffer()
            while True:
                frame = find_complete_frame(
                    self.rx_buffer
                )
                if frame is None:
                    break
                to, frm, cmd, payload_rx = frame
                if (
                    to == ADDR_PC
                    and cmd == CMD_E6
                ):
                    raise RuntimeError(
                        "Radio aborted write mid-stream: "
                        f"{payload_rx.hex(' ')}"
                    )

            percent = (
                index
                * 100.0
                / len(frames)
            )

            print_progress(
                percent,
                index,
                len(frames),
                unit="frames",
            )

        print()

        # -------------------------------------------------------------
        # E5 summary: "Icom Inc." + two hex digits (frame_count & 0xFF)
        # e.g. 956 frames → "Icom Inc.BC".  No trailing 0xFF.
        # -------------------------------------------------------------

        frame_count = (
            len(frames) & 0xFF
        )

        e5_payload = (
            b"Icom Inc."
            + f"{frame_count:02X}".encode(
                "ascii"
            )
        )

        self._send(
            build_frame(
                ADDR_RADIO,
                ADDR_PC,
                CMD_E5,
                e5_payload,
            )
        )

        # -------------------------------------------------------------
        # E6 result — the only write acknowledgement
        # -------------------------------------------------------------

        result = self._recv_until_cmd(
            CMD_E6,
            timeout=15.0,
        )

        if result != b"\x00":

            raise RuntimeError(
                "Radio rejected write: "
                f"{result.hex(' ')}"
            )

        print(
            "  Radio accepted memory write."
        )

        # After a successful write the display stays on CLONE until
        # the radio is power-cycled (same as CS-R20).  Still send the
        # terminator for protocol cleanliness.
        self.exit_clone_mode()

    def exit_clone_mode(self):

        """
        Send the clone-session close request.

        Some IC-R20 firmware versions may still display CLONE OUT
        until the radio is power-cycled. We nevertheless wait for
        any final protocol response before closing the serial port.
        """

        print(
            "  Closing clone session..."
        )

        self.rx_buffer.clear()

        self._send(
            build_frame(
                ADDR_RADIO,
                ADDR_PC,
                CMD_E2,
                MODEL
                + b"FFFFFFFFFFFFFFFF",
            )
        )

        try:

            response = self._recv_frame(
                timeout=5.0
            )

            if self.verbose:

                (
                    to,
                    frm,
                    cmd,
                    payload,
                ) = response

                print(
                    "  Clone termination response: "
                    f"TO={to:02X} "
                    f"FROM={frm:02X} "
                    f"CMD={cmd:02X} "
                    f"DATA={payload.hex(' ')}"
                )

        except TimeoutError:

            if self.verbose:

                print(
                    "  No final clone termination "
                    "frame received."
                )

    def _build_write_frames(
        self,
        memory: bytes,
    ):

        frames = []

        offset = 0

        while offset < SETTINGS_SIZE:

            # ---------------------------------------------------------
            # Protected areas
            # ---------------------------------------------------------

            skipped = False

            for start, end in NEVER_WRITE:

                if (
                    start
                    <= offset
                    <= end
                ):

                    offset = end + 1
                    skipped = True
                    break

            if skipped:
                continue

            # ---------------------------------------------------------
            # Maximum 32 bytes, aligned to 0x20.
            # ---------------------------------------------------------

            max_data = min(
                32,
                SETTINGS_SIZE - offset,
                0x20 - (
                    offset & 0x1F
                ),
            )

            # ---------------------------------------------------------
            # Region borders
            # ---------------------------------------------------------

            for border in WRITE_BORDERS:

                if (
                    offset
                    < border
                    < offset + max_data
                ):

                    max_data = (
                        border - offset
                    )

            # ---------------------------------------------------------
            # Protected regions
            # ---------------------------------------------------------

            for start, end in NEVER_WRITE:

                if (
                    offset < start
                    and
                    offset + max_data > start
                ):

                    max_data = (
                        start - offset
                    )

            if max_data <= 0:

                raise RuntimeError(
                    "Unable to generate E4 "
                    f"frame at 0x{offset:04X}"
                )

            data = memory[
                offset:
                offset + max_data
            ]

            body_without_checksum = (
                offset.to_bytes(
                    4,
                    "big",
                )
                + bytes([
                    len(data)
                ])
                + data
            )

            checksum = (
                -sum(
                    body_without_checksum
                )
            ) & 0xFF

            body = (
                body_without_checksum
                + bytes([
                    checksum
                ])
            )

            if (
                sum(body) & 0xFF
            ):

                raise RuntimeError(
                    "Internal checksum error "
                    f"at 0x{offset:04X}"
                )

            frames.append(body)

            offset += len(data)

        return frames

    def close(self):

        try:
            self.ser.close()
        except Exception:
            pass


# =============================================================================
# General helpers
# =============================================================================

def merge_ranges(ranges):

    if not ranges:
        return []

    ranges = sorted(ranges)

    merged = [
        ranges[0]
    ]

    for start, end in ranges[1:]:

        old_start, old_end = merged[-1]

        if start <= old_end:

            merged[-1] = (
                old_start,
                max(
                    old_end,
                    end,
                ),
            )

        else:

            merged.append(
                (start, end)
            )

    return merged


def print_progress(
    percent,
    current,
    total,
    unit="bytes",
):

    bar_width = 40

    filled = int(
        bar_width
        * percent
        / 100.0
    )

    bar = (
        "#"
        * filled
        + "-"
        * (
            bar_width
            - filled
        )
    )

    print(
        f"\r  [{bar}] "
        f"{percent:6.2f}% "
        f"{current}/{total} {unit}",
        end="",
        flush=True,
    )


# =============================================================================
# Bitmap helpers
# =============================================================================

def bitmap_bit(
    memory: bytes,
    base: int,
    channel: int,
) -> int:

    byte_offset = (
        base
        + channel // 8
    )

    bit = channel % 8

    if byte_offset >= len(memory):

        raise ValueError(
            f"Bitmap address outside image: "
            f"0x{byte_offset:04X}"
        )

    return (
        memory[byte_offset]
        >> bit
    ) & 1


def channel_is_empty(
    memory: bytes,
    channel: int,
) -> bool:

    return bool(
        bitmap_bit(
            memory,
            EMPTY_BITMAP,
            channel,
        )
    )


def channel_skip(
    memory: bytes,
    channel: int,
) -> str:

    if bitmap_bit(
        memory,
        PSKIP_BITMAP,
        channel,
    ):

        return "PSKIP"

    if bitmap_bit(
        memory,
        SKIP_BITMAP,
        channel,
    ):

        return "SKIP"

    return "OFF"


def channel_bank(
    memory: bytes,
    channel: int,
):
    """
    Return (bank_index, bank_slot) or (None, None).

    Bank table at 0x6997: 1000 × 2 bytes.
    byte0 = bank 0–25 (A–Z), 0xFF = none
    byte1 = slot 0–99
    """

    offset = BANK_TABLE + channel * 2

    if offset + 1 >= len(memory):
        return None, None

    bank = memory[offset]
    slot = memory[offset + 1]

    if bank > 25:
        return None, None

    return bank, slot


def bank_letter(bank: int) -> str:
    if bank is None or bank < 0 or bank > 25:
        return ""
    return chr(ord("A") + bank)


def get_bank_name(memory: bytes, bank: int) -> str:
    if bank is None or bank < 0 or bank > 25:
        return ""
    offset = BANK_NAMES + bank * 8
    return decode_radio_text(memory[offset:offset + 8])


def set_channel_empty(memory: bytearray, channel: int, empty: bool):
    byte_offset = EMPTY_BITMAP + channel // 8
    bit = channel % 8
    if empty:
        memory[byte_offset] |= (1 << bit)
    else:
        memory[byte_offset] &= ~(1 << bit)


def set_channel_skip(memory: bytearray, channel: int, skip: str):
    """skip: OFF / SKIP / PSKIP"""
    def set_bit(base, value):
        byte_offset = base + channel // 8
        bit = channel % 8
        if value:
            memory[byte_offset] |= (1 << bit)
        else:
            memory[byte_offset] &= ~(1 << bit)

    set_bit(SKIP_BITMAP, skip in ("SKIP", "PSKIP"))
    set_bit(PSKIP_BITMAP, skip == "PSKIP")


def set_channel_bank(memory: bytearray, channel: int, bank, slot):
    offset = BANK_TABLE + channel * 2
    if bank is None or bank < 0:
        memory[offset] = 0xFF
        memory[offset + 1] = 0xFF
    else:
        memory[offset] = bank & 0xFF
        memory[offset + 1] = (slot if slot is not None else 0) & 0xFF


# =============================================================================
# Radio text
# =============================================================================

def decode_radio_text(
    data: bytes,
) -> str:
    """
    Decode an IC-R20 text field (channel/bank name).

    Printable ASCII is kept; other bytes become {XX} tokens so icons
    survive a CSV round-trip.  Trailing spaces are stripped — blank
    names on the radio are stored as 0x20, not 0x00.
    """

    result = []

    for byte in data:

        if 0x20 <= byte <= 0x7E and byte not in (0x7B, 0x7D):
            # '{' / '}' reserved for {XX} tokens
            result.append(chr(byte))
        elif byte == 0x00:
            # Treat NULs like spaces so a zero-padded field still
            # decodes to an empty name after rstrip.
            result.append(" ")
        else:
            result.append(f"{{{byte:02X}}}")

    return "".join(result).rstrip(" ")


def encode_radio_text(text: str, length: int = 8) -> bytes:
    """
    Encode a name into a fixed-width radio text field.

    Blank/empty names become all spaces (0x20), matching CS-R20 and
    the radio UI.  Zero-filled names make the IC-R20 show the default
    "M:nnn" label instead of a blank name.
    """

    result = bytearray(b" " * length)
    if not text:
        return bytes(result)

    n = 0
    i = 0
    while i < len(text) and n < length:
        # {XX} token for non-ASCII / icon glyphs
        if (
            text[i] == "{"
            and i + 3 < len(text)
            and text[i + 3] == "}"
        ):
            try:
                result[n] = int(text[i + 1:i + 3], 16)
                n += 1
                i += 4
                continue
            except ValueError:
                pass

        c = text[i]
        if " " <= c < "\x7f":
            result[n] = ord(c)
        else:
            result[n] = ord("?")
        n += 1
        i += 1

    return bytes(result)


# =============================================================================
# Channel decoding
# =============================================================================

def decode_channels(
    memory: bytes,
):

    if len(memory) != SETTINGS_SIZE:

        raise ValueError(
            f"Expected {SETTINGS_SIZE} bytes, "
            f"got {len(memory)}"
        )

    channels = []

    for channel_number in range(
        NUM_CHANNELS
    ):

        offset = (
            CHANNEL_START
            + channel_number
            * CHANNEL_SIZE
        )

        rec = memory[
            offset:
            offset + CHANNEL_SIZE
        ]

        if len(rec) != CHANNEL_SIZE:
            break

        freq_hz = int.from_bytes(
            rec[0:4],
            "little",
        )

        offset_hz = int.from_bytes(
            rec[4:8],
            "little",
        )

        step_index = (
            rec[8] & 0x0F
        )

        ssb_step_index = (
            (rec[8] >> 4)
            & 0x0F
        )

        tone_mode = (
            (rec[11] >> 1)
            & 0x07
        )

        duplex_value = (
            (rec[11] >> 4)
            & 0x03
        )

        dtcs_reverse = bool(
            rec[11] & 0x40
        )

        dtcs_index = (
            rec[12] & 0x7F
        )

        ctcss_index = (
            ((rec[13] & 0x1F) << 1)
            | ((rec[12] >> 7) & 1)
        )

        # -------------------------------------------------------------
        # Mode
        # -------------------------------------------------------------

        raw_mode_byte = rec[13]

        raw_mode = (
            raw_mode_byte >> 5
        ) & 0x07

        mode = MODE_NAMES.get(
            raw_mode,
            f"?{raw_mode}",
        )

        # -------------------------------------------------------------
        # Name
        # -------------------------------------------------------------

        name = decode_radio_text(
            rec[14:22]
        )

        # -------------------------------------------------------------
        # Duplex
        # -------------------------------------------------------------

        if duplex_value == 1:
            duplex = "-"

        elif duplex_value == 2:
            duplex = "+"

        else:
            duplex = ""

        # -------------------------------------------------------------
        # Tone
        # -------------------------------------------------------------

        tone = ""

        if tone_mode == 4:

            if dtcs_index < len(
                DTCS_CODES
            ):

                tone = (
                    f"{DTCS_CODES[dtcs_index]:03d}"
                )

            else:

                tone = (
                    f"?{dtcs_index}"
                )

        elif tone_mode == 5:

            train_value = (
                int.from_bytes(
                    rec[10:12],
                    "little",
                )
                & 0x01FF
            )

            tone = (
                f"{train_value * 10:d}"
            )

        elif tone_mode == 2:

            if ctcss_index < len(
                CTCSS_TONES
            ):

                tone = (
                    f"{CTCSS_TONES[ctcss_index]:.1f}"
                )

            else:

                tone = (
                    f"?{ctcss_index}"
                )

        elif tone_mode == 0:

            tone = ""

        else:

            tone = TONE_MODE_NAMES.get(
                tone_mode,
                f"?{tone_mode}",
            )

        # -------------------------------------------------------------
        # Steps
        # -------------------------------------------------------------

        step = (
            STEP_NAMES[step_index]
            if step_index < len(STEP_NAMES)
            else f"?{step_index}"
        )

        ssb_step = (
            STEP_NAMES[ssb_step_index]
            if ssb_step_index < len(STEP_NAMES)
            else f"?{ssb_step_index}"
        )

        # -------------------------------------------------------------
        # Skip
        # -------------------------------------------------------------

        skip = channel_skip(
            memory,
            channel_number,
        )

        # -------------------------------------------------------------
        # Empty
        # -------------------------------------------------------------

        empty = channel_is_empty(
            memory,
            channel_number,
        )

        bank, bank_slot = channel_bank(
            memory,
            channel_number,
        )

        channels.append(
            {
                "ch": channel_number,
                "name": name,

                "freq_mhz": (
                    freq_hz / 1_000_000.0
                    if freq_hz
                    else 0.0
                ),

                "mode": mode,
                "raw_mode": raw_mode,
                "raw_mode_byte": (
                    f"{raw_mode_byte:02X}"
                ),

                "duplex": duplex,

                "offset_mhz": (
                    offset_hz / 1_000_000.0
                    if offset_hz
                    else 0.0
                ),

                "offset_khz": (
                    offset_hz / 1_000.0
                    if offset_hz
                    else 0.0
                ),

                "tone": tone,

                "tone_mode": (
                    TONE_MODE_NAMES.get(
                        tone_mode,
                        f"?{tone_mode}",
                    )
                ),

                "ctcss_index": ctcss_index,
                "dtcs_index": dtcs_index,

                "step": step,
                "ssb_step": ssb_step,
                "step_index": step_index,
                "ssb_step_index": ssb_step_index,

                "dtcs_reverse": (
                    dtcs_reverse
                ),

                "skip": skip,
                "empty": empty,

                "bank": bank,
                "bank_slot": bank_slot,
                "bank_letter": bank_letter(bank),

                "raw": rec.hex().upper(),
            }
        )

    return channels


# =============================================================================
# Programmed scan decoding
# =============================================================================

def decode_program_scans(
    memory: bytes,
):

    programs = []

    for program in range(
        PROGRAM_SCAN_COUNT
    ):

        base = (
            PROGRAM_SCAN_START
            + program
            * PROGRAM_SCAN_RECORD_SIZE
        )

        for edge_name in (
            "A",
            "B",
        ):

            if edge_name == "A":
                offset = base
            else:
                offset = (
                    base
                    + CHANNEL_SIZE
                )

            rec = memory[
                offset:
                offset + CHANNEL_SIZE
            ]

            if len(rec) != CHANNEL_SIZE:

                raise ValueError(
                    f"Program scan "
                    f"{program:02d}{edge_name} "
                    "extends beyond image"
                )

            freq_hz = int.from_bytes(
                rec[0:4],
                "little",
            )

            raw_mode_byte = rec[13]

            raw_mode = (
                raw_mode_byte >> 5
            ) & 0x07

            mode = MODE_NAMES.get(
                raw_mode,
                f"?{raw_mode}",
            )

            name = decode_radio_text(
                rec[14:22]
            )

            programs.append(
                {
                    "program": program,
                    "edge": edge_name,
                    "label": (
                        f"{program:02d}"
                        f"{edge_name}"
                    ),
                    "name": name,
                    "freq_mhz": (
                        freq_hz / 1_000_000.0
                        if freq_hz
                        else 0.0
                    ),
                    "mode": mode,
                    "raw_mode": raw_mode,
                    "raw_mode_byte": (
                        f"{raw_mode_byte:02X}"
                    ),
                    "raw": rec.hex().upper(),
                }
            )

    return programs


# =============================================================================
# Identification
# =============================================================================

def parse_identification(
    data: bytes,
) -> dict:

    if len(data) < 23:

        raise ValueError(
            "E1 response is too short: "
            f"{len(data)} bytes"
        )

    model = data[0:4]

    comment = (
        data[4:20]
        .rstrip(b"\x00")
        .decode(
            "ascii",
            errors="replace",
        )
    )

    region_info = data[20:23]

    rest = data[23:]

    firmware = ""
    track_blocks = []

    track_start = None

    for i in range(
        len(rest)
    ):

        candidate = rest[i:]

        if len(candidate) < 4:
            continue

        if len(candidate) % 4:
            continue

        try:

            groups = [
                candidate[j:j + 4].decode(
                    "ascii"
                )
                for j in range(
                    0,
                    len(candidate),
                    4,
                )
            ]

        except UnicodeDecodeError:
            continue

        if not groups:
            continue

        if not all(
            len(group) == 4
            and all(
                c in
                "0123456789abcdefABCDEF"
                for c in group
            )
            for group in groups
        ):
            continue

        if "FFFF" in groups:

            track_start = i
            break

    if track_start is not None:

        firmware_raw = (
            rest[:track_start]
        )

        track_raw = (
            rest[track_start:]
        )

        firmware = (
            firmware_raw
            .rstrip(b"\x00")
            .decode(
                "ascii",
                errors="replace",
            )
        )

        for i in range(
            0,
            len(track_raw) - 3,
            4,
        ):

            group = track_raw[
                i:i + 4
            ]

            try:

                text = group.decode(
                    "ascii"
                )

            except UnicodeDecodeError:
                break

            if not all(
                c in
                "0123456789abcdefABCDEF"
                for c in text
            ):
                break

            value = int(
                text,
                16,
            )

            if value == 0xFFFF:
                break

            track_blocks.append(
                value
            )

            if len(track_blocks) >= 32:
                break

    else:

        firmware_bytes = bytearray()

        for byte in rest:

            if 32 <= byte <= 126:
                firmware_bytes.append(
                    byte
                )
            else:
                break

        firmware = bytes(
            firmware_bytes
        ).decode(
            "ascii",
            errors="replace",
        )

    return {
        "model": model,
        "comment": comment,
        "region_info": region_info,
        "firmware": firmware,
        "track_blocks": track_blocks,
        "raw": data,
    }


# =============================================================================
# File handling
# =============================================================================

def save_icf(
    path: str,
    memory: bytes,
):

    if len(memory) != SETTINGS_SIZE:

        raise ValueError(
            "Refusing to save invalid image size: "
            f"{len(memory)}"
        )

    with open(
        path,
        "wb",
    ) as f:

        f.write(memory)

    print(
        f"Saved {len(memory):,} bytes to {path}"
    )


def load_icf(
    path: str,
) -> bytes:

    with open(
        path,
        "rb",
    ) as f:

        data = f.read()

    if len(data) != SETTINGS_SIZE:

        raise ValueError(
            f"{path}: expected "
            f"{SETTINGS_SIZE:,} bytes, "
            f"got {len(data):,}"
        )

    return data


# =============================================================================
# Recorder tracks (OKI ADPCM)
# =============================================================================

QUALITY_NAMES = {
    0: "LONG",
    1: "NORMAL",
    2: "FINE",
}

# 91-entry step table from OkiAdpcmDec.dll (verified bit-exact in ic-r20-studio)
OKI_STEPS = [
    16, 17, 18, 20, 21, 23, 25, 27, 29, 31, 34, 37, 40, 43, 46, 50, 54, 59, 63, 69,
    74, 80, 86, 93, 101, 109, 118, 127, 138, 149, 161, 173, 187, 202, 219, 236, 255, 275, 298, 321,
    347, 375, 405, 437, 472, 510, 551, 595, 643, 694, 750, 810, 875, 945, 1020, 1102, 1190, 1286, 1388, 1500,
    1620, 1749, 1889, 2040, 2204, 2380, 2570, 2776, 2998, 3238, 3497, 3777, 4079, 4406, 4758, 5139, 5550, 5994, 6474, 6991,
    7551, 8155, 8807, 9512, 10273, 11095, 11982, 12941, 13976, 15095, 16302,
]

OKI_ADJUST4 = [-2, -2, -2, -2, 2, 6, 9, 11]
OKI_ADJUST2 = [-2, 3]


class OkiAdpcm:
    """OKI ADPCM decoder (2-bit LONG / 4-bit NORMAL+FINE), matching OkiAdpcmDec.dll."""

    def __init__(self, bits_per_sample: int):
        if bits_per_sample not in (2, 4):
            raise ValueError("bits_per_sample must be 2 or 4")
        self.bits = bits_per_sample
        self.index = 0
        self.predicted = 0

    def reset(self):
        self.index = 0
        self.predicted = 0

    def decode_code(self, code: int) -> int:
        sign_mask = 1 << (self.bits - 1)
        step = OKI_STEPS[self.index]
        sign = -1 if (code & sign_mask) else 1
        for bit in range(self.bits - 2, -1, -1):
            if (code >> bit) & 1:
                self.predicted += sign * step
            step >>= 1
        self.predicted += sign * step

        magnitude = code & ~sign_mask & ((1 << self.bits) - 1)
        adj = OKI_ADJUST4 if self.bits == 4 else OKI_ADJUST2
        self.index = max(0, min(90, self.index + adj[magnitude]))
        self.predicted = max(-32768, min(32767, self.predicted))
        return self.predicted

    def decode(self, data: bytes) -> list:
        per_byte = 8 // self.bits
        mask = (1 << self.bits) - 1
        out = []
        for b in data:
            for k in range(per_byte - 1, -1, -1):
                out.append(self.decode_code((b >> (k * self.bits)) & mask))
        return out


def parse_tracks(memory: bytes) -> list:
    """
    Parse the track table at 0x72A1.

    Each entry is 3 bytes: end block (LE uint16), quality (0/1/2).
    End is exclusive: track N occupies [start, end).
    Track numbers are 1-based.
    """
    tracks = []
    start = 0
    for i in range(RECORDER_TRACK_MAX):
        off = RECORDER_TRACK_TABLE + i * 3
        if off + 2 >= len(memory):
            break
        end = int.from_bytes(memory[off:off + 2], "little")
        quality = memory[off + 2]
        if end == 0xFFFF or quality > 2:
            break
        if end > start:
            blocks = end - start
            bits = 2 if quality == 0 else 4
            rate = 16000 if quality == 2 else 8000
            audio_bytes = blocks * SOUND_BLOCK_PAYLOAD
            duration_s = (audio_bytes * 8.0) / bits / rate
            tracks.append({
                "number": len(tracks) + 1,
                "start": start,
                "end": end,
                "blocks": blocks,
                "quality": quality,
                "quality_name": QUALITY_NAMES.get(quality, f"?{quality}"),
                "sample_rate": rate,
                "bits": bits,
                "duration_s": duration_s,
            })
        start = end
    return tracks


def save_wav(path: str, pcm: list, sample_rate: int):
    """Write mono 16-bit PCM as a standard WAV file (stdlib only)."""
    import struct
    import wave

    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        frames = b"".join(struct.pack("<h", s) for s in pcm)
        w.writeframes(frames)


def try_save_mp3(wav_path: str, mp3_path: str) -> bool:
    """Convert WAV→MP3 via ffmpeg if available. Returns True on success."""
    import shutil
    import subprocess

    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        print("  (ffmpeg not found; skipping MP3 — install ffmpeg for --mp3)")
        return False
    try:
        subprocess.run(
            [
                ffmpeg, "-y", "-loglevel", "error",
                "-i", wav_path,
                "-codec:a", "libmp3lame", "-qscale:a", "4",
                mp3_path,
            ],
            check=True,
            timeout=120,
        )
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        print(f"  (ffmpeg MP3 failed: {exc})")
        return False


def save_icw(path: str, quality: int, blocks: bytes, comment: str = ""):
    """Write CS-R20 / icw2wav .icw text format."""
    if len(blocks) % SOUND_BLOCK_SIZE:
        raise ValueError(
            f"ICW data length {len(blocks)} not a multiple of {SOUND_BLOCK_SIZE}"
        )
    n_blocks = len(blocks) // SOUND_BLOCK_SIZE
    lines = [
        f"26990001{n_blocks:08X}{quality:02X}",
        f"#Comment={comment}",
    ]
    for off in range(0, len(blocks), 16):
        chunk = blocks[off:off + 16]
        lines.append(
            f"{off:08X}{len(chunk):02X}{chunk.hex().upper()}"
        )
    with open(path, "w", encoding="ascii", newline="\r\n") as f:
        f.write("\r\n".join(lines) + "\r\n")


def load_icw(path: str) -> dict:
    """Load a .icw file → {quality, blocks, comment}."""
    with open(path, "r", encoding="ascii", errors="replace") as f:
        lines = [ln.strip() for ln in f.readlines()]

    if not lines or not lines[0].upper().startswith("26990001"):
        raise ValueError(f"{path}: not an ICW file")

    header = lines[0]
    n_blocks = int(header[8:16], 16)
    quality = int(header[16:18], 16)
    if quality > 2:
        raise ValueError(f"bad quality {quality}")

    data = bytearray(n_blocks * SOUND_BLOCK_SIZE)
    comment = ""
    for line in lines[1:]:
        if not line:
            continue
        if line.startswith("#"):
            if line.upper().startswith("#COMMENT="):
                comment = line[9:]
            continue
        off = int(line[0:8], 16)
        length = int(line[8:10], 16)
        hexdata = line[10:10 + length * 2]
        chunk = bytes.fromhex(hexdata)
        if off + len(chunk) > len(data):
            raise ValueError(f"ICW line overruns buffer at 0x{off:08X}")
        data[off:off + len(chunk)] = chunk

    return {
        "quality": quality,
        "blocks": bytes(data),
        "comment": comment,
    }


def decode_track_pcm(quality: int, blocks: bytes) -> tuple:
    """Return (pcm_samples, sample_rate) from raw 512-byte blocks."""
    if len(blocks) % SOUND_BLOCK_SIZE:
        raise ValueError("block data misaligned")
    payload = bytearray()
    for i in range(0, len(blocks), SOUND_BLOCK_SIZE):
        payload.extend(
            blocks[i + SOUND_BLOCK_HEADER:i + SOUND_BLOCK_SIZE]
        )
    bits = 2 if quality == 0 else 4
    rate = 16000 if quality == 2 else 8000
    pcm = OkiAdpcm(bits).decode(bytes(payload))
    return pcm, rate


def export_track_files(
    base_path: str,
    quality: int,
    blocks: bytes,
    want_mp3: bool = False,
    comment: str = "",
):
    """Write .icw + .wav (+ optional .mp3) for one track."""
    icw_path = base_path + ".icw"
    wav_path = base_path + ".wav"
    save_icw(icw_path, quality, blocks, comment=comment)
    pcm, rate = decode_track_pcm(quality, blocks)
    save_wav(wav_path, pcm, rate)
    print(f"  → {icw_path}")
    print(f"  → {wav_path} ({len(pcm)} samples @ {rate} Hz)")
    if want_mp3:
        mp3_path = base_path + ".mp3"
        if try_save_mp3(wav_path, mp3_path):
            print(f"  → {mp3_path}")


# =============================================================================
# CSV export / import (CHIRP-compatible, intended to match ic-r20-studio)
# =============================================================================

CHIRP_COLUMNS = [
    "Location",
    "Name",
    "Frequency",
    "Duplex",
    "Offset",
    "Tone",
    "rToneFreq",
    "cToneFreq",
    "DtcsCode",
    "DtcsPolarity",
    "RxDtcsCode",
    "CrossMode",
    "Mode",
    "TStep",
    "Skip",
    "Power",
    "Comment",
    "URCALL",
    "RPT1CALL",
    "RPT2CALL",
    "DVCODE",
    "Bank",
    "BankSlot",
    "SsbStep",
]


def _step_text(step_name: str) -> str:
    """Convert internal step label like '5k' / '6.25k' to CHIRP number text."""
    if not step_name or step_name.startswith("?"):
        return ""
    return step_name.replace("k", "")


def _parse_step_khz(text: str) -> int:
    """Map step text (kHz) back to STEP_NAMES index, or -1."""
    text = (text or "").strip().rstrip("kK")
    if not text:
        return -1
    try:
        khz = float(text)
    except ValueError:
        return -1
    targets = [
        0.01, 0.1, 1, 5, 6.25, 8.33, 9, 10, 12.5, 15, 20, 25, 30, 50, 100,
    ]
    for i, t in enumerate(targets):
        if abs(t - khz) < 0.01:
            return i
    return -1


def csv_export(
    icf_path: str,
    csv_path: str,
):
    """Export used channels in CHIRP-compatible CSV (Bank/BankSlot/SsbStep extra)."""

    memory = load_icf(icf_path)
    channels = decode_channels(memory)

    rows = []
    for ch in channels:
        if ch["empty"] or ch["freq_mhz"] <= 0:
            continue

        tone_mode = ch["tone_mode"]
        if tone_mode == "TSQL":
            tone = "TSQL"
        elif tone_mode == "DTCS":
            tone = "DTCS"
        else:
            tone = ""

        ctcss = ""
        if ch["ctcss_index"] < len(CTCSS_TONES):
            ctcss = f"{CTCSS_TONES[ch['ctcss_index']]:.1f}"

        dtcs = ""
        if ch["dtcs_index"] < len(DTCS_CODES):
            dtcs = f"{DTCS_CODES[ch['dtcs_index']]:03d}"

        comment = ""
        if tone_mode not in ("OFF", "TSQL", "DTCS", ""):
            comment = tone_mode

        skip = ""
        if ch["skip"] == "SKIP":
            skip = "S"
        elif ch["skip"] == "PSKIP":
            skip = "P"

        pol = "RN" if ch["dtcs_reverse"] else "NN"

        rows.append({
            "Location": str(ch["ch"]),
            "Name": ch["name"],
            "Frequency": f"{ch['freq_mhz']:.6f}",
            "Duplex": ch["duplex"],
            "Offset": f"{ch['offset_mhz']:.6f}",
            "Tone": tone,
            "rToneFreq": ctcss,
            "cToneFreq": ctcss,
            "DtcsCode": dtcs,
            "DtcsPolarity": pol,
            "RxDtcsCode": dtcs,
            "CrossMode": "Tone->Tone",
            "Mode": ch["mode"],
            "TStep": _step_text(ch["step"]),
            "Skip": skip,
            "Power": "",
            "Comment": comment,
            "URCALL": "",
            "RPT1CALL": "",
            "RPT2CALL": "",
            "DVCODE": "",
            "Bank": ch["bank_letter"],
            "BankSlot": (
                str(ch["bank_slot"])
                if ch["bank"] is not None
                else ""
            ),
            "SsbStep": _step_text(ch["ssb_step"]),
        })

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CHIRP_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Exported {len(rows)} non-empty channels to {csv_path}")


def list_icf(icf_path: str):
    """Print used channels, bank names and track summary (like ir20 list)."""

    memory = load_icf(icf_path)
    channels = decode_channels(memory)

    used = [
        ch for ch in channels
        if not ch["empty"] and ch["freq_mhz"] > 0
    ]

    print(f"# {icf_path}")
    for ch in used:
        bank_str = ""
        if ch["bank"] is not None:
            bank_str = f"{ch['bank_letter']}{ch['bank_slot']:02d}"

        tone_mode = ch["tone_mode"]
        ctcss = (
            f"{CTCSS_TONES[ch['ctcss_index']]:5.1f}"
            if ch["ctcss_index"] < len(CTCSS_TONES)
            else "  ?  "
        )
        dtcs = (
            f"D{DTCS_CODES[ch['dtcs_index']]:03d}"
            if ch["dtcs_index"] < len(DTCS_CODES)
            else "D???"
        )

        print(
            f"{ch['ch']:4d} "
            f"{ch['freq_mhz']:12.6f} "
            f"{ch['mode']:<4} "
            f"{ch['name']:<8} "
            f"TS={ch['step']:<6} "
            f"{ch['duplex'] or ' ':<4} "
            f"{tone_mode:<5} "
            f"{ctcss} "
            f"{dtcs} "
            f"{ch['skip']:<5} "
            f"{bank_str}"
        )

    for b in range(26):
        name = get_bank_name(memory, b)
        if name:
            print(f"Bank {bank_letter(b)}: {name}")

    # Track table summary (exclusive end blocks, same as ic-r20-studio)
    for t in parse_tracks(memory):
        mins, secs = divmod(int(t["duration_s"]), 60)
        print(
            f"Track {t['number']:02d}: "
            f"blocks {t['start']}-{t['end'] - 1} "
            f"({t['blocks']}) "
            f"quality={t['quality_name']} "
            f"{mins:02d}:{secs:02d}"
        )


def _encode_channel_record(ch: dict) -> bytes:
    """Build a 22-byte channel record from a decoded-style dict."""
    rec = bytearray(22)

    freq_hz = int(round(ch["freq_mhz"] * 1_000_000))
    offset_hz = int(round(ch.get("offset_mhz", 0) * 1_000_000))

    rec[0:4] = freq_hz.to_bytes(4, "little")
    rec[4:8] = offset_hz.to_bytes(4, "little")

    step_index = ch.get("step_index", 0) & 0x0F
    ssb_step_index = ch.get("ssb_step_index", 0) & 0x0F
    rec[8] = step_index | (ssb_step_index << 4)

    tone_mode_map = {
        "OFF": 0, "TSQL": 2, "DTCS": 4, "TRAIN": 5, "MSK": 6, "VSC": 7,
    }
    tone_mode = tone_mode_map.get(ch.get("tone_mode", "OFF"), 0)

    duplex_value = {"": 0, "-": 1, "+": 2}.get(ch.get("duplex", ""), 0)
    dtcs_reverse = 0x40 if ch.get("dtcs_reverse") else 0

    # byte 11: tone mode bits 1-3, duplex 4-5, dtcs reverse bit 6
    rec[11] = (
        ((tone_mode & 0x07) << 1)
        | ((duplex_value & 0x03) << 4)
        | dtcs_reverse
    )

    dtcs_index = ch.get("dtcs_index", 0) & 0x7F
    ctcss_index = ch.get("ctcss_index", 0) & 0x3F
    rec[12] = dtcs_index | ((ctcss_index & 1) << 7)

    raw_mode = MODE_VALUES.get(ch.get("mode", "FM"), 0)
    rec[13] = ((ctcss_index >> 1) & 0x1F) | ((raw_mode & 0x07) << 5)

    # Blank names must be spaces (0x20), not NULs — otherwise the radio
    # shows the default "M:nnn" channel label.
    rec[14:22] = encode_radio_text(ch.get("name") or "", 8)

    return bytes(rec)


def csv_import(
    icf_path: str,
    csv_path: str,
    out_path: str,
    replace: bool = False,
):
    """Import CHIRP-style CSV into an .icf image."""

    memory = bytearray(load_icf(icf_path))
    channels = decode_channels(bytes(memory))

    with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
        # Detect delimiter
        sample = f.read(4096)
        f.seek(0)
        delim = ";" if sample.count(";") > sample.count(",") else ","
        reader = csv.DictReader(f, delimiter=delim)
        rows = list(reader)

    if not rows:
        print("CSV is empty")
        return

    # Normalise header keys
    def cell(row, *names):
        for n in names:
            for k, v in row.items():
                if k and k.strip().lower() == n.lower():
                    return (v or "").strip()
        return ""

    if replace:
        for i in range(NUM_CHANNELS):
            set_channel_empty(memory, i, True)
            # Zero the channel record
            off = CHANNEL_START + i * CHANNEL_SIZE
            memory[off:off + CHANNEL_SIZE] = b"\x00" * CHANNEL_SIZE
            set_channel_skip(memory, i, "OFF")
            set_channel_bank(memory, i, None, None)

    imported = 0
    warnings = []
    next_free = 0

    for n, row in enumerate(rows, 1):
        freq_s = cell(row, "Frequency")
        if not freq_s:
            continue
        try:
            mhz = float(freq_s)
        except ValueError:
            warnings.append(f"row {n}: bad frequency {freq_s!r}")
            continue
        if mhz <= 0:
            continue

        loc_s = cell(row, "Location")
        if loc_s.isdigit() and 0 <= int(loc_s) < NUM_CHANNELS:
            loc = int(loc_s)
        else:
            while next_free < NUM_CHANNELS and not channel_is_empty(bytes(memory), next_free):
                # after replace all are empty; otherwise skip non-empty
                if not replace and not channels[next_free]["empty"]:
                    next_free += 1
                    continue
                break
            # simpler free search
            while next_free < NUM_CHANNELS:
                if replace or channels[next_free]["empty"] or channels[next_free]["freq_mhz"] <= 0:
                    break
                next_free += 1
            if next_free >= NUM_CHANNELS:
                warnings.append(f"row {n}: no free channel")
                break
            loc = next_free
            next_free += 1

        name = cell(row, "Name")[:8]
        duplex = cell(row, "Duplex")
        if duplex not in ("", "-", "+"):
            duplex = ""

        offset_mhz = 0.0
        off_s = cell(row, "Offset")
        if off_s:
            try:
                offset_mhz = float(off_s)
            except ValueError:
                pass

        mode = cell(row, "Mode").upper() or "FM"
        if mode == "NFM":
            mode = "FM"
        if mode not in MODE_VALUES:
            warnings.append(f"row {n}: unknown mode {mode!r}, using FM")
            mode = "FM"

        tone = cell(row, "Tone").upper()
        comment = cell(row, "Comment").upper()
        if tone in ("TSQL", "TONE"):
            tone_mode = "TSQL"
        elif tone == "DTCS":
            tone_mode = "DTCS"
        elif comment in ("VSC", "TRAIN", "MSK"):
            tone_mode = comment
        else:
            tone_mode = "OFF"

        ctcss_index = 0
        tone_text = cell(row, "cToneFreq") or cell(row, "rToneFreq")
        if tone_text:
            try:
                val = float(tone_text)
                best = min(
                    range(len(CTCSS_TONES)),
                    key=lambda i: abs(CTCSS_TONES[i] - val),
                )
                ctcss_index = best
            except ValueError:
                pass

        dtcs_index = 0
        dtcs_s = cell(row, "DtcsCode")
        if dtcs_s:
            try:
                code = int(dtcs_s)
                if code in DTCS_CODES:
                    dtcs_index = DTCS_CODES.index(code)
            except ValueError:
                pass

        pol = cell(row, "DtcsPolarity").upper()
        dtcs_reverse = pol.startswith("R")

        step_index = _parse_step_khz(cell(row, "TStep"))
        if step_index < 0:
            step_index = 3  # 5k default
        ssb_step_index = _parse_step_khz(cell(row, "SsbStep"))
        if ssb_step_index < 0:
            ssb_step_index = step_index

        skip_s = cell(row, "Skip").upper()
        if skip_s in ("S", "SKIP"):
            skip = "SKIP"
        elif skip_s in ("P", "PSKIP"):
            skip = "PSKIP"
        else:
            skip = "OFF"

        bank = None
        bank_slot = 0
        bank_s = cell(row, "Bank").upper()
        if len(bank_s) == 1 and "A" <= bank_s <= "Z":
            bank = ord(bank_s) - ord("A")
            slot_s = cell(row, "BankSlot")
            try:
                bank_slot = max(0, min(99, int(slot_s))) if slot_s else 0
            except ValueError:
                bank_slot = 0

        ch = {
            "freq_mhz": mhz,
            "offset_mhz": offset_mhz,
            "name": name,
            "mode": mode,
            "duplex": duplex,
            "tone_mode": tone_mode,
            "ctcss_index": ctcss_index,
            "dtcs_index": dtcs_index,
            "dtcs_reverse": dtcs_reverse,
            "step_index": step_index,
            "ssb_step_index": ssb_step_index,
        }

        rec = _encode_channel_record(ch)
        off = CHANNEL_START + loc * CHANNEL_SIZE
        memory[off:off + CHANNEL_SIZE] = rec
        set_channel_empty(memory, loc, False)
        set_channel_skip(memory, loc, skip)
        set_channel_bank(memory, loc, bank, bank_slot)
        imported += 1

    save_icf(out_path, bytes(memory))
    print(f"Imported {imported} channels → {out_path}")
    for w in warnings:
        print(f"  ! {w}")


def csv_export_program_scans(
    icf_path: str,
    csv_path: str,
):

    memory = load_icf(
        icf_path
    )

    programs = decode_program_scans(
        memory
    )

    fieldnames = [
        "program",
        "edge",
        "label",
        "name",
        "freq_mhz",
        "mode",
        "raw_mode",
        "raw_mode_byte",
        "raw",
    ]

    with open(
        csv_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )

        writer.writeheader()

        writer.writerows(
            programs
        )

    programmed = [
        item
        for item in programs
        if item["freq_mhz"] > 0
    ]

    print(
        f"Exported {len(programmed)} "
        f"programmed scan edges to "
        f"{csv_path}"
    )


# =============================================================================
# Memory dump
# =============================================================================

def dump_region(
    memory: bytes,
    start: int,
    length: int,
):

    if start < 0:
        raise ValueError(
            "Start address cannot be negative"
        )

    if length < 0:
        raise ValueError(
            "Length cannot be negative"
        )

    end = start + length

    if end > len(memory):

        raise ValueError(
            f"Region 0x{start:04X}-"
            f"0x{end - 1:04X} exceeds "
            f"image size"
        )

    for address in range(
        start,
        end,
        16,
    ):

        chunk = memory[
            address:
            min(
                address + 16,
                end,
            )
        ]

        hex_part = " ".join(
            f"{byte:02X}"
            for byte in chunk
        )

        ascii_part = "".join(
            chr(byte)
            if 32 <= byte <= 126
            else "."
            for byte in chunk
        )

        print(
            f"{address:04X}: "
            f"{hex_part:<47}  "
            f"{ascii_part}"
        )


# =============================================================================
# Channel diagnostic
# =============================================================================

def print_channel_diagnostic(
    memory: bytes,
    start: int,
    count: int,
):

    channels = decode_channels(
        memory
    )

    end = min(
        start + count,
        len(channels)
    )

    print()

    print(
        " CH   FREQ          MODE "
        "RAW  BYTE  SKIP   NAME"
    )

    print(
        "----  ------------  ---- "
        "---  ----  -----  ----------------"
    )

    for channel in channels[
        start:end
    ]:

        print(
            f"{channel['ch']:4d}  "
            f"{channel['freq_mhz']:12.6f}  "
            f"{channel['mode']:4s} "
            f"{channel['raw_mode']:3d}  "
            f"{channel['raw_mode_byte']:>4s}  "
            f"{channel['skip']:5s}  "
            f"{channel['name']}"
        )


# =============================================================================
# Serial-port detection
# =============================================================================

def find_port() -> str:

    ports = list(
        serial.tools.list_ports.comports()
    )

    if not ports:

        print(
            "No serial ports found.\n"
            "Connect the OPC-1382 cable."
        )

        sys.exit(1)

    ftdi = []

    for port in ports:

        manufacturer = (
            port.manufacturer
            or ""
        ).lower()

        description = (
            port.description
            or ""
        ).lower()

        hwid = (
            port.hwid
            or ""
        ).lower()

        if (
            "ftdi" in manufacturer
            or "ftdi" in description
            or "0403" in hwid
        ):

            ftdi.append(
                port
            )

    if ftdi:

        selected = ftdi[0]

        print(
            "Found FTDI device: "
            f"{selected.device} "
            f"({selected.description})"
        )

        return selected.device

    print(
        "Available serial ports:"
    )

    for index, port in enumerate(
        ports,
        1,
    ):

        print(
            f"  {index}. "
            f"{port.device} — "
            f"{port.description}"
        )

    while True:

        choice = input(
            "Select port number or enter "
            "device path: "
        ).strip()

        if not choice:
            continue

        if choice.isdigit():

            index = int(
                choice
            )

            if (
                1
                <= index
                <= len(ports)
            ):

                return ports[
                    index - 1
                ].device

        for port in ports:

            if choice == port.device:
                return port.device

        print(
            "Invalid selection."
        )


# =============================================================================
# Info
# =============================================================================

def print_info(
    data: bytes,
):

    info = parse_identification(
        data
    )

    model = info["model"]

    print()

    print(
        f"Model:      "
        f"{model.hex(' ')}"
    )

    if model == bytes(
        [0x26, 0x99, 0x00, 0x01]
    ):

        print(
            "            IC-R20 identification"
        )

    elif model[:2] == MODEL[:2]:

        print(
            "            IC-R20"
        )

    print(
        f"Comment:    "
        f"{info['comment']}"
    )

    print(
        "Region:     "
        f"{info['region_info'].hex(' ')}"
    )

    print(
        f"Firmware:   "
        f"{info['firmware']}"
    )

    tracks = info[
        "track_blocks"
    ]

    if tracks:

        print(
            f"Tracks:     "
            f"{len(tracks)}"
        )

        for index, block in enumerate(
            tracks,
            1,
        ):

            print(
                f"  Track {index:02d}: "
                f"end block {block}"
            )

    else:

        print(
            "Tracks:     none"
        )

    print()


# =============================================================================
# Argument parser
# =============================================================================

def build_parser():

    parser = argparse.ArgumentParser(
        description=(
            "Icom IC-R20 clone tool "
            "(Python; CLI based on ic-r20-studio)"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  ir20.py info\n"
            "  ir20.py read backup.icf\n"
            "  ir20.py write backup.icf --yes\n"
            "  ir20.py list backup.icf\n"
            "  ir20.py csv-export backup.icf ch.csv\n"
            "  ir20.py csv-import backup.icf ch.csv out.icf\n"
            "  ir20.py csv-import backup.icf ch.csv out.icf --replace\n"
            "  ir20.py tracks\n"
            "  ir20.py download all out\n"
            "  ir20.py download 1 out --mp3\n"
            "  ir20.py icw track01.icw --mp3\n"
        ),
    )

    parser.add_argument(
        "--port",
        help="Serial port, e.g. /dev/ttyUSB0",
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show raw clone protocol frames",
    )

    sub = parser.add_subparsers(dest="command")

    sub.add_parser(
        "info",
        help="Query model, firmware and recorder tracks",
    )

    read_parser = sub.add_parser(
        "read",
        help="Read IC-R20 memory to .icf",
    )
    read_parser.add_argument("output", help="Output .icf file")

    write_parser = sub.add_parser(
        "write",
        help="Write an .icf file to the radio",
    )
    write_parser.add_argument("input", help="Input .icf file")
    write_parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip confirmation",
    )

    list_parser = sub.add_parser(
        "list",
        help="List used channels, banks and tracks from an .icf",
    )
    list_parser.add_argument("icf", help="Input .icf file")

    csv_parser = sub.add_parser(
        "csv-export",
        help="Export channels to CHIRP-compatible CSV",
    )
    csv_parser.add_argument("icf", help="Input .icf file")
    csv_parser.add_argument("csv", help="Output CSV file")

    csv_in = sub.add_parser(
        "csv-import",
        help="Import CHIRP-compatible CSV into an .icf image",
    )
    csv_in.add_argument("icf", help="Base .icf file")
    csv_in.add_argument("csv", help="Input CSV file")
    csv_in.add_argument("output", help="Output .icf file")
    csv_in.add_argument(
        "--replace",
        action="store_true",
        help="Clear all channels before import",
    )

    tracks_parser = sub.add_parser(
        "tracks",
        help="List recorder tracks on the radio",
    )

    dl_parser = sub.add_parser(
        "download",
        help="Download recorder track(s) to ICW/WAV (and optional MP3)",
    )
    dl_parser.add_argument(
        "which",
        help='"all" or a 1-based track number',
    )
    dl_parser.add_argument(
        "outdir",
        nargs="?",
        default=".",
        help="Output directory (default: .)",
    )
    dl_parser.add_argument(
        "--mp3",
        action="store_true",
        help="Also encode MP3 via ffmpeg if available",
    )

    icw_parser = sub.add_parser(
        "icw",
        help="Convert an offline .icw file to WAV (and optional MP3)",
    )
    icw_parser.add_argument("icw", help="Input .icw file")
    icw_parser.add_argument(
        "--mp3",
        action="store_true",
        help="Also encode MP3 via ffmpeg if available",
    )

    scan_parser = sub.add_parser(
        "scan-export",
        help="Export programmed scan edges from an .icf image",
    )
    scan_parser.add_argument("icf", help="Input .icf file")
    scan_parser.add_argument("csv", help="Output CSV file")

    dump_parser = sub.add_parser(
        "dump",
        help="Dump raw bytes from an .icf",
    )
    dump_parser.add_argument("icf", help="Input .icf file")
    dump_parser.add_argument(
        "--start",
        type=lambda value: int(value, 0),
        required=True,
        help="Start address, e.g. 0x6716",
    )
    dump_parser.add_argument(
        "--length",
        type=lambda value: int(value, 0),
        required=True,
        help="Number of bytes, e.g. 0x200",
    )

    diag_parser = sub.add_parser(
        "channel-dump",
        help="Decode a range of normal memory channels",
    )
    diag_parser.add_argument("icf", help="Input .icf file")
    diag_parser.add_argument(
        "--start",
        type=int,
        default=0,
        help="First channel",
    )
    diag_parser.add_argument(
        "--count",
        type=int,
        default=20,
        help="Number of channels",
    )

    return parser


# =============================================================================
# Main
# =============================================================================

def main():

    parser = build_parser()

    args = parser.parse_args()

    if not args.command:

        parser.print_help()

        return 1

    # -------------------------------------------------------------------------
    # CSV export does not need radio.
    # -------------------------------------------------------------------------

    if args.command == "csv-export":

        try:

            csv_export(
                args.icf,
                args.csv,
            )

        except Exception as exc:

            print(
                f"Error: {exc}",
                file=sys.stderr,
            )

            return 1

        return 0

    # -------------------------------------------------------------------------
    # CSV import.
    # -------------------------------------------------------------------------

    if args.command == "csv-import":

        try:

            csv_import(
                args.icf,
                args.csv,
                args.output,
                replace=args.replace,
            )

        except Exception as exc:

            print(
                f"Error: {exc}",
                file=sys.stderr,
            )

            return 1

        return 0

    # -------------------------------------------------------------------------
    # List image contents.
    # -------------------------------------------------------------------------

    if args.command == "list":

        try:

            list_icf(args.icf)

        except Exception as exc:

            print(
                f"Error: {exc}",
                file=sys.stderr,
            )

            return 1

        return 0

    # -------------------------------------------------------------------------
    # Offline ICW conversion (no radio).
    # -------------------------------------------------------------------------

    if args.command == "icw":

        try:

            track = load_icw(args.icw)
            base = args.icw
            if base.lower().endswith(".icw"):
                base = base[:-4]
            pcm, rate = decode_track_pcm(
                track["quality"],
                track["blocks"],
            )
            wav_path = base + ".wav"
            save_wav(wav_path, pcm, rate)
            qname = QUALITY_NAMES.get(
                track["quality"],
                f"?{track['quality']}",
            )
            duration_s = len(pcm) / rate
            mins, secs = divmod(int(duration_s), 60)
            print(
                f"{wav_path}: {qname}, "
                f"{mins:02d}:{secs:02d}"
            )
            if args.mp3:
                mp3_path = base + ".mp3"
                if try_save_mp3(wav_path, mp3_path):
                    print(f"{mp3_path}")

        except Exception as exc:

            print(
                f"Error: {exc}",
                file=sys.stderr,
            )

            return 1

        return 0

    # -------------------------------------------------------------------------
    # Programmed scan export.
    # -------------------------------------------------------------------------

    if args.command == "scan-export":

        try:

            csv_export_program_scans(
                args.icf,
                args.csv,
            )

        except Exception as exc:

            print(
                f"Error: {exc}",
                file=sys.stderr,
            )

            return 1

        return 0

    # -------------------------------------------------------------------------
    # Raw dump.
    # -------------------------------------------------------------------------

    if args.command == "dump":

        try:

            memory = load_icf(
                args.icf
            )

            dump_region(
                memory,
                args.start,
                args.length,
            )

        except Exception as exc:

            print(
                f"Error: {exc}",
                file=sys.stderr,
            )

            return 1

        return 0

    # -------------------------------------------------------------------------
    # Channel diagnostic.
    # -------------------------------------------------------------------------

    if args.command == "channel-dump":

        try:

            memory = load_icf(
                args.icf
            )

            print_channel_diagnostic(
                memory,
                args.start,
                args.count,
            )

        except Exception as exc:

            print(
                f"Error: {exc}",
                file=sys.stderr,
            )

            return 1

        return 0

    # -------------------------------------------------------------------------
    # Radio commands.
    # -------------------------------------------------------------------------

    port = (
        args.port
        or find_port()
    )

    radio = None

    try:

        radio = ICR20(
            port,
            verbose=args.verbose,
        )

        # ---------------------------------------------------------------------
        # INFO
        # ---------------------------------------------------------------------

        if args.command == "info":

            print(
                "Querying IC-R20..."
            )

            data = radio.query_id()

            print_info(
                data
            )

        # ---------------------------------------------------------------------
        # TRACKS
        # ---------------------------------------------------------------------

        elif args.command == "tracks":

            print("Querying IC-R20...")
            radio.query_id()
            # Track table lives in the settings image
            memory = radio.read_range(
                0,
                SETTINGS_SIZE - 1,
                label="settings (track table)",
            )
            tracks = parse_tracks(memory)
            if not tracks:
                print("No recorder tracks.")
            else:
                for t in tracks:
                    mins, secs = divmod(int(t["duration_s"]), 60)
                    print(
                        f"Track {t['number']:02d}: "
                        f"{t['quality_name']:<6} "
                        f"{mins:02d}:{secs:02d}  "
                        f"blocks {t['start']}-{t['end'] - 1} "
                        f"({t['blocks']})"
                    )
            radio.exit_clone_mode()

        # ---------------------------------------------------------------------
        # DOWNLOAD
        # ---------------------------------------------------------------------

        elif args.command == "download":

            import os

            outdir = args.outdir
            os.makedirs(outdir, exist_ok=True)

            print("Querying IC-R20...")
            radio.query_id()
            memory = radio.read_range(
                0,
                SETTINGS_SIZE - 1,
                label="settings (track table)",
            )
            tracks = parse_tracks(memory)
            if not tracks:
                print("No recorder tracks.")
                radio.exit_clone_mode()
                return 0

            which = args.which.strip().lower()
            if which == "all":
                pick = tracks
            else:
                try:
                    num = int(which)
                except ValueError:
                    print(
                        f"Error: expected 'all' or a track number, got {args.which!r}",
                        file=sys.stderr,
                    )
                    return 1
                pick = [t for t in tracks if t["number"] == num]
                if not pick:
                    print(
                        f"Error: track {num} not found "
                        f"(have {len(tracks)})",
                        file=sys.stderr,
                    )
                    return 1

            for t in pick:
                mins, secs = divmod(int(t["duration_s"]), 60)
                print(
                    f"Track {t['number']:02d}: "
                    f"{t['quality_name']} "
                    f"{mins:02d}:{secs:02d}"
                )
                raw = radio.read_sound_blocks(
                    t["start"],
                    t["end"],
                )
                base = os.path.join(
                    outdir,
                    f"IC-R20_track{t['number']:02d}",
                )
                export_track_files(
                    base,
                    t["quality"],
                    raw,
                    want_mp3=args.mp3,
                )

            radio.exit_clone_mode()

        # ---------------------------------------------------------------------
        # READ
        # ---------------------------------------------------------------------

        elif args.command == "read":

            print(
                "Reading IC-R20 memory "
                f"({SETTINGS_SIZE:,} bytes)..."
            )

            start = time.monotonic()

            memory = radio.read_settings()

            elapsed = (
                time.monotonic()
                - start
            )

            save_icf(
                args.output,
                memory,
            )

            print(
                f"Elapsed: "
                f"{elapsed:.1f}s"
            )

            # Same E2 FFFF… terminator used after tracks/download/write.
            # After a read this usually clears CLONE OUT; after a write some
            # firmwares still need a power-cycle (hardware limitation).
            radio.exit_clone_mode()

        # ---------------------------------------------------------------------
        # WRITE
        # ---------------------------------------------------------------------

        elif args.command == "write":

            memory = load_icf(
                args.input
            )

            print()

            print(
                f"Input image: "
                f"{len(memory):,} bytes"
            )

            if not args.yes:

                print()

                print(
                    "WARNING:"
                )

                print(
                    "Writing an image to the "
                    "IC-R20 will put the radio "
                    "into CLONE mode."
                )

                print(
                    "Make sure this image came "
                    "from an IC-R20 and has not "
                    "been corrupted."
                )

                print(
                    "The radio may require a "
                    "power-cycle after cloning."
                )

                answer = input(
                    "Continue? [y/N] "
                ).strip().lower()

                if answer != "y":

                    print(
                        "Aborted."
                    )

                    return 0

            print()

            print(
                "Writing IC-R20 memory..."
            )

            start = time.monotonic()

            radio.write_settings(
                memory
            )

            elapsed = (
                time.monotonic()
                - start
            )

            print()

            print(
                "Write accepted."
            )

            print(
                f"Elapsed: "
                f"{elapsed:.1f}s"
            )

            print(
                "If the display remains in "
                "CLONE/CLONE OUT mode, "
                "power-cycle the radio."
            )

    except KeyboardInterrupt:

        print(
            "\nInterrupted.",
            file=sys.stderr,
        )

        return 130

    except serial.SerialException as exc:

        print(
            f"Serial error: {exc}",
            file=sys.stderr,
        )

        return 1

    except TimeoutError as exc:

        print(
            f"Timeout: {exc}",
            file=sys.stderr,
        )

        return 1

    except Exception as exc:

        print(
            f"Error: {exc}",
            file=sys.stderr,
        )

        return 1

    finally:

        if radio is not None:
            radio.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
