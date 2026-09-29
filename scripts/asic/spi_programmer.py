"""
SPI programmer for HEEPidermis.

Parses an ELF executable and programs its loadable memory regions through
the HEEPidermis SPI slave interface.
"""

import argparse
import struct
import time

import serial
from elftools.elf.elffile import ELFFile


RAM_BASE = 0x00000000
RAM_SIZE = 0x00008000

SOC_CTRL_BASE = 0x20000000
BOOT_EXIT_LOOP = SOC_CTRL_BASE + 0x0C
BOOT_ADDRESS = SOC_CTRL_BASE + 0x10

PICO_MAX_DATA = 512
DEFAULT_CHUNK_SIZE = 512

SERIAL_READ_TIMEOUT_S = 0.25
SERIAL_WRITE_TIMEOUT_S = 2.0
REPLY_TIMEOUT_S = 3.0

READY_BANNER = b"HEEPidermis programmer ready"


def align_down(value, alignment):
    return value & ~(alignment - 1)


def align_up(value, alignment):
    return (value + alignment - 1) & ~(alignment - 1)


def format_range(start, end):
    if start == end:
        return f"0x{start:08X}"

    return f"0x{start:08X}-0x{end:08X}"


class HeepProgrammer:
    def __init__(
        self,
        port,
        baudrate=115200,
        chunk_size=DEFAULT_CHUNK_SIZE
    ):
        if chunk_size <= 0:
            raise ValueError("Chunk size must be > 0")

        if chunk_size > PICO_MAX_DATA:
            raise ValueError(
                f"Chunk size {chunk_size} exceeds Pico buffer size "
                f"({PICO_MAX_DATA} bytes)"
            )

        if chunk_size % 4 != 0:
            raise ValueError(
                "Chunk size must be a multiple of 4 bytes"
            )

        self.chunk_size = chunk_size

        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=SERIAL_READ_TIMEOUT_S,
            write_timeout=SERIAL_WRITE_TIMEOUT_S,
        )

        # Give USB CDC a moment to settle.
        #
        # The Pico may print its ready banner either before or after
        # this reset. _expect_ok() therefore also ignores a late banner.
        time.sleep(0.3)

        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _read_line(self, deadline):
        """
        Read one complete CR/LF-terminated line until an absolute deadline.

        Unlike Serial.readline(), this keeps accumulating a fragmented USB CDC
        response across individual pyserial read timeouts.
        """

        buf = bytearray()

        while time.monotonic() < deadline:
            byte = self.ser.read(1)

            if not byte:
                continue

            if byte == b"\n":
                return bytes(buf).rstrip(b"\r")

            buf.extend(byte)

        if buf:
            raise TimeoutError(
                f"Timed out with partial Pico response: "
                f"{bytes(buf)!r}"
            )

        return None

    def _expect_ok(self, address):
        deadline = time.monotonic() + REPLY_TIMEOUT_S

        while time.monotonic() < deadline:
            reply = self._read_line(deadline)

            if reply is None:
                break

            # The banner can race with the first command after opening USB.
            if reply == READY_BANNER:
                continue

            if reply == b"OK":
                return

            if reply.startswith(b"ERR"):
                raise RuntimeError(
                    f"Pico error at 0x{address:08X}: "
                    f"{reply.decode(errors='replace')}"
                )

            raise RuntimeError(
                f"Unexpected Pico response at "
                f"0x{address:08X}: {reply!r}"
            )

        raise TimeoutError(
            f"Timed out waiting for Pico response "
            f"at 0x{address:08X}"
        )

    def write(self, address, data):
        data = bytes(data)

        if address < 0 or address > 0xFFFFFFFF:
            raise ValueError(
                f"Invalid 32-bit address: 0x{address:X}"
            )

        if not data:
            return

        if address & 0x3:
            raise ValueError(
                f"Write address 0x{address:08X} "
                f"is not 4-byte aligned"
            )

        if len(data) % 4 != 0:
            raise ValueError(
                f"Write length {len(data)} "
                f"is not a multiple of 4 bytes"
            )

        if len(data) > PICO_MAX_DATA:
            raise ValueError(
                f"Transfer too large "
                f"({len(data)} bytes, max {PICO_MAX_DATA})"
            )

        if address > 0xFFFFFFFF - (len(data) - 1):
            raise ValueError(
                f"Write range starting at 0x{address:08X} "
                f"overflows 32-bit address space"
            )

        # PC -> Pico protocol is little-endian.
        #
        # The Pico converts:
        #
        # address:
        #   little-endian USB representation
        #       ->
        #   MSB-first SPI wire representation
        #
        # data:
        #   little-endian target-memory bytes
        #       ->
        #   MSB-first SPI word
        #
        packet = (
            b"W"
            + struct.pack("<I", address)
            + struct.pack("<H", len(data))
            + data
        )

        written = self.ser.write(packet)

        self.ser.flush()

        if written != len(packet):
            raise IOError(
                f"Short serial write: "
                f"sent {written}/{len(packet)} bytes"
            )

        # IMPORTANT:
        #
        # "OK" means that the Pico completed transmission of the SPI
        # transaction.
        #
        # It does NOT mean that the ASIC contents were read back and verified.
        self._expect_ok(address)

    def write_block(self, address, data):
        data = bytes(data)

        if address < 0 or address > 0xFFFFFFFF:
            raise ValueError(
                f"Invalid 32-bit address: 0x{address:X}"
            )

        if address & 0x3:
            raise ValueError(
                f"Block address 0x{address:08X} "
                f"is not 4-byte aligned"
            )

        if not data:
            return

        if len(data) % 4 != 0:
            raise ValueError(
                f"Block length {len(data)} "
                f"is not a multiple of 4 bytes"
            )

        if address > 0xFFFFFFFF - (len(data) - 1):
            raise ValueError(
                f"Block range starting at 0x{address:08X} "
                f"overflows 32-bit address space"
            )

        offset = 0

        while offset < len(data):
            chunk = data[
                offset:
                offset + self.chunk_size
            ]

            self.write(
                address + offset,
                chunk
            )

            offset += len(chunk)

    def write_u32(self, address, value):
        if value < 0 or value > 0xFFFFFFFF:
            raise ValueError(
                f"Invalid 32-bit value: 0x{value:X}"
            )

        # Target-memory representation is little-endian.
        #
        # Example:
        #
        # value = 0x00000180
        #
        # bytes sent PC -> Pico:
        #
        #   80 01 00 00
        #
        # Pico then emits on SPI:
        #
        #   00 00 01 80
        #
        self.write(
            address,
            struct.pack("<I", value)
        )

    def run(self, entry):
        print(
            f"Setting boot address to 0x{entry:08X}"
        )

        self.write_u32(
            BOOT_ADDRESS,
            entry
        )

        print("Releasing CPU")

        self.write_u32(
            BOOT_EXIT_LOOP,
            1
        )


def load_elf(
    filename,
    strict_partial_words=False
):
    """
    Build 32-bit-word-aligned SRAM write regions from ELF PT_LOAD segments.

    The ELF is little-endian.

    segment.data() is intentionally kept in raw target-memory byte order.
    SPI-specific byte ordering is handled only by the Pico.

    The SPI slave writes complete 32-bit words.

    If a PT_LOAD starts or ends in the middle of a word, bytes outside the
    ELF-defined image in that boundary word must therefore be supplied too.

    Default:
        zero-pad those bytes and print a warning.

    --strict-partial-words:
        reject the ELF instead.
    """

    image = bytearray(RAM_SIZE)

    # defined[i] == 1 means that the ELF explicitly defines this SRAM byte,
    # either through p_filesz data or through p_memsz BSS zero-initialization.
    defined = bytearray(RAM_SIZE)

    used_words = [
        False
    ] * (RAM_SIZE // 4)

    with open(filename, "rb") as f:
        elf = ELFFile(f)

        if elf.elfclass != 32:
            raise RuntimeError(
                f"Expected a 32-bit ELF, "
                f"got ELF{elf.elfclass}"
            )

        if not elf.little_endian:
            raise RuntimeError(
                "Expected a little-endian HEEPidermis ELF"
            )

        if elf.header["e_machine"] != "EM_RISCV":
            raise RuntimeError(
                f"Expected a RISC-V ELF, "
                f"got {elf.header['e_machine']}"
            )

        entry = int(
            elf.header["e_entry"]
        )

        if entry < 0 or entry > 0xFFFFFFFF:
            raise RuntimeError(
                f"ELF entry point 0x{entry:X} "
                f"is not a 32-bit address"
            )

        for segment in elf.iter_segments():
            if segment["p_type"] != "PT_LOAD":
                continue

            address = int(
                segment["p_paddr"]
            )

            vaddr = int(
                segment["p_vaddr"]
            )

            filesz = int(
                segment["p_filesz"]
            )

            memsz = int(
                segment["p_memsz"]
            )

            if memsz == 0:
                continue

            if address < 0 or address > 0xFFFFFFFF:
                raise RuntimeError(
                    f"Invalid PT_LOAD physical address "
                    f"0x{address:X}"
                )

            if filesz > memsz:
                raise RuntimeError(
                    f"Malformed PT_LOAD at "
                    f"0x{address:08X}: "
                    f"p_filesz ({filesz}) > "
                    f"p_memsz ({memsz})"
                )

            segment_end = (
                address + memsz
            )

            if segment_end > 0x100000000:
                raise RuntimeError(
                    f"PT_LOAD at 0x{address:08X} "
                    f"overflows 32-bit address space"
                )

            if (
                address < RAM_BASE
                or segment_end >
                RAM_BASE + RAM_SIZE
            ):
                raise RuntimeError(
                    f"Segment "
                    f"0x{address:08X}-"
                    f"0x{segment_end - 1:08X} "
                    f"is outside SRAM"
                )

            if address != vaddr:
                print(
                    f"Warning: "
                    f"PT_LOAD p_paddr="
                    f"0x{address:08X} "
                    f"differs from p_vaddr="
                    f"0x{vaddr:08X}; "
                    f"using p_paddr"
                )

            file_data = bytes(
                segment.data()
            )

            if len(file_data) != filesz:
                raise RuntimeError(
                    f"PT_LOAD at "
                    f"0x{address:08X}: "
                    f"expected {filesz} file bytes, "
                    f"got {len(file_data)}"
                )

            # ELF semantics:
            #
            # [p_filesz, p_memsz)
            #
            # is zero-initialized memory, i.e. BSS.
            mem_data = (
                file_data
                + b"\x00" *
                (memsz - filesz)
            )

            start = (
                address - RAM_BASE
            )

            end = (
                start + memsz
            )

            # Detect contradictory overlapping PT_LOAD bytes.
            #
            # Identical overlaps are allowed.
            for i, value in enumerate(mem_data):
                idx = (
                    start + i
                )

                if (
                    defined[idx]
                    and image[idx] != value
                ):
                    raise RuntimeError(
                        f"Conflicting PT_LOAD overlap "
                        f"at SRAM address "
                        f"0x{RAM_BASE + idx:08X}"
                    )

                image[idx] = value
                defined[idx] = 1

            first_word = (
                align_down(start, 4) // 4
            )

            last_word = (
                align_up(end, 4) // 4
            )

            for word in range(
                first_word,
                last_word
            ):
                used_words[word] = True

    if not any(used_words):
        raise RuntimeError(
            "ELF contains no non-empty "
            "PT_LOAD segments in SRAM"
        )

    #
    # Detect partial boundary words.
    #
    # Example:
    #
    # ELF defines:
    #
    #   0x181
    #   0x182
    #   0x183
    #
    # But SPI writes:
    #
    #   0x180-0x183
    #
    # therefore 0x180 must be supplied too.
    #
    padding_indices = []

    for word, used in enumerate(used_words):
        if not used:
            continue

        base = word * 4

        for idx in range(
            base,
            base + 4
        ):
            if not defined[idx]:
                padding_indices.append(
                    idx
                )

    if padding_indices:
        ranges = []

        range_start = (
            padding_indices[0]
        )

        previous = (
            padding_indices[0]
        )

        for idx in padding_indices[1:]:
            if idx != previous + 1:
                ranges.append(
                    (
                        range_start,
                        previous
                    )
                )

                range_start = idx

            previous = idx

        ranges.append(
            (
                range_start,
                previous
            )
        )

        description = ", ".join(
            format_range(
                RAM_BASE + start,
                RAM_BASE + end
            )
            for start, end in ranges
        )

        message = (
            "ELF requires zero-padding byte(s) "
            "outside its defined memory image "
            "because the SPI slave writes full "
            f"32-bit words: {description}"
        )

        if strict_partial_words:
            raise RuntimeError(
                message
            )

        print(
            f"Warning: {message}"
        )

    #
    # Merge consecutive used words into aligned write regions.
    #
    regions = []

    word = 0

    while word < len(used_words):
        if not used_words[word]:
            word += 1
            continue

        first = word

        while (
            word < len(used_words)
            and used_words[word]
        ):
            word += 1

        last = word

        start = first * 4
        end = last * 4

        regions.append(
            (
                RAM_BASE + start,
                bytes(
                    image[start:end]
                )
            )
        )

    return entry, regions


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Program HEEPidermis SRAM "
            "through Raspberry Pi Pico SPI"
        )
    )

    parser.add_argument(
        "elf",
        help="HEEPidermis ELF file",
    )

    parser.add_argument(
        "--port",
        required=True,
        help=(
            "Pico serial port, "
            "e.g. COM6 or /dev/ttyACM0"
        ),
    )

    parser.add_argument(
        "--chunk",
        type=int,
        default=DEFAULT_CHUNK_SIZE,
        help=(
            f"USB/SPI block size in bytes; "
            f"must be a multiple of 4 and <= "
            f"{PICO_MAX_DATA} "
            f"(default: "
            f"{DEFAULT_CHUNK_SIZE})"
        ),
    )

    parser.add_argument(
        "--run",
        action="store_true",
        help="Start the CPU after programming",
    )

    parser.add_argument(
        "--strict-partial-words",
        action="store_true",
        help=(
            "Reject ELF segments that require "
            "writing zero-padded bytes outside "
            "the ELF-defined memory image"
        ),
    )

    args = parser.parse_args()

    entry, regions = load_elf(
        args.elf,
        strict_partial_words=
            args.strict_partial_words,
    )

    if (
        args.run
        and not (
            RAM_BASE
            <= entry
            < RAM_BASE + RAM_SIZE
        )
    ):
        raise RuntimeError(
            f"ELF entry point "
            f"0x{entry:08X} "
            f"is outside SRAM"
        )

    total = sum(
        len(data)
        for _, data in regions
    )

    print(
        f"ELF entry point: "
        f"0x{entry:08X}"
    )

    print(
        f"Bytes to write:  "
        f"{total}"
    )

    for address, data in regions:
        print(
            f"  0x{address:08X} - "
            f"0x{address + len(data) - 1:08X} "
            f"({len(data)} bytes)"
        )

    programmer = HeepProgrammer(
        args.port,
        chunk_size=args.chunk,
    )

    try:
        written = 0

        for address, data in regions:
            print(
                f"Writing "
                f"0x{address:08X} ..."
            )

            programmer.write_block(
                address,
                data
            )

            written += len(data)

            print(
                f"  {written}/{total} bytes"
            )

        print(
            "Programming complete"
        )

        if args.run:
            programmer.run(entry)

            print(
                "CPU started"
            )

    finally:
        programmer.close()


if __name__ == "__main__":
    main()