"""Package the application ELF as a SHA-256 checked MCUboot BIN and HEX."""

import argparse
from pathlib import Path
import re
import subprocess
import sys
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "third_party/mcuboot/scripts"))

from click import ClickException
from imgtool import image, version
from intelhex import IntelHex

PRIMARY_ADDR = 0x0800C000
HEADER_SIZE = 0x200
SECONDARY_SIZE = 0x33800
MAX_IMAGE_SIZE = 0x32800
ALIGN = 8


def pack_image(elf_path, image_version):
    sections = subprocess.run(
        ["arm-none-eabi-objdump", "-h", str(elf_path)],
        check=True, capture_output=True, text=True,
    ).stdout
    vector = f"{PRIMARY_ADDR + HEADER_SIZE:08x}"
    if not re.search(rf"^\s*\d+\s+\.isr_vector\s+\S+\s+{vector}\s+{vector}\b",
                     sections, re.MULTILINE | re.IGNORECASE):
        raise RuntimeError(f"Expected application vectors at 0x{vector}.")

    bin_path = elf_path.with_name(f"{elf_path.stem}-mcuboot.bin")
    hex_path = elf_path.with_name(f"{elf_path.stem}-mcuboot.hex")
    with tempfile.TemporaryDirectory(prefix="mcuboot-image-", dir=elf_path.parent) as folder:
        temp = Path(folder)
        raw = temp / "application.bin"
        packed = temp / "image.bin"
        hex_file = temp / "image.hex"

        subprocess.run(
            ["arm-none-eabi-objcopy", "-O", "binary", str(elf_path), str(raw)],
            check=True,
        )
        firmware = image.Image(
            version=version.decode_version(image_version),
            header_size=HEADER_SIZE, pad_header=True,
            slot_size=SECONDARY_SIZE, max_sectors=104, align=ALIGN,
        )
        firmware.load(str(raw))
        firmware.create(None, "hash", None)
        firmware.save(str(packed))

        result, _, _, _ = image.Image.verify(str(packed), None)
        if result != image.VerifyResult.OK:
            raise RuntimeError(f"Image SHA-256 verification failed: {result.name}.")
        data = packed.read_bytes()
        if len(data) > MAX_IMAGE_SIZE:
            raise RuntimeError("Image exceeds the slot's usable space.")

        # The UART BIN ends at the TLV. Align only the copy for ST-Link.
        padded = data + b"\xff" * (-len(data) % ALIGN)
        image_hex = IntelHex()
        image_hex.frombytes(padded, offset=PRIMARY_ADDR)
        image_hex.write_hex_file(str(hex_file))

        packed.replace(bin_path)
        try:
            hex_file.replace(hex_path)
        except OSError:
            bin_path.unlink(missing_ok=True)
            raise

    print(f"MCUboot {image_version}: SHA-256 verified; {hex_path.name} and {bin_path.name}.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("version")
    args = parser.parse_args()
    try:
        pack_image(args.elf.resolve(), args.version)
    except (OSError, RuntimeError, ValueError, ClickException, subprocess.CalledProcessError) as error:
        print(f"MCUboot: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
