"""Read uncompressed little-endian PolyFEM VTU arrays for analysis tools."""

import base64
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


DTYPES = {
    "Float64": "<f8", "Float32": "<f4", "Int64": "<i8", "UInt64": "<u8",
    "Int32": "<i4", "UInt32": "<u4", "Int8": "i1", "UInt8": "u1",
}


def read_arrays(path, names=None):
    """Yield DataArray attributes and the corresponding uncompressed bytes."""
    data = Path(path).read_bytes()
    marker = re.search(rb'<AppendedData encoding="raw">\s*_', data)
    appended = None
    if marker:
        end_tag = b"</AppendedData>"
        end = data.rfind(end_tag)
        if end < marker.end():
            raise ValueError(f"Unterminated appended data in {path}")
        appended = memoryview(data)[marker.end():end]
        data = data[:marker.start()] + data[end + len(end_tag):]
    root = ET.fromstring(data)
    if (root.get("compressor") or root.get("header_type") != "UInt64"
            or root.get("byte_order", "LittleEndian") != "LittleEndian"):
        raise ValueError(f"Unsupported VTU encoding in {path}")

    for array in root.iter("DataArray"):
        if names is not None and array.get("Name") not in names:
            continue
        encoding = array.get("format")
        if encoding == "binary":
            block = memoryview(base64.b64decode(array.text or ""))
        elif encoding == "appended" and appended is not None:
            offset = int(array.attrib["offset"])
            if not 0 <= offset <= len(appended) - 8:
                raise ValueError(f"Invalid appended array offset in {path}")
            block = appended[offset:]
        else:
            raise ValueError(f"Unsupported VTU array encoding in {path}")
        if len(block) < 8:
            raise ValueError(f"Incomplete array header in {path}")
        length, = struct.unpack_from("<Q", block)
        if length > len(block) - 8 or (encoding == "binary" and length != len(block) - 8):
            raise ValueError(f"Invalid VTU array length in {path}")
        yield array.attrib, block[8:8 + length]
