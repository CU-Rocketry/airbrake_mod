import re
from cffi import FFI

ffi = FFI()

def parse_firmware_schema():
    header_files = [
        "../../src/Core/Inc/state.h",
        "../../src/Core/Inc/packets.h"
    ]

    firmware_schema = ""

    for filepath in header_files:
        with open(filepath, "r") as f:
            code = f.read()
        
        clean_code = re.sub(r'^\s*#.*$', '', code, flags=re.MULTILINE) # remove preprocessor directives
        firmware_schema += clean_code + "\n"

    ffi.cdef(firmware_schema, pack=1)
    return ffi

ffi = parse_firmware_schema()