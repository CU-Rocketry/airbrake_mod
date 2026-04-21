import serial
from cobs import cobs
from cffi import FFI

def init_cffi(header_path):
    ffi = FFI()
    
    csource = ""
    with open(header_path, 'r') as f:
        for line in f:
            stripped = line.strip()
            if not stripped.startswith("#") and not stripped.startswith("extern"):
                csource += line

    ffi.cdef(csource)
    return ffi

def read_telemetry(port, baudrate, header_path):
    ffi = init_cffi(header_path)
    
    struct_size = ffi.sizeof("state_t")

    ser = serial.Serial(port, baudrate)
    rx_buf = bytearray()
    
    try:
        while True:
            byte = ser.read() # read one byte at a time
            
            if byte == b'\x00': # COBS packet delimiter at end
                if len(rx_buf) > 0:
                    try:
                        decoded = cobs.decode(rx_buf)
                        
                        if len(decoded) == struct_size:
                            state = ffi.from_buffer("state_t *", decoded)
                            
                            print(f"t:{state.t} batt_v:{state.batt_v}")
                            
                        else:
                            print(f"Size mismatch: got {len(decoded)}, expected {struct_size}")
                            
                    except cobs.DecodeError:
                        print("COBS decode error - corrupted packet")
                        
                rx_buf.clear()
            else:
                rx_buf.extend(byte)
                
    except KeyboardInterrupt:
        print("Closing serial port.")
        ser.close()

if __name__ == "__main__":
    read_telemetry('COM18', 115200, '')