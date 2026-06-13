import socket
import struct
from backend.telemetry import send_hil_data
from backend.state import app_state
from gui.components.flags import get_flag

def simulink_worker(python_port=9090, simulink_port=9091):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', python_port))
    
    while True: # this will just run until the app is closed
        try:
            data, _ = sock.recvfrom(1024) # blocks until data is received. the packet size is prob way too big

            print(f"Received {len(data)} bytes from Simulink")
            
            # if connected to the MCU
            # and MCU HIL data source selected
            # and packet is 40 bytes for 10 floats
            if len(data) == 40 and app_state.connected and get_flag(app_state.current_flags, "FLAG_USE_HIL_DATA"):
                unpacked = struct.unpack('<10f', data) # little-endian, 10 floats
                
                pres = unpacked[0]
                accel = list(unpacked[1:4])
                omega = list(unpacked[4:7])
                mag = list(unpacked[7:10])
                
                send_hil_data(pres, accel, omega, mag) # send to telemetry thread to forward to MCU
                # TODO maybe also put this in the GUI?

                output = app_state.buffers['output'].get_latest()
                output = float(output[0]) # cast single element numpy array to float                
                sock.sendto(struct.pack('<f', output), ('127.0.0.1', simulink_port)) # send latest output command back to Simulink for its control input
                # print(f"Sent output command {output} back to Simulink")
                
        except Exception as e:
            print(f"Simulink worker error: {e}")