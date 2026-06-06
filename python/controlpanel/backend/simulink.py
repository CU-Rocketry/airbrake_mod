import socket
import struct
from backend.telemetry import State, send_hil_data

def simulink_worker(state: State, python_port=9090, simulink_port=9091):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', python_port))
    
    while True: # this will just run until the app is closed
        try:
            data, _ = sock.recvfrom(1024) # blocks until data is received. the packet size is prob way too big
            
            # if connected to the MCU
            # and MCU HIL data source selected
            # and packet is 40 bytes for 10 floats
            if len(data) == 40 and state.connected and state.use_hil_data:
                unpacked = struct.unpack('<10f', data) # little-endian, 10 floats
                
                pres = unpacked[0]
                accel = list(unpacked[1:4])
                omega = list(unpacked[4:7])
                mag = list(unpacked[7:10])
                
                send_hil_data(state, pres, accel, omega, mag) # send to telemetry thread to forward to MCU
                # TODO maybe also put this in the GUI?

                output = state.buffers['output'].get_latest()
                output = float(output[0]) # cast single element numpy array to float                
                sock.sendto(struct.pack('<f', output), ('127.0.0.1', simulink_port)) # send latest output command back to Simulink for its control input
                
        except Exception as e:
            print(f"Simulink worker error: {e}")