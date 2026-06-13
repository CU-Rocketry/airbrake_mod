import math
import queue

import serial
from cobs import cobs
from backend.state import app_state
from backend.firmware_schema import ffi

from imgui_bundle import hello_imgui # type: ignore

def telemetry_worker():
    """Runs in a background thread, constantly decoding serial into app_app_state."""    
    port = app_state.ports[app_state.current_port]
    baudrate = int(app_state.baudrates[app_state.current_baudrate])

    try:
        ser = serial.Serial(port, baudrate)
        app_state.connected = True
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        app_state.connected = False
        return

    raw_buffer = bytearray()
    
    try:
        # Check the threading event so the GUI can stop this loop
        while not app_state.stop_event.is_set():

            while not app_state.tx_queue.empty(): # if packets waiting to send
                try:
                    packet_bytes = app_state.tx_queue.get_nowait() # get packet bytes non-blocking
                    encoded = cobs.encode(packet_bytes) # cobs encode the packet
                    ser.write(encoded + b'\x00') # add null terminator and send
                except queue.Empty:
                    break
            
            # Non-blocking check so the loop can exit if stop_event is set
            if ser.in_waiting > 0:
                # Read everything available to prevent buffer overflow at 2Mbps
                raw_buffer += ser.read(ser.in_waiting) # append everything available to buffer                
                while b'\x00' in raw_buffer: # when there's at least one complete frame
                    frame, _, raw_buffer = raw_buffer.partition(b'\x00') # get the frame, keep the rest in buffer
                    
                    if len(frame) > 0:
                        try:
                            decoded = cobs.decode(frame)
                            
                            if len(decoded) > 0:
                                pkt_type = decoded[0] # read packet type byte
                                
                                # telemetry packet is type 0x01
                                # it should also match the length of the struct defined for it
                                if pkt_type == 0x01 and len(decoded) == ffi.sizeof("telemetry_packet_t"):
                                    parsed_state = ffi.from_buffer("telemetry_packet_t *", decoded)
                                    t_sec = parsed_state.t / 1000.0 
                                    app_state.latest_time = t_sec

                                    app_state.current_flags = parsed_state.flags
                                    
                                    for field_name, buf in app_state.buffers.items(): # for each buffer from app state
                                        if hasattr(parsed_state, field_name): # if the field was in the telemetry packet
                                            value = getattr(parsed_state, field_name) # get that field's value
                                            if buf.width > 1: # convert arrays to lists
                                                value = list(value)
                                            buf.add_point(t_sec, value) # add the point to the buffer
                                            
                                elif pkt_type == 0x02: # log packet
                                    # log level handling
                                    log_level = decoded[1] # 0-3 for debug,info,warning,error                                    
                                    level_map = {
                                        0: hello_imgui.LogLevel.debug,
                                        1: hello_imgui.LogLevel.info,
                                        2: hello_imgui.LogLevel.warning,
                                        3: hello_imgui.LogLevel.error
                                    }
                                    imgui_level = level_map.get(log_level, hello_imgui.LogLevel.error) # convert from number, defaulting to error if invalid level

                                    msg = decoded[2:].decode('utf-8', errors='ignore').rstrip('\x00').strip('\r\n') # decode remaining bytes into string, removing null terminator and newlines
                                    hello_imgui.log(imgui_level, msg)

                                else:
                                    print(f"Unknown packet type of {pkt_type} or invalid length of {len(decoded)}")
                                    
                        except cobs.DecodeError as e:
                            print(f"COBS decode error: {e}")
    finally:
        ser.close()
        app_state.connected = False

def send_command(state_flags_set_mask=0, state_flags_clear_mask=0):
    if not app_state.connected:
        return
        
    cmd = ffi.new("command_packet_t *")
    cmd.pkt_type = 0x03

    # Set/clear state flags based on masks
    cmd.state_flags_set = state_flags_set_mask
    cmd.state_flags_clear = state_flags_clear_mask

    cmd.mode = int(app_state.selected_mode_idx)    
    cmd.servo_cmd = float(math.degrees(app_state.servo_cmd_rad))
    
    raw_bytes = bytes(ffi.buffer(cmd))
    app_state.tx_queue.put(raw_bytes)

def send_hil_data(pres, accel, omega, mag):
    """Forwards hardware in the loop (HIL) sensor data from Simulink to MCU

    Args:
        state (State): Global state struct, used to queue tx data
        pres (float): Pressure data [Pa]
        accel (list): Accelerometer axes data [m/s^2]
        omega (list): Gyroscope axes data [rad/s]
        mag (list): Magnetometer axes data [mGauss]
    """
    if not app_state.connected:
        return
        
    hil = ffi.new("hil_packet_t *")
    hil.pkt_type = 0x10
    hil.pres_pa = pres
    for i in range(3):
        hil.accel_ms2[i] = accel[i]
        hil.omega_rads[i] = omega[i]
        hil.mag_mgauss[i] = mag[i]
        
    raw_bytes = bytes(ffi.buffer(hil))
    app_state.tx_queue.put(raw_bytes)