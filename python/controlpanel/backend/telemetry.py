import math
import queue

import serial
from cobs import cobs
from cffi import FFI
from backend.state import State

from imgui_bundle import hello_imgui # type: ignore

ffi = FFI()

# ensure to paste the __attribute__((packed)) before the name of the struct after the definition

ffi.cdef("""
// Telemetry
typedef struct {
	uint8_t pkt_type; // always 0x01 for telemetry

	// Time
	uint32_t t; // [ms] since boot

	// Launch detect
	uint32_t is_launched;

	// Power
	float batt_v; // [V]
	float batt_i; // [A]

    // Sensors
    float pres_pa;

    // Body frame sensors
    float accel_b[3]; // [m/s/s] in body frame. *proper acceleration
    float omega_b[3]; // [rad/s] in body frame
    float mag_b[3]; // [mgauss] in body frame

    // State estimation
    float quat[4]; // body to inertial rotation already I think
    float accel_e[3]; // [m/s/s] in inertial frame

    float p_ground; // [Pa]
    float alt_agl; // [m] AGL with + up
    float vel_z; // [m] with + up

    // Control
    // TODO
    float output; // 0 to 1 mapping to air brakes deployment range

    // Servo
    float servo_cmd; // [deg]
    float servo_fdbk; // [deg]
} telemetry_packet_t;

typedef struct {
    uint8_t pkt_type; // always 0x02 for log
    uint8_t lvl;
    char message[127]; // there's extra space even in just the first 254 bytes then
} log_packet_t;

// Flash
// for 2 packets per 256 byte page we have max 128 bytes = 32 floats
typedef struct {
	uint32_t t; // [ms] since boot 4 bytes
	float batt_v; // [V] 8
	float batt_i; // [A] 12
	float accel_b[3]; // [m/s/s] in body frame. *proper acceleration 24
	float omega_b[3]; // [rad/s] in body frame 36
	float mag_b[3]; // [mgauss] in body frame 48
    float quat[4]; // body to inertial rotation already I think 64
    float accel_e[3]; // [m/s/s] in inertial frame 76
    float p_ground; // [Pa] 80
    float alt_agl; // [m] AGL with + up 84
    float vel_z; // [m] with + up 88
    float output; // 0 to 1 mapping to air brakes deployment range 92
    float servo_cmd; // [deg] 96
    float servo_fdbk; // [deg] 100
    // 28 bytes = 7 floats remaining
} flash_packet_t;

// Command
typedef struct {
    uint8_t pkt_type; // always 0x03 for cmd

    uint8_t mode_en; // requests mode change (substitute for rotating selector)
    uint8_t mode; // mode to change to

    uint8_t launch_detect_en; // set to 1 to trigger launch detect

    uint8_t servo_cmd_en;
    float servo_cmd; // [deg]

    uint8_t use_hil_data; // set to 1 to enable HIL mode else 0 sources data from real sensors
} command_packet_t;

// HIL
typedef struct {
	uint8_t pkt_type; // always 0x10 for HIL data

	float pres_pa;
	float accel_ms2[3];
	float omega_rads[3];
	float mag_mgauss[3];
} hil_packet_t;
""", pack=1)

def telemetry_worker(state: State):
    """Runs in a background thread, constantly decoding serial into app_state."""    
    port = state.ports[state.current_port]
    baudrate = int(state.baudrates[state.current_baudrate])

    try:
        ser = serial.Serial(port, baudrate)
        state.connected = True
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        state.connected = False
        return

    raw_buffer = bytearray()
    
    try:
        # Check the threading event so the GUI can stop this loop
        while not state.stop_event.is_set():

            while not state.tx_queue.empty(): # if packets waiting to send
                try:
                    packet_bytes = state.tx_queue.get_nowait() # get packet bytes non-blocking
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
                                    state.latest_time = t_sec
                                    
                                    for field_name, buf in state.buffers.items(): # for each buffer from app state
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
                                    
                        except cobs.DecodeError as e:
                            print(f"COBS decode error: {e}")
    finally:
        ser.close()
        state.connected = False

def send_command(state: State, force_launch=False):
    if not state.connected:
        return
        
    cmd = ffi.new("command_packet_t *")
    cmd.pkt_type = 0x03
    
    # Persistent overrides pulled directly from GUI state
    cmd.mode_en = 1 if state.mode_override_active else 0
    cmd.mode = int(state.selected_mode_idx)
    
    cmd.servo_cmd_en = 1 if state.servo_override_active else 0
    cmd.servo_cmd = float(math.degrees(state.manual_servo_rad))
    
    cmd.use_hil_data = 1 if state.use_hil_data else 0
    
    cmd.launch_detect_en = 1 if state.force_launch_detect else 0
    
    raw_bytes = bytes(ffi.buffer(cmd))
    state.tx_queue.put(raw_bytes)

def send_hil_data(state: State, pres, accel, omega, mag):
    """Forwards hardware in the loop (HIL) sensor data from Simulink to MCU

    Args:
        state (State): Global state struct, used to queue tx data
        pres (float): Pressure data [Pa]
        accel (list): Accelerometer axes data [m/s^2]
        omega (list): Gyroscope axes data [rad/s]
        mag (list): Magnetometer axes data [mGauss]
    """
    if not state.connected:
        return
        
    hil = ffi.new("hil_packet_t *")
    hil.pkt_type = 0x10
    hil.pres_pa = pres
    for i in range(3):
        hil.accel_ms2[i] = accel[i]
        hil.omega_rads[i] = omega[i]
        hil.mag_mgauss[i] = mag[i]
        
    raw_bytes = bytes(ffi.buffer(hil))
    state.tx_queue.put(raw_bytes)