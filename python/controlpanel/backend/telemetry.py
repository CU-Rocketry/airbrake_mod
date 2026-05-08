import queue

import serial
from cobs import cobs
from cffi import FFI
from backend.state import State

ffi = FFI()

# ensure to paste the __attribute__((packed)) before the name of the struct after the definition

ffi.cdef("""
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
} __attribute__((packed)) telemetry_packet_t;

typedef struct {
    uint8_t pkt_type; // always 0x02 for log
    char message[127]; // there's extra space even in just the first 254 bytes then
} __attribute__((packed)) log_packet_t;
         
// Command
typedef struct {
    uint8_t pkt_type; // always 0x03 for cmd

    uint8_t mode_en; // requests mode change (substitute for rotating selector)
    uint8_t mode; // mode to change to

    uint8_t launch_detect_en; // set to 1 to trigger launch detect

    uint8_t servo_cmd_en;
    float servo_cmd; // [deg]
} __attribute__((packed)) command_packet_t;

// HIL
typedef struct {
	uint8_t pkt_type; // always 0x10 for HIL data

	float pres_pa;
	float accel_ms2[3];
	float omega_rads[3];
	float mag_mgauss[3];
} __attribute__((packed)) hil_packet_t;
""")

def telemetry_worker(state: State):
    """Runs in a background thread, constantly decoding serial into app_state."""
    struct_size = ffi.sizeof("state_t")
    
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
    print("Listening for telemetry...")
    
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
                byte = ser.read(1)
                
                if byte == b'\x00':
                    if len(raw_buffer) > 0:
                        try:
                            decoded = cobs.decode(raw_buffer)
                            
                            if len(decoded) > 0:
                                pkt_type = decoded[0] # read first byte (packet type)
                                
                                # telemetry packet must start with type 0x01 and match struct
                                if pkt_type == 0x01 and len(decoded) == ffi.sizeof("telemetry_packet_t"):
                                    parsed_state = ffi.from_buffer("telemetry_packet_t *", decoded)
                                    
                                    t_sec = parsed_state.t / 1000.0 
                                    state.latest_time = t_sec
                                    
                                    for field_name, buf in state.buffers.items():
                                        if hasattr(parsed_state, field_name):
                                            value = getattr(parsed_state, field_name)
                                            if buf.width > 1:
                                                value = list(value)
                                            buf.add_point(t_sec, value)
                                            
                                # log must start with 0x02 and can be variable length, null terminated
                                elif pkt_type == 0x02:
                                    parsed_log = ffi.from_buffer("log_packet_t *", decoded)
                                    msg = ffi.string(parsed_log.message).decode('utf-8', errors='ignore') # convert char array to Python string
                                    print(f"LOG: {msg}") # TODO route to GUI
                                        
                        except cobs.DecodeError:
                            pass # Silently drop corrupted frames
                            
                    raw_buffer.clear()
                else:
                    raw_buffer.extend(byte)
    finally:
        ser.close()
        state.connected = False
        print("Serial connection closed.")

def send_command(state: State, mode=None, manual_launch_detect=False, servo_cmd=None):
    """Sends command triggered by GUI back to MCU, helpful during testing.

    Args:
        state (State): Global state struct, used to queue tx data
        mode (TODO, optional): Manual mode selection override. Defaults to None.
        manual_launch_detect (bool, optional): Manual launch detect trigger. Defaults to False.
        servo_cmd (TODO, optional): Manual servo angle [deg]. Defaults to None.
    """
    if not state.connected:
        return

    # blank command struct, just with packet type set to command        
    cmd = ffi.new("command_packet_t *")
    cmd.pkt_type = 0x03
    
    if mode is not None: # if a mode is specified, enable mode change and set the mode value
        cmd.mode_en = 1
        cmd.mode = int(mode)
    else:
        cmd.mode_en = 0
        cmd.mode = 0
        
    cmd.launch_detect_en = 1 if manual_launch_detect else 0 # if launch detect button pressed
    
    if servo_cmd is not None: # if servo command specified, enable and set value
        cmd.servo_cmd_en = 1
        cmd.servo_cmd = float(servo_cmd)
    else:
        cmd.servo_cmd_en = 0
        cmd.servo_cmd = 0.0
    
    # Convert struct to bytes and queue to be sent
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
        
    hil = ffi.new("hil_rx_packet_t *")
    hil.pkt_type = 0x10
    hil.pres_pa = pres
    for i in range(3):
        hil.accel_ms2[i] = accel[i]
        hil.omega_rads[i] = omega[i]
        hil.mag_mgauss[i] = mag[i]
        
    raw_bytes = bytes(ffi.buffer(hil))
    state.tx_queue.put(raw_bytes)