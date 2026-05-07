import serial
from cobs import cobs
from cffi import FFI
from backend.state import State

ffi = FFI()

ffi.cdef("""
typedef struct {
	// Time
	uint32_t t; // [ms] since boot
	uint32_t launch_t; // [ms] launch detect
	uint32_t elapsed_t; // [ms] since launch detected

	// Launch detect
	uint32_t is_launched;

	// Power
	float batt_v; // [V]
	float batt_i; // [A]

    // Sensors
    float accel_ms2[3];
    float omega_rads[3];
    float mag_mgauss[3];
//    float pres_hpa;
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

    // Servo
    float servo_cmd; // [deg]
    float servo_fdbk; // [deg]
} state_t;
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