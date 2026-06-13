import threading
from gui.scrollingbuffer import ScrollingBuffer
import serial.tools.list_ports
import queue
from collections import deque

class State:
    def __init__(self):
        self.ports = [port.device for port in serial.tools.list_ports.comports()]
        self.current_port = 0
        
        self.baudrates = ["9600", "115200", "2000000"]
        self.current_baudrate = 1
        
        self.connected = False

        # Threading control
        self.telemetry_thread : threading.Thread = None
        self.stop_event = threading.Event()
        self.latest_time = 0.0 # [s]

        # Simulink worker
        self.simulink_thread : threading.Thread = None
        
        # Dictionary of buffers. 
        # Keys MUST match the C struct field names exactly.
        self.buffer_size = 5000
        self.buffers = {
            'batt_v': ScrollingBuffer(max_size=self.buffer_size),
            'batt_i': ScrollingBuffer(max_size=self.buffer_size),

            'accel_ms2': ScrollingBuffer(max_size=self.buffer_size, width=3),
            'omega_rads': ScrollingBuffer(max_size=self.buffer_size, width=3),
            'mag_mgauss': ScrollingBuffer(max_size=self.buffer_size, width=3),
            'pres_pa': ScrollingBuffer(max_size=self.buffer_size),

            'accel_b': ScrollingBuffer(max_size=self.buffer_size, width=3),
            'omega_b': ScrollingBuffer(max_size=self.buffer_size, width=3),
            'mag_b': ScrollingBuffer(max_size=self.buffer_size, width=3),

            'quat': ScrollingBuffer(max_size=self.buffer_size, width=4), # w,x,y,z
            'accel_e': ScrollingBuffer(max_size=self.buffer_size, width=3),

            'p_ground': ScrollingBuffer(max_size=self.buffer_size),
            'alt_agl': ScrollingBuffer(max_size=self.buffer_size),
            'vel_z': ScrollingBuffer(max_size=self.buffer_size),

            'predicted': ScrollingBuffer(max_size=self.buffer_size),
            'output': ScrollingBuffer(max_size=self.buffer_size),
            'p_contrib': ScrollingBuffer(max_size=self.buffer_size),
            'i_contrib': ScrollingBuffer(max_size=self.buffer_size),

            'servo_cmd': ScrollingBuffer(max_size=self.buffer_size),
            'servo_fdbk': ScrollingBuffer(max_size=self.buffer_size),
        }

        self.current_flags = 0 # flags is a uint32_t

        self.logs = deque(maxlen=100) # COBS logging packets added here for display in GUI
        self.tx_queue = queue.Queue() # Packet queue for encoding and transmission

        # GUI controls
        self.servo_cmd_rad = 0.0         
        self.mode_names = ["0 IDLE", "1 TEST_UI", "2 TEST_SIMULINK", "3 TEST_SERVO", 
                           "4 TEST_SENSORS", "5 TEST_FLASH", "6 TEST_CONTROL", "7 LAUNCH_DETECT"]
        self.selected_mode_idx = 0

    def get_ports(self):
        ports = serial.tools.list_ports.comports() # get COM ports from OS
        self.ports = [port.device for port in ports] # get port names
            
        if self.current_port >= len(self.ports): # if the current port index is now invalid
            self.current_port = 0

app_state = State()