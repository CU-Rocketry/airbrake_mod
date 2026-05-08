import threading
from gui.scrollingbuffer import ScrollingBuffer
import serial.tools.list_ports
import queue
from collections import deque

class State:
    def __init__(self):
        self.ports = [port.device for port in serial.tools.list_ports.comports()]
        self.current_port = 0
        
        self.baudrates = ["9600", "115200"]
        self.current_baudrate = 1
        
        self.connected = False

        # Threading control
        self.telemetry_thread = None
        self.stop_event = threading.Event()
        self.latest_time = 0.0 # [s]
        
        # Dictionary of buffers. 
        # Keys MUST match the C struct field names exactly.
        self.buffer_size = 1000
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

            'servo_cmd': ScrollingBuffer(max_size=self.buffer_size),
            'servo_fdbk': ScrollingBuffer(max_size=self.buffer_size),
        }

        self.logs = deque(maxlen=100) # COBS logging packets added here for display in GUI
        self.tx_queue = queue.Queue() # Packet queue for encoding and transmission

    def get_ports(self):
        ports = serial.tools.list_ports.comports() # get COM ports from OS
        self.ports = [port.device for port in ports] # get port names
            
        if self.current_port >= len(self.ports): # if the current port index is now invalid
            self.current_port = 0