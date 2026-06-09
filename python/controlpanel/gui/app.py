from imgui_bundle import imgui, hello_imgui # type: ignore

from backend.state import app_state

import threading
from backend.telemetry import telemetry_worker
from backend.simulink import simulink_worker

from components.connection import draw_serial_connection_window
from components.windows import draw_command_window
from components.logs import draw_logs_window
from components.power import draw_power
from components.servo import draw_servo_window
from components.sensors_body import draw_sensors_body
from components.sensors_raw import draw_sensors_raw
from components.state_estimation import draw_state_estimation
from components.control import draw_control

app_state.simulink_thread = threading.Thread(
    target=simulink_worker,
    args=(app_state,),
    daemon=True # daemon so it will automatically close when the main app closes
)
app_state.simulink_thread.start()

def connect_cb(port, baud):
    app_state.stop_event.clear()
    app_state.telemetry_thread = threading.Thread(
        target=telemetry_worker,
        args=(app_state,),
        daemon=True
    )
    app_state.telemetry_thread.start()

def disconnect_cb():
    app_state.stop_event.set()
    if app_state.telemetry_thread:
        app_state.telemetry_thread.join(timeout=1.0)

def get_layout_params():
    """Builds the HelloImGui runner parameters and default docking layout."""
    params = hello_imgui.RunnerParams()
    params.app_window_params.window_title = "Air Brakes Control Panel"
    params.app_window_params.window_geometry.size = (1200, 800)
    params.fps_idling.fps_idle = 30
    
    # Enable automatic full-screen docking
    params.imgui_window_params.default_imgui_window_type = hello_imgui.DefaultImGuiWindowType.provide_full_screen_dock_space

    docking = hello_imgui.DockingParams()
    docking.layout_condition = hello_imgui.DockingLayoutCondition.application_start

    # LeftSpace for connection, command, and logs, maindockspace for plots
    split_left = hello_imgui.DockingSplit()
    split_left.initial_dock = "MainDockSpace"
    split_left.new_dock = "LeftSpace"
    split_left.direction = imgui.Dir_.left
    split_left.ratio = 0.25

    # Left bottom split for logs
    split_left_bottom = hello_imgui.DockingSplit()
    split_left_bottom.initial_dock = "LeftSpace"
    split_left_bottom.new_dock = "LeftBottomSpace"
    split_left_bottom.direction = imgui.Dir_.down
    split_left_bottom.ratio = 0.5

    docking.docking_splits = [split_left, split_left_bottom]

    windows = []

    def route(name, space, draw_func):
        w = hello_imgui.DockableWindow()
        w.label = name
        w.dock_space_name = space
        w.call_begin_end = False # We call imgui.begin() ourselves
        w.gui_function = draw_func
        
        windows.append(w)

    route("Serial Connection", "LeftSpace", lambda: draw_serial_connection_window(app_state, connect_cb, disconnect_cb))
    route("Commands", "LeftSpace", lambda: draw_command_window(app_state))
    route("Logging", "LeftBottomSpace", draw_logs_window)
   
    route("Power", "MainDockSpace", draw_power)
    route("Servo", "MainDockSpace", draw_servo_window)
    route("Sensors raw", "MainDockSpace", draw_sensors_raw)
    route("Sensors body", "MainDockSpace", draw_sensors_body)
    route("State Estimation", "MainDockSpace", draw_state_estimation)
    route("Control", "MainDockSpace", draw_control)

    docking.dockable_windows = windows
    params.docking_params = docking
    return params