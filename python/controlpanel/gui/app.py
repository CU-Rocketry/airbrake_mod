import threading
from imgui_bundle import imgui, hello_imgui # type: ignore
from backend.state import State
from backend.telemetry import telemetry_worker
from gui.components.connection import draw_serial_connection_window
from gui.components.plots import LinePlot, QuatPlot
from backend.telemetry import telemetry_worker
from backend.simulink import simulink_worker
from gui.components.windows import draw_command_window, draw_servo_window, draw_logs_window

app_state = State()

app_state.simulink_thread = threading.Thread(
    target=simulink_worker,
    args=(app_state,),
    daemon=True # daemon so it will automatically close when the main app closes
)
app_state.simulink_thread.start()

plot_voltage = LinePlot("Battery Voltage", "Voltage [V]")
plot_voltage.add_line("Voltage", app_state.buffers['batt_v'])
plot_current = LinePlot("Battery Current", "Current [A]")
plot_current.add_line("Current", app_state.buffers['batt_i'])

plot_servo = LinePlot("Servo", "Angle [deg]")
plot_servo.add_line("Command", app_state.buffers['servo_cmd'])
plot_servo.add_line("Feedback", app_state.buffers['servo_fdbk'])

plot_accel_raw = LinePlot("Raw Acceleration", "Accel [m/s^2]")
plot_accel_raw.add_line("X", app_state.buffers['accel_ms2'], col=0)
plot_accel_raw.add_line("Y", app_state.buffers['accel_ms2'], col=1)
plot_accel_raw.add_line("Z", app_state.buffers['accel_ms2'], col=2)

plot_omega_raw = LinePlot("Raw Angular Rate", "Omega [rad/s]")
plot_omega_raw.add_line("X", app_state.buffers['omega_rads'], col=0)
plot_omega_raw.add_line("Y", app_state.buffers['omega_rads'], col=1)
plot_omega_raw.add_line("Z", app_state.buffers['omega_rads'], col=2)

plot_pres = LinePlot("Barometer", "Pressure [Pa]")
plot_pres.add_line("Pressure", app_state.buffers['pres_pa'])

plot_accel_b = LinePlot("Body Acceleration", "Accel [m/s^2]")
plot_accel_b.add_line("X", app_state.buffers['accel_b'], col=0)
plot_accel_b.add_line("Y", app_state.buffers['accel_b'], col=1)
plot_accel_b.add_line("Z", app_state.buffers['accel_b'], col=2)

plot_omega_b = LinePlot("Body Angular Rate", "Omega [rad/s]")
plot_omega_b.add_line("X", app_state.buffers['omega_b'], col=0)
plot_omega_b.add_line("Y", app_state.buffers['omega_b'], col=1)
plot_omega_b.add_line("Z", app_state.buffers['omega_b'], col=2)

plot_quat_components = LinePlot("Orientation", "Quaternion")
plot_quat_components.add_line("W", app_state.buffers['quat'], col=0)
plot_quat_components.add_line("X", app_state.buffers['quat'], col=1)
plot_quat_components.add_line("Y", app_state.buffers['quat'], col=2)
plot_quat_components.add_line("Z", app_state.buffers['quat'], col=3)

plot_quat_orientation = QuatPlot("Vehicle Attitude")
plot_quat_orientation.set_buffer(app_state.buffers['quat'])

plot_accel_e = LinePlot("Inertial Acceleration", "Accel [m/s^2]")
plot_accel_e.add_line("X", app_state.buffers['accel_e'], col=0)
plot_accel_e.add_line("Y", app_state.buffers['accel_e'], col=1)
plot_accel_e.add_line("Z", app_state.buffers['accel_e'], col=2)

plot_p_ground = LinePlot("Ground Pressure", "Pressure [Pa]")
plot_p_ground.add_line("Ground Pressure", app_state.buffers['p_ground'])

plot_alt_agl = LinePlot("Altitude AGL", "Altitude [m]")
plot_alt_agl.add_line("Altitude", app_state.buffers['alt_agl'])

plot_vel_z = LinePlot("Vertical Velocity", "Velocity [m/s]")
plot_vel_z.add_line("Vertical Velocity", app_state.buffers['vel_z'])

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

    # 1. Route our complex modular windows using lambdas
    route("Serial Connection", "LeftSpace", lambda: draw_serial_connection_window(app_state, connect_cb, disconnect_cb))
    route("Commands", "LeftSpace", lambda: draw_command_window(app_state))
    # route("Logging", "LeftBottomSpace", lambda: draw_logs_window(app_state))
    route("Logging", "LeftBottomSpace", draw_logs_window)
    route("Servo", "MainDockSpace", lambda: draw_servo_window(app_state, plot_servo))

    # 2. Wrap our simple plots into mini-functions and route them
    def draw_power():
        imgui.begin("Power")
        plot_voltage.render(app_state.latest_time)
        plot_current.render(app_state.latest_time)
        imgui.end()
    route("Power", "MainDockSpace", draw_power)

    def draw_sensors_raw():
        imgui.begin("Sensors raw")
        plot_accel_raw.render(app_state.latest_time)
        plot_omega_raw.render(app_state.latest_time)
        plot_pres.render(app_state.latest_time)
        imgui.end()
    route("Sensors raw", "MainDockSpace", draw_sensors_raw)

    def draw_sensors_body():
        imgui.begin("Sensors body")
        plot_accel_b.render(app_state.latest_time)
        plot_omega_b.render(app_state.latest_time)
        imgui.end()
    route("Sensors body", "MainDockSpace", draw_sensors_body)

    def draw_orientation():
        imgui.begin("Orientation")
        plot_quat_components.render(app_state.latest_time)
        plot_quat_orientation.render()
        imgui.end()
    route("Orientation", "MainDockSpace", draw_orientation)

    def draw_estimation():
        imgui.begin("Estimation inertial")
        plot_accel_e.render(app_state.latest_time)
        plot_p_ground.render(app_state.latest_time)
        plot_alt_agl.render(app_state.latest_time)
        plot_vel_z.render(app_state.latest_time)
        imgui.end()
    route("Estimation inertial", "MainDockSpace", draw_estimation)

    docking.dockable_windows = windows
    params.docking_params = docking
    return params