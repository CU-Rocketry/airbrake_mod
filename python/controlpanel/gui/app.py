import threading
from imgui_bundle import imgui
from backend.state import State
from backend.telemetry import telemetry_worker
from gui.components.connection import draw_serial_connection_window
from gui.components.plots import LinePlot, QuatPlot

# 1. Initialize Global App State
app_state = State()

# 2. Setup Plot Components

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

# 3. Connection Callbacks
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

# 4. Main GUI Layout
def gui():
    # Top Level Control Window
    draw_serial_connection_window(app_state, connect_callback=connect_cb, disconnect_callback=disconnect_cb)

    imgui.begin("Power")
    plot_voltage.render(app_state.latest_time)
    plot_current.render(app_state.latest_time)
    imgui.end()

    imgui.begin("Servo")
    plot_servo.render(app_state.latest_time)
    imgui.end()

    imgui.begin("Sensors raw")
    plot_accel_raw.render(app_state.latest_time)
    plot_omega_raw.render(app_state.latest_time)
    plot_pres.render(app_state.latest_time)
    imgui.end()

    imgui.begin("Sensors body")
    plot_accel_b.render(app_state.latest_time)
    plot_omega_b.render(app_state.latest_time)
    imgui.end()

    imgui.begin("Orientation")
    plot_quat_components.render(app_state.latest_time)
    plot_quat_orientation.render()
    imgui.end()

    imgui.begin("Estimation inertial")
    plot_accel_e.render(app_state.latest_time)
    plot_p_ground.render(app_state.latest_time)
    plot_alt_agl.render(app_state.latest_time)
    plot_vel_z.render(app_state.latest_time)
    imgui.end()