from imgui_bundle import imgui
from backend.state import app_state
from gui.components.plots import LinePlot, QuatPlot

# State estimation
# Madgwick
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

# Kalman
plot_p_ground = LinePlot("Ground Pressure", "Pressure [Pa]")
plot_p_ground.add_line("Ground Pressure", app_state.buffers['p_ground'])

plot_alt_agl = LinePlot("Altitude AGL", "Altitude [m]")
plot_alt_agl.add_line("Altitude", app_state.buffers['alt_agl'])

plot_vel_z = LinePlot("Vertical Velocity", "Velocity [m/s]")
plot_vel_z.add_line("Vertical Velocity", app_state.buffers['vel_z'])

def draw_state_estimation():
    imgui.begin("State Estimation")
    imgui.text("Madgwick filter")
    plot_quat_components.render(app_state.latest_time)
    plot_quat_orientation.render()
    imgui.separator()
    imgui.text("Kalman filter")
    plot_accel_e.render(app_state.latest_time)
    plot_p_ground.render(app_state.latest_time)
    plot_alt_agl.render(app_state.latest_time)
    plot_vel_z.render(app_state.latest_time)
    imgui.end()