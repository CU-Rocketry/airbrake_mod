from imgui_bundle import imgui
from backend.state import app_state
from gui.components.plots import LinePlot

# Body frame sensors
plot_accel_b = LinePlot("Body Acceleration", "Accel [m/s^2]")
plot_accel_b.add_line("X", app_state.buffers['accel_b'], col=0)
plot_accel_b.add_line("Y", app_state.buffers['accel_b'], col=1)
plot_accel_b.add_line("Z", app_state.buffers['accel_b'], col=2)

plot_omega_b = LinePlot("Body Angular Rate", "Omega [rad/s]")
plot_omega_b.add_line("X", app_state.buffers['omega_b'], col=0)
plot_omega_b.add_line("Y", app_state.buffers['omega_b'], col=1)
plot_omega_b.add_line("Z", app_state.buffers['omega_b'], col=2)

plot_mag_b = LinePlot("Body Magnetometer", "Flux Density [mgauss]")
plot_mag_b.add_line("X", app_state.buffers['mag_b'], col=0)
plot_mag_b.add_line("Y", app_state.buffers['mag_b'], col=1)
plot_mag_b.add_line("Z", app_state.buffers['mag_b'], col=2)

def draw_sensors_body():
    imgui.begin("Sensors body")
    plot_accel_b.render(app_state.latest_time)
    plot_omega_b.render(app_state.latest_time)
    plot_mag_b.render(app_state.latest_time)
    imgui.end()