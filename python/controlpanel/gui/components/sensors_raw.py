from imgui_bundle import imgui
from backend.state import app_state
from gui.components.plots import LinePlot

# Raw sensors
plot_pres = LinePlot("Barometer", "Pressure [Pa]")
plot_pres.add_line("Pressure", app_state.buffers['pres_pa'])

plot_accel_ms2 = LinePlot("Acceleration", "Accel [m/s^2]")
plot_accel_ms2.add_line("X", app_state.buffers['accel_ms2'], col=0)
plot_accel_ms2.add_line("Y", app_state.buffers['accel_ms2'], col=1)
plot_accel_ms2.add_line("Z", app_state.buffers['accel_ms2'], col=2)

plot_omega_rads = LinePlot("Angular Rate", "Omega [rad/s]")
plot_omega_rads.add_line("X", app_state.buffers['omega_rads'], col=0)
plot_omega_rads.add_line("Y", app_state.buffers['omega_rads'], col=1)
plot_omega_rads.add_line("Z", app_state.buffers['omega_rads'], col=2)

plot_mag_mgauss = LinePlot("Magnetometer", "Flux Density [mgauss]")
plot_mag_mgauss.add_line("X", app_state.buffers['mag_mgauss'], col=0)
plot_mag_mgauss.add_line("Y", app_state.buffers['mag_mgauss'], col=1)
plot_mag_mgauss.add_line("Z", app_state.buffers['mag_mgauss'], col=2)

def draw_sensors_raw():
    imgui.begin("Sensors raw")
    plot_pres.render(app_state.latest_time)
    plot_accel_ms2.render(app_state.latest_time)
    plot_omega_rads.render(app_state.latest_time)
    plot_mag_mgauss.render(app_state.latest_time)
    imgui.end()