from imgui_bundle import imgui
from backend.state import app_state
from gui.components.plots import LinePlot

plot_voltage = LinePlot("Battery Voltage", "Voltage [V]")
plot_voltage.add_line("Voltage", app_state.buffers['batt_v'])
plot_current = LinePlot("Battery Current", "Current [A]")
plot_current.add_line("Current", app_state.buffers['batt_i'])

def draw_power():
    imgui.begin("Power")
    plot_voltage.render(app_state.latest_time)
    plot_current.render(app_state.latest_time)
    imgui.end()