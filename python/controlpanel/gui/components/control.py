from imgui_bundle import imgui
from backend.state import app_state
from gui.components.plots import LinePlot
from gui.components.flags import draw_state_flag_toggle, draw_state_flag_readonly, STATE_FLAGS
from backend.telemetry import send_command

plot_predicted = LinePlot("Predicted Apogee", "Prediction [m]")
plot_predicted.add_line("Predicted", app_state.buffers['predicted'])

plot_output = LinePlot("Output", "Deployment command [0-1]")
plot_output.add_line("Output", app_state.buffers['output'])

plot_contrib = LinePlot("PI Controller", "Contribution [0-1]")
plot_contrib.add_line("P", app_state.buffers['p_contrib'])
plot_contrib.add_line("I", app_state.buffers['i_contrib'])

def draw_control():
    imgui.begin("Control")
    imgui.text("Deployment Lockout Conditions")
    draw_state_flag_readonly("Lockout Accel", app_state.current_flags, "FLAG_LOCKOUT_ACCEL")
    draw_state_flag_readonly("Lockout Altitude", app_state.current_flags, "FLAG_LOCKOUT_ALTITUDE")
    draw_state_flag_readonly("Lockout Elapsed", app_state.current_flags, "FLAG_LOCKOUT_ELAPSED")
    draw_state_flag_readonly("Lockout Apogee", app_state.current_flags, "FLAG_LOCKOUT_APOGEE")
    draw_state_flag_readonly("Lockout Attitude", app_state.current_flags, "FLAG_LOCKOUT_ATTITUDE")
    imgui.separator()
    plot_predicted.render(app_state.latest_time)
    imgui.separator()
    plot_output.render(app_state.latest_time)
    plot_contrib.render(app_state.latest_time)
    imgui.end()