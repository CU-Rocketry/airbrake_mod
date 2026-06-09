from imgui_bundle import imgui
from backend.state import app_state
from gui.components.plots import LinePlot
from gui.components.flags import draw_state_flag_toggle, draw_state_flag_readonly, STATE_FLAGS
from backend.telemetry import send_command

plot_servo = LinePlot("Servo", "Angle [deg]")
plot_servo.add_line("Command", app_state.buffers['servo_cmd'])
plot_servo.add_line("Feedback", app_state.buffers['servo_fdbk'])

def draw_servo_window():
    imgui.begin("Servo")
    changed, is_set, mask = draw_state_flag_toggle("Manual servo command", app_state.current_flags, "FLAG_SERVO_OVERRIDE_EN")
    if changed:
        if is_set: send_command(app_state, state_flags_set_mask=mask)
        else:      send_command(app_state, state_flags_clear_mask=mask)
    
    if app_state.current_flags & STATE_FLAGS.get("FLAG_SERVO_OVERRIDE_EN", 0): # if manual servo command enabled, show slider
        imgui.same_line()
        servo_cmd_changed, app_state.servo_cmd_rad = imgui.slider_angle("Command Angle", app_state.servo_cmd_rad, 0, 180.0)
        if servo_cmd_changed:
            send_command(app_state)
    
    imgui.separator()
    plot_servo.render(app_state.latest_time)
    imgui.end()