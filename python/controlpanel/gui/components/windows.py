from imgui_bundle import imgui
from backend.telemetry import send_command
import math

def draw_command_window(app_state):
    imgui.begin("Commands")
    
    changed_hil, app_state.use_hil_data = imgui.checkbox("Enable HIL Data Source", app_state.use_hil_data)
    if changed_hil: send_command(app_state)
        
    imgui.separator()
    imgui.text("System Mode:")
    
    changed_mode_en, app_state.mode_override_active = imgui.checkbox("Override Switch", app_state.mode_override_active)
    imgui.same_line()
    imgui.set_next_item_width(150)
    
    changed_mode, app_state.selected_mode_idx = imgui.combo("##ModeSelect", app_state.selected_mode_idx, app_state.mode_names)
    if changed_mode:
        app_state.mode_override_active = True
        
    if changed_mode_en or changed_mode:
        send_command(app_state)
        
    imgui.separator()
    
    imgui.push_style_color(imgui.Col_.button, imgui.ImColor.hsv(0, 0.6, 0.6).value)
    imgui.push_style_color(imgui.Col_.button_hovered, imgui.ImColor.hsv(0, 0.7, 0.7).value)
    imgui.push_style_color(imgui.Col_.button_active, imgui.ImColor.hsv(0, 0.8, 0.8).value)
    
    if imgui.button("FORCE LAUNCH DETECT"):
        # Set, Sync, and instantly Clear the trigger
        app_state.force_launch_detect = True
        send_command(app_state)
        app_state.force_launch_detect = False
        
    imgui.pop_style_color(3)
    imgui.end()


def draw_servo_window(app_state, plot_servo):
    imgui.begin("Servo")
    changed_override, app_state.servo_override_active = imgui.checkbox("Manual Override", app_state.servo_override_active)
    
    if app_state.servo_override_active:
        imgui.same_line()
        changed_angle, app_state.manual_servo_rad = imgui.slider_angle("Command Angle", app_state.manual_servo_rad, 0, 180.0)
        if changed_override or changed_angle:
            send_command(app_state)
    elif changed_override:
        send_command(app_state)
    
    imgui.separator()
    plot_servo.render(app_state.latest_time)
    imgui.end()


def draw_logs_window(app_state):
    imgui.begin("Logging")
    imgui.begin_child("LogRegion", imgui.ImVec2(0, 0), False, imgui.WindowFlags_.horizontal_scrollbar)
    
    for log_msg in app_state.logs:
        imgui.text_unformatted(log_msg)
        
    if imgui.get_scroll_y() >= imgui.get_scroll_max_y():
        imgui.set_scroll_here_y(1.0)
        
    imgui.end_child()
    imgui.end()