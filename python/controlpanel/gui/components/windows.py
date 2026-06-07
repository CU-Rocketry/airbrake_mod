from imgui_bundle import imgui
from backend.telemetry import send_command
from imgui_bundle import hello_imgui # type: ignore
from backend.state import State
from plots import LinePlot

def draw_command_window(app_state: State):
    imgui.begin("Commands")

    imgui.text("Hardware In the Loop Testing")
    use_hil_data_changed, app_state.use_hil_data = imgui.checkbox("Enable HIL data", app_state.use_hil_data) # outputs whether changed and new value
    if use_hil_data_changed:
        send_command(app_state)
        
    imgui.separator()

    # Mode selector
    imgui.text("Mode")
    mode_en_changed, app_state.mode_en = imgui.checkbox("Override Mode", app_state.mode_en)
    imgui.same_line()
    imgui.set_next_item_width(150)
    
    mode_changed, app_state.selected_mode_idx = imgui.combo("##ModeSelect", app_state.selected_mode_idx, app_state.mode_names)
    if mode_changed:
        app_state.mode_en = True
    if mode_en_changed or mode_changed:
        send_command(app_state)
        
    imgui.separator()

    # State estimation
    imgui.text("State Estimation")
    imgui.push_style_color(imgui.Col_.button, imgui.ImColor.hsv(0, 0.6, 0.6).value)
    imgui.push_style_color(imgui.Col_.button_hovered, imgui.ImColor.hsv(0, 0.7, 0.7).value)
    imgui.push_style_color(imgui.Col_.button_active, imgui.ImColor.hsv(0, 0.8, 0.8).value)
    
    if imgui.button("FORCE LAUNCH DETECT"):
        app_state.force_launch_detect = True # set launch detect flag in state
        send_command(app_state) # send command to MCU to trigger launch detect
        app_state.force_launch_detect = False # reset so it isn't used again until next button press
        
    imgui.pop_style_color(3)

    imgui.end()


def draw_servo_window(app_state: State, plot_servo: LinePlot):
    imgui.begin("Servo")
    servo_cmd_en_changed, app_state.servo_cmd_en = imgui.checkbox("Manual servo command", app_state.servo_cmd_en)
    
    if app_state.servo_cmd_en: # if manual servo command enabled, show slider
        imgui.same_line()
        servo_cmd_changed, app_state.servo_cmd_rad = imgui.slider_angle("Command Angle", app_state.servo_cmd_rad, 0, 180.0)
        if servo_cmd_en_changed or servo_cmd_changed:
            send_command(app_state)
    elif servo_cmd_en_changed: # if not enabled and checkbox was just changed, send command to disable
        send_command(app_state)
    
    imgui.separator()
    plot_servo.render(app_state.latest_time)
    imgui.end()


def draw_logs_window(app_state: State):
    imgui.begin("Logging")
    # imgui.begin_child("LogRegion", imgui.ImVec2(0, 0), False, imgui.WindowFlags_.horizontal_scrollbar)
    
    # for log_msg in app_state.logs:
    #     imgui.text_unformatted(log_msg)
        
    # if imgui.get_scroll_y() >= imgui.get_scroll_max_y():
    #     imgui.set_scroll_here_y(1.0)
        
    # imgui.end_child()
    hello_imgui.log_gui()
    imgui.end()