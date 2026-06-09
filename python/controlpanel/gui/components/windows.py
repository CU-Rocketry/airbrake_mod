from imgui_bundle import imgui
from backend.telemetry import send_command
from imgui_bundle import hello_imgui # type: ignore
from backend.state import State
from gui.components.plots import LinePlot

def draw_command_window(app_state: State):
    imgui.begin("Commands")

    imgui.text("Hardware In the Loop Testing")
    changed, is_set, mask = draw_state_flag_toggle("Enable HIL data", app_state.current_flags, "FLAG_USE_HIL_DATA")
    if changed:
        if is_set: send_command(app_state, state_flags_set_mask=mask)
        else: send_command(app_state, state_flags_clear_mask=mask)
        
    imgui.separator()

    # Mode selector
    imgui.text("Mode")
    changed, is_set, mask = draw_state_flag_toggle("Override Mode", app_state.current_flags, "FLAG_MODE_OVERRIDE_EN")
    if changed:
        if is_set: send_command(app_state, state_flags_set_mask=mask)
        else:      send_command(app_state, state_flags_clear_mask=mask)

    imgui.same_line()
    imgui.set_next_item_width(150)
    
    mode_changed, app_state.selected_mode_idx = imgui.combo("##ModeSelect", app_state.selected_mode_idx, app_state.mode_names)
    if mode_changed: # user changed mode so automatically set override flag on
        mode_mask = STATE_FLAGS.get("FLAG_MODE_OVERRIDE_EN", 0)
        send_command(app_state, state_flags_set_mask=mode_mask)
    
    imgui.end()





def draw_logs_window():
    imgui.begin("Logging")
    hello_imgui.log_gui()
    imgui.end()