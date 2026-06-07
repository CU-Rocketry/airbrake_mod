from imgui_bundle import imgui
from backend.firmware_schema import ffi

STATE_FLAGS = ffi.typeof("state_flags_t").relements
STATE_FLAGS = {}

def draw_state_flag_readonly(label: str, current_flags: int, flag_name: str):
    mask = STATE_FLAGS.get(flag_name, 0) # get bitmask for requested flag
    
    if mask == 0: # if flag not defined, show error
        imgui.text_colored((1.0, 0.0, 0.0, 1.0), f"Flag not found: {label}")
        return

    is_set = bool(current_flags & mask)
    
    imgui.begin_disabled()
    imgui.checkbox(label, is_set)
    imgui.end_disabled()

def draw_state_flag_toggle(label: str, current_flags: int, flag_name: str) -> tuple[bool, bool]:
    mask = STATE_FLAGS.get(flag_name, 0) # get bitmask for requested flag

    if mask == 0: # if flag not defined, show error
        imgui.text_colored((1.0, 0.0, 0.0, 1.0), f"Flag not found: {label}")
        return False, False

    is_set = bool(current_flags & mask) # determine if flag is currently set
    
    changed, value = imgui.checkbox(label, is_set) # toggleable checkbox
    return changed, value # return whether changed and the new value