from imgui_bundle import imgui
from backend.firmware_schema import ffi

STATE_FLAGS = ffi.typeof("state_flag_t").relements

def get_flag(current_flags: int, flag_name: str) -> bool:
    mask = STATE_FLAGS.get(flag_name, 0) # get bitmask for requested flag
    if mask == 0:
        raise ValueError(f"Requested flag not defined: {flag_name}")
    return bool(current_flags & mask)

def draw_state_flag_readonly(label: str, current_flags: int, flag_name: str):
    is_set = get_flag(current_flags, flag_name)
    
    imgui.begin_disabled()
    imgui.checkbox(label, is_set)
    imgui.end_disabled()

def draw_state_flag_toggle(label: str, current_flags: int, flag_name: str) -> tuple[bool, bool, int]:
    is_set = get_flag(current_flags, flag_name)
    mask = STATE_FLAGS.get(flag_name, 0) # get bitmask for requested flag, safe if flag doesn't exist because get_flag would have already raised an error right?
    
    changed, value = imgui.checkbox(label, is_set) # toggleable checkbox
    return changed, value, mask # return whether changed, the new value, and the mask for this flag