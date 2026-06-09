from imgui_bundle import imgui
from backend.state import app_state

def draw_serial_connection_window(connect_callback: callable = None, disconnect_callback: callable = None):
    imgui.set_next_window_size(imgui.ImVec2(400,200), imgui.Cond_.once)
    imgui.begin("Serial Connection")

    imgui.set_next_item_width(150)
    changed_port, app_state.current_port = imgui.combo(
        "Serial Port",
        app_state.current_port,
        app_state.ports
    )
    
    imgui.same_line()
    if imgui.button("Detect"):
        app_state.get_ports()

    changed_baud, app_state.current_baudrate = imgui.combo(
        "Baud Rate",
        app_state.current_baudrate,
        app_state.baudrates
    )

    imgui.separator()

    if app_state.connected:
        # Red button for disconnect
        imgui.push_style_color(imgui.Col_.button, imgui.ImColor.hsv(0, 0.6, 0.6).value)
        imgui.push_style_color(imgui.Col_.button_hovered, imgui.ImColor.hsv(0, 0.7, 0.7).value)
        imgui.push_style_color(imgui.Col_.button_active, imgui.ImColor.hsv(0, 0.8, 0.8).value)

        if imgui.button("Disconnect"):
            if disconnect_callback:
                disconnect_callback()

        imgui.pop_style_color(3)
    else:
        # Greenish default connect button
        imgui.push_style_color(imgui.Col_.button, imgui.ImColor.hsv(0.33, 0.6, 0.6).value)
        imgui.push_style_color(imgui.Col_.button_hovered, imgui.ImColor.hsv(0.33, 0.7, 0.7).value)
        imgui.push_style_color(imgui.Col_.button_active, imgui.ImColor.hsv(0.33, 0.8, 0.8).value)

        if imgui.button("Connect"):
            if connect_callback:
                port = app_state.ports[app_state.current_port]
                baud = app_state.baudrates[app_state.current_baudrate]
                connect_callback(port, baud)
        
        imgui.pop_style_color(3)

    imgui.end()