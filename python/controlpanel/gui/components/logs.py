from imgui_bundle import imgui, hello_imgui # type: ignore

def draw_logs_window():
    imgui.begin("Logging")
    hello_imgui.log_gui()
    imgui.end()