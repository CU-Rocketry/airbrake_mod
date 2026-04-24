from imgui_bundle import immapp
from gui.app import gui

if __name__ == "__main__":
    immapp.run(gui, window_title="Air Brakes Control Panel", with_implot=True, with_implot3d=True)