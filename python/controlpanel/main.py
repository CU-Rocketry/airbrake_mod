from imgui_bundle import immapp
from gui.app import get_layout_params

if __name__ == "__main__":
    runner_params = get_layout_params()
    
    addons = immapp.AddOnsParams()
    addons.with_implot = True
    addons.with_implot3d = True
    
    immapp.run(runner_params=runner_params, add_ons_params=addons)