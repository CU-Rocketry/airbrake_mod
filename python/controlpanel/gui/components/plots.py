from imgui_bundle import implot, imgui, implot3d
from gui.scrollingbuffer import ScrollingBuffer
import numpy as np

class TelemetryPlot:
    def __init__(self, title, y_label, history=10.0, max_points=2000):
        self.title = title
        self.y_label = y_label
        self.history = history
        
        # Dictionary to hold multiple lines on the same plot
        # e.g., {'accel_x': ScrollingBuffer, 'accel_y': ScrollingBuffer}
        self.lines = {} 
        self.max_points = max_points

    def add_line(self, line_name, buffer_ref, col=None):
        # Store a tuple of (buffer, column_index)
        self.lines[line_name] = (buffer_ref, col)

    def update_data(self, line_name, t_sec, value):
        if line_name in self.lines:
            self.lines[line_name].add_point(t_sec, value)

    def render(self, current_time):
        x_flags = implot.AxisFlags_.none
        y_flags = implot.AxisFlags_.auto_fit
        
        if implot.begin_plot(self.title, size=(-1, imgui.get_text_line_height() * 15)):
            implot.setup_axes("Time (s)", self.y_label, x_flags, y_flags)
            
            # Autoscroll the X axis
            implot.setup_axis_limits(
                implot.ImAxis_.x1, 
                current_time - self.history, 
                current_time, 
                implot.Cond_.always
            )
            
            # Plot each line
            for name, (buffer, col) in self.lines.items():
                if buffer.size > 0:
                    xs, ys = buffer.get_data(col=col)
                    implot.plot_line(name, xs, ys)
                    
            implot.end_plot()


class OrientationPlot3D:
    def __init__(self, title):
        self.title = title
        self.quat_buffer = None

    def set_buffer(self, buffer_ref):
        self.quat_buffer = buffer_ref

    def render(self):
        if self.quat_buffer is None:
            return

        # 1. Fetch the latest [w, x, y, z] quaternion
        q = self.quat_buffer.get_latest()
        w, x, y, z = q

        # Normalize to prevent scaling distortion
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-6:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        else:
            w, x, y, z = 1.0, 0.0, 0.0, 0.0

        # 2. Build the Rotation Matrix
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x**2 + y**2)]
        ])

        # 3. Calculate rotated Body frame unit vectors (Length = 1.0)
        x_axis = R @ np.array([1.0, 0.0, 0.0])
        y_axis = R @ np.array([0.0, 1.0, 0.0])
        z_axis = R @ np.array([0.0, 0.0, 1.0])

        # 4. Render the 3D Plot
        if implot3d.begin_plot(self.title, size=imgui.ImVec2(-1, 500)):
            x_flags = implot3d.AxisFlags_.none
            y_flags = implot3d.AxisFlags_.none
            z_flags = implot3d.AxisFlags_.invert
            
            implot3d.setup_axes("North", "East", "Down", x_flags, y_flags, z_flags)
            
            lim_flags = implot3d.Cond_.always
            implot3d.setup_axis_limits(implot3d.ImAxis3D_.x, -1, 1, lim_flags)
            implot3d.setup_axis_limits(implot3d.ImAxis3D_.y, -1, 1, lim_flags)
            implot3d.setup_axis_limits(implot3d.ImAxis3D_.z, -1, 1, lim_flags)

            def draw_vector(name, vec, color_vec4, line_weight=4.0):
                xs = np.array([0.0, vec[0]], dtype=np.float32)
                ys = np.array([0.0, vec[1]], dtype=np.float32)
                zs = np.array([0.0, vec[2]], dtype=np.float32)
                
                # Create the Spec object to hold the properties you found in the demo
                spec = implot3d.Spec()
                spec.line_weight = line_weight
                spec.line_color = color_vec4
                
                # Pass the spec object directly to PlotLine
                implot3d.plot_line(name, xs, ys, zs, spec=spec)

            # plot body axes with thicker lines
            draw_vector("Body X", x_axis, imgui.ImVec4(1.0, 0.2, 0.2, 1.0), line_weight=4.0)
            draw_vector("Body Y", y_axis, imgui.ImVec4(0.2, 1.0, 0.2, 1.0), line_weight=4.0)
            draw_vector("Body Z", z_axis, imgui.ImVec4(0.2, 0.2, 1.0, 1.0), line_weight=4.0)

            # Plot fixed inertial axes for reference
            draw_vector("Inertial X", np.array([0.5, 0.0, 0.0]), imgui.ImVec4(1, 0, 0, 0.5), line_weight=2.0)
            draw_vector("Inertial Y", np.array([0.0, 0.5, 0.0]), imgui.ImVec4(0, 1, 0, 0.5), line_weight=2.0)
            draw_vector("Inertial Z", np.array([0.0, 0.0, 0.5]), imgui.ImVec4(0, 0, 1, 0.5), line_weight=2.0)

            implot3d.end_plot()