import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R

SERIAL_PORT = 'COM18'  
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.view_init(azim=210, elev=20)

def update(frame):
    try:
        if ser.in_waiting > 0:
            
            latest_line = None
            while ser.in_waiting > 0: # read until we get most recent line to stay up to date
                try:
                    latest_line = ser.readline().decode('utf-8').strip()
                except UnicodeDecodeError:
                    pass
            
            if latest_line:
                clean_line = latest_line.replace('q:[', '').replace(']', '') # get rid of q:[ and ]
                parts = clean_line.split(',')
                
                if len(parts) == 4:
                    qw = float(parts[0])
                    qx = float(parts[1])
                    qy = float(parts[2])
                    qz = float(parts[3])

                    ax.cla()

                    ax.set_xlim([-2, 2])
                    ax.set_ylim([2, -2])
                    ax.set_zlim([2, -2])
                    ax.set_xlabel('North (Xe)')
                    ax.set_ylabel('East (Ye)')
                    ax.set_zlabel('Down (Ze)')

                    rotation = R.from_quat([qw, qx, qy, qz], scalar_first=True) # scipy uses x y z w

                unit_x = [2, 0, 0]
                unit_y = [0, 1, 0]
                unit_z = [0, 0, 1]

                rot_x = rotation.apply(unit_x)
                rot_y = rotation.apply(unit_y)
                rot_z = rotation.apply(unit_z)

                # Draw fixed global axes (faint and dashed)
                ax.quiver(0, 0, 0, 1, 0, 0, color='r', alpha=0.3, linestyle='dashed', length=1.5)
                ax.quiver(0, 0, 0, 0, 1, 0, color='g', alpha=0.3, linestyle='dashed', length=1.5)
                ax.quiver(0, 0, 0, 0, 0, 1, color='b', alpha=0.3, linestyle='dashed', length=1.5)

                # Draw the rotating local axes (RGB convention: Red=X, Green=Y, Blue=Z)
                ax.quiver(0, 0, 0, rot_x[0], rot_x[1], rot_x[2], color='r', length=1.0, linewidth=3)
                ax.quiver(0, 0, 0, rot_y[0], rot_y[1], rot_y[2], color='g', length=1.0, linewidth=3)
                ax.quiver(0, 0, 0, rot_z[0], rot_z[1], rot_z[2], color='b', length=1.0, linewidth=3)

    except Exception as e:
        # Catch decoding or parsing errors gracefully (e.g., corrupted serial bytes)
        pass

# Create the animation, attempting to update every 50ms (20 FPS)
ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

plt.show()

# Cleanup when the matplotlib window is closed
ser.close()