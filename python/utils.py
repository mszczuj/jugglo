#%%
import numpy as np
import madgw
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from numpy_ringbuffer import RingBuffer

class TimeseriesVisualizer:
    def __init__(self, rb_capacity):
        self.rb_a = RingBuffer(capacity=rb_capacity, dtype=(np.float32, 3))
        self.rb_g = RingBuffer(capacity=rb_capacity, dtype=(np.float32, 3))
        self.rb_rpy = RingBuffer(capacity=rb_capacity, dtype=(np.float32, 3))

        self.rb_a.extend(np.full((rb_capacity, 3), fill_value=np.nan, dtype=np.float32))
        self.rb_g.extend(np.full((rb_capacity, 3), fill_value=np.nan, dtype=np.float32))
        self.rb_rpy.extend(np.full((rb_capacity, 3), fill_value=np.nan, dtype=np.float32))

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
        l_ax1 = ax1.plot(self.rb_a[:, :3], label="ax,ay,az".split(','))
        l_ax2 = ax2.plot(self.rb_g[:, :3], label="gx,gy,gz".split(','))
        l_ax3 = ax3.plot(self.rb_rpy[:, :], label="roll,pitch,yaw".split(','))

        ax1.set_title("Accelerometer")
        ax1.set_ylabel("Acceleration (g)")
        ax1.legend(loc='upper right')

        ax2.set_title("Gyroscope")
        ax2.set_ylabel("Angular rate (deg/s)")
        ax2.legend(loc='upper right')

        ax3.set_title("Orientation (Euler angles)")
        ax3.set_ylabel("Angle (rad)")
        ax3.set_xlabel("Samples")
        ax3.legend(loc='upper right') 

        fig.tight_layout()

        plt.show(block=False)
        plt.pause(0.01)


        self.fig = fig
        self.axes = (ax1, ax2, ax3)
        self.lines = (l_ax1, l_ax2, l_ax3)


    def append_data(self, ax, gx, rpy):
        self.rb_a.append(np.array(ax))
        self.rb_g.append(np.array(gx))
        self.rb_rpy.append(np.array(rpy))
        
    def draw(self):

        self.lines[0][0].set_ydata(self.rb_a[:, 0])
        self.lines[0][1].set_ydata(self.rb_a[:, 1])
        self.lines[0][2].set_ydata(self.rb_a[:, 2])
    
        self.lines[1][0].set_ydata(self.rb_g[:, 0])
        self.lines[1][1].set_ydata(self.rb_g[:, 1])
        self.lines[1][2].set_ydata(self.rb_g[:, 2])

        self.lines[2][0].set_ydata(self.rb_rpy[:, 0])
        self.lines[2][1].set_ydata(self.rb_rpy[:, 1])
        self.lines[2][2].set_ydata(self.rb_rpy[:, 2])

        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
class Box3DView:
    
    def __init__(self):
        self.fig3d = plt.figure(figsize=(8, 8))
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        
        # Define box vertices (unit cube centered at origin)
        vertices = np.array([
            [-1, -1, -0.5], [1, -1, -0.5], [1, 1, -0.5], [-1, 1, -0.5],  # bottom face
            [-1, -1, 0.5], [1, -1, 0.5], [1, 1, 0.5], [-1, 1, 0.5]       # top face
        ]) * 0.5

        # Define the 6 faces of the box
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # left
            [vertices[1], vertices[2], vertices[6], vertices[5]]   # right
        ]

        self.box_vectices = vertices

        self.poly3d = Poly3DCollection(faces, alpha=0.7, facecolor='cyan', edgecolor='black', linewidth=2)
        self.ax3d.add_collection3d(self.poly3d)

        self.axes_basis = np.eye(3)
        self.axes_length = 1.0
        self.axes_quiver = self.ax3d.quiver(
            np.zeros(3), np.zeros(3), np.zeros(3),
            self.axes_basis[:, 0] * self.axes_length,
            self.axes_basis[:, 1] * self.axes_length,
            self.axes_basis[:, 2] * self.axes_length,
            color=['r', 'g', 'b'],
            linewidth=2,
            arrow_length_ratio=0.15,
        )

        self.ax3d.set_xlabel('X')
        self.ax3d.set_ylabel('Y')
        self.ax3d.set_zlabel('Z')
        # ax3d.set_title('3D Box with Quaternion Rotation')
        plt.show(block=False)
        plt.pause(0.01)

    def update_3d_view(self, quat:madgw.Quaternion):
        quat = quat.normalize()
        rotated_vertices = np.array([quat.rotate_vector(v) for v in self.box_vectices])
        rotated_axes = np.array([quat.rotate_vector(axis) for axis in self.axes_basis]) * self.axes_length
        
        # Update faces with rotated vertices
        rotated_faces = [
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[2], rotated_vertices[3]],
            [rotated_vertices[4], rotated_vertices[5], rotated_vertices[6], rotated_vertices[7]],
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[5], rotated_vertices[4]],
            [rotated_vertices[2], rotated_vertices[3], rotated_vertices[7], rotated_vertices[6]],
            [rotated_vertices[0], rotated_vertices[3], rotated_vertices[7], rotated_vertices[4]],
            [rotated_vertices[1], rotated_vertices[2], rotated_vertices[6], rotated_vertices[5]]
        ]
        
        self.poly3d.set_verts(rotated_faces)
        self.axes_quiver.remove()
        self.axes_quiver = self.ax3d.quiver(
            np.zeros(3), np.zeros(3), np.zeros(3),
            rotated_axes[:, 0],
            rotated_axes[:, 1],
            rotated_axes[:, 2],
            color=['r', 'g', 'b'],
            linewidth=2,
            arrow_length_ratio=0.15,
        )
        self.fig3d.canvas.draw()
        self.fig3d.canvas.flush_events()

        self.ax3d.set_xlim([-2, 2])
        self.ax3d.set_ylim([-2, 2])
        self.ax3d.set_zlim([-2, 2])


