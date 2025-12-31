#%%
%matplotlib qt 
import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt


df = pd.read_csv('data.csv', index_col=False)
frames = np.load('cam_frames.npy')


fig, ax = plt.subplots(figsize=(8, 4))
fig_cam, ax_cam = plt.subplots(figsize=(4, 4))
ax_cam.axis('off')
img_display = ax_cam.imshow(frames[0])

acc = df[['accel_x', 'accel_y', 'accel_z']].values

ax.plot(acc)
vline = ax.axvline(0, color='red', linestyle='--', linewidth=1)
ax.set_title("Click to move marker")

def on_click(event):
    if event.inaxes != ax:
        return
    vline.set_xdata([event.xdata, event.xdata])
    fig.canvas.draw_idle()
    frame_num = int(df.loc[int(event.xdata)]['frame_num'])
    img_display.set_data(frames[frame_num])
    fig_cam.canvas.draw_idle()


fig.canvas.mpl_connect('button_press_event', on_click)
