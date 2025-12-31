import cv2
import matplotlib.pyplot as plt
from skimage import color, measure, morphology
import numpy as np
import sys
import select
from collections import deque
from read import ImuMeasurement

def detect_red_ball(frame_rgb):
    """Return centroid (y, x) of largest red blob and mask."""
    hsv = color.rgb2hsv(frame_rgb)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]

    # Red wraps hue=0, so combine low and high ranges.
    red_mask = (
        ((h < 0.05) | (h > 0.95)) &
        (s > 0.4) &
        (v > 0.2)
    )

    # Clean small noise.
    red_mask = morphology.remove_small_objects(red_mask, min_size=250)

    labeled = measure.label(red_mask)
    regions = measure.regionprops(labeled)
    if not regions:
        return None, red_mask

    # Pick the largest region by area.
    region = max(regions, key=lambda r: r.area)
    cy, cx = region.centroid
    return (int(cy), int(cx)), red_mask

class LineBuffer:
    """Non-blocking stdin line reader that buffers partial lines between frames."""
    def __init__(self, max_bytes_per_frame=4096):
        self.buf = bytearray()
        self.max_bytes_per_frame = max_bytes_per_frame
        self.ready_lines = deque()

    def poll(self):
        """Poll stdin for available data, accumulate lines; non-blocking."""
        stream = sys.stdin.buffer
        read_so_far = 0
        while read_so_far < self.max_bytes_per_frame:
            ready, _, _ = select.select([stream], [], [], 0)
            if not ready:
                break
            peeked = stream.peek(self.max_bytes_per_frame - read_so_far)
            if not peeked:
                break
            to_read = min(len(peeked), self.max_bytes_per_frame - read_so_far)
            chunk = stream.read(to_read)
            if not chunk:
                break
            self.buf.extend(chunk)
            read_so_far += len(chunk)

        # Split complete lines
        while True:
            nl = self.buf.find(b"\n")
            if nl == -1:
                break
            line = self.buf[:nl]
            del self.buf[:nl + 1]
            self.ready_lines.append(ImuMeasurement.from_csv_row(str(line, "utf-8")))

    def pop_all(self):
        lines = list(self.ready_lines)
        self.ready_lines.clear()
        return lines


cap = cv2.VideoCapture(0, cv2.CAP_ANY)
if not cap.isOpened():
    raise SystemExit("Could not open camera 0")

fig, ax = plt.subplots()
ok, frame0 = cap.read()
if not ok:
    raise SystemExit("Failed to read initial frame")

frame_rgb = cv2.cvtColor(frame0, cv2.COLOR_BGR2RGB)
ax_img = ax.imshow(frame_rgb)
plt.ion()
plt.show()

line_buffer = LineBuffer()

running = True
def on_close(event):
    global running
    running = False

fig.canvas.mpl_connect("close_event", on_close)

frame_no = 0
frames = []

print ("frame_num,timestamp_ms,device_sequence_number,sensor_id,battery_mv,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,sequence_number,checksum_good,host_timestamp", flush=True)
while running:
    ok, frame = cap.read()

    if not ok:
        print("Failed to read frame")
        break

    # Non-blocking read of stdin; collect full lines for this frame
    line_buffer.poll()
    lines = line_buffer.pop_all()
    # if lines:
        # print (len(lines), "IMU lines received", flush=True)
    for l in lines:
        print (f"{frame_no},{l.to_csv_row()}", flush=True)
    
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # centroid, mask = detect_red_ball(frame_rgb)

    # overlay = frame.copy()
    # if centroid:
    #     cy, cx = centroid
    #     cv2.circle(overlay, (cx, cy), 15, (0, 255, 0), 2)
    # overlay_rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)

    overlay_rgb = frame_rgb
    ax_img.set_data(overlay_rgb)
    fig.canvas.draw()
    fig.canvas.flush_events()

    if not running or not plt.fignum_exists(fig.number):
        break

    # plt.pause(0.01)

    
    frames.append(frame_rgb)        
    frame_no += 1



np.save("cam_frames.npy", np.array(frames))
cap.release()

