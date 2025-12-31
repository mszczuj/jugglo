import numpy as np
import matplotlib.pyplot as plt
import utils
import madgw
import utils
import time
import sys


timeseries_vis = utils.TimeseriesVisualizer(rb_capacity=400 * 4)
vis_box = utils.Box3DView()

m = madgw.Madgwick(sampleperiod=1/400, beta=1.0, zeta=0.0)

samples_num = 0
last_time = time.time()

loop_no = 0
while True:
    l = sys.stdin.readline()
    loop_no += 1
    if time.time() - last_time >= 1.0:
        print(f"sampling_rate = {samples_num} samples/sec")
        samples_num = 0
        last_time = time.time()

    samples_num += 1

    d = list(map(float, l.strip().split(','))) # ax,ay,az, gx,gy,gz
    m.update_imu(gyroscope=np.deg2rad(d[3:6]), accelerometer=d[0:3])
    timeseries_vis.append_data(ax=d[0:3], gx=d[3:6], rpy=m.quaternion.to_euler_angles())
    if loop_no % 50 == 0:        
        vis_box.update_3d_view(m.quaternion)
        timeseries_vis.draw()

