import sys
import time 

last_time = time.time()
samples_num = 0

while True:
    line = sys.stdin.readline()
    samples_num += 1
    
    if time.time() - last_time >= 1.0:
        print(f"sampling_rate = {samples_num} samples/sec")
        samples_num = 0
        last_time = time.time()

