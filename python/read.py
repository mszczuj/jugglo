#%%
from dataclasses import dataclass
import serial
import struct
import argparse
import socket
import time
import matplotlib.pyplot as plt
from numpy_ringbuffer  import RingBuffer
import numpy as np


## this must match the definitions in imu_base.hpp and main.cpp
aRes = 16.0 / 32768.0    	# ares value for full range (16g) readings
gRes = 2000.0 / 32768.0	    # gres value for full range (2000dps) readings

# Python struct format string for ImuDatagram (without magic header)
# Format: I (timestamp_ms) + H (sequence_number) + B (sensor_id) + 6h (accel_gyro)
IMUD_FORMAT = '<IHB6hH'
IMUD_SIZE = struct.calcsize(IMUD_FORMAT)

@dataclass
class ImuDatagram:
    timestamp_ms: int
    sequence_number: int
    sensor_id: int
    accel_gyro: tuple
    checksum_good: bool 
    host_timestamp: float 

def imu_checksum16(data) -> int:
    """
    Internet checksum (RFC 1071).
    Returns a 16-bit integer (0..65535).
    """
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError("data must be bytes-like")

    b = bytes(data)
    if len(b) % 2 == 1:
        b += b"\x00"  # pad odd length with one zero byte

    s = 0
    for i in range(0, len(b), 2):
        word = (b[i] << 8) + b[i + 1]
        s += word

    # fold carries
    s = (s & 0xFFFF) + (s >> 16)
    s = (s & 0xFFFF) + (s >> 16)  # in case another carry was created

    return (~s) & 0xFFFF

def parse_imu_datagram(data):
    """Parse IMU datagram from bytes"""
    timestamp_ms, sequence_number, sensor_id, *accel_gyro, checksum = struct.unpack(IMUD_FORMAT, data)
    checksum_good = checksum == imu_checksum16(data[:-2])
    return ImuDatagram(
        timestamp_ms=timestamp_ms,
        sequence_number=sequence_number,
        sensor_id=sensor_id,
        accel_gyro=(accel_gyro[3] * aRes, accel_gyro[4] * aRes, accel_gyro[5] * aRes,
                       accel_gyro[0] * gRes, accel_gyro[1] * gRes, accel_gyro[2] * gRes),
        checksum_good=checksum_good,
        host_timestamp= time.time()
    )

def read_datagram_from_serial(port, baudrate=115_200, timeout=1):    
    with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
        header = b'IMUD'
        buffer = bytearray()
        while True:
            while True: # find header
                byte = ser.read(1)
                # this is just ot check visually on terminal if we parse it properly
                # print ('.', end='', flush=True)
                if byte:
                    buffer.append(byte[0])
                    if len(buffer) > len(header):
                        buffer.pop(0)
                    if bytes(buffer) == header:                    
                        buffer.clear()
                        break            
                
            # rest of the datagram
            datagram = ser.read(IMUD_SIZE)
            if len(datagram) == IMUD_SIZE:  
                imu_data = parse_imu_datagram(datagram)
                yield imu_data

def read_datagram_from_socket(host, port, timeout=1.0):
    """Generator that binds a UDP socket and yields parsed datagrams from clients."""
    header = b'IMUD'
    header_len = len(header) # 4 bytes
    packet_len = header_len + IMUD_SIZE # 20bytes

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.settimeout(timeout)
        # print('UDP server listening on {}:{}'.format(host, port))

        while True:
            try:
                data, addr = server.recvfrom(packet_len*20)  # read a packet; allow some slack
            except socket.timeout:
                continue

            offset = 0
            while True:
                start = data.find(header, offset)
                if start == -1:
                    break
                end = start + packet_len
                if end > len(data):
                    break  # incomplete datagram at end of packet

                datagram = data[start + header_len:end]
                imu_data = parse_imu_datagram(datagram)
                yield imu_data

                offset = end


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Read IMU datagrams from serial port.")
    parser.add_argument("--serial_port", type=str, default="/dev/ttyACM0", help="Serial port (e.g., /dev/ttyACM0 or COM3) (default: /dev/ttyACM0)")
    parser.add_argument("--serial", type=bool, default=False, help="Read from serial port (default: False)")
    parser.add_argument("--server_host", type=str, default="0.0.0.0", help="Server host to bind (default:0.0.0.0)")
    parser.add_argument("--server_port", type=int, default=50555, help="Server port to bind (default:50555)")
    parser.add_argument("--stats", type=bool, default=False, help="Online stats (default: False)")
        
    args = parser.parse_args()

    if args.serial:
        in_stream = read_datagram_from_serial(args.serial_port, 115200, timeout=1)
    else : 
        in_stream = read_datagram_from_socket(args.server_host, args.server_port, timeout=1.0)

    if args.stats:
        rb_t = RingBuffer(capacity=400, dtype=float)
        rb_err = RingBuffer(capacity=400, dtype=bool)
        rb_device_time = RingBuffer(capacity=400, dtype=int)
        bad_seq_count = 0        
        for i, d in enumerate(in_stream):
            rb_t.append(d.host_timestamp)
            rb_err.append(not d.checksum_good)
            if d.checksum_good:
                rb_device_time.append(d.timestamp_ms)
            # print (d.sequence_number, flush=True)

            if i ==0 :
                last_seq_num = d.sequence_number
            else:
                last_seq_num = (last_seq_num +1) % 65536
                if d.sequence_number != last_seq_num: 
                    last_seq_num = d.sequence_number
                    bad_seq_count += 1

            if i % 25 == 0:
                duration = rb_t[-1] - rb_t[-0]
                sample_rate = len(rb_t)/duration
                err_rate = np.sum(rb_err[-50:])
                dt = np.diff(np.array(rb_device_time))
                
                print(f"Sample Rate: {sample_rate:.2f} Hz, Checksum Error Rate: {err_rate}, Bad Sequence Count: {bad_seq_count}, dev time: {np.mean(dt):.2f} ± {np.std(dt):.2f}", flush=True)

                bad_seq_count = 0 
                
    else:  # just print the data
        for i, d in enumerate(in_stream):
            if d.checksum_good:
                # pass
                print(str(d.accel_gyro)[1:-1], flush=True)
            # print (d.sequence_number, d.timestamp_ms, d.host_timestamp, flush=True)
