
import matplotlib.pyplot as plt 
from numpy_ringbuffer  import RingBuffer
import sys
import numpy as np 


class CommunicationStats:
    def __init__(self, window_size=100,  plot_every_n=5):

        self.plot_every_n = plot_every_n
        
        self.rb_sample_rate = RingBuffer(capacity=window_size, dtype=float)
        self.rb_err_rate = RingBuffer(capacity=window_size, dtype=float)
        self.rb_bad_seq_rate = RingBuffer(capacity=window_size, dtype=float)

        self.fig, self.ax = plt.subplots(1,1, figsize=(12,4))
        self.ax_b = self.ax.twinx()
        self.fig.tight_layout()

        plt.show(block=False)
        plt.pause(0.01) # allow to paint the window


    def update(self, i, sample_rate, error_rate, bad_sequence_rate):
        self.rb_sample_rate.append(sample_rate)
        self.rb_err_rate.append(error_rate)
        self.rb_bad_seq_rate.append(bad_sequence_rate)

        if i % self.plot_every_n == 0:

            self.ax.clear()
            self.ax.plot(self.rb_sample_rate, label='Sample Rate (Hz)')
            self.ax.set_title('IMU Sample Rate')
            self.ax.set_xlabel('Samples (last 100)')
            self.ax.set_ylabel('Sample Rate (Hz)')
            self.ax.grid(True)
            self.ax.legend(loc = 'upper left')

            a = np.array(self.rb_sample_rate)
            amin, amax = np.min(a), np.max(a)
            self.ax.set_ylim(50 * (amin // 50), 50 * ((amax + 49) // 50))
            
            self.ax_b.clear()
            self.ax_b.bar( np.arange(len(self.rb_err_rate)), self.rb_err_rate, color='r', label='Checksum Errors', alpha=0.5)
            self.ax_b.bar( np.arange(len(self.rb_bad_seq_rate)), self.rb_bad_seq_rate, color='m', label='Bad Sequence Numbers', alpha=0.5)            

            self.ax_b.legend(loc='upper right')
            a1, a2 = np.array(self.rb_err_rate), np.array(self.rb_bad_seq_rate)
            amax = max(np.max(a1), np.max(a2))
            self.ax_b.set_ylim(0, ((amax + 5) // 5) * 5)
            
            self.fig.canvas.draw()
        self.fig.canvas.flush_events()
            
if __name__ == "__main__":

    stats = CommunicationStats(window_size=100, plot_every_n=5)
    i = 0
    while True:
        l = sys.stdin.readline()
        l = l.strip()
        print (l)        
        ssr, ser, sbd, _ = l.split(',')

        stats.update(i,        
            float(ssr[ssr.index(":")+1:-3]),
            float(ser[ser.index(":")+1:]),
            float(sbd[sbd.index(":")+1:]))
        i +=1

