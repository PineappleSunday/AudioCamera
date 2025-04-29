import serial
import matplotlib.pyplot as plt
import numpy as np
import sys  # For exiting the script cleanly
from scipy.signal import firwin, lfilter

# --- Configuration ---
serial_port_name = 'COM3'
baud_rate        = 921600
plot_window_size = 500       # Number of L/R pairs to display
samples_per_read = 64
bytes_per_sample = 4         # sizeof(int32_t)
bytes_per_pair   = bytes_per_sample * 2
bytes_to_read    = samples_per_read * bytes_per_pair
Fs               = 32000     # sampling rate
FFTlen           = 512
halfFFT          = FFTlen // 2

# --- Filter Parameters ---
filter_cutoff_freq = 8000    # desired cutoff Hz
numtaps            = 60      # length of FIR

# Precompute FIR taps and initial state (two channels)
taps = firwin(numtaps, filter_cutoff_freq/(Fs/2), window='hamming', pass_zero=True)
# zi shape = (numtaps-1, num_channels)
zi = np.zeros((numtaps-1, 2))

# --- Serial Port Setup ---
print(f"Connecting to {serial_port_name} at {baud_rate} baud…")
try:
    s = serial.Serial(serial_port_name, baud_rate, timeout=10)
    print("Serial port open.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

# --- Allocate Rolling Buffers ---
raw_buffer  = np.zeros((plot_window_size, 2), dtype=np.int32)
filt_buffer = np.zeros_like(raw_buffer)

# Time and freq vectors
time_vector = np.arange(plot_window_size)
freq_vector = np.fft.rfftfreq(FFTlen, d=1/Fs)

# --- Plot Initialization ---
plt.ion()
fig, (ax_time, ax_fft, ax_fir) = plt.subplots(3, 1, figsize=(8, 6))

# Time-series lines
line_L, = ax_time.plot(time_vector, raw_buffer[:,0], 'b-', label='Left')
line_R, = ax_time.plot(time_vector, raw_buffer[:,1], 'r-', label='Right')
ax_time.set_title(f'Real-Time Raw Data (Last {plot_window_size} Pairs)')
ax_time.set_xlabel('Sample Index')
ax_time.set_ylabel('ADC (int32)')
ax_time.legend(loc='upper right');  ax_time.grid()

# FFT line (left channel)
line_fft, = ax_fft.plot(freq_vector, np.zeros_like(freq_vector), 'k-')
ax_fft.set_title('FFT (Raw Left Channel)')
ax_fft.set_xlabel('Freq (Hz)')
ax_fft.set_ylabel('Magnitude')
ax_fft.set_xlim(0, Fs/2);  ax_fft.grid()

# FIR-filtered lines
line_fir_L, = ax_fir.plot(time_vector, filt_buffer[:,0], 'g-', label='FIR Left')
line_fir_R, = ax_fir.plot(time_vector, filt_buffer[:,1], 'm-', label='FIR Right')
ax_fir.set_title(f'FIR Low-Pass ({filter_cutoff_freq} Hz)')
ax_fir.set_xlabel('Sample Index')
ax_fir.set_ylabel('Filtered Value')
ax_fir.legend(loc='upper right');  ax_fir.grid()

plt.tight_layout()
plt.show()

def is_figure_open(fig_handle):
    return plt.fignum_exists(fig_handle.number)

print("Starting acquisition… close figure to stop.")
try:
    while is_figure_open(fig):
        if s.in_waiting < bytes_to_read:
            plt.pause(0.001)
            continue

        # 1) Read & byte-swap
        raw = s.read(bytes_to_read)
        ba  = bytearray(raw)
        for i in range(0, len(ba), bytes_per_sample):
            ba[i:i+bytes_per_sample] = ba[i:i+bytes_per_sample][::-1]
        data  = np.frombuffer(ba, dtype='<i4').reshape(-1,2)  # shape=(64,2)

        # 2) Update raw_buffer
        raw_buffer = np.roll(raw_buffer, -samples_per_read, axis=0)
        raw_buffer[-samples_per_read:] = data

        # 3) Filter *just the new block*, carrying filter state
        filt_block, zi = lfilter(taps, 1.0, data, axis=0, zi=zi)

        # 4) Update filt_buffer
        filt_buffer = np.roll(filt_buffer, -samples_per_read, axis=0)
        filt_buffer[-samples_per_read:] = filt_block

        # 5) Update time plot
        line_L.set_ydata(raw_buffer[:,0])
        line_R.set_ydata(raw_buffer[:,1])
        ax_time.relim();  ax_time.autoscale_view()

        # 6) Update FFT (on *raw* left channel)
        Y  = np.fft.rfft(raw_buffer[:,0], n=FFTlen)
        P  = np.abs(Y) / FFTlen
        line_fft.set_ydata(P)
        ax_fft.relim();  ax_fft.autoscale_view()

        # 7) Update FIR plot
        line_fir_L.set_ydata(filt_buffer[:,0])
        line_fir_R.set_ydata(filt_buffer[:,1])
        ax_fir.relim();  ax_fir.autoscale_view()

        # 8) Refresh
        fig.canvas.flush_events()
        plt.pause(0.001)

except KeyboardInterrupt:
    pass
finally:
    print("Stopping… closing serial port.")
    s.close()
    plt.ioff()
    plt.close(fig)
