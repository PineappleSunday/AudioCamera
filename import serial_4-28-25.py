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


# --- Filter Parameters ---
filter_cutoff_freq = 12000    # desired cutoff Hz
numtaps         = 60       # keep this for your LPF if you like
hp_numtaps      = 61       # must be odd
hp_fir_cutoff   = 200

# Precompute FIR taps and initial state (two channels)
taps = firwin(numtaps, filter_cutoff_freq/(Fs/2), window='hamming', pass_zero=True)
hp_taps   = firwin(hp_numtaps,
                   cutoff=hp_fir_cutoff/(Fs/2),
                   window='hamming',
                   pass_zero=False)
# zi shape = (numtaps-1, num_channels)
zi = np.zeros((numtaps-1, 2))
zi_hp_fir = np.zeros((hp_numtaps-1, 2))

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
hp_buffer_fir = np.zeros_like(raw_buffer)

# Time and freq vectors
time_vector = np.arange(plot_window_size)
freq_vector = np.fft.rfftfreq(FFTlen, d=1/Fs)

# Cross-correlation settings
N = raw_buffer.shape[0]
full_lags = np.arange(-N+1, N)
max_lag = 60      # +/- 60 samples
mask   = np.abs(full_lags) <= max_lag
plot_lags = full_lags[mask]

# --- Plot Initialization ---
plt.ion()
fig, (ax_time, ax_fft, ax_fir, ax_cross) = plt.subplots(4, 1, figsize=(8, 6))

# Time-series lines
line_L, = ax_time.plot(time_vector, raw_buffer[:,0], 'b-', label='Left')
line_R, = ax_time.plot(time_vector, raw_buffer[:,1], 'r-', label='Right')
ax_time.set_title(f'Real-Time Raw Data (Last {plot_window_size} Pairs)')
ax_time.set_xlabel('Sample Index')
ax_time.set_ylabel('ADC (int32)')
ax_time.legend(loc='upper right');  ax_time.grid()

# FFT line (left channel)
line_fft_l, = ax_fft.plot(freq_vector, np.zeros_like(freq_vector), 'b-', label='FFT Left')

# FFT line (right channel)
line_fft_r, = ax_fft.plot(freq_vector, np.zeros_like(freq_vector), 'r-', label='FFT Right')
# FFT Plot Settings
ax_fft.set_title('FFT (Raw Left-Right Channel)')
ax_fft.set_xlabel('Freq (Hz)')
ax_fft.set_ylabel('Magnitude')
ax_fft.set_xlim(0, Fs/2)
ax_fft.legend(loc='upper right');  ax_fft.grid()


# FIR-filtered lines
line_L_hp, = ax_time.plot(time_vector, hp_buffer_fir[:,0],
                          'c--', label=f'FIR-HPF ≥{hp_fir_cutoff}Hz Left')
line_R_hp, = ax_time.plot(time_vector, hp_buffer_fir[:,1],
                          'y--', label=f'FIR-HPF ≥{hp_fir_cutoff}Hz Right')

line_fir_L, = ax_fir.plot(time_vector, filt_buffer[:,0], 'g-', label='FIR Left')
line_fir_R, = ax_fir.plot(time_vector, filt_buffer[:,1], 'm-', label='FIR Right')
ax_fir.set_title(f'FIR Low-Pass ({filter_cutoff_freq} Hz)')
ax_fir.set_xlabel('Sample Index')
ax_fir.set_ylabel('Filtered Value')
ax_fir.legend(loc='upper right');  ax_fir.grid()

# Cross-correlation plot (left-right channel)
cross_correlation = np.correlate(raw_buffer[:,0], raw_buffer[:,1], mode='full')
# Cross-correlation plot settings
# set up the line (once) with x-data = plot_lags
line_cross, = ax_cross.plot(plot_lags, np.zeros_like(plot_lags),'k-',label='Cross‐Corr')
line_cross_fir, = ax_cross.plot(plot_lags, np.zeros_like(plot_lags), 'm--', label='FIR Corr')
ax_cross.set_xlim(-max_lag, max_lag)
ax_cross.set_ylim(-1, 1)
ax_cross.set_title('Cross-Correlation (Raw Left-Right)')
ax_cross.set_xlabel('Lag')
ax_cross.set_ylabel('Correlation Value')    
ax_cross.legend(loc='upper right');  ax_cross.grid()


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
        if len(raw) != bytes_to_read:
            continue
        ba  = bytearray(raw)
        for i in range(0, len(ba), bytes_per_sample):
            ba[i:i+bytes_per_sample] = ba[i:i+bytes_per_sample][::-1]
        data  = np.frombuffer(ba, dtype='<i4').reshape(-1,2)  # shape=(64,2)

        # 2) Update raw_buffer
        raw_buffer = np.roll(raw_buffer, -samples_per_read, axis=0)
        raw_buffer[-samples_per_read:] = data

        # 3) Filter *just the new block*, carrying filter state
        filt_block, zi = lfilter(taps, [1.0],data, axis=0, zi=zi)

        # 4) Update filt_buffer
        hp_block_fir, zi_hp_fir = lfilter(hp_taps, [1], data, axis=0, zi=zi_hp_fir)
        hp_buffer_fir   = np.roll(hp_buffer_fir, -samples_per_read, axis=0)
        hp_buffer_fir[-samples_per_read:] = hp_block_fir
        filt_buffer = np.roll(filt_buffer, -samples_per_read, axis=0)
        filt_buffer[-samples_per_read:] = filt_block

        # 5) Update time plot

        line_L.set_ydata(raw_buffer[:,0])
        line_R.set_ydata(raw_buffer[:,1])
        ax_time.relim();  ax_time.autoscale_view()

        # 6) Update FFT (on *raw* left channel)
        Y  = np.fft.rfft(raw_buffer[:,0], n=FFTlen)
        P  = np.abs(Y) / FFTlen
        line_fft_l.set_ydata(P)
        ax_fft.relim();  ax_fft.autoscale_view()
        
        #6a) Update FFT (on *raw* right channel)
        Z = np.fft.rfft(raw_buffer[:,1], n=FFTlen)
        P2 = np.abs(Z) / FFTlen
        line_fft_r.set_ydata(P2)
        ax_fft.relim();  ax_fft.autoscale_view()    
        
        
        # 7) Update FIR plot
        line_L_hp.set_ydata(hp_buffer_fir[:,0])
        line_R_hp.set_ydata(hp_buffer_fir[:,1])
        line_fir_L.set_ydata(filt_buffer[:,0])
        line_fir_R.set_ydata(filt_buffer[:,1])
        ax_fir.relim();  ax_fir.autoscale_view()
        
        # 8a) copy and demean into float
        L = raw_buffer[:,0].astype(float)
        R = raw_buffer[:,1].astype(float)
        
        L_fir = hp_buffer_fir[:,0].astype(float)
        R_fir = hp_buffer_fir[:,1].astype(float)
        
        L_fir -= L_fir.mean()
        R_fir -= R_fir.mean()
        
        L -= L.mean()
        R -= R.mean()

        # 8b) full (unnormalized) corr, cast to float to avoid overflow
        xc = np.correlate(L, R, mode='full').astype(float)
        xc_fir = np.correlate(L_fir, R_fir, mode='full').astype(float)

        # 8c) normalize to get a coefficient in [-1,1]
        norm = np.sqrt((L*L).sum() * (R*R).sum())
        norm_fir = np.sqrt((L_fir*L_fir).sum() * (R_fir*R_fir).sum())
        xc /= (norm + 1e-16)    # avoid divide-by-zero
        xc_fir /= (norm_fir + 1e-16)    # avoid divide-by-zero

        # 8d) update only the window you care about
        xc_window = xc[mask]
        xc_window_fir = xc_fir[mask]

        line_cross.set_ydata(xc_window)
        line_cross_fir.set_ydata(xc_window_fir)

        # 8e) find the peak delay
        peak_idx     = np.argmax(np.abs(xc_window))
        peak_idx_fir = np.argmax(np.abs(xc_window_fir))
        lag_samples  = plot_lags[peak_idx]
        lag_samples_fir = plot_lags[peak_idx_fir]
        time_delay_s = lag_samples / Fs
        time_delay_s_fir = lag_samples_fir / Fs
        ax_cross.set_title(f'Raw Lag {lag_samples} samples → {time_delay_s*1e6:.1f} μs\n FIR Lag {lag_samples_fir} samples → {time_delay_s_fir*1e6:.1f} μs')
        
        
        # 9) Refresh
        fig.canvas.flush_events()
        plt.pause(0.001)

except KeyboardInterrupt:
    pass
finally:
    print("Stopping… closing serial port.")
    s.close()
    plt.ioff()
    plt.close(fig)
