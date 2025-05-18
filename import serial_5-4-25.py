import serial
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import firwin, lfilter

from collections import deque

# --- Configuration ---
serial_port_name = 'COM3'
baud_rate        = 921600
plot_window_size = 500       # samples in history
samples_per_read = 64
bytes_per_sample = 4         # sizeof(int32_t)
bytes_to_read    = samples_per_read * bytes_per_sample * 2  # stereo
Fs               = 32000     # sampling rate
FFTlen           = 512

# --- HP FIR Filter Design ---
hp_numtaps    = 61      # odd
hp_cutoff_hz  = 2000
hp_taps       = firwin(hp_numtaps,
                       hp_cutoff_hz/(Fs/2),
                       window='hamming',
                       pass_zero=False)
zi_hp = np.zeros((hp_numtaps-1, 2))

# --- Buffers & Lags ---
raw_buf = np.zeros((plot_window_size, 2), dtype=np.int32)
hp_buf  = np.zeros_like(raw_buf)
lags    = np.arange(-plot_window_size+1, plot_window_size)
max_lag = 60
mask    = np.abs(lags) <= max_lag
plot_lags = lags[mask]

lag_hist = deque(maxlen=5)    # 5-frame window

def gcc_phat(x, y, fs):
    """
    Estimate time delay between x and y using GCC-PHAT.
    
    Args:
        x, y : 1D arrays of the same length
        fs   : sampling rate in Hz

    Returns:
        lag         : sample delay (positive => y lags x)
        tau         : time delay in seconds (lag / fs)
        cc          : cross-correlation sequence (length 2*Nfft)
        lags        : lag vector corresponding to cc
    """
    N = len(x) + len(y) - 1
    # zero-pad to next power of 2 for speed (optional)
    Nfft = 1 << (N - 1).bit_length()

    X = np.fft.rfft(x, n=Nfft)
    Y = np.fft.rfft(y, n=Nfft)
    
    R = X * np.conj(Y)
    R /= np.abs(R) + np.finfo(float).eps   # PHAT weighting

    cc = np.fft.irfft(R, n=Nfft*2)         # length = 2*Nfft
    # build lag vector
    lags = np.arange(-Nfft, Nfft)
    
    # restrict to plausible lags (optional)
    max_shift = min(len(x), len(y))
    center = Nfft
    window = slice(center - max_shift, center + max_shift + 1)
    cc_window = cc[window]
    lags_window = lags[window]

    idx = np.argmax(np.abs(cc_window))
    lag = lags_window[idx]
    tau = lag / float(fs)
    return lag, tau, cc_window, lags_window

# --- Plot Setup ---
plt.ion()
fig, (ax_time, ax_fft, ax_xcorr) = plt.subplots(3, 1, figsize=(8, 6))

# time-series: raw (blue) & HPF (cyan)
t = np.arange(plot_window_size)
line_raw_l, = ax_time.plot(t, raw_buf[:,0], 'b-', label='Raw L')
line_raw_r, = ax_time.plot(t, raw_buf[:,1], 'r-', label='Raw R')
line_hp_l,  = ax_time.plot(t, hp_buf[:,0],  'c--', label=f'HPF ≥{hp_cutoff_hz}Hz L')
line_hp_r, = ax_time.plot(t, hp_buf[:,1], 'm--', label=f'HPF ≥{hp_cutoff_hz}Hz R')
ax_time.set_ylabel('Amplitude'); ax_time.legend(loc='upper right'); ax_time.grid()
ax_time.legend(loc='upper right')

# FFT of HPF left channel
f = np.fft.rfftfreq(FFTlen, 1/Fs)
line_fft, = ax_fft.plot(f, np.zeros_like(f), 'g-')
ax_fft.set_xlim(0, Fs/2)
ax_fft.set_ylabel('Magnitude'); ax_fft.grid()

# Cross-corr of HPF L vs R
line_xc, = ax_xcorr.plot(plot_lags, np.zeros_like(plot_lags), 'k-')
ax_xcorr.set_xlim(plot_lags[0], plot_lags[-1])
ax_xcorr.set_ylim(-1,1)
ax_xcorr.set_xlabel('Lag (samples)'); ax_xcorr.set_ylabel('Corr'); ax_xcorr.grid()
# 1) add a twin-x axis for “Direction” labels
ax_dir = ax_xcorr.twiny()
ax_dir.set_xlim(ax_xcorr.get_xlim())
ax_dir.set_xticks([-max_lag-60, max_lag+60])
ax_dir.set_xticklabels(['Left', 'Right'])
ax_dir.xaxis.set_ticks_position('top')
ax_dir.xaxis.set_label_position('top')
ax_dir.set_xlabel('Direction')
dir_dot, = ax_xcorr.plot(0, 0, 'ro', markersize=10)


plt.tight_layout()
plt.show()

# --- Serial Port ---
try:
    s = serial.Serial(serial_port_name, baud_rate, timeout=10)
except serial.SerialException as e:
    print("Serial error:", e); sys.exit(1)

def fig_alive(fig):
    return plt.fignum_exists(fig.number)

# --- Acquisition Loop ---
while fig_alive(fig):
    if s.in_waiting < bytes_to_read:
        plt.pause(0.001)
        continue

    raw = s.read(bytes_to_read)
    if len(raw) != bytes_to_read:
        continue

    # byte-swap to little-endian int32
    ba = bytearray(raw)
    for i in range(0, len(ba), bytes_per_sample):
        ba[i:i+bytes_per_sample] = ba[i:i+bytes_per_sample][::-1]
    data = np.frombuffer(ba, '<i4').reshape(-1,2)

    # update raw history
    raw_buf = np.roll(raw_buf, -samples_per_read, axis=0)
    raw_buf[-samples_per_read:] = data

    # HP FIR filter block
    hp_block, zi_hp = lfilter(hp_taps, [1.0], data, axis=0, zi=zi_hp)
    hp_buf = np.roll(hp_buf, -samples_per_read, axis=0)
    hp_buf[-samples_per_read:] = hp_block

    # 1) update time-series (left channel only shown)
    # Remove these lines if you want to remove the raw data
    #line_raw_l.set_ydata(raw_buf[:,0])
    #line_raw_r.set_ydata(raw_buf[:,1])
    
    line_hp_l.set_ydata(hp_buf[:,0])
    line_hp_r.set_ydata(hp_buf[:,1])
    ax_time.relim(); ax_time.autoscale_view()

    # 2) FFT on HPF history (left channel)
    H = np.fft.rfft(hp_buf[:,0], n=FFTlen)
    line_fft.set_ydata(np.abs(H)/FFTlen)
    ax_fft.relim(); ax_fft.autoscale_view()

    # 3) GCC-PHAT cross-correlation on HPF history
    L = hp_buf[:,0].astype(float)
    R = hp_buf[:,1].astype(float)

    # call gcc_phat to get lag, time delay, and the CC sequence+lags
    lag_samples, tau, cc_vals, cc_lags = gcc_phat(L, R, Fs)

    # optionally restrict to the same ±max_lag window
    mask2 = np.abs(cc_lags) <= max_lag
    line_xc.set_xdata(cc_lags[mask2])
    line_xc.set_ydata(cc_vals[mask2])

    # 3a) smooth the lag and update the direction dot
    lag_hist.append(lag_samples)
    lag_med = np.median(lag_hist)
    dir_dot.set_xdata(lag_med)

    # keep the top “Direction” axis in sync
    ax_dir.set_xlim(ax_xcorr.get_xlim())

    # refresh
    fig.canvas.flush_events()
    plt.pause(0.001)

# cleanup
s.close()
plt.ioff()
plt.close(fig)
