import serial
import matplotlib.pyplot as plt
import numpy as np
import sys  # For exiting the script cleanly

# --- Configuration ---
serial_port_name = 'COM3'     # Serial port name
baud_rate        = 921600      # Baud rate
plot_window_size = 500        # Number of data points (L/R pairs) to display
samples_per_read = 64         # How many L/R pairs to read at once
bytes_per_sample = 4           # sizeof(int32_t)
bytes_per_pair   = bytes_per_sample * 2 # L + R = 8 bytes
bytes_to_read    = samples_per_read * bytes_per_pair # 64 * 8 = 512 bytes
Fs               = 32000       # audio sampling rate (Hz)
FFTlen           = 512         # FFT length (power of 2 for efficiency)
halfFFT          = FFTlen // 2

# --- Serial Port Setup ---
print(f"Attempting to connect to {serial_port_name} at {baud_rate} baud...")
try:
    s = serial.Serial(serial_port_name, baud_rate, timeout=10)
    print("Serial port connected successfully.")
except serial.SerialException as e:
    print(f"Error connecting to {serial_port_name}: {e}")
    sys.exit(1)

# --- Plot Initialization ---
print("Initializing plot...")
plot_data  = np.zeros((plot_window_size, 2), dtype=np.int32)
time_vector = np.arange(1, plot_window_size + 1)
freq_vector = np.fft.fftfreq(FFTlen, 1/Fs)[:halfFFT]

plt.ion()
fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(8, 6))

line_L, = ax_time.plot(time_vector, plot_data[:, 0], 'b-', label='Left')
line_R, = ax_time.plot(time_vector, plot_data[:, 1], 'r-', label='Right')
ax_time.set_title(f'Real-Time BINARY Serial Data (Last {plot_window_size} Pairs)')
ax_time.set_xlabel('Sample Index')
ax_time.set_ylabel('ADC Value (int32)')
ax_time.legend(loc='best')
ax_time.grid(True)
ax_time.relim()
ax_time.autoscale_view()

line_fft, = ax_fft.plot(freq_vector, np.zeros(halfFFT), 'k-', label='FFT (Left)')
ax_fft.set_title('FFT (Left Channel)')
ax_fft.set_xlabel('Frequency (Hz)')
ax_fft.set_ylabel('Magnitude')
ax_fft.set_xlim(0, Fs/2)
ax_fft.grid(True)

plt.tight_layout()
plt.show()
print("Starting real-time acquisition... Close the figure window to stop.")

def is_figure_open(fig_handle):
    return plt.fignum_exists(fig_handle.number)

keep_running = True

try:
    while keep_running and is_figure_open(fig):
        try:
            bytes_available = s.in_waiting
        except Exception as e:
            print(f"Serial port error: {e}")
            break

        if bytes_available >= bytes_to_read:
            new_bytes = s.read(bytes_to_read)
            if len(new_bytes) != bytes_to_read:
                print(f"Warning: expected {bytes_to_read} bytes, got {len(new_bytes)}. Skipping.")
                continue

            # --- Reorder each 4-byte word in place ---
            ba = bytearray(new_bytes)
            for i in range(0, len(ba), bytes_per_sample):
                ba[i:i+bytes_per_sample] = ba[i:i+bytes_per_sample][::-1]

            # Convert to int32 little-endian and reshape into (samples_per_read, 2)
            data  = np.frombuffer(ba, dtype='<i4')
            pairs = data.reshape(-1, 2)

            # Roll plot buffer and append new data
            plot_data = np.roll(plot_data, -samples_per_read, axis=0)
            plot_data[-samples_per_read:] = pairs

            # Update time-domain plot
            line_L.set_ydata(plot_data[:, 0])
            line_R.set_ydata(plot_data[:, 1])
            ax_time.relim()
            ax_time.autoscale_view()

            # --- Compute & update FFT (left channel) ---
            Y = np.fft.fft(plot_data[:, 0], FFTlen)
            P2 = np.abs(Y / FFTlen)
            P1 = P2[:halfFFT]
            line_fft.set_ydata(P1)
            ax_fft.relim()
            ax_fft.autoscale_view()
            
            fig.canvas.flush_events()
            plt.pause(0.001)

except KeyboardInterrupt:
    pass
finally:
    print("Stopping acquisition and closing serial port...")
    s.close()
    plt.ioff()
    plt.close(fig)