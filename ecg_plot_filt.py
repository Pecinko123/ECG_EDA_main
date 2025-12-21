import serial
import re
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter

# ============================================================================
# CONFIGURATION MACROS - ONLY CHANGE THESE VALUES
# ============================================================================

SERIAL_PORT = 'COM6'  # Windows - change to '/dev/ttyUSB0' for Linux or '/dev/cu.usbserial-1410' for macOS
BAUD_RATE = 115200

# Collection settings
COLLECTION_TIME = 5  # seconds
SAMPLING_RATE = 256  # SPS

# Filter settings
LOWCUT_FREQ = 0.5  # Hz - removes baseline wander
HIGHCUT_FREQ = 40 # Hz - removes noise/muscle artifacts
FILTER_ORDER = 4  # Higher = steeper but more phase distortion

# Smoothing filter settings
SAVGOL_WINDOW = 11  # Window length (must be odd) - default 11 ≈ 43ms at 256 SPS
SAVGOL_POLYORDER = 3  # Polynomial order (default 3 = cubic)

# ============================================================================
# END OF CONFIGURATION - NO NEED TO MODIFY BELOW
# ============================================================================

# Regex to extract ECG value from your format: "ECG: <value> | Count: <count>"
data_pattern = re.compile(r'ECG:\s*([-+]?\d+)\s*\|\s*Count:\s*([-+]?\d+)')

# Storage
ecg_buffer = []
timestamps = []


def design_ecg_filter(fs=SAMPLING_RATE, lowcut=LOWCUT_FREQ, highcut=HIGHCUT_FREQ, order=FILTER_ORDER):
    """
    Design a Butterworth bandpass filter for ECG signal
    
    Parameters:
    - fs: Sampling frequency (Hz)
    - lowcut: Low cutoff frequency (Hz) - removes baseline wander
    - highcut: High cutoff frequency (Hz) - removes noise/muscle artifacts
    - order: Filter order (higher = steeper but more phase distortion)
    
    Returns:
    - b, a: Filter coefficients for scipy.signal.filtfilt()
    """
    nyquist = fs / 2.0
    
    # Normalize frequencies to Nyquist frequency
    low = lowcut / nyquist
    high = highcut / nyquist
    
    # Ensure frequencies are within valid range (0, 1)
    if low <= 0:
        low = 0.001
    if high >= 1:
        high = 0.999
    
    # Design the filter
    b, a = butter(order, [low, high], btype='band', analog=False)
    
    return b, a


def apply_ecg_filter(ecg_signal, b, a):
    """
    Apply zero-phase digital filter to ECG signal using filtfilt
    (forward-backward filtering for zero phase distortion)
    
    Parameters:
    - ecg_signal: Input ECG signal array
    - b, a: Filter coefficients from design_ecg_filter()
    
    Returns:
    - filtered_ecg: Filtered ECG signal
    """
    if len(ecg_signal) < max(len(b), len(a)):
        print("Warning: Signal too short for filtering, returning original")
        return np.array(ecg_signal)
    
    # filtfilt applies filter forward and backward (zero phase distortion)
    filtered_ecg = filtfilt(b, a, ecg_signal)
    return filtered_ecg


def apply_smoothing_filter(signal, window_length=SAVGOL_WINDOW, polyorder=SAVGOL_POLYORDER):
    """
    Apply Savitzky-Golay smoothing filter to signal
    (polynomial smoothing - preserves peaks better than moving average)
    
    Parameters:
    - signal: Input signal array
    - window_length: Length of filter window (must be odd, default 11 ≈ 43ms at 256 SPS)
    - polyorder: Order of polynomial (default 3 = cubic)
    
    Returns:
    - smoothed_signal: Smoothed signal
    """
    if len(signal) < window_length:
        print("Warning: Signal too short for smoothing filter")
        return np.array(signal)
    
    # Ensure window_length is odd
    if window_length % 2 == 0:
        window_length += 1
    
    # Apply Savitzky-Golay filter (preserves signal features better than moving average)
    smoothed_signal = savgol_filter(signal, window_length, polyorder)
    return smoothed_signal


def extract_data(raw_line):
    """Extract ECG value from serial line in format: 'ECG: <value> | Count: <count>'"""
    match = data_pattern.search(raw_line)
    if match:
        ecg = int(match.group(1))
        count = int(match.group(2))
        return ecg, count
    return None, None


def main():
    print("=" * 80)
    print(f"ECG Data Logger with Butterworth Bandpass + Savitzky-Golay Smoothing Filters")
    print("=" * 80)
    print(f"Serial Port: {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"Sampling Rate: {SAMPLING_RATE} SPS")
    print(f"Collection Time: {COLLECTION_TIME} seconds")
    print(f"\nFilter Settings:")
    print(f"  Butterworth Bandpass: {LOWCUT_FREQ} - {HIGHCUT_FREQ} Hz (order {FILTER_ORDER})")
    print(f"  Savitzky-Golay Smoothing: window={SAVGOL_WINDOW}, polyorder={SAVGOL_POLYORDER}")
    print("Waiting for data...\n")
    
    # Design ECG filter
    b_ecg, a_ecg = design_ecg_filter(fs=SAMPLING_RATE, lowcut=LOWCUT_FREQ, highcut=HIGHCUT_FREQ, order=FILTER_ORDER)
    print(f"✓ Filter 1 designed: Butterworth Bandpass for ECG ({LOWCUT_FREQ}-{HIGHCUT_FREQ} Hz, order {FILTER_ORDER}) @ {SAMPLING_RATE} SPS")
    print(f"✓ Filter 2 designed: Savitzky-Golay Smoothing (window={SAVGOL_WINDOW}, polyorder={SAVGOL_POLYORDER})")
    print("=" * 80 + "\n")
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(1)  # Wait for connection to stabilize
    
    try:
        start_time = time.time()
        sample_count = 0
        
        while (time.time() - start_time) < COLLECTION_TIME:
            try:
                raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if raw_line:
                    ecg, count = extract_data(raw_line)
                    
                    if ecg is not None and count is not None:
                        # Collect ECG samples
                        ecg_buffer.append(ecg)
                        
                        # Store timestamp
                        elapsed = time.time() - start_time
                        timestamps.append(elapsed)
                        sample_count += 1
                        
                        # Progress indicator
                        progress_bar = "█" * int((elapsed / COLLECTION_TIME) * 50)
                        print(f"\r[{progress_bar:<50}] {elapsed:.1f}s / {COLLECTION_TIME}s - {sample_count} samples", end='', flush=True)
                        
            except Exception as e:
                continue
        
        print("\n\n" + "=" * 80)
        print("DATA COLLECTION COMPLETE - FILTERING AND PLOTTING")
        print("=" * 80)
        print(f"✓ ECG samples collected:  {len(ecg_buffer)}")
        
        # Calculate sampling rates
        actual_ecg_sps = len(ecg_buffer) / COLLECTION_TIME
        print(f"✓ ECG sampling rate:      {actual_ecg_sps:.1f} SPS")
        
        # Apply bandpass filter to ECG signal
        print("✓ Applying Butterworth bandpass filter to ECG...")
        ecg_filtered = apply_ecg_filter(np.array(ecg_buffer), b_ecg, a_ecg)
        
        # Apply smoothing filter to bandpass-filtered ECG
        print("✓ Applying Savitzky-Golay smoothing filter to ECG...")
        ecg_smoothed = apply_smoothing_filter(ecg_filtered, window_length=SAVGOL_WINDOW, polyorder=SAVGOL_POLYORDER)
        print(f"✓ Filters applied successfully")
        
        # Create figure with subplots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 12))
        fig.suptitle(f'ECG Signal - {COLLECTION_TIME} Second Collection @ {SAMPLING_RATE} SPS (with Filtering & Smoothing)', 
                     fontsize=16, fontweight='bold')
        
        # Generate time axis for ECG
        ecg_time = np.linspace(0, COLLECTION_TIME, len(ecg_buffer))
        
        # Plot original ECG
        ax1.plot(ecg_time, ecg_buffer, color='blue', linewidth=0.8, label='ECG (Original)', alpha=0.7)
        ax1.set_xlabel('Time (seconds)', fontsize=11)
        ax1.set_ylabel('ECG (µV)', fontsize=11)
        ax1.set_title(f'ECG Signal - Original - {len(ecg_buffer)} samples @ {actual_ecg_sps:.1f} SPS', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right', fontsize=10)
        
        # Add statistics to original ECG plot
        ecg_min, ecg_max = min(ecg_buffer), max(ecg_buffer)
        ecg_mean = np.mean(ecg_buffer)
        ecg_stats_text = f'Min: {ecg_min}\nMax: {ecg_max}\nMean: {ecg_mean:.1f}'
        ax1.text(0.02, 0.95, ecg_stats_text, transform=ax1.transAxes, 
                fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Plot bandpass filtered ECG
        ax2.plot(ecg_time, ecg_filtered, color='red', linewidth=0.8, label=f'ECG (Bandpass {LOWCUT_FREQ}-{HIGHCUT_FREQ} Hz)')
        ax2.set_xlabel('Time (seconds)', fontsize=11)
        ax2.set_ylabel('ECG (µV)', fontsize=11)
        ax2.set_title(f'ECG Signal - Butterworth Bandpass ({LOWCUT_FREQ}-{HIGHCUT_FREQ} Hz, order {FILTER_ORDER})', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right', fontsize=10)
        
        # Add statistics to bandpass filtered ECG plot
        ecg_filt_min, ecg_filt_max = np.min(ecg_filtered), np.max(ecg_filtered)
        ecg_filt_mean = np.mean(ecg_filtered)
        ecg_filt_stats_text = f'Min: {ecg_filt_min:.0f}\nMax: {ecg_filt_max:.0f}\nMean: {ecg_filt_mean:.1f}'
        ax2.text(0.02, 0.95, ecg_filt_stats_text, transform=ax2.transAxes, 
                fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5))
        
        # Plot smoothed ECG
        ax3.plot(ecg_time, ecg_smoothed, color='purple', linewidth=1.2, label='ECG (Bandpass + Savitzky-Golay)', alpha=0.9)
        ax3.set_xlabel('Time (seconds)', fontsize=11)
        ax3.set_ylabel('ECG (µV)', fontsize=11)
        ax3.set_title(f'ECG Signal - Smoothed (Savitzky-Golay: window={SAVGOL_WINDOW}, polyorder={SAVGOL_POLYORDER})', fontsize=12)
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc='upper right', fontsize=10)
        
        # Add statistics to smoothed ECG plot
        ecg_smooth_min, ecg_smooth_max = np.min(ecg_smoothed), np.max(ecg_smoothed)
        ecg_smooth_mean = np.mean(ecg_smoothed)
        ecg_smooth_stats_text = f'Min: {ecg_smooth_min:.0f}\nMax: {ecg_smooth_max:.0f}\nMean: {ecg_smooth_mean:.1f}'
        ax3.text(0.02, 0.95, ecg_smooth_stats_text, transform=ax3.transAxes, 
                fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='plum', alpha=0.5))
        
        plt.tight_layout()
        plt.show()
        
        print("\n" + "=" * 80)
        print("SIGNAL STATISTICS")
        print("=" * 80)
        print(f"\nECG Signal (Original):")
        print(f"  Min:      {ecg_min:8d} µV")
        print(f"  Max:      {ecg_max:8d} µV")
        print(f"  Range:    {ecg_max - ecg_min:8d} µV")
        print(f"  Mean:     {ecg_mean:8.1f} µV")
        
        print(f"\nECG Signal (Bandpass {LOWCUT_FREQ}-{HIGHCUT_FREQ} Hz):")
        print(f"  Min:      {ecg_filt_min:8.1f} µV")
        print(f"  Max:      {ecg_filt_max:8.1f} µV")
        print(f"  Range:    {ecg_filt_max - ecg_filt_min:8.1f} µV")
        print(f"  Mean:     {ecg_filt_mean:8.1f} µV")
        
        print(f"\nECG Signal (Bandpass + Savitzky-Golay Smoothed):")
        print(f"  Min:      {ecg_smooth_min:8.1f} µV")
        print(f"  Max:      {ecg_smooth_max:8.1f} µV")
        print(f"  Range:    {ecg_smooth_max - ecg_smooth_min:8.1f} µV")
        print(f"  Mean:     {ecg_smooth_mean:8.1f} µV")
        
        if ecg_max - ecg_min > 0:
            noise_reduction = ((ecg_max - ecg_min) - (ecg_smooth_max - ecg_smooth_min)) / (ecg_max - ecg_min) * 100
            print(f"  Noise reduction: {noise_reduction:.1f}%")
        
        print("\n" + "=" * 80)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        ser.close()
        print("Serial port closed")


if __name__ == '__main__':
    main()
