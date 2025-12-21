import serial
import re
import time
import matplotlib.pyplot as plt
import numpy as np

# ============================================================================
# CONFIGURATION MACROS - ONLY CHANGE THESE VALUES
# ============================================================================

SERIAL_PORT = 'COM6'  # Windows - change to '/dev/ttyUSB0' for Linux or '/dev/cu.usbserial-1410' for macOS
BAUD_RATE = 115200

# Collection settings
COLLECTION_TIME = 5  # seconds
SAMPLING_RATE = 256  # SPS

# ============================================================================
# END OF CONFIGURATION - NO NEED TO MODIFY BELOW
# ============================================================================

# Regex to extract ECG value from your format: "ECG: <value> | Count: <count>"
data_pattern = re.compile(r'ECG:\s*([-+]?\d+)\s*\|\s*Count:\s*([-+]?\d+)')

# Storage
ecg_buffer = []
timestamps = []


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
    print(f"ECG Data Logger - Raw Signal (No Filtering)")
    print("=" * 80)
    print(f"Serial Port: {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"Sampling Rate: {SAMPLING_RATE} SPS")
    print(f"Collection Time: {COLLECTION_TIME} seconds")
    print("Waiting for data...\n")
    
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
        print("DATA COLLECTION COMPLETE - PLOTTING")
        print("=" * 80)
        print(f"✓ ECG samples collected:  {len(ecg_buffer)}")
        
        # Calculate sampling rates
        actual_ecg_sps = len(ecg_buffer) / COLLECTION_TIME
        print(f"✓ ECG sampling rate:      {actual_ecg_sps:.1f} SPS")
        
        # Create figure
        fig, ax = plt.subplots(1, 1, figsize=(14, 6))
        fig.suptitle(f'ECG Signal - {COLLECTION_TIME} Second Collection @ {SAMPLING_RATE} SPS (Raw, No Filtering)', 
                     fontsize=16, fontweight='bold')
        
        # Generate time axis for ECG
        ecg_time = np.linspace(0, COLLECTION_TIME, len(ecg_buffer))
        
        # Plot original ECG
        ax.plot(ecg_time, ecg_buffer, color='blue', linewidth=0.8, label='ECG (Raw)', alpha=0.8)
        ax.set_xlabel('Time (seconds)', fontsize=11)
        ax.set_ylabel('ECG (µV)', fontsize=11)
        ax.set_title(f'ECG Signal - Raw - {len(ecg_buffer)} samples @ {actual_ecg_sps:.1f} SPS', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        
        # Add statistics to plot
        ecg_min, ecg_max = min(ecg_buffer), max(ecg_buffer)
        ecg_mean = np.mean(ecg_buffer)
        ecg_std = np.std(ecg_buffer)
        ecg_stats_text = f'Min: {ecg_min}\nMax: {ecg_max}\nMean: {ecg_mean:.1f}\nStd Dev: {ecg_std:.1f}'
        ax.text(0.02, 0.95, ecg_stats_text, transform=ax.transAxes, 
                fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        plt.show()
        
        print("\n" + "=" * 80)
        print("SIGNAL STATISTICS")
        print("=" * 80)
        print(f"\nECG Signal (Raw):")
        print(f"  Min:       {ecg_min:8d} µV")
        print(f"  Max:       {ecg_max:8d} µV")
        print(f"  Range:     {ecg_max - ecg_min:8d} µV")
        print(f"  Mean:      {ecg_mean:8.1f} µV")
        print(f"  Std Dev:   {ecg_std:8.1f} µV")
        
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
