'''
Plotting Real vs Measured Distance and Error Measurement for IR and Aruco Sensors
This script reads sensor data from 'sensor_data.csv', processes it to filter out invalid readings,
and generates two plots:
1. Real vs Measured Distance (Raw Data)
2. Error Measurement (Filtered Data)
'''
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# Load data
df = pd.read_csv('sensor_data.csv')

# --- Data Preparation ---
# Filter IR for error graph (remove saturated values)
ir_valid = df[df['IR (cm)'] < 63].copy()
ir_valid['Error'] = ir_valid['IR (cm)'] - ir_valid['Real (cm)']

# Filter Aruco for error graph (remove undetected values)
aruco_valid = df[df['Aruco (cm)'] > 0].copy()
aruco_valid['Error'] = aruco_valid['Aruco (cm)'] - aruco_valid['Real (cm)']

# --- Plotting ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

# --- Graph 1: Real vs Measured (Raw Data) ---
ax1.plot(df['Real (cm)'], df['IR (cm)'], marker='o', label='IR (Raw)', color='blue', alpha=0.6)
ax1.plot(df['Real (cm)'], df['Aruco (cm)'], marker='s', label='Aruco (Raw)', color='green', alpha=0.6)
ax1.plot([0, 100], [0, 100], linestyle='--', color='red', label='Ideal')

ax1.set_title('1. Real vs Measured Distance (Raw)')
ax1.set_xlabel('Real Distance (cm)')
ax1.set_ylabel('Measured Distance (cm)')

# Finer Axis Settings for Graph 1
ax1.xaxis.set_major_locator(ticker.MultipleLocator(5)) # X-ticks every 5 cm
ax1.yaxis.set_major_locator(ticker.MultipleLocator(5)) # Y-ticks every 5 cm
ax1.grid(True, which='both', linestyle='-', alpha=0.5)
ax1.legend()

# --- Graph 2: Error Measurement (Filtered) ---
ax2.plot(ir_valid['Real (cm)'], ir_valid['Error'], marker='o', label='IR Error', color='blue')
ax2.plot(aruco_valid['Real (cm)'], aruco_valid['Error'], marker='s', label='Aruco Error', color='green')
ax2.axhline(0, color='red', linestyle='--', label='Zero Error')

ax2.set_title('2. Error Measurement (Filtered)')
ax2.set_xlabel('Real Distance (cm)')
ax2.set_ylabel('Error (cm)')

# Finer Axis Settings for Graph 2
ax2.set_ylim(-10, 10)                                   # Limit -10 to 10
ax2.xaxis.set_major_locator(ticker.MultipleLocator(5))  # X-ticks every 5 cm
ax2.yaxis.set_major_locator(ticker.MultipleLocator(1))  # Y-ticks every 1 cm
ax2.grid(True, which='both', linestyle='-', alpha=0.5)
ax2.legend()

plt.tight_layout()
plt.show()