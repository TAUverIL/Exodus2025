import re
import sys
import numpy as np
import matplotlib.pyplot as plt

# Usage: python CTE_Extraction3.py [log_file]
log_file = 'ros2_log_KSteer_0_5.txt'

# Read log file
try:
    with open(log_file, 'r') as f:
        lines = f.readlines()
except FileNotFoundError:
    print(f"Log file '{log_file}' not found.")
    sys.exit(1)

# Containers for pose, path, and metrics
delta_times, deltas, dist_times, dists = [], [], [], []
pose_x, pose_y, closest_x, closest_y = [], [], [], []
pose_times = []

# Parse log
for line in lines:
    if 'CORRECTED_POSE' in line:
        m = re.search(r"x=\s*([\-\d\.]+),\s*y=\s*([\-\d\.]+)", line)
        t_match = re.search(r"\[(\d+\.\d+)\]", line)
        if m and t_match:
            pose_x.append(float(m.group(1)))
            pose_y.append(float(m.group(2)))
            pose_times.append(float(t_match.group(1)))
    if 'Closest Point' in line and 'in Map' in line:
        m = re.search(r"\(X Y\).*?\s*([\-\d\.]+)\s+([\-\d\.]+)\)", line)
        if m:
            closest_x.append(float(m.group(1)))
            closest_y.append(float(m.group(2)))
    if 'Delta:' in line and 'X-track Err' in line:
        m = re.search(r"\[(\d+\.\d+)\].*Delta:\s*([\-\d\.]+)", line)
        if m:
            delta_times.append(float(m.group(1)))
            deltas.append(float(m.group(2)))
    if 'Distance to nearest point' in line:
        m = re.search(r"\[(\d+\.\d+)\].*Distance to nearest point:\s*([\-\d\.]+)", line)
        if m:
            dist_times.append(float(m.group(1)))
            dists.append(float(m.group(2)))

# Waypoint send and receive times (absolute ROS time)  -- updated for latest log
abs_sent_times = [
    1748180164.260061870,  # Sending waypoint (10.00, 0.00)
    1748180183.228169634,  # Sending waypoint (10.00, -25.00)
    1748180239.825969101,  # Sending waypoint (-25.00, -25.00)
    1748180316.121921737   # Sending waypoint (5.00, -55.00)
]
abs_recv_times = [
    1748180183.226132482,  # ✅ Waypoint reached first
    1748180239.824891450,  # ✅ Waypoint reached second
    1748180316.118054624,  # ✅ Waypoint reached third
    1748180428.052589493   # ✅ Waypoint reached fourth
]
# Corresponding targets
targets = [
    (10.00, 0.00),
    (10.00, -25.00),
    (-25.00, -25.00),
    (5.00, -55.00)
]

# Normalize time base
t0 = abs_sent_times[0]
sent_times = np.array(abs_sent_times) - t0
recv_times = np.array(abs_recv_times) - t0

# If your log timestamps are already absolute ROS times:
pose_times = np.array(pose_times) - t0
delta_times = np.array(delta_times) - t0
dist_times  = np.array(dist_times)  - t0

print("  pose_times    :", pose_times.min(), "→", pose_times.max())
print("  delta_times   :", delta_times.min(), "→", delta_times.max())
print("  dist_times    :", dist_times.min(), "→", dist_times.max())
print("  sent_times    :", sent_times)
print("  recv_times    :", recv_times)

# Convert lists to arrays
deltas = np.array(deltas)
dists = np.array(dists)
pose_x = np.array(pose_x)
pose_y = np.array(pose_y)
closest_x = np.array(closest_x)
closest_y = np.array(closest_y)

# Find minimal distance to each target during each segment
# Use pre-spike distance measurements to get accurate cross-track error
min_events = []  # (t_min, idx, clean_distance)
for i, (ts, tr) in enumerate(zip(sent_times, recv_times)):
    mask = (pose_times >= ts) & (pose_times <= tr)
    if not np.any(mask): continue
    xs, ys, ts_seg = pose_x[mask], pose_y[mask], pose_times[mask]
    tx, ty = targets[i]
    d2t = np.hypot(xs - tx, ys - ty)
    idx_min = np.argmin(d2t)
    t_closest = ts_seg[idx_min]
    
    # Get distance measurement from a stable period before waypoint reached
    # Look for measurements in the final approach phase, avoiding the spike
    pre_spike_window = 2.0  # seconds before waypoint reached
    window_start = max(tr - pre_spike_window, ts)
    window_end = tr - 0.1  # Stop just before the spike
    
    # Find distance measurements in the stable approach window
    stable_mask = (dist_times >= window_start) & (dist_times <= window_end)
    
    if np.any(stable_mask):
        stable_dists = dists[stable_mask]
        stable_times = dist_times[stable_mask]
        
        # Find the minimum distance in this stable window
        min_stable_idx = np.argmin(stable_dists)
        clean_distance = stable_dists[min_stable_idx]
    else:
        # Fallback: use interpolated value from stable data
        # Find a time point just before potential spike
        safe_time = tr - 0.5
        clean_distance = float(np.interp(safe_time, dist_times, dists))
    
    min_events.append((t_closest, i, clean_distance))

# --- Plot 1: Robot Pose vs Reference Path ---
plt.figure(figsize=(8, 6))
plt.plot(pose_x, pose_y, '-', linewidth=2, label='Robot Pose')
plt.plot(closest_x, closest_y, '--', linewidth=2, label='Reference Path')
plt.title('Robot Pose vs Reference Path')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend(loc='best')
plt.grid(True)
plt.axis('equal')
plt.show()

# --- Plot 2: Delta and Distance over Time with markers ---
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
# Plot curves
axs[0].plot(delta_times, deltas, '-', label='Delta', color='#007BA7')
axs[1].plot(dist_times, dists, '-', label='Distance to Path')
# Titles and labels
axs[0].set_title('Delta over Time')
axs[0].set_ylabel('Delta [deg]')
axs[1].set_title('Distance to Path over Time')
axs[1].set_ylabel('Distance [m]')
axs[1].set_xlabel('Time [s]')
# Markers and collect handles for legend
handles0, labels0 = axs[0].get_legend_handles_labels()
handles1, labels1 = axs[1].get_legend_handles_labels()
colors = [
    '#FF7F0E',  # Warm Orange
    '#2CA02C',  # Forest Green
    '#D62728',  # Brick Red
    '#9467BD',  # Slate Purple
]
for tmin, idx, clean_dist in min_events:
    # Delta subplot marker
    h0 = axs[0].scatter(tmin, np.interp(tmin, delta_times, deltas),
                        marker='x', color=colors[idx], s=100)
    # Distance subplot marker - use clean distance from before spike
    h1 = axs[1].scatter(tmin, clean_dist,
                        marker='o', color=colors[idx], s=100)
    label = f"Closest to WP{idx+1}"
    handles0.append(h0); labels0.append(label)
    handles1.append(h1); labels1.append(label)
# Final legends
axs[0].legend(handles0, labels0, loc='upper right')
axs[1].legend(handles1, labels1, loc='upper right')
# Grid
axs[0].grid(True)
axs[1].grid(True)
plt.tight_layout()
plt.show()

# --- Export data to text file ---
output_filename = 'plot_data_output_0_5.txt'
with open(output_filename, 'w') as out:
    out.write('Robot Pose Points (X, Y):\n')
    for x, y in zip(pose_x, pose_y):
        out.write(f"{x:.6f}, {y:.6f}\n")
    out.write('\nReference Path Points (X, Y):\n')
    for x, y in zip(closest_x, closest_y):
        out.write(f"{x:.6f}, {y:.6f}\n")
    out.write('\nSteering Data (Time [s], Delta [deg]):\n')
    for t, d in zip(delta_times, deltas):
        out.write(f"{t:.6f}, {d:.6f}\n")
    out.write('\nDistance Data (Time [s], Distance [m]):\n')
    for t, dist in zip(dist_times, dists):
        out.write(f"{t:.6f}, {dist:.6f}\n")
    out.write('\nClosest-to-Waypoint Events (WP index, Time [s], Delta [deg], Distance [m]):\n')
    # Use clean distance measurements from before spikes
    for tmin, idx, clean_dist in min_events:
        d_val = float(np.interp(tmin, delta_times, deltas))
        out.write(f"WP{idx+1}, {tmin:.6f}, {d_val:.6f}, {clean_dist:.6f}\n")

print(f"Data exported to {output_filename}")
print("\nGoal Achievement Summary:")
for tmin, idx, clean_dist in min_events:
    print(f"WP{idx+1} at {targets[idx]}: Cross-track Error = {clean_dist:.3f}m at t={tmin:.1f}s")