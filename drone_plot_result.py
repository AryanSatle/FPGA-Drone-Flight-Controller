import csv
import matplotlib.pyplot as plt

time, setpoint, altitude, error, control = [], [], [], [], []

with open('drone_pid_output.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        time.append(int(row['Time']))
        setpoint.append(int(row['Setpoint']))
        altitude.append(int(row['Altitude']))
        error.append(int(row['Error']))
        control.append(int(row['ControlSignal']))

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

ax1.plot(time, setpoint, 'r--', label='Setpoint (Target)')
ax1.plot(time, altitude, 'b-', linewidth=2, label='Actual Altitude')
ax1.axvspan(75, 85, color='gray', alpha=0.2, label='Wind Disturbance')
ax1.set_title('Drone Altitude Tracking')
ax1.set_ylabel('Altitude')
ax1.grid(True, linestyle=':', alpha=0.7)
ax1.legend()

ax2.plot(time, error, 'g-', linewidth=2, label='Error (Setpoint - Altitude)')
ax2.axvspan(75, 85, color='gray', alpha=0.2)
ax2.set_title('System Error over Time')
ax2.set_ylabel('Error Margin')
ax2.grid(True, linestyle=':', alpha=0.7)
ax2.legend()

ax3.plot(time, control, 'm-', linewidth=2, label='PID Output (Thrust/Power)')
ax3.axvspan(75, 85, color='gray', alpha=0.2)
ax3.set_title('Controller Effort')
ax3.set_xlabel('Time (Clock Cycles)')
ax3.set_ylabel('Control Signal')
ax3.grid(True, linestyle=':', alpha=0.7)
ax3.legend()

plt.tight_layout()
plt.savefig('verification_plots.png')
plt.show()
