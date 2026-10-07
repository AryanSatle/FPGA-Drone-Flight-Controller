"""This code provides an interactive Jupyter Notebook simulation of a drone using a PID controller 
to maintain altitude against wind disturbances.
"""

import ipywidgets as widgets
from IPython.display import display, clear_output, HTML
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import random

kp_slider = widgets.FloatSlider(value=1.5, min=0.0, max=10.0, step=0.1, description='Kp (Power):')
ki_slider = widgets.FloatSlider(value=0.1, min=0.0, max=2.0, step=0.01, description='Ki (Memory):')
kd_slider = widgets.FloatSlider(value=0.5, min=0.0, max=5.0, step=0.1, description='Kd (Brakes):')
noise_slider = widgets.FloatSlider(value=2.0, min=0.0, max=10.0, step=0.5, description='Wind Noise:')
run_button = widgets.Button(description="Run Simulation & Plot", button_style='success')
output_area = widgets.Output()

def run_sim_and_plot(b):
    with output_area:
        clear_output(wait=True)
        
        duration, dt = 15.0, 0.1
        steps = int(duration / dt)
        setpoint = 10.0

        Kp, Ki, Kd = kp_slider.value, ki_slider.value, kd_slider.value
        max_noise = noise_slider.value

        alt, vel, error_prior, integral = 0.0, 0.0, 0.0, 0.0
        history_alt = []
        history_time = [i * dt for i in range(steps)]

        for i in range(steps):
            time = i * dt
            error = setpoint - alt
            integral += error * dt
            derivative = (error - error_prior) / dt

            output = (Kp * error) + (Ki * integral) + (Kd * derivative)
            noise = random.uniform(-max_noise, max_noise) if time > 6.0 else 0

            accel = output - 9.8 + noise
            vel += accel * dt
            alt += vel * dt

            if alt < 0: alt, vel = 0, 0
            history_alt.append(alt)
            error_prior = error

        plt.figure(figsize=(10, 5))
        plt.plot(history_time, [setpoint]*steps, 'r--', label="Target Setpoint", alpha=0.8)
        plt.plot(history_time, history_alt, color='#1f77b4', linewidth=2, label="Drone Altitude")
        
        plt.axvspan(6, 15, color='gray', alpha=0.15, label="Disturbance Zone")
        
        plt.title(f"PID Tuning Results (Kp={Kp}, Ki={Ki}, Kd={Kd})", fontsize=12)
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (m)")
        plt.ylim(0, max(max(history_alt) + 2, 15))
        plt.grid(True, linestyle=':', alpha=0.7)
        plt.legend(loc='lower right')
        plt.show()

run_button.on_click(run_sim_and_plot)

print("Adjust the sliders and click the button to see the flight path.")
display(kp_slider, ki_slider, kd_slider, noise_slider, run_button, output_area)

Kp = kp_slider.value
Ki = ki_slider.value
Kd = kd_slider.value
max_noise = noise_slider.value

duration = 15.0
dt = 0.1
steps = int(duration / dt)
setpoint = 10.0

altitude = 0.0
velocity = 0.0
error_prior = 0.0
integral = 0.0
history_alt = []

for i in range(steps):
    time = i * dt
    error = setpoint - altitude
    integral += error * dt
    derivative = (error - error_prior) / dt

    control_output = (Kp * error) + (Ki * integral) + (Kd * derivative)
    noise = random.uniform(-max_noise, max_noise) if time > 6.0 else 0

    acceleration = control_output - 9.8 + noise
    velocity += acceleration * dt
    altitude += velocity * dt

    if altitude < 0: 
        altitude = 0
        velocity = 0

    history_alt.append(altitude)
    error_prior = error

fig, ax = plt.subplots(figsize=(5, 7))
ax.set_xlim(-5, 5)
ax.set_ylim(0, 20)
ax.axhline(setpoint, color='red', linestyle='--', label="Target")
ax.set_title(f"Flight Status\nKp={Kp}, Ki={Ki}, Kd={Kd}")
ax.set_xticks([])

drone_body, = ax.plot([], [], 'ks', markersize=12)
prop_line, = ax.plot([], [], 'k-', linewidth=2)
status_text = ax.text(-4, 18, '', fontsize=10, color='red')

def init():
    drone_body.set_data([], [])
    prop_line.set_data([], [])
    status_text.set_text('')
    return drone_body, prop_line, status_text

def update(frame):
    curr_alt = history_alt[frame]
    time = frame * dt

    drone_body.set_data([0], [curr_alt])
    prop_line.set_data([-0.7, 0.7], [curr_alt + 0.1, curr_alt + 0.1])

    if time > 6.0 and max_noise > 0:
        status_text.set_text("WIND GUSTS ACTIVE!")
    else:
        status_text.set_text("Stable Air")

    return drone_body, prop_line, status_text

ani = FuncAnimation(fig, update, frames=range(0, steps, 2), init_func=init, blit=True, interval=50)

plt.close()
display(HTML(ani.to_html5_video()))
